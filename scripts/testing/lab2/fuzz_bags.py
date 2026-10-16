#!/usr/bin/env python3
"""TA fuzz test of the Lab 2 Part 2 gates: do they catch bad runs?

    python3 scripts/testing/lab2/fuzz_bags.py BAG [--reference REF] [--seed N] [--n 30]
                                              [--out tmp/fuzz2] [--jobs 4] [--keep]
                                              [--only SUBSTR] [--list] [--gate] [--no-bags]

BAG is a bench (or car) bag of the graded route that both gates pass. It is
never modified: it is read once, and if its camera topic is empty (bench bags
recorded before the synthetic camera) a copy with a synthetic 960 x 600, 10 Hz
camera is written to OUT/base/ first, since check_bags would otherwise reject
every mutant for the camera alone. The prepared base must ACCEPT and grade
VALID, or the run stops.

Each catalogue entry writes a mutated copy of the base under OUT/bags/, runs

    check_bags.check_bag            what `just lab2 check-bags` runs
    route_grade.read_route_bag      what `just lab2 grade --score` runs
      + score_route
    audit.audit_bag (plot off)      what `just lab2 audit` runs, when present

and compares the verdicts with the entry's expectation:

    REJECT    check_bags must reject the bag
    INVALID   the route grade must be invalid (scores 0); a bag the grader
              cannot grade at all (no route) counts as invalid
    VALID     check_bags accepts and the grade is valid; a metric predicate
              (speed RMSE up, overshoot captured, stop error doubled past the
              line) may be attached and must hold
    -         not asserted

The audit is expected to say INVALID whenever check_bags or the grade is
expected to fail, and VALID when both are expected to pass.

Per case it prints CAUGHT (a bad bag was caught), OK (a good bag stayed good
and its metrics moved as expected), MISSED (a bad bag passed: a real gate
finding), FALSE (a good bag was rejected or invalidated) or METRIC (a metric
did not move the expected way), or CRASH (a gate raised instead of giving a
verdict: the bag is not accepted, but the student sees a traceback). Exit
status 1 if any case is MISSED, FALSE, METRIC or CRASH. Results also go to OUT/results.json.

--gate also runs the full-stack bench gate (`just lab2 gate`) on mutant
SpeedPID submissions built from tmp/sub_ref (sign flip, always neutral, max PWM
always, NaN every 50th call), each one inside
`ROS_DOMAIN_ID=73 flock tmp/bench.lock`, and expects FAIL with a matching
reason. --no-bags skips the bag catalogue.

The gates need ROS (rosbag2_py, the message types); without it in the
environment the script re-executes itself under scripts/env.sh.
"""

import argparse
import concurrent.futures as cf
import json
import math
import os
import random
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

REPO = Path(__file__).resolve().parents[3]
TEMPLATE = REPO / 'src/lab/autosdv_waypoint_lab/config/courses/lab2_uturn'
HALL = REPO / 'src/lab/autosdv_waypoint_lab/config/courses/hall_demo'
SUB_REF = REPO / 'tmp/sub_ref'

# topic names, as bag_meta has them (kept here so the pure part imports without ROS)
KIN = '/localization/kinematic_state'
FIXES = '/localization/pose_estimator/pose_with_covariance'
VEL = '/vehicle/status/velocity_status'
GATE_CMD = '/control/command/control_cmd'
TRACKER_CMD = '/control/trajectory_follower/control_cmd'
PWM = '/autosdv/actuator_node/debug/pwm_values'
PID = '/autosdv/actuator_node/debug/pid_values'
PROGRESS = '/planning/waypoint_path_publisher/progress'
MRM = '/system/fail_safe/mrm_state'
MODE = '/api/operation_mode/state'
CAMERA = '/lab2/camera/image/compressed'
META = '/lab2/record_meta'
REQUIRED = [KIN, FIXES, VEL, GATE_CMD, TRACKER_CMD, PWM, PID, PROGRESS, MRM, MODE,
            '/tf', '/tf_static', CAMERA, META]

MODE_STOP, AUTONOMOUS = 1, 2
MRM_NORMAL, MRM_OPERATING = 1, 2
METRICS = ['speed_rmse', 'speed_rmse_straight', 'speed_rmse_turn', 'overshoot',
           'stop_error', 'stop_signed', 'path_dev_max']


# ====================================================================== expectations (pure)

@dataclass
class Expect:
    check: Optional[str] = None          # 'REJECT' | 'ACCEPT' | None
    grade: Optional[str] = None          # 'INVALID' | 'VALID' | None
    metric: Optional[Callable] = None    # (base: dict, mut: dict) -> (ok, detail)
    note: str = ''
    audit_want: Optional[str] = None     # overrides the audit verdict derived below

    @property
    def audit(self) -> Optional[str]:
        if self.audit_want:
            return self.audit_want
        if self.check == 'REJECT' or self.grade == 'INVALID':
            return 'INVALID'
        if self.check == 'ACCEPT' and self.grade == 'VALID':
            return 'VALID'
        return None

    def label(self) -> str:
        return f'{self.check or "-"}/{self.grade or "-"}'


def combine(expects: List[Expect]) -> Expect:
    """The expectation of several mutations applied together: the worst of them."""
    def one(vals, bad, good):
        if bad in vals:
            return bad
        return good if vals and all(v == good for v in vals) else None
    return Expect(one([e.check for e in expects], 'REJECT', 'ACCEPT'),
                  one([e.grade for e in expects], 'INVALID', 'VALID'))


def classify(exp: Expect, got: dict) -> Tuple[str, List[str]]:
    """(status, details) of one case. got: check_ok, grade_valid (None = ungradable),
    audit (None = not run), metric (None or (ok, detail))."""
    details, missed, false, metric_bad = [], False, False, False
    g_check = 'ACCEPT' if got['check_ok'] else 'REJECT'
    g_grade = 'VALID' if got.get('grade_valid') else 'INVALID'
    pairs = [('check', exp.check, g_check), ('grade', exp.grade, g_grade)]
    if got.get('audit') in ('VALID', 'INVALID'):
        pairs.append(('audit', exp.audit, got['audit']))
    for name, want, have in pairs:
        if want is None or want == have:
            continue
        if want in ('REJECT', 'INVALID'):
            missed = True
            details.append(f'{name}: expected {want}, got {have}')
        else:
            false = True
            details.append(f'{name}: expected {want}, got {have}')
    if got.get('metric') is not None:
        ok, d = got['metric']
        details.append(d)
        metric_bad = not ok
    if missed:
        return 'MISSED', details
    if got.get('errors'):
        return 'CRASH', details + list(got['errors'])
    if false:
        return 'FALSE', details
    if metric_bad:
        return 'METRIC', details
    return ('CAUGHT' if exp.audit == 'INVALID' else 'OK'), details


def monotonic(xs: List[float]) -> bool:
    return all(b > a for a, b in zip(xs, xs[1:]))


# ====================================================================== bag model

class Bag:
    """A whole bag in memory: [topic, t_ns, payload] records, payload raw CDR bytes
    until a mutation asks for the message."""

    def __init__(self, topics, recs, sidecar):
        self.topics = topics            # [rosbag2_py.TopicMetadata], in bag order
        self.types = {t.name: t.type for t in topics}
        self.recs = recs
        self.sidecar = sidecar          # lab2_meta.json contents, or None

    def clone(self) -> 'Bag':
        return Bag(list(self.topics), [list(r) for r in self.recs],
                   json.loads(json.dumps(self.sidecar)) if self.sidecar is not None else None)

    def of(self, topic):
        return [r for r in self.recs if r[0] == topic]

    def msg(self, r):
        if isinstance(r[2], (bytes, bytearray, memoryview)):
            from rclpy.serialization import deserialize_message
            r[2] = deserialize_message(bytes(r[2]), _cls(self.types[r[0]]))
        return r[2]

    def raw(self, r) -> bytes:
        if isinstance(r[2], (bytes, bytearray, memoryview)):
            return bytes(r[2])
        from rclpy.serialization import serialize_message
        return serialize_message(r[2])

    def copy_msg(self, r):
        from rclpy.serialization import deserialize_message
        return deserialize_message(self.raw(r), _cls(self.types[r[0]]))

    def insert(self, topic, t_ns, msg):
        self.recs.append([topic, int(t_ns), msg])

    def drop(self, topic, keep_topic=False):
        self.recs = [r for r in self.recs if r[0] != topic]
        if not keep_topic:
            self.topics = [t for t in self.topics if t.name != topic]
            self.types.pop(topic, None)

    def span(self):
        ts = [r[1] for r in self.recs]
        return min(ts), max(ts)


_CLS = {}


def _cls(type_name):
    if type_name not in _CLS:
        from rosidl_runtime_py.utilities import get_message
        _CLS[type_name] = get_message(type_name)
    return _CLS[type_name]


def load_bag(path) -> Bag:
    import rosbag2_py
    from autosdv_coach_lab.calibration.bagio import _storage_id
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=_storage_id(str(path))),
                rosbag2_py.ConverterOptions('', ''))
    topics = reader.get_all_topics_and_types()
    recs = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        recs.append([topic, t_ns, data])
    side = Path(path) / 'lab2_meta.json'
    sidecar = json.loads(side.read_text()) if side.is_file() else None
    return Bag(topics, recs, sidecar)


def write_bag(bag: Bag, out: Path):
    import rosbag2_py
    if out.exists():
        shutil.rmtree(out)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(out), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    for t in bag.topics:
        w.create_topic(t)
    known = set(bag.types)
    for r in sorted(bag.recs, key=lambda r: r[1]):
        if r[0] in known:
            w.write(r[0], bag.raw(r), int(r[1]))
    del w
    if bag.sidecar is not None:
        (out / 'lab2_meta.json').write_text(json.dumps(bag.sidecar, indent=2, sort_keys=True))


def stamp(msg, t_ns):
    if hasattr(msg, 'header'):
        msg.header.stamp.sec, msg.header.stamp.nanosec = int(t_ns // 10**9), int(t_ns % 10**9)
    elif hasattr(msg, 'stamp'):
        msg.stamp.sec, msg.stamp.nanosec = int(t_ns // 10**9), int(t_ns % 10**9)


def camera_frame(t_ns, size=(960, 600)):
    import cv2
    import numpy as np
    from sensor_msgs.msg import CompressedImage
    img = np.full((size[1], size[0], 3), 90, np.uint8)
    cv2.putText(img, 'fuzz_bags synthetic camera', (40, size[1] // 2), cv2.FONT_HERSHEY_SIMPLEX,
                1.2, (255, 255, 255), 2)
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 60])
    m = CompressedImage()
    m.header.frame_id = 'zedxm_left_camera_optical_frame'
    stamp(m, t_ns)
    m.format = 'jpeg'
    m.data = buf.tobytes()
    return m


def add_camera(bag: Bag, hz=10.0, size=(960, 600), t0=None, t1=None):
    """Replace the camera topic with a synthetic stream (one JPEG, re-stamped)."""
    from rclpy.serialization import serialize_message
    a, b = bag.span()
    t0, t1 = t0 if t0 is not None else a, t1 if t1 is not None else b
    bag.drop(CAMERA, keep_topic=CAMERA in bag.types)
    if CAMERA not in bag.types:
        import rosbag2_py
        meta = rosbag2_py.TopicMetadata(name=CAMERA, type='sensor_msgs/msg/CompressedImage',
                                        serialization_format='cdr')
        bag.topics.append(meta)
        bag.types[CAMERA] = meta.type
    proto = camera_frame(t0, size)
    t, step = t0, int(1e9 / hz)
    while t <= t1:
        stamp(proto, t)
        bag.recs.append([CAMERA, t, serialize_message(proto)])
        t += step


# ====================================================================== the base run

@dataclass
class Ctx:
    """What the mutations need to know about the base run (bag times, seconds)."""
    base: str
    car: str = ''
    team: str = ''
    bench: bool = False
    reference: Optional[str] = None
    t_engage: float = math.nan
    t90: float = math.nan
    t_ramp: float = math.nan
    t_rest: float = math.nan
    t_finish: Optional[float] = None
    t_end: float = math.nan
    w1: Tuple[float, float] = (math.nan, math.nan)     # 1 s on the out leg, nearest the path
    w1_normal: Tuple[float, float] = (0.0, 1.0)        # outward normal there (worst side)
    w1_tangent: Tuple[float, float] = (1.0, 0.0)
    w1_dev: float = math.nan
    w2: Tuple[float, float] = (math.nan, math.nan)     # 2 s on the return leg
    w2_normal: Tuple[float, float] = (0.0, 1.0)
    w2_tangent: Tuple[float, float] = (1.0, 0.0)
    stop_dir: Tuple[float, float] = (1.0, 0.0)
    metrics: Dict[str, float] = field(default_factory=dict)
    out: str = ''


def find_window(geo, kin, lo, hi, length, leg):
    """The `length`-second span on a straight leg (1 = before the turn, 2 = after)
    whose worst |cross-track| is smallest: (t0, worst, normal, tangent)."""
    a, b = geo.turns[0]
    samples = []
    for t, (x, y) in kin:
        if lo <= t <= hi:
            p = geo.path.project(x, y)
            on = (p.s < a - 0.5) if leg == 1 else (p.s > b + 0.5)
            samples.append((t, on, p.cross_track, p.yaw))
    best = None
    for i, (t0, on0, _, _) in enumerate(samples):
        if not on0:
            continue
        span = [s for s in samples[i:] if s[0] <= t0 + length]
        if span[-1][0] < t0 + length - 0.2 or not all(s[1] for s in span):
            continue
        worst = max(abs(s[2]) for s in span)
        if best is None or worst < best[1]:
            side = 1.0 if max(span, key=lambda s: abs(s[2]))[2] >= 0 else -1.0
            yaw = span[0][3]
            best = (t0, worst, (-math.sin(yaw) * side, math.cos(yaw) * side),
                    (math.cos(yaw), math.sin(yaw)))
    if best is None:
        raise SystemExit(f'no {length} s straight window on leg {leg} of the base run')
    return best


def build_ctx(base: str, reference: Optional[str]) -> Ctx:
    from autosdv_coach_lab.calibration.bagio import read_bag
    from autosdv_waypoint_lab import bag_meta as bm
    from autosdv_waypoint_lab import route_grade as rg
    msgs = read_bag(base, rg.ROUTE_TOPICS)
    meta = bm.meta_from_messages(msgs, base) or {}
    m, _ = rg.read_route_bag(base, None, None, msgs=msgs)
    c = Ctx(base, str(meta.get('car_id') or ''), str(meta.get('team_id') or ''),
            bool(meta.get('bench')), reference)
    c.t_engage, c.t90, c.t_ramp, c.t_rest = m.t_engage, m.t90, m.t_ramp, m.t_rest
    c.t_finish = next((t for t, p in msgs.get(PROGRESS, []) if p.finished), None)
    c.t_end = max(t for lst in msgs.values() for t, _ in lst)
    geo = rg.geometry_from_course(rg.template_dir('uturn'), 'uturn')
    kin = [(t, (x.pose.pose.position.x, x.pose.pose.position.y)) for t, x in msgs[KIN]]
    t0, c.w1_dev, c.w1_normal, c.w1_tangent = find_window(geo, kin, c.t90 + 1.0, c.t_ramp, 1.0, 1)
    c.w1 = (t0, t0 + 1.0)
    t0, _, c.w2_normal, c.w2_tangent = find_window(geo, kin, c.t90 + 1.0, c.t_ramp, 2.0, 2)
    c.w2 = (t0, t0 + 2.0)
    c.stop_dir = (geo.dir_x, geo.dir_y)
    c.metrics = {k: getattr(m, k) for k in METRICS}
    return c


# ====================================================================== mutations

def T(t_s):
    return int(round(t_s * 1e9))


def set_meta(bag: Bag, fn):
    for r in bag.of(META):
        m = bag.msg(r)
        d = json.loads(m.data)
        fn(d)
        m.data = json.dumps(d, sort_keys=True)
    if bag.sidecar is not None:
        fn(bag.sidecar)


def each(bag, topic, fn, t0=-math.inf, t1=math.inf):
    for r in bag.of(topic):
        t = r[1] * 1e-9
        if t0 <= t <= t1:
            fn(bag.msg(r), t)


def offset_kin(bag, t0, t1, dx, dy):
    def f(m, t):
        m.pose.pose.position.x += dx
        m.pose.pose.position.y += dy
    each(bag, KIN, f, t0, t1)


def scale_vel(bag, k, t0=-math.inf, t1=math.inf):
    def f(m, t):
        m.longitudinal_velocity = float(m.longitudinal_velocity) * k
    each(bag, VEL, f, t0, t1)


def scale_cmd(bag, k):
    def f(m, t):
        m.longitudinal.velocity = float(m.longitudinal.velocity) * k
    each(bag, GATE_CMD, f)
    each(bag, TRACKER_CMD, f)


def inject(bag, topic, t_s, **fields):
    """A copy of the topic's last message before t_s with `fields` set, at t_s."""
    recs = sorted(bag.of(topic), key=lambda r: r[1])
    proto = next((r for r in reversed(recs) if r[1] <= T(t_s)), recs[0])
    m = bag.copy_msg(proto)
    for k, v in fields.items():
        setattr(m, k, v)
    stamp(m, T(t_s))
    bag.insert(topic, T(t_s), m)


def truncate(bag, t_s):
    bag.recs = [r for r in bag.recs if r[1] <= T(t_s)]


def time_reverse(bag):
    a, b = bag.span()
    for r in bag.recs:
        r[1] = a + b - r[1]


def dup_timestamps(bag):
    from rclpy.serialization import serialize_message  # noqa: F401 (raw() uses it)
    topics = {VEL, KIN, GATE_CMD, MODE, MRM, PROGRESS}
    bag.recs += [[r[0], r[1], bag.raw(r)] for r in bag.recs if r[0] in topics]


def reverse_header_stamps(bag, topic=VEL):
    recs = sorted(bag.of(topic), key=lambda r: r[1])
    stamps = [(bag.msg(r).header.stamp.sec, bag.msg(r).header.stamp.nanosec) for r in recs]
    for r, (s, ns) in zip(recs, reversed(stamps)):
        bag.msg(r).header.stamp.sec, bag.msg(r).header.stamp.nanosec = s, ns


def swap_adjacent(bag, topic=VEL):
    recs = sorted(bag.of(topic), key=lambda r: r[1])
    for i in range(0, len(recs) - 1, 2):
        recs[i][1], recs[i + 1][1] = recs[i + 1][1], recs[i][1]


def noise_vel(bag, ctx, sigma, rng):
    def f(m, t):
        if abs(m.longitudinal_velocity) > 0.05:
            m.longitudinal_velocity = float(m.longitudinal_velocity) + rng.gauss(0.0, sigma)
    each(bag, VEL, f, ctx.t_engage, ctx.t_ramp)


def nan_vel(bag, ctx):
    n = [0]

    def f(m, t):
        if n[0] < 3:
            m.longitudinal_velocity = math.nan
            n[0] += 1
    each(bag, VEL, f, ctx.w1[0], ctx.w1[1])


def nan_pose(bag, ctx):
    recs = sorted([r for r in bag.of(KIN) if ctx.w1[0] <= r[1] * 1e-9 <= ctx.w1[1]],
                  key=lambda r: r[1])
    m = bag.msg(recs[len(recs) // 2])
    m.pose.pose.position.x = math.nan
    m.pose.pose.position.y = math.nan


def never_engaged(bag):
    each(bag, MODE, lambda m, t: setattr(m, 'mode', MODE_STOP if m.mode == AUTONOMOUS else m.mode))


def camera_rate(bag, keep_every):
    recs = sorted(bag.of(CAMERA), key=lambda r: r[1])
    drop = {id(r) for i, r in enumerate(recs) if i % keep_every}
    bag.recs = [r for r in bag.recs if id(r) not in drop]


def camera_half(bag, ctx):
    a = (ctx.t_engage + ctx.t_rest) / 2
    bag.recs = [r for r in bag.recs if not (r[0] == CAMERA and r[1] * 1e-9 > a)]


def camera_size(bag):
    a, b = bag.span()
    add_camera(bag, size=(640, 400), t0=a, t1=b)


def swapped_meta(course_dir):
    def fn(d):
        from autosdv_waypoint_lab import bag_meta as bm
        new = bm.build_meta(str(course_dir), d.get('car_id', ''), d.get('team_id', ''),
                            d.get('git_commit', ''))
        for k in ('course_name', 'course_hash', 'route_hash', 'route', 'route_kind'):
            d[k] = new[k]
        d['course_dir'] = str(course_dir)
    return fn


def modified_course(out: Path) -> Path:
    """A copy of the template with its out leg moved 0.1 m and v_ref 1.0: still a U-turn."""
    d = out / 'courses' / 'lab2_uturn_modified'
    if not d.is_dir():
        shutil.copytree(TEMPLATE, d)
        lines = []
        for line in (d / 'path.csv').read_text().splitlines():
            body = line.split('#')[0].strip()
            if body:
                a = body.split(',')
                a[1] = f'{float(a[1]) + 0.1:.4f}'
                line = ','.join(a)
            lines.append(line)
        (d / 'path.csv').write_text('\n'.join(lines) + '\n')
        y = (d / 'course.yaml').read_text()
        y = re.sub(r'^v_ref:.*$', 'v_ref: 1.0', y, flags=re.M)
        (d / 'course.yaml').write_text(y)
    return d


# ---------------------------------------------------------------- metric predicates

def m_unchanged(tol):
    def f(b, m):
        bad = [k for k in METRICS
               if not (math.isnan(b[k]) and math.isnan(m[k])) and not abs(m[k] - b[k]) <= tol]
        return (not bad), ('metrics unchanged (tol %g)' % tol if not bad
                           else 'changed: ' + ', '.join(f'{k} {b[k]:.4f}->{m[k]:.4f}' for k in bad))
    return f


def m_up(key, by=1e-4):
    def f(b, m):
        ok = m[key] > b[key] + by
        return ok, f'{key} {b[key]:.4f} -> {m[key]:.4f} ({"up" if ok else "NOT up"})'
    return f


def m_overshoot(spike):
    def f(b, m):
        ok = m['overshoot'] >= spike - 0.02 and m['overshoot'] > b['overshoot'] + 0.5 * spike
        return ok, f'overshoot {b["overshoot"]:.4f} -> {m["overshoot"]:.4f} (spike {spike})'
    return f


def m_stop(shift):
    def f(b, m):
        d0, d1, e1 = b['stop_signed'], m['stop_signed'], m['stop_error']
        want = abs(d1) + max(0.0, d1)
        ok = abs((d1 - d0) - shift) < 0.01 and abs(e1 - want) < 1e-6
        kind = 'double' if d1 > 0 else 'single'
        if shift > 0:
            ok = ok and d1 > 0 and abs(e1 - 2 * d1) < 1e-6
        else:
            ok = ok and d1 < 0 and abs(e1 + d1) < 1e-6
        return ok, (f'stop signed {d0:+.3f} -> {d1:+.3f}, stop_error {b["stop_error"]:.3f} -> '
                    f'{e1:.3f} (counted {kind})')
    return f


def m_path_dev(lo, hi):
    def f(b, m):
        v = m['path_dev_max']
        ok = lo <= v <= hi
        return ok, f'path_dev_max {b["path_dev_max"]:.3f} -> {v:.3f} (expected {lo}..{hi})'
    return f


# ---------------------------------------------------------------- the catalogue

@dataclass
class Case:
    name: str
    apply: Callable          # (bag, ctx, rng) -> None
    expect: Expect
    composable: bool = True  # may join a random combination
    series: Optional[Tuple[str, float]] = None
    rank: int = 0            # canonical order inside a combination


R, A, I, V = 'REJECT', 'ACCEPT', 'INVALID', 'VALID'


def catalogue(out: Path) -> List[Case]:
    cs: List[Case] = []

    def add(name, fn, exp, **kw):
        cs.append(Case(name, fn, exp, rank=len(cs), **kw))

    # metadata and course
    add('meta:route_hash_altered', lambda b, c, r: set_meta(b, lambda d: d.update(route_hash='0' * 64)),
        Expect(R))
    add('meta:course_hash_altered', lambda b, c, r: set_meta(b, lambda d: d.update(course_hash='0' * 64)),
        Expect(A, V, note='by design: course_hash covers the surveyed markers, only route_hash is compared'),
        composable=False)
    add('meta:route_kind_other', lambda b, c, r: set_meta(b, lambda d: d.update(route_kind='circle')),
        Expect(R, I))
    add('meta:v_ref_1.0', lambda b, c, r: set_meta(b, lambda d: d['route'].update(v_ref=1.0)), Expect(R))
    add('meta:car_altered', lambda b, c, r: set_meta(b, lambda d: d.update(car_id='Z')), Expect(R))
    add('meta:car_blank', lambda b, c, r: set_meta(b, lambda d: d.update(car_id='')), Expect(R))
    add('meta:team_missing', lambda b, c, r: set_meta(b, lambda d: d.pop('team_id', None)), Expect(R))
    add('meta:team_altered', lambda b, c, r: set_meta(b, lambda d: d.update(team_id='99')), Expect(R))
    add('meta:bench_flag', lambda b, c, r: set_meta(b, lambda d: d.update(bench=True)),
        Expect(R, audit_want=V, note='by design the audit checks a bench-flagged bag in bench mode; '
                                     'check_bags rejects it as a submission'), composable=False)

    def no_meta(b, c, r):
        b.drop(META)
        b.sidecar = None
    add('meta:none_at_all', no_meta, Expect(R, I))
    add('route:swapped_hall_demo', lambda b, c, r: set_meta(b, swapped_meta(HALL)), Expect(R, I))
    add('route:modified_uturn', lambda b, c, r: set_meta(b, swapped_meta(modified_course(out))),
        Expect(R))

    # progress
    add('progress:laps_planned_2', lambda b, c, r: each(b, PROGRESS, lambda m, t: setattr(m, 'laps', 2)),
        Expect(R))
    add('progress:laps_done_0',
        lambda b, c, r: each(b, PROGRESS, lambda m, t: setattr(m, 'laps_done', 0)), Expect(R))

    # MRM, mode
    def mrm_mid(b, c, r):
        tm = sum(c.w1) / 2
        inject(b, MRM, tm, state=MRM_OPERATING)
        inject(b, MRM, tm + 0.1, state=MRM_NORMAL)
    add('mrm:0.1s_mid_run', mrm_mid, Expect(R, I))

    def mrm_after(b, c, r):
        inject(b, MRM, c.t_finish + 1.0, state=MRM_OPERATING)
        inject(b, MRM, c.t_finish + 1.1, state=MRM_NORMAL)
    add('mrm:after_finish', mrm_after, Expect(A, V, metric=m_unchanged(1e-9)))

    def dis_mid(b, c, r):
        tm = sum(c.w1) / 2
        inject(b, MODE, tm, mode=MODE_STOP)
        inject(b, MODE, tm + 0.5, mode=AUTONOMOUS)
    add('mode:disengage_mid_run', dis_mid, Expect(R, I))

    def dis_after(b, c, r):
        inject(b, MODE, max(c.t_finish, c.t_rest) + 4.0, mode=MODE_STOP)
    add('mode:disengage_after_finish', dis_after, Expect(A, V))

    # localization
    add('loc:lateral_0.35m_1s',
        lambda b, c, r: offset_kin(b, *c.w1, 0.35 * c.w1_normal[0], 0.35 * c.w1_normal[1]),
        Expect(R, I, metric=m_path_dev(0.30, 0.40)))
    add('loc:lateral_0.25m_1s',
        lambda b, c, r: offset_kin(b, *c.w1, 0.25 * c.w1_normal[0], 0.25 * c.w1_normal[1]),
        Expect(A, V, metric=m_path_dev(0.24, 0.30)))
    add('loc:jump_lateral_0.6m_2s',
        lambda b, c, r: offset_kin(b, *c.w2, 0.6 * c.w2_normal[0], 0.6 * c.w2_normal[1]),
        Expect(R, I))
    add('loc:jump_along_1.0m_1s',
        lambda b, c, r: offset_kin(b, c.w2[0], c.w2[0] + 1.0, 1.0 * c.w2_tangent[0],
                                   1.0 * c.w2_tangent[1]),
        Expect(R, I, note='a teleport along the path: the pose jumps 1 m ahead of what '
                          'the wheel speed allows'), composable=False)
    add('loc:nan_pose_1_sample', lambda b, c, r: nan_pose(b, c), Expect(None, I))

    # timestamps
    add('time:whole_bag_reversed', lambda b, c, r: time_reverse(b), Expect(R, I), composable=False)
    add('time:duplicated_timestamps', lambda b, c, r: dup_timestamps(b),
        Expect(A, V, metric=m_unchanged(1e-3)))
    add('time:header_stamps_reversed', lambda b, c, r: reverse_header_stamps(b),
        Expect(A, V, metric=m_unchanged(1e-9)))
    add('time:adjacent_velocity_swapped', lambda b, c, r: swap_adjacent(b),
        Expect(A, V, metric=m_unchanged(0.01)), composable=False)

    # speed
    add('speed:nan_3_samples', lambda b, c, r: nan_vel(b, c), Expect(None, I))

    def x13(b, c, r):
        scale_vel(b, 1.3)
        scale_cmd(b, 1.3)
    add('speed:all_x1.3', x13, Expect(R))
    add('speed:target_cmd_0.90', lambda b, c, r: scale_cmd(b, 0.90 / 0.80), Expect(R))
    add('speed:target_cmd_0.83', lambda b, c, r: scale_cmd(b, 0.83 / 0.80),
        Expect(A, note='inside the 0.05 m/s tolerance'), composable=False)
    for s in (0.005, 0.01, 0.02, 0.04):
        add(f'speed:noise_sigma_{s}', (lambda s: lambda b, c, r: noise_vel(b, c, s, r))(s),
            Expect(A, V, metric=m_up('speed_rmse_straight')), series=('noise', s),
            composable=(s == 0.01))
    for e in (0.01, 0.02, 0.05, 0.10):
        add(f'metric:measured_x{1 + e:.2f}_steady',
            (lambda e: lambda b, c, r: scale_vel(b, 1 + e, c.t90, c.t_ramp))(e),
            Expect(A, V, metric=m_up('speed_rmse_straight')), series=('scale', e), composable=False)
    add('metric:overshoot_spike_0.15',
        lambda b, c, r: each(b, VEL, lambda m, t: setattr(m, 'longitudinal_velocity',
                                                          float(m.longitudinal_velocity) + 0.15),
                             c.t90 + 1.0, c.t90 + 1.3),
        Expect(A, V, metric=m_overshoot(0.15)), composable=False)
    add('metric:stop_past_+0.2m',
        lambda b, c, r: offset_kin(b, c.t_rest, math.inf, 0.2 * c.stop_dir[0], 0.2 * c.stop_dir[1]),
        Expect(A, V, metric=m_stop(0.2)), composable=False)
    add('metric:stop_short_-0.2m',
        lambda b, c, r: offset_kin(b, c.t_rest, math.inf, -0.2 * c.stop_dir[0], -0.2 * c.stop_dir[1]),
        Expect(A, V, metric=m_stop(-0.2)))

    # camera
    add('camera:3.3Hz', lambda b, c, r: camera_rate(b, 3), Expect(R))
    add('camera:640x400', lambda b, c, r: camera_size(b), Expect(R))
    add('camera:first_half_only', lambda b, c, r: camera_half(b, c), Expect(R))

    # late, global mutations (order matters inside a combination)
    add('mode:never_engaged', lambda b, c, r: never_engaged(b), Expect(R, I))
    add('truncate:before_stop', lambda b, c, r: truncate(b, c.t_ramp - 2.0), Expect(R, I))
    add('truncate:just_after_finish',
        lambda b, c, r: truncate(b, max(c.t_finish, c.t_rest) + 0.3), Expect(None, V), composable=False)
    for t in (KIN, PWM, CAMERA):
        add(f'empty:{t}', (lambda t: lambda b, c, r: b.drop(t, keep_topic=True))(t),
            Expect(R, I if t == KIN else None))
    grade_dropped = {KIN: I, VEL: I, GATE_CMD: I, PROGRESS: I, MODE: I}
    for t in REQUIRED:
        add(f'drop:{t}', (lambda t: lambda b, c, r: b.drop(t))(t), Expect(R, grade_dropped.get(t)))
    return cs


# ====================================================================== evaluation

def evaluate(path: str, ctx: Ctx) -> dict:
    from autosdv_waypoint_lab import check_bags as cb
    from autosdv_waypoint_lab import route_grade as rg
    out = {'check_ok': False, 'check_reasons': [], 'grade_valid': None, 'gate_failures': [],
           'metrics': {k: math.nan for k in METRICS}, 'total': 0.0, 'audit': None, 'errors': []}
    try:
        v = cb.check_bag(path, [], ctx.car or None, ctx.team or None, bench_ok=ctx.bench)
        out['check_ok'], out['check_reasons'] = v.ok, list(v.reasons)
    except Exception as e:      # a crash is a rejection, and worth reporting
        out['check_reasons'] = [f'check_bags raised {type(e).__name__}: {e}']
        out['errors'].append(out['check_reasons'][0])
    try:
        m, _ = rg.read_route_bag(path)
        if m.route is None:
            out['gate_failures'] = ['ungradable: ' + '; '.join(m.notes)]
        else:
            ref = REF_CACHE.get(ctx.reference)
            s = rg.score_route(m, ref)
            out['grade_valid'] = not s.invalid
            out['gate_failures'] = list(s.gate_failures)
            out['total'] = s.total
            out['metrics'] = {k: float(getattr(m, k)) for k in METRICS}
    except Exception as e:
        out['gate_failures'] = [f'route_grade raised {type(e).__name__}: {e}']
        out['errors'].append(out['gate_failures'][0])
    if AUDIT is not None:
        try:
            a, _ = AUDIT.audit_bag(path, ctx.car or None, ctx.team or None, ctx.reference,
                                   None, None, plot=False)
            out['audit'] = a.verdict
        except Exception as e:
            out['audit'] = 'ERROR'
            out['errors'].append(f'audit raised {type(e).__name__}: {e}')
    return out


REF_CACHE: Dict[Optional[str], object] = {None: None}
AUDIT = None
BASE: Optional[Bag] = None
CTX: Optional[Ctx] = None


def run_case(job) -> dict:
    """One mutant: build, write, evaluate, classify. Runs in a worker process."""
    name, parts, seed, keep = job
    cases = {c.name: c for c in catalogue(Path(CTX.out))}
    chosen = sorted((cases[p] for p in parts), key=lambda c: c.rank)
    rng = random.Random(seed)
    bag = BASE.clone()
    t0 = time.time()
    for c in chosen:
        c.apply(bag, CTX, rng)
    exp = chosen[0].expect if len(chosen) == 1 else combine([c.expect for c in chosen])
    safe = re.sub(r'[^A-Za-z0-9_.+-]+', '_', name).strip('_')
    path = Path(CTX.out) / 'bags' / safe
    write_bag(bag, path)
    got = evaluate(str(path), CTX)
    if exp.metric is not None and not all(math.isnan(v) for v in got['metrics'].values()):
        try:
            got['metric'] = exp.metric(CTX.metrics, got['metrics'])
        except Exception as e:
            got['metric'] = (False, f'metric check raised {e}')
    elif exp.metric is not None:
        got['metric'] = (False, 'no metrics: the bag could not be graded')
    status, details = classify(exp, got)
    if not keep and status in ('CAUGHT', 'OK'):
        shutil.rmtree(path, ignore_errors=True)
    got.pop('metric', None)
    return {'name': name, 'parts': [c.name for c in chosen], 'expect': exp.label(),
            'expect_audit': exp.audit, 'note': exp.note, 'status': status, 'details': details,
            'series': chosen[0].series if len(chosen) == 1 else None,
            'bag': str(path), 'seconds': round(time.time() - t0, 1), **got}


def _init_worker(base, ctx, ref, audit_ok):
    global BASE, CTX, AUDIT
    BASE, CTX = base, ctx
    REF_CACHE[ctx.reference] = ref
    if audit_ok:
        from autosdv_waypoint_lab import audit
        AUDIT = audit


# ====================================================================== the bench gate fuzz

GATE_MUTANTS = {
    'sign_flip': ('return _orig(self, target_speed, 2.0 * target_speed - measured_speed, dt)',
                  r'did not move|median speed|did not finish|MRM|peak speed'),
    'always_neutral': ('return self.init_pwm', r'did not move|did not finish|median speed'),
    'always_max_pwm': ('return self.max_pwm', r'peak speed|median speed|MRM|still moving'),
    'nan_every_50': ('self._fuzz_n = getattr(self, "_fuzz_n", 0) + 1\n'
                     '    if self._fuzz_n % 50 == 0:\n'
                     '        return float("nan")\n'
                     '    return _orig(self, target_speed, measured_speed, dt)',
                     r'SpeedPID fault'),
}


def make_mutant(out: Path, name: str, body: str) -> Path:
    d = out / 'gate' / name
    if d.exists():
        shutil.rmtree(d)
    shutil.copytree(SUB_REF, d, ignore=shutil.ignore_patterns('__pycache__', 'build', 'install', 'log'))
    f = next(d.rglob('lab2_waypoint/speed_pid.py'))
    f.write_text(f.read_text() + (
        f'\n\n# --- fuzz_bags.py mutant: {name}\n'
        '_orig = SpeedPID.update\n\n\n'
        'def _fuzz_update(self, target_speed, measured_speed, dt):\n'
        f'    {body}\n\n\n'
        'SpeedPID.update = _fuzz_update\n'))
    return d


def run_gate(out: Path, timeout: float) -> List[dict]:
    results = []
    for name, (body, want) in GATE_MUTANTS.items():
        d = make_mutant(out, name, body)
        log = d / 'gate.log'
        cmd = (f'cd {REPO} && ROS_DOMAIN_ID=73 flock tmp/bench.lock '
               f'just lab2 gate submission:={d}')
        print(f'--- gate {name}: {cmd}', flush=True)
        t0 = time.time()
        with open(log, 'w') as f:
            p = subprocess.Popen(['bash', '-c', cmd], stdout=f, stderr=subprocess.STDOUT,
                                 start_new_session=True, env={**os.environ, 'ROS_DOMAIN_ID': '73'})
            try:
                rc = p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, 15)
                try:
                    rc = p.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, 9)
                    rc = p.wait()
        text = log.read_text(errors='replace')
        lines = [ln for ln in text.splitlines() if re.match(r'^(PASS|FAIL)\s', ln)]
        verdict = lines[-1] if lines else '(no PASS/FAIL line)'
        if verdict.startswith('PASS'):
            status = 'MISSED'
        elif verdict.startswith('FAIL') and re.search(want, verdict):
            status = 'CAUGHT'
        elif verdict.startswith('FAIL'):
            status = 'CAUGHT?'      # failed, but not for the reason expected
        else:
            status = 'ERROR'
        r = {'mutant': name, 'status': status, 'verdict': verdict, 'expected_reason': want,
             'exit': rc, 'seconds': round(time.time() - t0), 'log': str(log)}
        print(f'{status:8s} gate:{name:16s} {verdict}  ({r["seconds"]} s)', flush=True)
        results.append(r)
    return results


# ====================================================================== main

def ensure_ros():
    try:
        import rosbag2_py  # noqa: F401
        import autosdv_waypoint_lab  # noqa: F401
        import autosdv_coach_lab  # noqa: F401
    except ImportError:
        if os.environ.get('FUZZ_BAGS_REEXEC'):
            raise SystemExit('fuzz_bags: rosbag2_py / autosdv_waypoint_lab not importable even '
                             'after sourcing scripts/env.sh (built the workspace?)')
        os.environ['FUZZ_BAGS_REEXEC'] = '1'
        os.execvp('bash', ['bash', '-c',
                           'source "$1/scripts/env.sh" >/dev/null 2>&1; shift; exec python3 "$@"',
                           'bash', str(REPO), os.path.abspath(__file__), *sys.argv[1:]])


def prepare_base(src: str, out: Path) -> str:
    """The source bag, or a copy of it with a synthetic camera when its camera is empty."""
    bag = load_bag(src)
    if any(r[0] == CAMERA for r in bag.recs):
        return src
    dst = out / 'base' / Path(src).name
    print(f'base: {src} has no camera frames; writing a copy with a synthetic 960x600 10 Hz '
          f'camera to {dst}', flush=True)
    add_camera(bag)
    write_bag(bag, dst)
    return str(dst)


def fmt(x):
    return 'n/a' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:.3f}'


def main(argv=None):
    ap = argparse.ArgumentParser(description='Fuzz the Lab 2 Part 2 gates with mutated bags.')
    ap.add_argument('bag', nargs='?', help='a bag of the graded route that passes both gates')
    ap.add_argument('--reference', default=None, help="the car's reference bag, for the points")
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--n', type=int, default=30, help='random combinations (default 30)')
    ap.add_argument('--out', default=str(REPO / 'tmp/fuzz2'))
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--keep', action='store_true', help='keep every mutant bag (default: the '
                    'failing ones only)')
    ap.add_argument('--only', default=None, help='run the cases whose name contains this')
    ap.add_argument('--list', action='store_true', help='print the catalogue and exit')
    ap.add_argument('--gate', action='store_true', help='also fuzz the bench gate (ROS, minutes)')
    ap.add_argument('--no-bags', action='store_true', help='skip the bag catalogue')
    ap.add_argument('--gate-timeout', type=float, default=600.0)
    args = ap.parse_args(argv)
    out = Path(args.out).resolve()

    if args.list:
        for c in catalogue(out):
            print(f'{c.expect.label():16s} {"combo" if c.composable else "     "}  {c.name}'
                  + (f'   ({c.expect.note})' if c.expect.note else ''))
        return 0
    if not args.bag and not args.no_bags:
        ap.error('BAG is required (or --no-bags with --gate)')
    ensure_ros()
    out.mkdir(parents=True, exist_ok=True)
    t_start = time.time()
    failures = 0
    report = {'bag': args.bag, 'reference': args.reference, 'seed': args.seed}

    if not args.no_bags:
        from autosdv_waypoint_lab import route_grade as rg
        try:
            from autosdv_waypoint_lab import audit  # noqa: F401
            audit_ok = True
        except ImportError:
            audit_ok = False
            print('audit: autosdv_waypoint_lab.audit not importable; check_bags and the grade only')
        base_path = prepare_base(args.bag, out)
        ctx = build_ctx(base_path, args.reference)
        ctx.out = str(out)
        modified_course(out)
        ref = rg.read_route_bag(args.reference)[0] if args.reference else None
        _init_worker(load_bag(base_path), ctx, ref, audit_ok)
        print(f'base {base_path}: car={ctx.car} team={ctx.team} bench={ctx.bench}  engage '
              f'{ctx.t_engage:.1f}, steady {ctx.t90 - ctx.t_engage:.1f}-{ctx.t_ramp - ctx.t_engage:.1f} s, '
              f'rest {ctx.t_rest - ctx.t_engage:.1f} s; out-leg window '
              f'{ctx.w1[0] - ctx.t_engage:.1f} s (worst dev {ctx.w1_dev:.3f} m)', flush=True)
        b0 = evaluate(base_path, ctx)
        print(f'base verdict: check {"ACCEPT" if b0["check_ok"] else "REJECT"}, grade '
              f'{"VALID" if b0["grade_valid"] else "INVALID"}, audit {b0["audit"]}; '
              + ', '.join(f'{k} {fmt(b0["metrics"][k])}' for k in METRICS), flush=True)
        if not (b0['check_ok'] and b0['grade_valid']):
            print('the base bag must pass both gates:', b0['check_reasons'] + b0['gate_failures'])
            return 2
        report['base'] = {'path': base_path, **b0}

        cases = catalogue(out)
        jobs = [(c.name, [c.name], args.seed * 1000 + i, args.keep) for i, c in enumerate(cases)]
        rng = random.Random(args.seed)
        pool = [c.name for c in cases if c.composable]
        for i in range(args.n):
            parts = rng.sample(pool, rng.choice((2, 3)))
            jobs.append((f'combo{i:02d}:' + '+'.join(parts), parts, args.seed * 1000 + 500 + i,
                         args.keep))
        if args.only:
            jobs = [j for j in jobs if args.only in j[0]]
        print(f'{len(jobs)} cases, {args.jobs} jobs\n', flush=True)
        results = []
        ctx_fork = __import__('multiprocessing').get_context('fork')
        with cf.ProcessPoolExecutor(args.jobs, mp_context=ctx_fork) as ex:
            futs = {ex.submit(run_case, j): j for j in jobs}
            for f in cf.as_completed(futs):
                try:
                    r = f.result()
                except Exception as e:
                    j = futs[f]
                    r = {'name': j[0], 'parts': j[1], 'status': 'ERROR', 'details': [repr(e)],
                         'expect': '?', 'check_ok': None, 'grade_valid': None, 'audit': None,
                         'metrics': {}}
                results.append(r)
                got = (f'{"ACCEPT" if r.get("check_ok") else "REJECT"}/'
                       f'{"VALID" if r.get("grade_valid") else "INVALID"}'
                       f'/{r.get("audit") or "-"}')
                print(f'{r["status"]:7s} {r["name"]:60s} expect {r["expect"]:15s} got {got}'
                      + (f'   {"; ".join(r["details"])}' if r['details'] else ''), flush=True)
        order = {j[0]: i for i, j in enumerate(jobs)}
        results.sort(key=lambda r: order[r['name']])

        # series: a metric must grow with the mutation's size
        series_out = []
        for group, key in (('scale', 'speed_rmse_straight'), ('noise', 'speed_rmse_straight')):
            pts = sorted((r['series'][1], r['metrics'].get(key, math.nan)) for r in results
                         if r.get('series') and r['series'][0] == group)
            if len(pts) >= 2:
                ok = monotonic([ctx.metrics[key]] + [v for _, v in pts])
                series_out.append({'group': group, 'metric': key, 'points': pts, 'monotonic': ok})
                print(f'{"OK" if ok else "METRIC":7s} series:{group:10s} {key} '
                      f'{fmt(ctx.metrics[key])} -> ' + ' -> '.join(f'{fmt(v)} (@{x})' for x, v in pts))
                failures += 0 if ok else 1

        counts: Dict[str, int] = {}
        for r in results:
            counts[r['status']] = counts.get(r['status'], 0) + 1
        bad = [r for r in results if r['status'] in ('MISSED', 'FALSE', 'METRIC', 'CRASH', 'ERROR')]
        failures += len(bad)
        print('\nsummary: ' + ', '.join(f'{k} {v}' for k, v in sorted(counts.items()))
              + f'  ({len(results)} cases)')
        for r in bad:
            print(f'  {r["status"]}: {r["name"]}  expect {r["expect"]}; ' + '; '.join(r['details']))
            print(f'      check: {r.get("check_reasons")}  grade: {r.get("gate_failures")}  '
                  f'bag: {r.get("bag")}')
        report.update(results=results, series=series_out, counts=counts)

    if args.gate:
        if not SUB_REF.is_dir():
            print(f'gate: no reference submission at {SUB_REF}')
            failures += 1
        else:
            g = run_gate(out, args.gate_timeout)
            report['gate'] = g
            failures += sum(1 for r in g if r['status'] not in ('CAUGHT',))

    report['seconds'] = round(time.time() - t_start)
    if args.no_bags and (out / 'results.json').is_file():     # keep the last bag run's results
        try:
            old = json.loads((out / 'results.json').read_text())
            old.update(gate=report.get('gate'), gate_seconds=report['seconds'])
            report = old
        except ValueError:
            pass
    (out / 'results.json').write_text(json.dumps(report, indent=2, default=str))
    print(f'\n{out / "results.json"}  ({report["seconds"]} s)')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
