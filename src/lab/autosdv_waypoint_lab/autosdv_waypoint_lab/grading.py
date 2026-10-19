"""Lab 2 (waypoint) run metrics, from a rosbag2 recording.

    lab2_grade_run BAG [BAG ...] [--course DIR] [--json]

Per run, over the time the car was engaged (AUTONOMOUS) and moving:

    laps            laps completed (waypoint_path_publisher's progress)
    finished        the run reached its end and the car stopped there
    xt_rmse/xt_max  cross-track error of the LOCALIZED pose against the
                    course polyline (path_tracker's debug), m
    true_xt_*       the same for the TRUE pose, on the bench only
                    (vehicle_bench_plant/odometry); the gap between the two
                    is localization error, not tracking error
    loc_err_*       distance between the localized and the true pose (bench)
    speed_rmse      actuator view: commanded speed (vehicle_cmd_gate's output)
                    against the wheel-speed report, while commanded > 0.05 m/s
    mrm             MRM activations (mrm_state leaving NORMAL) before the run
                    finished; a parked car's after it do not count
    disengages      AUTONOMOUS -> anything else before the run finished

The pass thresholds are the plan's (PASS below); the grader prints them next
to each number and does not decide marks.
"""

import argparse
import bisect
import json
import math
import os
import sys
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

PROGRESS = '/planning/waypoint_path_publisher/progress'
TRACKER = '/control/path_tracker/debug'
CONTROL = '/control/command/control_cmd'
VELOCITY = '/vehicle/status/velocity_status'
TRUE_POSE = '/vehicle_bench_plant/odometry'
KINEMATIC = '/localization/kinematic_state'
MRM = '/system/fail_safe/mrm_state'
MODE = '/api/operation_mode/state'
TOPICS = [PROGRESS, TRACKER, CONTROL, VELOCITY, TRUE_POSE, KINEMATIC, MRM, MODE]

PASS = {'laps': ('>=', 3), 'xt_rmse': ('<=', 0.15), 'xt_max': ('<=', 0.40),
        'speed_rmse': ('<=', 0.15), 'mrm': ('<=', 0), 'disengages': ('<=', 0)}

AUTONOMOUS = 2    # autoware_adapi_v1_msgs/OperationModeState.AUTONOMOUS
MRM_NORMAL = 1    # autoware_adapi_v1_msgs/MrmState.NORMAL


@dataclass
class RunMetrics:
    name: str
    laps: int = 0
    laps_planned: int = 0
    finished: bool = False
    duration: float = 0.0
    xt_rmse: float = math.nan
    xt_max: float = math.nan
    true_xt_rmse: float = math.nan
    true_xt_max: float = math.nan
    loc_err_rmse: float = math.nan
    loc_err_max: float = math.nan
    speed_rmse: float = math.nan
    mean_speed: float = math.nan
    mrm: int = 0
    disengages: int = 0
    notes: List[str] = field(default_factory=list)


def rms(xs: Sequence[float]) -> float:
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else math.nan


def max_abs(xs: Sequence[float]) -> float:
    return max(abs(x) for x in xs) if xs else math.nan


class Intervals:
    """Time intervals [t0, t1) over which a condition held."""

    def __init__(self, spans: List[Tuple[float, float]]):
        self.spans = spans
        self.starts = [s for s, _ in spans]

    def contains(self, t: float) -> bool:
        i = bisect.bisect_right(self.starts, t) - 1
        return i >= 0 and self.spans[i][0] <= t < self.spans[i][1]


def engaged_intervals(modes: List[Tuple[float, int]], t_end: float) -> Intervals:
    spans, start = [], None
    for t, mode in modes:
        if mode == AUTONOMOUS and start is None:
            start = t
        elif mode != AUTONOMOUS and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, t_end))
    return Intervals(spans)


def latest_before(ts: List[float], vals: list, t: float):
    i = bisect.bisect_right(ts, t) - 1
    return vals[i] if i >= 0 else None


def course_distance(course_dir: Optional[str]):
    """A function (x, y) -> distance from the course polyline, or None."""
    if not course_dir:
        return None
    from autosdv_waypoint_lab.course import load_course
    from autosdv_waypoint_lab.path import Path, load_csv
    c = load_course(course_dir)
    x, y, v = load_csv(c.path_file, c.v_ref)
    if c.closed:
        x, y, v = x + [x[0]], y + [y[0]], v + [v[0]]
    path = Path(x, y, v)
    return lambda px, py: abs(path.project(px, py).cross_track)


def metrics_from_messages(name: str, msgs: Dict[str, list],
                          course_dir: Optional[str] = None) -> RunMetrics:
    m = RunMetrics(name)
    t_all = [t for lst in msgs.values() for t, _ in lst]
    if not t_all:
        m.notes.append('empty bag')
        return m
    t_end = max(t_all)
    modes = [(t, int(msg.mode)) for t, msg in msgs.get(MODE, [])]
    engaged = engaged_intervals(modes, t_end)
    if not engaged.spans:
        m.notes.append('never AUTONOMOUS')

    prog = msgs.get(PROGRESS, [])
    if prog:
        last = prog[-1][1]
        m.laps = int(max(p.laps_done for _, p in prog))
        m.laps_planned = int(last.laps)
        m.finished = bool(last.finished)
    else:
        m.notes.append(f'no {PROGRESS}')

    # the run ends where the path publisher says it finished
    t_finish = next((t for t, p in prog if p.finished), None)

    xt = []
    speeds = []
    for t, d in msgs.get(TRACKER, []):
        if engaged.contains(t) and d.speed > 0.05 and math.isfinite(d.cross_track):
            xt.append(d.cross_track)
            speeds.append(d.speed)
    m.xt_rmse, m.xt_max = rms(xt), max_abs(xt)
    m.mean_speed = sum(speeds) / len(speeds) if speeds else math.nan
    if engaged.spans:
        m.duration = sum(min(b, t_finish or b) - a for a, b in engaged.spans if a < (t_finish or b))

    dist = course_distance(course_dir)
    truth = msgs.get(TRUE_POSE, [])
    if truth:
        kin = msgs.get(KINEMATIC, [])
        kin_t = [t for t, _ in kin]
        kin_v = [k for _, k in kin]
        txt, lerr = [], []
        for t, o in truth:
            if not engaged.contains(t) or o.twist.twist.linear.x <= 0.05:
                continue
            p = o.pose.pose.position
            if dist is not None:
                txt.append(dist(p.x, p.y))
            k = latest_before(kin_t, kin_v, t)
            if k is not None:
                kp = k.pose.pose.position
                lerr.append(math.hypot(kp.x - p.x, kp.y - p.y))
        m.true_xt_rmse, m.true_xt_max = rms(txt), max_abs(txt)
        m.loc_err_rmse, m.loc_err_max = rms(lerr), max_abs(lerr)
        if dist is None:
            m.notes.append('true cross-track needs --course')

    vel = msgs.get(VELOCITY, [])
    vel_t = [t for t, _ in vel]
    vel_v = [float(v.longitudinal_velocity) for _, v in vel]
    err = []
    for t, c in msgs.get(CONTROL, []):
        if not engaged.contains(t) or c.longitudinal.velocity <= 0.05:
            continue
        meas = latest_before(vel_t, vel_v, t)
        if meas is not None:
            err.append(c.longitudinal.velocity - meas)
    m.speed_rmse = rms(err)

    prev = MRM_NORMAL
    for t, s in msgs.get(MRM, []):
        st = int(s.state)
        if st != MRM_NORMAL and prev == MRM_NORMAL and (t_finish is None or t < t_finish):
            m.mrm += 1
        prev = st
    was = False
    for t, mode in modes:
        now = mode == AUTONOMOUS
        if was and not now and (t_finish is None or t < t_finish):
            m.disengages += 1
        was = now
    return m


def grade_bag(path: str, course_dir: Optional[str] = None) -> RunMetrics:
    from autosdv_coach_lab.calibration.bagio import read_bag
    msgs = read_bag(path, TOPICS)
    return metrics_from_messages(os.path.basename(os.path.normpath(path)), msgs, course_dir)


def verdict(key, value):
    if key not in PASS or value is None or (isinstance(value, float) and math.isnan(value)):
        return ''
    op, lim = PASS[key]
    ok = value >= lim if op == '>=' else value <= lim
    return f'{"pass" if ok else "FAIL"} ({op} {lim})'


def format_metrics(m: RunMetrics) -> str:
    rows = [
        ('laps', f'{m.laps} / {m.laps_planned}', verdict('laps', m.laps)),
        ('finished', str(m.finished), ''),
        ('engaged time', f'{m.duration:.1f} s', ''),
        ('mean speed', f'{m.mean_speed:.2f} m/s', ''),
        ('cross-track RMSE', f'{m.xt_rmse:.3f} m', verdict('xt_rmse', m.xt_rmse)),
        ('cross-track max', f'{m.xt_max:.3f} m', verdict('xt_max', m.xt_max)),
        ('speed-tracking RMSE', f'{m.speed_rmse:.3f} m/s', verdict('speed_rmse', m.speed_rmse)),
        ('MRM activations', str(m.mrm), verdict('mrm', m.mrm)),
        ('disengages', str(m.disengages), verdict('disengages', m.disengages)),
    ]
    if not math.isnan(m.loc_err_rmse):
        rows += [
            ('bench: true cross-track RMSE', f'{m.true_xt_rmse:.3f} m', ''),
            ('bench: true cross-track max', f'{m.true_xt_max:.3f} m', ''),
            ('bench: localization error RMSE', f'{m.loc_err_rmse:.3f} m', ''),
            ('bench: localization error max', f'{m.loc_err_max:.3f} m', ''),
        ]
    w = max(len(r[0]) for r in rows)
    lines = [m.name] + [f'  {k:<{w}}  {v:<14} {c}'.rstrip() for k, v, c in rows]
    lines += [f'  note: {n}' for n in m.notes]
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Lab 2 (waypoint) run metrics from rosbag2 recordings.')
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--course', default=None,
                    help='course directory, for the true cross-track on bench bags')
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--score', action='store_true',
                    help='route grading (lab2_uturn): points and validity; see route_grade.py')
    ap.add_argument('--reference', action='append', default=[], metavar='BAG',
                    help='with --score: the car\'s TA reference bag of the route')
    ap.add_argument('--route', choices=['uturn'], default=None,
                    help='with --score: the route, when the bag has no metadata (one bag only)')
    args = ap.parse_args(argv)
    if args.score:
        return score_main(args)
    out = [grade_bag(b, args.course) for b in args.bags]
    if args.json:
        print(json.dumps([asdict(m) for m in out], indent=2))
    else:
        print('\n\n'.join(format_metrics(m) for m in out))
    return 0


def score_main(args):
    from autosdv_waypoint_lab import route_grade as rg
    if args.route and len(args.bags) > 1:
        print('--route names one route; give one bag', file=sys.stderr)
        return 2
    refs = {}
    for r in args.reference:
        m, _ = rg.read_route_bag(r, None, args.course)
        if m.route is None:
            print(f'reference {r}: {"; ".join(m.notes)}', file=sys.stderr)
            return 2
        if m.route in refs:
            print(f'two references for {m.route}', file=sys.stderr)
            return 2
        refs[m.route] = m
    scores, out, ungraded = [], [], 0
    for b in args.bags:
        m, meta = rg.read_route_bag(b, args.route, args.course)
        if m.route is None and m.ungraded:
            print(rg.format_ungraded(m))
            ungraded += 1
            continue
        if m.route is None:
            print(f'{b}: {"; ".join(m.notes)}', file=sys.stderr)
            return 2
        s = rg.score_route(m, refs.get(m.route))
        if m.route not in refs:
            m.notes.append('no reference bag for this route: fixed absolute pass lines')
        scores.append(s)
        out.append((m, s, meta))
    if not out:
        return 0
    if args.json:
        print(json.dumps([{'metrics': asdict(m), 'points': s.points, 'total': s.total,
                           'max': s.max_total, 'lines': s.lines, 'gate_failures': s.gate_failures}
                          for m, s, _ in out], indent=2))
    else:
        print('\n\n'.join(rg.format_route(m, s) for m, s, _ in out))
    if len(scores) > 1:
        tot, mx = rg.team_total(scores)
        print(f'\nteam total: {tot:.2f} / {mx:g}')
        for m, s_, _ in out:
            if s_.invalid:
                print(f'  {m.name}: INVALID (scores 0)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
