"""lab2_check_bags: is this a bag the grader should accept?

    lab2_check_bags BAG [BAG ...] [--course DIR ...] [--car A] [--team 3]

Each bag is rejected, with the reasons printed, if it has:

  - a missing required topic (bag_meta.REQUIRED_TOPICS),
  - no metadata, or no car / team ID in it,
  - a camera stream under MIN_CAMERA_HZ, not 960 x 600, or not covering the run,
  - a course that is not the template: its route_hash differs from the
    template's (or from a surveyed copy given with --course, which may be
    repeated). The route hash covers path.csv, the lap count, v_ref and the
    start pose; the markers are free, since a survey measures them,
  - a speed other than the target (metadata v_ref, and the speed actually
    commanded on the plateau),
  - too few laps (circles: 3) or a run that did not finish,
  - an MRM or a disengage before the finish,
  - no AUTONOMOUS period at all.

With several bags it also checks that they are the three routes, from one
car and one team. Exit status 1 if any bag is rejected.
"""

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from autosdv_waypoint_lab import bag_meta as bm
from autosdv_waypoint_lab import grading as g
from autosdv_waypoint_lab import route_grade as rg

MIN_CAMERA_HZ = 8.0          # recorded at 10
SPEED_TOLERANCE = 0.05       # m/s, commanded plateau vs the 0.8 m/s target
CAMERA_COVERAGE = 0.9        # fraction of the engaged time the camera must cover


@dataclass
class Verdict:
    bag: str
    route: Optional[str] = None
    car: Optional[str] = None
    team: Optional[str] = None
    reasons: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.reasons


def scan_bag(path: str):
    """({topic: [count, t_first, t_last]}, first camera message bytes), without
    deserializing anything."""
    import rosbag2_py
    from autosdv_coach_lab.calibration.bagio import _storage_id
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=_storage_id(path)),
                rosbag2_py.ConverterOptions('', ''))
    info: Dict[str, list] = {t.name: [0, None, None] for t in reader.get_all_topics_and_types()}
    cam = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        r = info[topic]
        t = t_ns * 1e-9
        r[0] += 1
        r[1] = t if r[1] is None else r[1]
        r[2] = t
        if topic == bm.CAMERA_TOPIC and cam is None:
            cam = data
    return info, cam


def camera_size(raw: bytes):
    try:
        import cv2
        import numpy as np
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import CompressedImage
        msg = deserialize_message(raw, CompressedImage)
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        return None if img is None else (img.shape[1], img.shape[0])
    except Exception:
        return None


def check_messages(name: str, topics: Dict[str, list], msgs: Dict[str, list], meta: Optional[dict],
                   course_dirs=(), cam_size=None,
                   expect_car: Optional[str] = None, expect_team: Optional[str] = None) -> Verdict:
    v = Verdict(name)
    # topics
    for t in bm.REQUIRED_TOPICS:
        if t not in topics or topics[t][0] == 0:
            v.reasons.append(f'missing topic {t}' if t not in topics else f'topic {t} is empty')
    for t in bm.OPTIONAL_TOPICS:
        if t not in topics or topics[t][0] == 0:
            v.warnings.append(f'optional topic {t} missing')

    # metadata and course
    route = bm.route_from_meta(meta)
    v.route = route
    if not meta:
        v.reasons.append('no metadata (record with `just lab2 record`)')
    else:
        v.car, v.team = str(meta.get('car_id') or ''), str(meta.get('team_id') or '')
        if not v.car or not v.team:
            v.reasons.append('metadata has no car or team ID')
        if expect_car and v.car != expect_car:
            v.reasons.append(f'car {v.car!r}, expected {expect_car!r}')
        if expect_team and v.team != expect_team:
            v.reasons.append(f'team {v.team!r}, expected {expect_team!r}')
        if meta.get('git_dirty'):
            v.warnings.append('recorded from a modified git tree')
        if route is None:
            v.reasons.append('cannot tell which route this bag drove')
        else:
            allowed = []
            for d in course_dirs:
                try:
                    allowed.append(bm.course_hashes(d)['route_hash'])
                except OSError as e:
                    v.reasons.append(f'cannot read course {d}: {e}')
            if not course_dirs:
                td = rg.template_dir(route)
                if td:
                    allowed.append(bm.course_hashes(td)['route_hash'])
            if allowed and meta.get('route_hash') not in allowed:
                v.reasons.append('modified course: route hash does not match the template '
                                 '(path, laps, v_ref or start pose changed)')
            if not allowed:
                v.warnings.append('no template course to compare the route hash with')
            vref = float((meta.get('route') or {}).get('v_ref', math.nan))
            if not math.isclose(vref, rg.TARGET_SPEED, abs_tol=1e-6):
                v.reasons.append(f'course v_ref {vref} m/s, expected {rg.TARGET_SPEED} m/s')

    # run
    m = g.metrics_from_messages(name, msgs)
    t_all = [t for lst in msgs.values() for t, _ in lst]
    modes = [(t, int(x.mode)) for t, x in msgs.get(g.MODE, [])]
    eng = g.engaged_intervals(modes, max(t_all) if t_all else 0.0)
    if not eng.spans:
        v.reasons.append('never engaged (no AUTONOMOUS period)')
    else:
        if m.mrm:
            v.reasons.append(f'{m.mrm} MRM activation(s) during the run')
        if m.disengages:
            v.reasons.append('left autonomous mode before the finish')
        if not m.finished:
            v.reasons.append('the run did not finish')
        if route in ('circle_cw', 'circle_ccw') and m.laps < rg.LAPS_REQUIRED:
            v.reasons.append(f'{m.laps} lap(s) completed, {rg.LAPS_REQUIRED} required')
        if route in ('circle_cw', 'circle_ccw') and m.laps_planned and m.laps_planned < rg.LAPS_REQUIRED:
            v.reasons.append(f'course set to {m.laps_planned} lap(s), {rg.LAPS_REQUIRED} required')
        # circle gate: localized radius error (INVALID bag, re-run)
        if route in ('circle_cw', 'circle_ccw') and msgs.get(g.KINEMATIC) and msgs.get(g.VELOCITY):
            td = (course_dirs[0] if len(course_dirs) == 1 else None) or rg.template_dir(route)
            geo = rg.geometry_from_course(td, route) if td else None
            rm = rg.route_metrics(name, msgs, route, geo)
            if geo and not math.isnan(rm.radius_rms):
                if rm.radius_rms > rg.RADIUS_RMS_MAX:
                    v.reasons.append(f'radius error RMS {rm.radius_rms:.3f} m > {rg.RADIUS_RMS_MAX} m')
                if rm.radius_max > rg.RADIUS_MAX:
                    v.reasons.append(f'radius error max {rm.radius_max:.3f} m > {rg.RADIUS_MAX} m')
            elif geo:
                v.reasons.append('radius error not measurable (no steady window)')
        # commanded plateau
        t0 = eng.spans[0][0]
        cmd = sorted(c.longitudinal.velocity for t, c in msgs.get(g.CONTROL, []) if eng.contains(t))
        if cmd:
            top = cmd[int(0.95 * (len(cmd) - 1))]
            if abs(top - rg.TARGET_SPEED) > SPEED_TOLERANCE:
                v.reasons.append(f'commanded speed peaks at {top:.2f} m/s, expected {rg.TARGET_SPEED} m/s')
        # camera
        c = topics.get(bm.CAMERA_TOPIC)
        if c and c[0] > 1:
            span = c[2] - c[1]
            hz = (c[0] - 1) / span if span > 0 else 0.0
            if hz < MIN_CAMERA_HZ:
                v.reasons.append(f'camera at {hz:.1f} Hz, at least {MIN_CAMERA_HZ} required')
            t_fin = next((t for t, p in msgs.get(g.PROGRESS, []) if p.finished), None)
            t_end = min(max(b for _, b in eng.spans), t_fin) if t_fin else max(b for _, b in eng.spans)
            run = max(t_end - t0, 1e-6)
            covered = max(0.0, min(c[2], t_end) - max(c[1], t0))
            if covered < CAMERA_COVERAGE * run:
                v.reasons.append(f'camera covers {100 * covered / run:.0f} % of the run, '
                                 f'{100 * CAMERA_COVERAGE:.0f} % required')
            if cam_size is not None and tuple(cam_size) != bm.CAMERA_SIZE:
                v.reasons.append(f'camera image {cam_size[0]}x{cam_size[1]}, expected '
                                 f'{bm.CAMERA_SIZE[0]}x{bm.CAMERA_SIZE[1]}')
    return v



def check_bag(path: str, course_dirs=(), expect_car=None, expect_team=None) -> Verdict:
    from autosdv_coach_lab.calibration.bagio import read_bag
    name = os.path.basename(os.path.normpath(path))
    try:
        topics, cam = scan_bag(path)
    except Exception as e:
        return Verdict(name, reasons=[f'cannot read the bag: {e}'])
    msgs = read_bag(path, [g.PROGRESS, g.CONTROL, g.MRM, g.MODE, g.VELOCITY, g.KINEMATIC, bm.META_TOPIC])
    meta = bm.meta_from_messages(msgs, path)
    return check_messages(name, topics, msgs, meta, course_dirs,
                          camera_size(cam) if cam else None, expect_car, expect_team)


def check_set(verdicts: List[Verdict]) -> List[str]:
    """Problems with the bags as a set of three."""
    out = []
    routes = [v.route for v in verdicts if v.route]
    for r in bm.ROUTES:
        if len(verdicts) >= 3 and routes.count(r) == 0:
            out.append(f'no bag for route {r}')
        if routes.count(r) > 1:
            out.append(f'route {r} submitted {routes.count(r)} times')
    if len({v.car for v in verdicts if v.car}) > 1:
        out.append('bags from different cars')
    if len({v.team for v in verdicts if v.team}) > 1:
        out.append('bags from different teams')
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description='Check Lab 2 submission bags.')
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--course', action='append', default=[], metavar='DIR',
                    help='the template or surveyed course directory the bags must match '
                         '(repeat for several routes); default: the packaged templates')
    ap.add_argument('--car', default=None, help='the car ID the bags must carry')
    ap.add_argument('--team', default=None, help='the team ID the bags must carry')
    args = ap.parse_args(argv)
    verdicts = [check_bag(b, args.course, args.car, args.team) for b in args.bags]
    for v in verdicts:
        print(f'{"ACCEPT" if v.ok else "REJECT"}  {v.bag}  route={v.route}  car={v.car}  team={v.team}')
        for r in v.reasons:
            print(f'    reject: {r}')
        for w in v.warnings:
            print(f'    warn:   {w}')
    set_problems = check_set(verdicts) if len(verdicts) > 1 else []
    for p in set_problems:
        print(f'SET: {p}')
    return 0 if all(v.ok for v in verdicts) and not set_problems else 1


if __name__ == '__main__':
    raise SystemExit(main())
