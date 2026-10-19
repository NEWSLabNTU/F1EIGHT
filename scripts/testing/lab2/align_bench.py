#!/usr/bin/env python3
"""survey-align on the vehicle bench: a field with a misplaced origin board, recovered.

    python3 scripts/testing/lab2/align_bench.py [--out tmp/lab2_align] [--run]

Builds, from the graded route lab2_uturn (its boards placed where its table says):

  OUT/truth    the field as it really is: the route's boards, except the stage-1
               origin board (the anchor, id 24), which stands --shift off, turned
               --yaw-err deg and leaning --lean deg back; start_pose = where the
               car really stands, --start (dx, dy, dyaw deg) off the taped box.
               The bench plant starts there; mock_aruco_camera renders these boards.
  OUT/survey   what the survey (`just aruco-map survey 24`) makes of it: every board in
               the misplaced origin board's frame (autosdv_marker_survey.simulate.
               stage1_map, the survey's own frame code), as a new survey writes it.

With --run it then runs `scripts/aruco_map/survey_align.py OUT/survey --bench --truth
OUT/truth`
(bench_straight_driver drives --distance m at 0.3 m/s) and compares the aligned
boards with the truth expressed in the frame of the car's ACTUAL start pose
(which is what the alignment defines): PASS within --tol-pos m and --tol-yaw deg.
`--compare` alone compares an existing OUT/survey. Bench only; nothing here
touches a car.
"""

import argparse
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
ROUTE = REPO / 'src/lab/autosdv_waypoint_lab/config/courses/lab2_uturn'


def build(out: Path, shift, yaw_err, lean, start, anchor=24):
    from autosdv_marker_survey.simulate import stage1_map
    from autosdv_waypoint_lab import course as course_lib
    d = yaml.safe_load((ROUTE / 'course.yaml').read_text())
    height = float(d.get('marker_height', 0.25))
    truth = {int(m['id']): [float(m['x']), float(m['y']), height, float(m['yaw']), 0.0, 0.0]
             for m in d['markers']}
    t = truth[anchor]
    t[0] += shift[0]
    t[1] += shift[1]
    t[3] += math.radians(yaw_err)
    t[4] = math.radians(lean)
    truth = {k: tuple(v) for k, v in truth.items()}

    tdir = out / 'truth'
    if tdir.exists():
        shutil.rmtree(tdir)
    tdir.mkdir(parents=True)
    tc = {k: d[k] for k in ('path', 'closed', 'laps', 'v_ref') if k in d}
    tc['start_pose'] = {'x': start[0], 'y': start[1], 'yaw': math.radians(start[2])}
    tc.update({'marker_size': d['marker_size'], 'dictionary': d['dictionary'], 'marker_height': height})
    tc['markers'] = [{'id': k, 'x': round(v[0], 4), 'y': round(v[1], 4), 'z': v[2], 'yaw': round(v[3], 5),
                      'pitch': round(v[4], 5), 'roll': 0.0} for k, v in sorted(truth.items())]
    (tdir / 'course.yaml').write_text(
        '# align_bench TRUTH: the field as it really is (lab2_uturn boards, the anchor misplaced),\n'
        '# start_pose = where the car really stands. Bench only.\n' + yaml.safe_dump(tc, sort_keys=False))
    shutil.copyfile(ROUTE / 'path.csv', tdir / 'path.csv')
    _maps(tdir)

    stage1, _ = stage1_map(truth, anchor)
    sdir = out / 'survey'
    if sdir.exists():
        shutil.rmtree(sdir)
    sdir.mkdir(parents=True)
    sc = {'path': 'path.csv', 'closed': True, 'laps': 3, 'v_ref': 0.8,
          'start_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
          'marker_size': d['marker_size'], 'dictionary': d['dictionary'],
          'survey': {'origin_marker': anchor, 'note': 'align_bench: simulated stage 1, no noise'},
          'markers': stage1}
    (sdir / 'course.yaml').write_text(
        f'# align_bench STAGE 1: the boards in the frame of board {anchor} (misplaced). Bench only.\n'
        + yaml.safe_dump(sc, sort_keys=False, default_flow_style=None))
    _maps(sdir)
    return truth


def _maps(d: Path):
    from autosdv_waypoint_lab import course as course_lib
    c = course_lib.load_course(str(d))
    (d / 'lanelet2_map.osm').write_text(course_lib.make_osm(c))
    (d / 'ar_tag.param.yaml').write_text(course_lib.make_ar_tag_params(c))
    (d / 'map_projector_info.yaml').write_text('projector_type: local\n')


def compare(out: Path, tol_pos: float, tol_yaw: float) -> bool:
    tc = yaml.safe_load((out / 'truth' / 'course.yaml').read_text())
    sp = tc['start_pose']
    sx, sy, syaw = float(sp['x']), float(sp['y']), float(sp['yaw'])
    al = yaml.safe_load((out / 'survey' / 'course.yaml').read_text())
    if not isinstance(al.get('alignment'), dict):
        print(f'{out / "survey"} is not aligned')
        return False
    got = {int(m['id']): m for m in al['markers']}
    c, s = math.cos(syaw), math.sin(syaw)
    worst_p, worst_y = 0.0, 0.0
    print('board   truth (start frame)        aligned                   error')
    for m in tc['markers']:
        dx, dy = float(m['x']) - sx, float(m['y']) - sy
        ex, ey, eyaw = c * dx + s * dy, -s * dx + c * dy, float(m['yaw']) - syaw
        g = got.get(int(m['id']))
        if g is None:
            print(f'  {m["id"]}: missing from the aligned course')
            return False
        dp = math.hypot(g['x'] - ex, g['y'] - ey)
        dyaw = abs(math.degrees(math.atan2(math.sin(g['yaw'] - eyaw), math.cos(g['yaw'] - eyaw))))
        worst_p, worst_y = max(worst_p, dp), max(worst_y, dyaw)
        print(f'  {m["id"]:3d}  ({ex:7.3f}, {ey:7.3f}) {math.degrees(eyaw):7.1f}   '
              f'({g["x"]:7.3f}, {g["y"]:7.3f}) {math.degrees(g["yaw"]):7.1f}   '
              f'{100 * dp:5.1f} cm {dyaw:5.2f} deg')
    # without stage 2 the run would use stage 1 as if it were the route frame
    s1 = yaml.safe_load((out / 'survey' / 'stage1' / 'course.yaml').read_text())
    raw = max(math.hypot(float(a['x']) - (c * (float(b['x']) - sx) + s * (float(b['y']) - sy)),
                         float(a['y']) - (-s * (float(b['x']) - sx) + c * (float(b['y']) - sy)))
              for a in s1['markers'] for b in tc['markers'] if int(a['id']) == int(b['id']))
    ok = worst_p <= tol_pos and worst_y <= tol_yaw
    print(f'worst: {100 * worst_p:.1f} cm, {worst_y:.2f} deg (allowed {100 * tol_pos:.0f} cm, '
          f'{tol_yaw:g} deg); the stage-1 map as it stood was up to {raw:.2f} m off')
    print('ALIGN BENCH ' + ('PASS' if ok else 'FAIL'))
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=str(REPO / 'tmp' / 'lab2_align'))
    ap.add_argument('--shift', default='0.12,-0.07', help='the anchor board off its spot, x,y [m]')
    ap.add_argument('--yaw-err', type=float, default=3.0, help='the anchor board turned [deg]')
    ap.add_argument('--lean', type=float, default=10.0, help='the anchor board leaning back [deg]')
    ap.add_argument('--start', default='0.04,-0.03,1.5', help='the car off the taped box: dx,dy [m],dyaw [deg]')
    ap.add_argument('--distance', type=float, default=3.0)
    ap.add_argument('--run', action='store_true', help='run survey-align on the bench, then compare')
    ap.add_argument('--compare', action='store_true', help='only compare an existing OUT')
    ap.add_argument('--tol-pos', type=float, default=0.03)
    ap.add_argument('--tol-yaw', type=float, default=0.5)
    a = ap.parse_args(argv)
    out = Path(a.out).resolve()
    if not a.compare:
        shift = [float(v) for v in a.shift.split(',')]
        start = [float(v) for v in a.start.split(',')]
        build(out, shift, a.yaw_err, a.lean, start)
        print(f'truth  {out / "truth"}  (anchor 24 off by {shift} m, {a.yaw_err:g} deg, leaning {a.lean:g} deg;'
              f' car at {start})')
        print(f'stage1 {out / "survey"}')
        if not a.run:
            return 0
        r = subprocess.run([sys.executable, str(REPO / 'scripts/aruco_map/survey_align.py'), str(out / 'survey'), '--bench',
                            '--truth', str(out / 'truth'),
                            '--distance', str(a.distance)], cwd=REPO)
        if r.returncode != 0:
            print('ALIGN BENCH FAIL: survey-align did not align')
            return 1
    return 0 if compare(out, a.tol_pos, a.tol_yaw) else 1


if __name__ == '__main__':
    sys.exit(main())
