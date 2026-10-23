"""`just lab3 check-lanelet MAP`: is the lanelet2 loop drivable, before the field?

    lab3_check_lanelet MAP_DIR [--no-sim] [--min-length 15] [--width 0.6 1.2]
                       [--clearance 0.25] [--band 0.05 0.8] [--timeout S] [--keep-log]

Static requirements, then a lap in the planning simulator:

  1 map check   scripts/map/check_map.py passes (pose_source ndt) and projector_type is Local
  2 closed chain  the road lanelets form one closed chain by successor relations
  3 clockwise   the loop's centreline, seen from above, has negative signed area
  4 lane width  every centreline sample is between the two width limits
  5 clearance   the centreline keeps --clearance from every map point in the height band
                above the floor, so walls and desks count and the floor does not
  6 loop length at least --min-length
  6 turn radius no curve tighter than the car's minimum turning radius,
                wheel_base / tan(max_steer_angle) from vehicle_info.param.yaml
  7 sim lap     a full lap in the planning simulator (lab3_sim_lap), unless --no-sim

The limits are provisional settings. Exit 0 only when every row passes. The result is
recorded in the map folder's lab3_map.yaml (section lanelet_check).
"""

import argparse
import math
import os
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np
import yaml

from autosdv_lab3 import mapdir
from autosdv_lab3.route import Loop, MapError, find_chain, point_polyline_distance, read_lanelets

DEFAULTS = dict(min_length=15.0, width=(0.6, 1.2), clearance=0.25, band=(0.05, 0.8))
CURVE_WINDOW = 1.0   # m of centreline the heading change is measured over
# a hand-drawn polyline turns at its vertices; over a 1 m window a 90 deg corner drawn
# sharp reads 0.64 m and an arc of radius R reads R, whatever its vertex spacing
VEHICLE_INFO_REL = 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_description/config/vehicle_info.param.yaml'


@dataclass
class Result:
    requirement: str
    ok: Optional[bool]    # None: not run (an earlier failure leaves nothing to measure)
    value: str = ''
    reason: str = ''


def repo_root() -> Optional[Path]:
    """The AutoSDV checkout: $AUTOSDV_ROOT, else up from this file, else up from cwd."""
    if os.environ.get('AUTOSDV_ROOT'):
        return Path(os.environ['AUTOSDV_ROOT'])
    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        for d in [start] + list(start.parents):
            if (d / 'versions.yaml').is_file() and (d / 'scripts/map/check_map.py').is_file():
                return d
    return None


def vehicle_info() -> dict:
    path = None
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('autosdv_vehicle_description')) / 'config/vehicle_info.param.yaml'
    except (ImportError, LookupError):
        pass
    if path is None or not path.is_file():
        root = repo_root()
        path = root / VEHICLE_INFO_REL if root else None
    if path is None or not path.is_file():
        raise FileNotFoundError('vehicle_info.param.yaml (autosdv_vehicle_description) not found')
    return yaml.safe_load(path.read_text())['/**']['ros__parameters']


# Autoware's planner, not the car, sets the floor: the planning validator rejects any
# trajectory curving tighter than radius 1 m (curvature 1.0), and with its 2 m path
# interval the RC simulator could not drive 1.2 m corners while 1.6 m ones passed.
# PROVISIONAL until a real car drives the loop.
PLANNER_MIN_RADIUS = 1.5


def min_turning_radius() -> float:
    v = vehicle_info()
    return float(v['wheel_base']) / math.tan(float(v['max_steer_angle']))


# ------------------------------------------------------------------ the checks

def check_map_files(map_dir: str) -> Result:
    root = repo_root()
    if root is None:
        return Result('1 map check', False, '', 'scripts/map/check_map.py not found (set AUTOSDV_ROOT)')
    sys.path.insert(0, str(root / 'scripts/map'))
    try:
        import check_map
    finally:
        sys.path.pop(0)
    lines, _ = check_map.run_check(Path(map_dir), 'ndt', 'occupancy_grid.yaml', None)
    # Autoware's loader builds the metadata itself for a single PCD, and the Lab 3
    # folder has none, so that one line does not count
    fails = [ln for ln in lines if ln.status == 'FAIL' and ln.name != 'pointcloud_map_metadata.yaml']
    if fails:
        return Result('1 map check', False, f'{len(fails)} failed',
                      '; '.join(f'{ln.name}: {ln.message}' for ln in fails))
    with open(os.path.join(map_dir, mapdir.PROJECTOR)) as f:
        ptype = (yaml.safe_load(f) or {}).get('projector_type')
    if ptype != 'Local':
        return Result('1 map check', False, f'projector_type {ptype}',
                      'must be Local: the map is in the board frame, not on the globe'
                      + (' (lowercase "local" is deprecated in Autoware)' if ptype == 'local' else ''))
    return Result('1 map check', True, 'projector_type Local')


def floor_height(z: np.ndarray) -> float:
    """The floor: the most populated 2 cm z bin within 0.5 m of the map origin's height
    (the anchor puts the origin on the floor; this absorbs a small anchoring offset)."""
    near = z[np.abs(z) < 0.5]
    if near.size == 0:
        return 0.0
    hist, edges = np.histogram(near, bins=np.arange(-0.5, 0.5001, 0.02))
    i = int(np.argmax(hist))
    return float(0.5 * (edges[i] + edges[i + 1]))


def clearance(loop: Loop, pcd_path: str, band) -> tuple:
    """(min clearance, where, floor z, points in band)."""
    from reflective_pose_core.pointcloud_io import read_pcd
    pts = read_pcd(pcd_path).points
    floor = floor_height(pts[:, 2])
    keep = (pts[:, 2] >= floor + band[0]) & (pts[:, 2] <= floor + band[1])
    obstacles = pts[keep, :2]
    if len(obstacles) == 0:
        return math.inf, None, floor, 0
    try:
        from scipy.spatial import cKDTree
        d, _ = cKDTree(obstacles).query(loop.xy)
    except ImportError:
        d = np.array([np.min(np.hypot(*(obstacles - q).T)) for q in loop.xy])
    i = int(np.argmin(d))
    return float(d[i]), loop.xy[i], floor, int(len(obstacles))


def lane_widths(loop: Loop) -> np.ndarray:
    """Width at every centreline sample: distance to the left bound plus to the right."""
    out = []
    for lid in loop.order:
        ll = loop.lanelets[lid]
        c = ll.centre[:, :2]
        out.append(point_polyline_distance(c, ll.left[:, :2]) + point_polyline_distance(c, ll.right[:, :2]))
    return np.concatenate(out)


def tightest_radius(loop: Loop, window: float = CURVE_WINDOW) -> tuple:
    """(radius, where): window / |heading change over window|, minimised round the loop."""
    closed = np.vstack([loop.xy, loop.xy[:1]])
    seg = np.diff(closed, axis=0)
    heading = np.arctan2(seg[:, 1], seg[:, 0])
    turn = np.angle(np.exp(1j * (np.roll(heading, -1) - heading)))   # at the end of each segment
    s_vertex = np.append(loop.s[1:], loop.length)                     # where each turn happens
    s2 = np.concatenate([s_vertex, s_vertex + loop.length])
    c2 = np.concatenate([[0.0], np.cumsum(np.concatenate([turn, turn]))])
    i = np.arange(len(s_vertex))
    j = np.searchsorted(s2, s_vertex + window, side='right')   # last vertex in window: j - 1
    # the turns strictly after vertex i up to vertex j - 1 bend the segments between them,
    # so a regular polygon inscribed in an arc of radius R reads R exactly
    total = np.abs(c2[j] - c2[i + 1])
    length = s2[j - 1] - s2[i]
    curvature = np.where(length > 0.5 * window, total / np.maximum(length, 1e-9), 0.0)
    k = int(np.argmax(curvature))
    if curvature[k] < 1e-9:
        return math.inf, None
    return float(1.0 / curvature[k]), loop.xy[k]


def static_checks(map_dir: str, min_length=DEFAULTS['min_length'], width=DEFAULTS['width'],
                  clearance_min=DEFAULTS['clearance'], band=DEFAULTS['band']) -> List[Result]:
    res = [check_map_files(map_dir)]
    try:
        lanelets = read_lanelets(os.path.join(map_dir, mapdir.LANELET))
    except MapError as e:
        res.append(Result('2 closed chain', False, '', str(e)))
        return res + [Result(n, None, '', 'needs a readable map') for n in
                      ('3 clockwise', '4 lane width', '5 clearance', '6 loop length', '6 turn radius')]
    order, problem = find_chain(lanelets)
    if problem:
        res.append(Result('2 closed chain', False, f'{len(order)} of {len(lanelets)} chained', problem))
        return res + [Result(n, None, '', 'needs the closed chain') for n in
                      ('3 clockwise', '4 lane width', '5 clearance', '6 loop length', '6 turn radius')]
    res.append(Result('2 closed chain', True, f'{len(order)} lanelets'))
    loop = Loop.from_chain(lanelets, order)

    area = loop.signed_area()
    res.append(Result('3 clockwise', area < 0, f'signed area {area:+.1f} m^2',
                      '' if area < 0 else 'the loop runs counter-clockwise seen from above: '
                      'redraw it the other way round (lanelets point the way the car drives)'))

    w = lane_widths(loop)
    ok = bool(width[0] <= w.min() and w.max() <= width[1])
    res.append(Result('4 lane width', ok, f'{w.min():.2f}-{w.max():.2f} m',
                      '' if ok else f'must stay within {width[0]:g}-{width[1]:g} m'))

    pcd = os.path.join(map_dir, mapdir.PCD)
    if os.path.isfile(pcd):
        d, where, floor, n = clearance(loop, pcd, band)
        ok = d >= clearance_min
        at = f' at ({where[0]:.2f}, {where[1]:.2f})' if where is not None else ''
        res.append(Result('5 clearance', ok, f'{d:.2f} m{at}',
                          (f'floor at z {floor:+.2f}, {n} points {band[0]:g}-{band[1]:g} m above it'
                           if ok else f'the centreline passes {d:.2f} m from a wall or desk{at}; '
                           f'keep {clearance_min:g} m (points {band[0]:g}-{band[1]:g} m above the '
                           f'floor at z {floor:+.2f})')))
    else:
        res.append(Result('5 clearance', False, '', f'{mapdir.PCD} missing'))

    ok = loop.length >= min_length
    res.append(Result('6 loop length', ok, f'{loop.length:.1f} m',
                      '' if ok else f'at least {min_length:g} m'))

    r_car = min_turning_radius()
    r_min = max(r_car, PLANNER_MIN_RADIUS)
    r, where = tightest_radius(loop)
    ok = r >= r_min
    at = f' at ({where[0]:.2f}, {where[1]:.2f})' if where is not None else ''
    res.append(Result('6 turn radius', ok, f'{r:.2f} m{at}',
                      f'minimum {r_min:.2f} m (car {r_car:.2f} m, planner {PLANNER_MIN_RADIUS:g} m)'
                      if ok else
                      f'tighter than {r_min:.2f} m (car {r_car:.2f} m, planner '
                      f'{PLANNER_MIN_RADIUS:g} m): round the corner{at}'))
    return res


def table(results: List[Result]) -> str:
    rows = [('requirement', 'result', 'measured', 'reason')]
    for r in results:
        rows.append((r.requirement, {True: 'pass', False: 'FAIL', None: '-'}[r.ok], r.value, r.reason))
    w = [max(len(row[k]) for row in rows) for k in range(3)]
    return '\n'.join(f'  {a:<{w[0]}}  {b:<{w[1]}}  {c:<{w[2]}}  {d}'.rstrip() for a, b, c, d in rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Check a Lab 3 lanelet2 loop, then drive it in the simulator.')
    ap.add_argument('map_dir')
    ap.add_argument('--no-sim', action='store_true', help='static checks only')
    ap.add_argument('--min-length', type=float, default=DEFAULTS['min_length'], help='m')
    ap.add_argument('--width', type=float, nargs=2, default=DEFAULTS['width'], metavar=('MIN', 'MAX'))
    ap.add_argument('--clearance', type=float, default=DEFAULTS['clearance'], help='m')
    ap.add_argument('--band', type=float, nargs=2, default=DEFAULTS['band'], metavar=('LOW', 'HIGH'),
                    help='height band above the floor whose points are obstacles, m')
    ap.add_argument('--timeout', type=float, default=None, help='simulator lap timeout, s')
    ap.add_argument('--keep-log', action='store_true', help="keep the simulator's run directory")
    args = ap.parse_args(argv)
    if not os.path.isdir(args.map_dir):
        print(f'error: {args.map_dir} is not a directory', file=sys.stderr)
        return 2

    results = static_checks(args.map_dir, args.min_length, tuple(args.width),
                            args.clearance, tuple(args.band))
    static_ok = all(r.ok for r in results)
    sim = None
    if args.no_sim:
        results.append(Result('7 sim lap', None, '', 'skipped (--no-sim)'))
    elif not static_ok:
        results.append(Result('7 sim lap', None, '', 'not run: fix the rows above first'))
    else:
        print(table(results))
        print('\nstatic requirements pass; driving a lap in the planning simulator ...', flush=True)
        from autosdv_lab3 import sim_lap
        sim = sim_lap.run_lap(args.map_dir, timeout=args.timeout, keep_log=args.keep_log)
        results.append(Result('7 sim lap', sim.ok, sim.value if sim.ok else sim.reason,
                              '' if sim.ok else sim.value))
    print(table(results))
    passed = all(r.ok for r in results if not (args.no_sim and r.requirement == '7 sim lap'))
    data = {
        'passed': bool(passed),
        'sim': not args.no_sim,
        'settings': {'min_length': args.min_length, 'width': list(args.width),
                     'clearance': args.clearance, 'band': list(args.band)},
        'lanelet_sha256': mapdir.sha256(os.path.join(args.map_dir, mapdir.LANELET))
        if os.path.isfile(os.path.join(args.map_dir, mapdir.LANELET)) else None,
        'results': [asdict(r) for r in results],
    }
    if sim is not None:
        data['sim_lap'] = sim.as_dict()
    mapdir.record_step(args.map_dir, 'lanelet_check', data)
    print('\nPASS' if passed else '\nFAIL')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
