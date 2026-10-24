"""`just lab3 grade`: the week-2 field marks from the team's bags, on fixed lines.

    lab3_grade --map TEAM_MAP --run RUN_BAG [--checkpoints CP_BAG --marks MARKS]
               [--sheet SHEET] [--fallback-map DIR ...] [-o REPORT]

No TA reference run: every line below is a fixed number, and the checkpoint marks
are measured once from the board, whose frame every map shares.

| Item | Marks | Read from |
|---|---|---|
| Board initialization | 5 | run bag: INITIALIZED within INIT_LIMIT of launch, no RViz pose, |
|  |  | the first NDT pose where the board put the car |
| Checkpoint accuracy | 15 | checkpoint bag: the mean pose of each stop of >= STILL_MIN against its mark |
| Autonomous laps | 15 | run bag: the first four legs lab3_route set, 3.75 each |
| Staying localized | 10 | run bag over the graded legs: four checks, 2.5 each |
| Fallback map | -10 | run bag: the map hash the stack loaded |

A leg counts when the car arrives at its goal under autonomous control, with no
route set by hand in between and no takeover on the TA's sheet. The transmitter's
manual switch is hardware and leaves no trace in a bag; the TA notes it.

MARKS (YAML), measured in the board frame:

    marks:
      - {id: A, x: 2.00, y: -1.50, yaw_deg: 90}

SHEET (YAML, the TA's run sheet; every key optional):

    team: 3
    takeovers: [2]          # graded legs 1-4 that ended in a transmitter takeover
    fallback_maps: [DIR]    # the TA's fallback map folders (also --fallback-map)

Every number here is provisional until the TA dry run.
"""

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

from autosdv_lab3 import cars, mapdir

# board initialization
INIT_LIMIT = 30.0            # s from launch to INITIALIZED
INIT_POS_TOL = 0.5           # m: first NDT pose after INITIALIZED against the board's pose
INIT_YAW_TOL = math.radians(10.0)
# checkpoints
STILL_RADIUS = 0.05          # m the pose may wander and still be one stop
STILL_MIN = 3.0              # s
MATCH_RADIUS = 1.0           # m: a stop farther than this from a mark is not at it
POS_FULL, POS_ZERO = 0.10, 0.40                       # m
HEAD_FULL, HEAD_ZERO = math.radians(3.0), math.radians(15.0)
# laps
GRADED_LEGS = 4
LEG_MARKS = 3.75
ARRIVE_TOL = 2.5             # m: the mission planner arrives within 2 m of the goal
AUTO_SHARE = 0.9             # of the leg, from when the car starts moving to arrival
MOVING = 0.05                # m/s: the car has started the leg
ROUTE_MATCH = 0.3            # m: a route whose goal is this close to the leg's is the script's
ROUTE_LEAD = 5.0             # s: the route appears before the script announces the leg
# staying localized
JUMP = 0.3                   # m beyond what the speed explains, between NDT poses
NVTL_SHARE = 0.95
RESIDUAL_MAX = 0.10          # m, median scan-to-map distance
RESIDUAL_CAP = 1.0           # m: farther points are people and moved chairs, not error
SCAN_PERIOD = 1.0            # s between the scans the residual samples
SCAN_POINTS = 3000
NVTL_DEFAULT = 1.3           # ndt_indoor.param.yaml, when it cannot be read
FALLBACK_PENALTY = -10.0

ODOM = '/localization/kinematic_state'
NDT_POSE = '/localization/pose_estimator/pose_with_covariance'
NVTL = '/localization/pose_estimator/nearest_voxel_transformation_likelihood'
INIT_STATE = '/api/localization/initialization_state'
ROUTE_STATE = '/api/routing/state'
OP_MODE = '/api/operation_mode/state'
ROUTE = '/planning/mission_planning/route'
INITIALPOSE = '/initialpose'
RUN_META = '/lab3/run_meta'
EVENTS = '/lab3/route_events'
RECORD_META = '/lab3/record_meta'

INITIALIZED = 3              # LocalizationInitializationState
ARRIVED = 3                  # RouteState
AUTONOMOUS = 2               # OperationModeState


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def ramp(err: float, full: float, zero: float) -> float:
    """1 at or under `full`, 0 at or over `zero`, linear between."""
    return float(np.clip((zero - err) / (zero - full), 0.0, 1.0))


# ------------------------------------------------------------------ the bag, as arrays

@dataclass
class Run:
    """What grading reads from one bag. Times are bag receive times, in seconds,
    except the stamps the AD API and lab3_run_meta carry (both on the stack's clock)."""
    start: float = 0.0
    end: float = 0.0
    record_meta: dict = field(default_factory=dict)
    run_meta: dict = field(default_factory=dict)
    odom: np.ndarray = field(default_factory=lambda: np.zeros((0, 5)))   # t x y yaw v
    ndt: np.ndarray = field(default_factory=lambda: np.zeros((0, 4)))    # t x y yaw
    nvtl: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))   # t value
    init_states: List[Tuple[float, float, int]] = field(default_factory=list)  # t, stamp, state
    board: List[Tuple[float, float, float, float]] = field(default_factory=list)  # t x y yaw
    initialposes: List[float] = field(default_factory=list)
    op: List[Tuple[float, int]] = field(default_factory=list)
    route_state: List[Tuple[float, int]] = field(default_factory=list)
    routes: List[Tuple[float, float, float]] = field(default_factory=list)  # t, goal x, y
    events: List[Tuple[float, dict]] = field(default_factory=list)
    lidar_topic: Optional[str] = None
    tf_static: List[tuple] = field(default_factory=list)  # (parent, child, 4x4)

    def moving_from(self, t0: float, t1: float) -> Optional[float]:
        """The first time in [t0, t1] the car moves faster than MOVING."""
        o = self.odom
        if not len(o):
            return None
        sel = (o[:, 0] >= t0) & (o[:, 0] <= t1) & (np.abs(o[:, 4]) > MOVING)
        return float(o[sel, 0][0]) if sel.any() else None

    def pose_at(self, t: float) -> Optional[Tuple[float, float, float]]:
        if len(self.odom) == 0 or t < self.odom[0, 0] - 0.5 or t > self.odom[-1, 0] + 0.5:
            return None
        o = self.odom
        i = int(np.clip(np.searchsorted(o[:, 0], t), 0, len(o) - 1))
        return (float(np.interp(t, o[:, 0], o[:, 1])), float(np.interp(t, o[:, 0], o[:, 2])),
                float(o[i, 3]))


def _matrix(tr) -> np.ndarray:
    q, p = tr.rotation, tr.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [p.x, p.y, p.z]
    return m


def chain(edges, root: str, leaf: str) -> Optional[np.ndarray]:
    """T(root <- leaf) through the static tree, walking parent to child."""
    children: Dict[str, list] = {}
    for parent, child, m in edges:
        children.setdefault(parent, []).append((child, m))
    stack = [(root, np.eye(4))]
    seen = set()
    while stack:
        frame, m = stack.pop()
        if frame == leaf:
            return m
        if frame in seen:
            continue
        seen.add(frame)
        stack.extend((c, m @ cm) for c, cm in children.get(frame, []))
    return None


def _open(path):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    return reader


def read_run(path: str) -> Run:
    """Every small topic grading needs; the LiDAR is left to scans()."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = _open(path)
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    r = Run()
    for topic in cars.LIDAR_TOPIC.values():
        if topic in topics:
            r.lidar_topic = topic
    wanted = {ODOM, NDT_POSE, NVTL, INIT_STATE, ROUTE_STATE, OP_MODE, ROUTE, INITIALPOSE,
              RUN_META, EVENTS, RECORD_META, '/tf_static', cars.BOARD_POSE_RUN}
    types = {t: get_message(topics[t]) for t in wanted if t in topics}
    odom, ndt, nvtl = [], [], []
    first = None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        t = stamp * 1e-9
        first = t if first is None else first
        r.end = t
        if topic not in types:
            continue
        m = deserialize_message(raw, types[topic])
        if topic == ODOM:
            p = m.pose.pose
            odom.append((t, p.position.x, p.position.y, yaw_of(p.orientation),
                         m.twist.twist.linear.x))
        elif topic == NDT_POSE:
            p = m.pose.pose
            ndt.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        elif topic == NVTL:
            nvtl.append((t, m.data))
        elif topic == INIT_STATE:
            r.init_states.append((t, m.stamp.sec + m.stamp.nanosec * 1e-9, m.state))
        elif topic == cars.BOARD_POSE_RUN:
            p = m.pose.pose
            r.board.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        elif topic == INITIALPOSE:
            r.initialposes.append(t)
        elif topic == OP_MODE:
            r.op.append((t, m.mode))
        elif topic == ROUTE_STATE:
            r.route_state.append((t, m.state))
        elif topic == ROUTE:
            if len(m.segments):
                r.routes.append((t, m.goal_pose.position.x, m.goal_pose.position.y))
        elif topic == EVENTS:
            r.events.append((t, json.loads(m.data)))
        elif topic == RUN_META:
            r.run_meta = json.loads(m.data)
        elif topic == RECORD_META:
            r.record_meta = json.loads(m.data)
        elif topic == '/tf_static':
            for tr in m.transforms:
                r.tf_static.append((tr.header.frame_id, tr.child_frame_id,
                                    _matrix(tr.transform)))
    r.start = first or 0.0
    sidecar = os.path.join(path, 'lab3_meta.json')
    if not r.record_meta and os.path.isfile(sidecar):
        with open(sidecar) as f:
            r.record_meta = json.load(f)
    r.odom = np.array(odom, float).reshape(-1, 5)
    r.ndt = np.array(ndt, float).reshape(-1, 4)
    r.nvtl = np.array(nvtl, float).reshape(-1, 2)
    return r


def scans(path: str, topic: str, t0: float, t1: float, period: float = SCAN_PERIOD):
    """(t, frame, Nx3 points) for one LiDAR scan per `period` inside [t0, t1]."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    reader = _open(path)
    cloud_type = get_message('sensor_msgs/msg/PointCloud2')
    last = -1e18
    while reader.has_next():
        name, raw, stamp = reader.read_next()
        t = stamp * 1e-9
        if name != topic or t < t0 or t - last < period:
            continue
        if t > t1:
            break
        last = t
        m = deserialize_message(raw, cloud_type)
        pts = point_cloud2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(float)
        yield t, m.header.frame_id, xyz


# ------------------------------------------------------------------ the items

def grade_init(r: Run) -> dict:
    out = {'item': 'board initialization', 'max': 5.0, 'score': 0.0}
    init = next(((t, s) for t, s, st in r.init_states if st == INITIALIZED), None)
    launch = r.run_meta.get('launch_time')
    checks = {}
    if init is None:
        out['reason'] = 'never INITIALIZED in this bag'
        return out
    t_init, stamp = init
    before = [b for b in r.board if b[0] <= t_init + 1.0]
    if not before and t_init - r.start < 1.0:
        # the latched INITIALIZED came in with the recorder's first moment: the board
        # pose that started it, which is not latched, happened before the bag began
        out.update(score=None, reason='recording started after INITIALIZED; start the '
                   'recorder before the stack. TA decides')
        return out
    if launch is None:
        checks['time'] = {'ok': None, 'note': 'no /lab3/run_meta: launch time unknown'}
    elif not 0.0 <= stamp - launch <= 3600.0:
        checks['time'] = {'ok': None, 'note': 'the stack and lab3_run_meta disagree on the '
                          'clock (sim time against wall time?)'}
    else:
        dt = stamp - launch
        checks['time'] = {'ok': dt <= INIT_LIMIT, 'seconds': round(dt, 1), 'limit': INIT_LIMIT}
    manual = [t for t in r.initialposes if t <= t_init + 1.0]
    checks['no_rviz_pose'] = {'ok': not manual, 'count': len(manual)}
    after = r.ndt[r.ndt[:, 0] >= t_init] if len(r.ndt) else r.ndt
    if not before or not len(after):
        checks['from_board'] = {'ok': False, 'note': 'no board pose before INITIALIZED'
                                if not before else 'no NDT pose after INITIALIZED'}
    else:
        b, n = before[-1], after[0]
        d = math.hypot(n[1] - b[1], n[2] - b[2])
        dyaw = abs(float(wrap(n[3] - b[3])))
        checks['from_board'] = {'ok': d <= INIT_POS_TOL and dyaw <= INIT_YAW_TOL,
                                'distance_m': round(d, 3), 'yaw_deg': round(math.degrees(dyaw), 1)}
    out['checks'] = checks
    oks = [c['ok'] for c in checks.values()]
    if None in oks:
        out['score'] = None
        out['reason'] = 'not measurable from the bag; TA decides'
    else:
        out['score'] = 5.0 if all(oks) else 0.0
    return out


def stops(odom: np.ndarray, radius: float = STILL_RADIUS, min_s: float = STILL_MIN):
    """Each stretch of at least min_s in which the pose stays within radius of where it
    started: (t0, t1, mean x, mean y, mean yaw)."""
    out, i, n = [], 0, len(odom)
    while i < n:
        j = i
        while j + 1 < n and math.hypot(odom[j + 1, 1] - odom[i, 1],
                                       odom[j + 1, 2] - odom[i, 2]) <= radius:
            j += 1
        if odom[j, 0] - odom[i, 0] >= min_s:
            seg = odom[i:j + 1]
            yaw = math.atan2(np.sin(seg[:, 3]).mean(), np.cos(seg[:, 3]).mean())
            out.append((seg[0, 0], seg[-1, 0], seg[:, 1].mean(), seg[:, 2].mean(), yaw))
            i = j + 1
        else:
            i += 1
    return out


def grade_checkpoints(r: Optional[Run], marks: List[dict]) -> dict:
    out = {'item': 'checkpoint accuracy', 'max': 15.0, 'score': 0.0, 'marks': []}
    if r is None or not marks:
        out['score'] = None
        out['reason'] = 'no checkpoint bag or no marks file'
        return out
    found = stops(r.odom)
    total = 0.0
    for mk in marks:
        mx, my, myaw = float(mk['x']), float(mk['y']), math.radians(float(mk['yaw_deg']))
        # the first stop at the mark, not the best of several: no picking
        near = [s for s in found if math.hypot(s[2] - mx, s[3] - my) <= MATCH_RADIUS]
        row = {'id': mk.get('id')}
        if not near:
            row.update(score=0.0, note=f'no stop of {STILL_MIN:g} s within {MATCH_RADIUS:g} m')
        else:
            s = near[0]
            pos = math.hypot(s[2] - mx, s[3] - my)
            head = abs(float(wrap(s[4] - myaw)))
            sc = 0.5 * ramp(pos, POS_FULL, POS_ZERO) + 0.5 * ramp(head, HEAD_FULL, HEAD_ZERO)
            row.update(position_m=round(pos, 3), heading_deg=round(math.degrees(head), 1),
                       still_s=round(s[1] - s[0], 1), score=round(sc, 3))
        total += row['score']
        out['marks'].append(row)
    out['score'] = round(15.0 * total / len(marks), 2)
    return out


def legs(r: Run) -> List[dict]:
    """The legs lab3_route set, in time order, each with its window and outcome."""
    sets = [(t, e) for t, e in r.events if e.get('event') == 'leg_set']
    out = []
    for k, (t, e) in enumerate(sets):
        t_end = sets[k + 1][0] if k + 1 < len(sets) else r.end
        gx, gy = e['goal'][0], e['goal'][1]
        leg = {'run': e.get('run'), 'lap': e.get('lap'), 'leg': e.get('leg'),
               't_set': t, 't_end': t_end, 'goal': [gx, gy]}
        t_arr = next((ts for ts, st in r.route_state if t < ts <= t_end and st == ARRIVED), None)
        problems = []
        if t_arr is None:
            problems.append('did not arrive')
        else:
            leg['t_arrived'] = t_arr
            p = r.pose_at(t_arr)
            if p is None or math.hypot(p[0] - gx, p[1] - gy) > ARRIVE_TOL:
                problems.append('arrived away from the goal')
            t_move = r.moving_from(t, t_arr)
            share = None if t_move is None else autonomous_share(r.op, t_move, t_arr)
            leg['autonomous_share'] = None if share is None else round(share, 3)
            if share is None or share < AUTO_SHARE:
                problems.append('not under autonomous control')
        lo, hi = t - ROUTE_LEAD, leg.get('t_arrived', t_end)
        hand = [rt for rt in r.routes if lo <= rt[0] <= hi
                and math.hypot(rt[1] - gx, rt[2] - gy) > ROUTE_MATCH]
        if hand:
            problems.append('a route set by hand')
        leg['problems'] = problems
        leg['t_close'] = leg.get('t_arrived', t_end)
        out.append(leg)
    return out


def autonomous_share(op: List[Tuple[float, int]], t0: float, t1: float) -> Optional[float]:
    """Of [first autonomous moment after t0, t1], the share spent autonomous.

    legs() starts t0 when the car starts moving: between legs the mode sits in STOP
    for seconds before the script's engage takes (5 s measured in the RC simulator),
    and that wait is not the team's. A disengage once moving stops the car, and counts."""
    if t1 <= t0:
        return None
    mode, segs = None, []
    for t, m in op:
        if t <= t0:
            mode = m
            continue
        if t >= t1:
            break
        segs.append((t, m))
    edges = [(t0, mode)] + segs + [(t1, None)]
    auto = span = 0.0
    started = False
    for (ta, ma), (tb, _) in zip(edges, edges[1:]):
        started = started or ma == AUTONOMOUS
        if started:
            span += tb - ta
            auto += (tb - ta) if ma == AUTONOMOUS else 0.0
    return auto / span if span > 0 else None


def grade_laps(r: Run, takeovers: List[int]) -> Tuple[dict, List[dict]]:
    out = {'item': 'autonomous laps', 'max': GRADED_LEGS * LEG_MARKS, 'score': 0.0, 'legs': []}
    graded = legs(r)[:GRADED_LEGS]
    for k, leg in enumerate(graded, start=1):
        problems = list(leg['problems'])
        if k in takeovers:
            problems.append('takeover (TA sheet)')
        ok = not problems
        out['legs'].append({'graded_leg': k, 'lap': leg['lap'], 'leg': leg['leg'],
                            'run': leg['run'], 'ok': ok, 'problems': problems,
                            'autonomous_share': leg.get('autonomous_share')})
        out['score'] += LEG_MARKS if ok else 0.0
    if len(graded) < GRADED_LEGS:
        out['note'] = f'{len(graded)} of {GRADED_LEGS} legs set by lab3_route in this bag'
    return out, graded


def residuals(run_bag: str, r: Run, map_points: np.ndarray, t0: float, t1: float,
              rng=None) -> List[float]:
    """Median nearest-map-point distance of each sampled scan, placed by the localization."""
    from scipy.spatial import cKDTree
    rng = rng or np.random.default_rng(0)
    tree = cKDTree(map_points)
    out = []
    for t, frame, xyz in scans(run_bag, r.lidar_topic, t0, t1):
        pose = r.pose_at(t)
        base_lidar = chain(r.tf_static, 'base_link', frame)
        if pose is None or base_lidar is None or not len(xyz):
            continue
        if len(xyz) > SCAN_POINTS:
            xyz = xyz[rng.choice(len(xyz), SCAN_POINTS, replace=False)]
        # base_link on the floor in the map's plane: z from the map's own floor at 0
        c, s = math.cos(pose[2]), math.sin(pose[2])
        map_base = np.array([[c, -s, 0, pose[0]], [s, c, 0, pose[1]], [0, 0, 1, 0], [0, 0, 0, 1]])
        pts = (map_base @ base_lidar @ np.c_[xyz, np.ones(len(xyz))].T).T[:, :3]
        d, _ = tree.query(pts, distance_upper_bound=RESIDUAL_CAP)
        d = d[np.isfinite(d)]
        if len(d):
            out.append(float(np.median(d)))
    return out


def grade_localized(r: Run, graded: List[dict], nvtl_threshold: float,
                    residual_median: Optional[float]) -> dict:
    out = {'item': 'staying localized', 'max': 10.0, 'score': 0.0}
    if not graded:
        out['reason'] = 'no graded legs: nothing to measure over'
        return out
    t0, t1 = graded[0]['t_set'], graded[-1]['t_close']
    checks = {}
    reinit = [st for t, _, st in r.init_states if t0 <= t <= t1 and st != INITIALIZED]
    checks['no_reinitialization'] = {'ok': not reinit, 'count': len(reinit)}
    n = r.ndt[(r.ndt[:, 0] >= t0) & (r.ndt[:, 0] <= t1)] if len(r.ndt) else r.ndt
    jumps = 0
    worst = 0.0
    for a, b in zip(n, n[1:]):
        v = abs(float(np.interp(b[0], r.odom[:, 0], r.odom[:, 4]))) if len(r.odom) else 0.0
        excess = math.hypot(b[1] - a[1], b[2] - a[2]) - v * (b[0] - a[0])
        worst = max(worst, excess)
        jumps += excess > JUMP
    checks['no_pose_jump'] = {'ok': len(n) > 1 and jumps == 0, 'jumps': int(jumps),
                              'worst_m': round(worst, 3), 'ndt_poses': int(len(n))}
    v = r.nvtl[(r.nvtl[:, 0] >= t0) & (r.nvtl[:, 0] <= t1), 1] if len(r.nvtl) else np.zeros(0)
    share = float((v >= nvtl_threshold).mean()) if len(v) else None
    checks['nvtl'] = {'ok': share is not None and share >= NVTL_SHARE,
                      'share': None if share is None else round(share, 3),
                      'threshold': nvtl_threshold, 'samples': int(len(v))}
    checks['residual'] = {'ok': None if residual_median is None else residual_median <= RESIDUAL_MAX,
                          'median_m': None if residual_median is None else round(residual_median, 3),
                          'limit_m': RESIDUAL_MAX}
    out['checks'] = checks
    out['window_s'] = round(t1 - t0, 1)
    for c in checks.values():
        out['score'] += 2.5 if c['ok'] else 0.0
    if checks['residual']['ok'] is None:
        out['note'] = 'residual not measured (no map or no LiDAR in the bag): 2.5 withheld'
    return out


def grade_map(r: Run, team_map: str, fallback_maps: List[str]) -> dict:
    out = {'item': 'fallback map', 'max': 0.0, 'score': 0.0}
    loaded = r.run_meta.get('map_hash') or r.record_meta.get('map_hash')
    out['loaded_hash'] = loaded
    team = mapdir.folder_hash(team_map) if team_map else None
    falls = {mapdir.folder_hash(d): d for d in fallback_maps}
    if loaded is None:
        out.update(score=None, reason='the bag names no map hash; TA decides')
    elif loaded == team:
        out['map'] = 'team'
    elif loaded in falls:
        out.update(map='fallback', fallback_dir=falls[loaded], score=FALLBACK_PENALTY)
    else:
        out.update(map='unknown', score=None,
                   reason='neither the handed-in map nor a fallback map: changed after the freeze?')
    return out


def nvtl_from_config() -> float:
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('autosdv_lab3'), 'config',
                            'ndt_indoor.param.yaml')
        with open(path) as f:
            doc = yaml.safe_load(f)
    except Exception:
        return NVTL_DEFAULT

    def find(d):
        if isinstance(d, dict):
            for k, v in d.items():
                if k == 'converged_param_nearest_voxel_transformation_likelihood':
                    return float(v)
                got = find(v)
                if got is not None:
                    return got
        return None
    return find(doc) or NVTL_DEFAULT


# ------------------------------------------------------------------ CLI

def grade(args) -> dict:
    sheet = {}
    if args.sheet:
        with open(args.sheet) as f:
            sheet = yaml.safe_load(f) or {}
    marks = []
    if args.marks:
        with open(args.marks) as f:
            marks = (yaml.safe_load(f) or {}).get('marks', [])
    fallback_maps = list(args.fallback_map or []) + [os.path.expanduser(d) for d in
                                                     sheet.get('fallback_maps', [])]
    run = read_run(args.run)
    cp = read_run(args.checkpoints) if args.checkpoints else None

    items = [grade_init(run), grade_checkpoints(cp, marks)]
    laps, graded = grade_laps(run, [int(k) for k in sheet.get('takeovers', [])])
    items.append(laps)
    mapped = grade_map(run, args.map, fallback_maps)

    resid = None
    map_dir = mapped.get('fallback_dir', args.map)
    pcd = os.path.join(map_dir, mapdir.PCD) if map_dir else None
    if graded and run.lidar_topic and pcd and os.path.isfile(pcd) and not args.no_residual:
        from reflective_pose_core.pointcloud_io import read_cloud
        per_scan = residuals(args.run, run, np.asarray(read_cloud(pcd).points, float),
                             graded[0]['t_set'], graded[-1]['t_close'])
        resid = float(np.median(per_scan)) if per_scan else None
    threshold = args.nvtl_threshold if args.nvtl_threshold is not None else nvtl_from_config()
    items.append(grade_localized(run, graded, threshold, resid))
    items.append(mapped)

    scores = [i['score'] for i in items]
    total = None if None in scores else round(sum(scores), 2)
    return {'lab': 'lab3', 'part': 'week 2 field', 'team': sheet.get('team'),
            'run_bag': os.path.abspath(args.run),
            'checkpoint_bag': os.path.abspath(args.checkpoints) if args.checkpoints else None,
            'items': items, 'total': total, 'max': 45.0,
            'provisional': 'every line is provisional until the TA dry run'}


def print_report(rep: dict):
    for i in rep['items']:
        sc = '  —  ' if i['score'] is None else f'{i["score"]:5.2f}'
        mx = f'/ {i["max"]:g}' if i['max'] else '(penalty)'
        print(f'{i["item"]:<22} {sc} {mx}' + (f'   {i["reason"]}' if i.get('reason') else ''))
        for leg in i.get('legs', []):
            print(f'    leg {leg["graded_leg"]} (lap {leg["lap"]} leg {leg["leg"]}): '
                  + ('ok' if leg['ok'] else ', '.join(leg['problems'])))
        for mk in i.get('marks', []):
            print(f'    mark {mk["id"]}: ' + ', '.join(f'{k} {v}' for k, v in mk.items() if k != 'id'))
        for name, c in (i.get('checks') or {}).items():
            state = '—' if c['ok'] is None else ('ok' if c['ok'] else 'FAIL')
            rest = ', '.join(f'{k} {v}' for k, v in c.items() if k != 'ok')
            print(f'    {name}: {state}  {rest}')
    tot = '—' if rep['total'] is None else f'{rep["total"]:.2f}'
    print(f'{"total":<22} {tot} / {rep["max"]:g}   ({rep["provisional"]})')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='lab3_grade', description='Lab 3 week-2 field marks.')
    ap.add_argument('--map', required=True, help="the team's handed-in map folder")
    ap.add_argument('--run', required=True, help='the laps bag (just lab3 record --kind run)')
    ap.add_argument('--checkpoints', help='the checkpoint bag')
    ap.add_argument('--marks', help='the checkpoint marks, YAML in the board frame')
    ap.add_argument('--sheet', help="the TA's run sheet, YAML")
    ap.add_argument('--fallback-map', action='append', help="a TA fallback map folder")
    ap.add_argument('--nvtl-threshold', type=float,
                    help='default: ndt_indoor.param.yaml, the value the stack ran with')
    ap.add_argument('--no-residual', action='store_true', help='skip the scan-to-map residual')
    ap.add_argument('-o', '--output', help='write the report as YAML')
    args = ap.parse_args(argv)
    rep = grade(args)
    print_report(rep)
    if args.output:
        with open(args.output, 'w') as f:
            yaml.safe_dump(rep, f, sort_keys=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
