"""The clockwise loop as a route: shared by the lanelet check, its simulator lap and
`just lab3 route` on the car.

    lab3_route --map DIR [--engage] [--laps N]

The map side is pure Python: the lanelet2 file is read the way Autoware's Local
projector reads it (local_x / local_y / ele node tags), the road lanelets are
ordered into one chain by their shared end points (lanelet2's own `follows` test:
the next lanelet's bounds start on the very nodes the previous one's end on), and
a route is cut from it.

A lap is a start, a goal just behind the start, and waypoints spaced around the
loop between them, so the mission planner cannot take the short way. Two limits
of Autoware 1.5.0 cut it into legs, each an ordinary route that ends in a stop:

- the mission planner refuses a route that enters any lanelet twice ("Loop
  detected within route!"), so a leg's start and goal lie in different lanelets;
- the behavior path planner builds its path from backward_path_length (5 m)
  behind the car, along the route; a goal inside that stretch ends the path behind
  the car, the trajectory doubles back, the planning validator flags curvature and
  relative angle, and the car takes an MRM without moving. So a leg's goal stays
  more than BACKWARD_CLEAR behind its own start, round the loop.

A full lap is therefore two legs on a loop under about 2 x (BACKWARD_CLEAR + gap);
longer loops need more.

The ROS side drives Autoware's AD API (names and types as in
autoware/adapi_specs of Autoware 1.5.0): /api/routing/clear_route (ClearRoute),
/api/routing/set_route_points (SetRoutePoints), /api/routing/state (RouteState,
transient local), /api/operation_mode/change_to_autonomous (ChangeOperationMode),
/api/operation_mode/state, /api/localization/initialize (InitializeLocalization).
"""

import argparse
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from autosdv_lab3 import mapdir

START_OFFSET = 0.5   # m into the first lanelet: clear of the junction, so its lanelet is unambiguous
# m from the goal forward to the start. More than the mission planner's
# arrival_check_longitudinal_overshoot_distance (2.0 m in Autoware 1.5.0): a car at rest
# that close past the goal already counts as arrived, before it has moved
GOAL_GAP = 2.5
WAYPOINT_SPACING = 3.0
# behavior_path_planner backward_path_length (5.0 m in Autoware 1.5.0) plus a margin
BACKWARD_CLEAR = 6.0
BOUNDARY_MARGIN = 0.3  # keep waypoints this far from a lanelet junction
GOAL_MARGIN = 1.0      # and goals this far
RESAMPLE = 0.1       # m between centreline samples


class MapError(ValueError):
    """The lanelet2 file cannot be read as a loop map."""


Pose2D = Tuple[float, float, float]   # x, y, yaw


# ------------------------------------------------------------------ geometry

def polyline_length(p: np.ndarray) -> float:
    return float(np.sum(np.hypot(*np.diff(p[:, :2], axis=0).T))) if len(p) > 1 else 0.0


def resample(p: np.ndarray, n: int) -> np.ndarray:
    """n points evenly spaced by arc length along polyline p."""
    seg = np.hypot(*np.diff(p[:, :2], axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 0.0:
        return np.repeat(p[:1], n, axis=0)
    t = np.linspace(0.0, s[-1], n)
    return np.column_stack([np.interp(t, s, p[:, k]) for k in range(p.shape[1])])


def nearest_on_polyline(q: np.ndarray, p: np.ndarray) -> np.ndarray:
    """The point of polyline p (N,2+) nearest each point in q (M,2+), xy only."""
    a, b = p[:-1, :2], p[1:, :2]
    ab = b - a
    denom = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    out = np.empty((len(q), 2))
    for i in range(0, len(q), 2048):   # chunked: M x N can be large for a long loop
        qq = q[i:i + 2048, None, :2]
        t = np.clip(np.sum((qq - a) * ab, axis=2) / denom, 0.0, 1.0)
        proj = a + t[..., None] * ab
        k = np.argmin(np.linalg.norm(qq - proj, axis=2), axis=1)
        out[i:i + 2048] = proj[np.arange(len(k)), k]
    return out


def point_polyline_distance(q: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Distance (xy) from each point in q (M,2) to polyline p (N,2)."""
    return np.linalg.norm(q[:, :2] - nearest_on_polyline(q, p), axis=1)


def centreline(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """The line midway between two bounds, evenly resampled.

    Pairing the bounds by arc-length fraction drifts on a lanelet that mixes a
    straight and a curve (the inner bound is shorter only along the curve), so start
    there and pull each point to the midpoint of its nearest points on the two bounds."""
    n = max(2, int(math.ceil(max(polyline_length(left), polyline_length(right)) / RESAMPLE)) + 1)
    c = 0.5 * (resample(left[:, :2], n) + resample(right[:, :2], n))
    for _ in range(3):
        inner = 0.5 * (nearest_on_polyline(c[1:-1], left) + nearest_on_polyline(c[1:-1], right))
        c = np.vstack([c[:1], inner, c[-1:]])
        c = resample(c, n)
    return c


def signed_area(xy: np.ndarray) -> float:
    """Shoelace area of the closed polygon xy, positive counter-clockwise (z up)."""
    x, y = xy[:, 0], xy[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def point_in_polygon(x: float, y: float, poly: np.ndarray) -> bool:
    px, py = poly[:, 0], poly[:, 1]
    qx, qy = np.roll(px, -1), np.roll(py, -1)
    crosses = ((py > y) != (qy > y)) & (x < (qx - px) * (y - py) / np.where(qy != py, qy - py, 1e-12) + px)
    return bool(np.count_nonzero(crosses) % 2)


# ------------------------------------------------------------------ the map

@dataclass
class Lanelet:
    id: int
    left: np.ndarray            # (N, 3) map-frame points
    right: np.ndarray
    left_ends: Tuple[str, str]  # node ids at the bound's front and back
    right_ends: Tuple[str, str]
    tags: Dict[str, str]
    centre: np.ndarray = field(default=None)

    def __post_init__(self):
        self.centre = centreline(self.left, self.right)

    @property
    def polygon(self) -> np.ndarray:
        return np.vstack([self.left[:, :2], self.right[::-1, :2]])

    def follows(self, prev: 'Lanelet') -> bool:
        return (prev.left_ends[1] == self.left_ends[0]
                and prev.right_ends[1] == self.right_ends[0])


def read_lanelets(osm_path: str) -> Dict[int, Lanelet]:
    """Road lanelets of a Local-projector lanelet2 file, in map-frame metres."""
    try:
        root = ET.parse(osm_path).getroot()
    except (ET.ParseError, OSError) as e:
        raise MapError(f'cannot read {osm_path}: {e}') from e
    nodes = {}
    for n in root.findall('node'):
        tags = {t.get('k'): t.get('v') for t in n.findall('tag')}
        if 'local_x' not in tags or 'local_y' not in tags:
            continue
        nodes[n.get('id')] = (float(tags['local_x']), float(tags['local_y']),
                              float(tags.get('ele', 0.0) or 0.0))
    ways = {w.get('id'): [nd.get('ref') for nd in w.findall('nd')] for w in root.findall('way')}
    out = {}
    for rel in root.findall('relation'):
        tags = {t.get('k'): t.get('v') for t in rel.findall('tag')}
        if tags.get('type') != 'lanelet' or tags.get('subtype', 'road') != 'road':
            continue
        roles = {m.get('role'): m.get('ref') for m in rel.findall('member') if m.get('type') == 'way'}
        bounds = []
        for role in ('left', 'right'):
            refs = ways.get(roles.get(role))
            if not refs or len(refs) < 2:
                raise MapError(f'lanelet {rel.get("id")}: no {role} bound with two or more nodes')
            missing = [r for r in refs if r not in nodes]
            if missing:
                raise MapError(f'lanelet {rel.get("id")}: nodes {missing[:3]} carry no '
                               'local_x/local_y (projector_type: Local needs them)')
            bounds.append((np.array([nodes[r] for r in refs]), (refs[0], refs[-1])))
        lid = int(rel.get('id'))
        out[lid] = Lanelet(lid, bounds[0][0], bounds[1][0], bounds[0][1], bounds[1][1], tags)
    if not out:
        raise MapError(f'{osm_path}: no road lanelets')
    return out


def find_chain(lanelets: Dict[int, Lanelet]) -> Tuple[List[int], str]:
    """The road lanelets as one closed successor chain, from the lowest id.

    Returns (order, problem); problem is '' when the chain is closed and covers
    every road lanelet."""
    succ = {a: [b for b in lanelets if lanelets[b].follows(lanelets[a])] for a in lanelets}
    order = [min(lanelets)]
    while True:
        cur = order[-1]
        nxt = succ[cur]
        if not nxt:
            x, y = lanelets[cur].centre[-1, :2]
            return order, (f'lanelet {cur} has no successor (ends at x {x:.2f}, y {y:.2f}): '
                           'the next lanelet must start on the same two nodes')
        if len(nxt) > 1:
            return order, f'lanelet {cur} branches to {nxt}: one lane, no forks'
        if nxt[0] == order[0]:
            break
        if nxt[0] in order:
            return order, f'the chain from {order[0]} runs into a loop at {nxt[0]} without returning'
        order.append(nxt[0])
    extra = sorted(set(lanelets) - set(order))
    if extra:
        return order, f'road lanelets {extra} are not on the loop'
    return order, ''


@dataclass
class Loop:
    lanelets: Dict[int, Lanelet]
    order: List[int]
    xy: np.ndarray        # (N, 2) centreline around the loop, not repeating the first point
    lane: np.ndarray      # (N,) lanelet id of each sample
    s: np.ndarray         # (N,) arc length of each sample from the start of order[0]
    length: float
    starts: Dict[int, float]   # arc length where each lanelet begins

    @classmethod
    def from_chain(cls, lanelets, order):
        xy, lane, starts, s0 = [], [], {}, 0.0
        for lid in order:
            c = lanelets[lid].centre[:, :2]
            starts[lid] = s0
            xy.append(c[:-1])     # the next lanelet starts where this one ends
            lane.append(np.full(len(c) - 1, lid))
            s0 += polyline_length(c)
        xy, lane = np.vstack(xy), np.concatenate(lane)
        closed = np.vstack([xy, xy[:1]])
        seg = np.hypot(*np.diff(closed, axis=0).T)
        s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
        return cls(lanelets, order, xy, lane, s, float(np.sum(seg)), starts)

    def signed_area(self) -> float:
        return signed_area(self.xy)

    def wrap(self, s: float) -> float:
        return s % self.length

    def pose_at(self, s: float) -> Pose2D:
        s = self.wrap(s)
        closed = np.vstack([self.xy, self.xy[:1]])
        ss = np.append(self.s, self.length)
        i = min(int(np.searchsorted(ss, s, side='right')) - 1, len(self.xy) - 1)
        a, b = closed[i], closed[i + 1]
        t = (s - ss[i]) / max(ss[i + 1] - ss[i], 1e-9)
        p = a + t * (b - a)
        return float(p[0]), float(p[1]), math.atan2(b[1] - a[1], b[0] - a[0])

    def lane_at(self, s: float) -> int:
        s = self.wrap(s)
        i = int(np.searchsorted(self.s, s, side='right')) - 1
        return int(self.lane[max(i, 0)])

    def project(self, x: float, y: float) -> float:
        """Arc length of the centreline point nearest (x, y)."""
        closed = np.vstack([self.xy, self.xy[:1]])
        a, b = closed[:-1], closed[1:]
        ab = b - a
        t = np.clip(np.sum((np.array([x, y]) - a) * ab, axis=1)
                    / np.maximum(np.sum(ab * ab, axis=1), 1e-12), 0.0, 1.0)
        d = np.linalg.norm(a + t[:, None] * ab - np.array([x, y]), axis=1)
        i = int(np.argmin(d))
        return float(self.s[i] + t[i] * np.hypot(*ab[i]))

    def contains(self, x: float, y: float, tol: float = 0.0) -> bool:
        """(x, y) inside some lanelet, or within tol of the centreline band's edge."""
        for ll in self.lanelets.values():
            if point_in_polygon(x, y, ll.polygon):
                return True
        if tol > 0.0:
            return any(point_polyline_distance(np.array([[x, y]]), np.vstack([ll.polygon, ll.polygon[:1]]))[0] <= tol
                       for ll in self.lanelets.values())
        return False


def load_loop(map_dir: str) -> Loop:
    lanelets = read_lanelets(os.path.join(map_dir, mapdir.LANELET))
    order, problem = find_chain(lanelets)
    if problem:
        raise MapError(problem)
    return Loop.from_chain(lanelets, order)


# ------------------------------------------------------------------ the route

@dataclass
class Leg:
    goal: Pose2D
    waypoints: List[Pose2D]
    s_goal: float
    s_waypoints: List[float]


@dataclass
class LoopRoute:
    start: Pose2D
    goal: Pose2D              # the lap's goal: just behind the start
    waypoints: List[Pose2D]   # every waypoint of every leg, in driving order
    legs: List[Leg]           # routes driven one after another, each ending in a stop
    s_start: float
    length: float


def _clear_of_junctions(loop: Loop, s: float, margin: float = BOUNDARY_MARGIN) -> float:
    """Move s off a lanelet junction, so the lanelet it picks is unambiguous."""
    s = loop.wrap(s)
    for s0 in list(loop.starts.values()) + [loop.length]:
        d = s - s0
        if abs(d) < margin:
            return loop.wrap(s0 + math.copysign(margin, d if d else 1.0))
    return s


def _leg(loop: Loop, s_from: float, distance: float, spacing: float) -> Leg:
    s_goal = loop.wrap(s_from + distance)
    n = max(2, int(round(distance / spacing)))
    s_wp = []
    for k in range(1, n):
        s = _clear_of_junctions(loop, s_from + distance * k / n)
        ahead = loop.wrap(s - s_from)
        if 0.5 < ahead < distance - 0.5:
            s_wp.append(s)
    return Leg(loop.pose_at(s_goal), [loop.pose_at(s) for s in s_wp], s_goal, s_wp)


def loop_route(map_dir, start: Optional[Pose2D] = None, gap: float = GOAL_GAP,
               spacing: float = WAYPOINT_SPACING) -> LoopRoute:
    """A clockwise-as-drawn lap from `start` (default: just into the first lanelet) to
    just behind it. `map_dir` is a map folder or an already loaded Loop."""
    loop = map_dir if isinstance(map_dir, Loop) else load_loop(map_dir)
    if loop.length < 3.0 * gap:
        raise MapError(f'the loop is {loop.length:.1f} m long: too short for a lap route')
    if start is None:
        first = loop.order[0]
        s0 = loop.starts[first] + min(START_OFFSET, 0.3 * polyline_length(loop.lanelets[first].centre))
    else:
        s0 = loop.project(start[0], start[1])
    s_goal = _clear_of_junctions(loop, s0 - gap, GOAL_MARGIN)
    lap = loop.wrap(s_goal - s0) or loop.length
    # as few legs as keep every goal BACKWARD_CLEAR behind its leg's start, and no
    # leg ending in the lanelet it starts in
    n = max(1, math.ceil(lap / (loop.length - BACKWARD_CLEAR)))
    while True:
        cuts = [s0] + [_clear_of_junctions(loop, s0 + lap * k / n, GOAL_MARGIN) for k in range(1, n)] + [s_goal]
        spans = [loop.wrap(b - a) for a, b in zip(cuts, cuts[1:])]
        if all(loop.lane_at(a) != loop.lane_at(b) and loop.length - d > BACKWARD_CLEAR
               for a, b, d in zip(cuts, cuts[1:], spans)):
            break
        n += 1
        if n > 8:
            raise MapError('no lap route: the lanelets are too long for the loop '
                           '(split them so that no lanelet is longer than a third of the loop)')
    legs = [_leg(loop, a, d, spacing) for a, d in zip(cuts, spans)]
    return LoopRoute(start=loop.pose_at(s0) if start is None else tuple(start),
                     goal=legs[-1].goal,
                     waypoints=[w for leg in legs for w in leg.waypoints],
                     legs=legs, s_start=s0, length=loop.length)


# ------------------------------------------------------------------ ROS: the AD API

def _quat(yaw: float):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def _pose_msg(p: Pose2D):
    from geometry_msgs.msg import Pose
    m = Pose()
    m.position.x, m.position.y = float(p[0]), float(p[1])
    (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w) = _quat(p[2])
    return m


class AdApi:
    """The AD API calls a lap needs, on an existing rclpy node (spun by these calls)."""

    def __init__(self, node):
        from autoware_adapi_v1_msgs.msg import (LocalizationInitializationState,
                                                OperationModeState, RouteState)
        from autoware_adapi_v1_msgs.srv import (ChangeOperationMode, ClearRoute,
                                                InitializeLocalization, SetRoutePoints)
        from nav_msgs.msg import Odometry
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self.node = node
        self.RouteState, self.OperationModeState = RouteState, OperationModeState
        self.LocState = LocalizationInitializationState
        self._srv = {
            'clear': node.create_client(ClearRoute, '/api/routing/clear_route'),
            'set': node.create_client(SetRoutePoints, '/api/routing/set_route_points'),
            'auto': node.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous'),
            'init': node.create_client(InitializeLocalization, '/api/localization/initialize'),
        }
        self._types = {'clear': ClearRoute, 'set': SetRoutePoints, 'auto': ChangeOperationMode,
                       'init': InitializeLocalization}
        self.route_state = None
        self.op = None
        self.loc_state = None
        self.odom = None
        self._engaged_at, self._seen_auto = 0.0, False
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(RouteState, '/api/routing/state',
                                 lambda m: setattr(self, 'route_state', m.state), latched)
        node.create_subscription(OperationModeState, '/api/operation_mode/state',
                                 lambda m: setattr(self, 'op', m), latched)
        node.create_subscription(LocalizationInitializationState,
                                 '/api/localization/initialization_state',
                                 lambda m: setattr(self, 'loc_state', m.state), latched)
        node.create_subscription(Odometry, '/localization/kinematic_state',
                                 lambda m: setattr(self, 'odom', m), 10)

    def spin(self, seconds: float = 0.1):
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=seconds)

    def wait_for(self, predicate, timeout: float, abort=None) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.1)
            if predicate():
                return True
            if abort is not None and abort():
                return False
        return False

    def call(self, which: str, request=None, timeout: float = 10.0) -> Tuple[bool, str]:
        import rclpy
        cli = self._srv[which]
        if not cli.wait_for_service(timeout_sec=timeout):
            return False, f'{cli.srv_name} not available'
        fut = cli.call_async(request if request is not None else self._types[which].Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            return False, f'{cli.srv_name} did not answer'
        return bool(res.status.success), res.status.message or f'code {res.status.code}'

    def pose(self) -> Optional[Pose2D]:
        if self.odom is None:
            return None
        p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        return p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def initialize(self, pose: Pose2D) -> Tuple[bool, str]:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        req = self._types['init'].Request()
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.pose = _pose_msg(pose)
        req.pose = [m]
        return self.call('init', req, timeout=30.0)

    def set_route(self, leg: Leg, timeout: float = 60.0) -> Tuple[bool, str]:
        """Clear whatever route is there, then set this leg's, retrying while the
        planner is still coming up (it answers 'not ready' rather than waiting)."""
        req = self._types['set'].Request()
        req.header.frame_id = 'map'
        req.goal = _pose_msg(leg.goal)
        req.waypoints = [_pose_msg(w) for w in leg.waypoints]
        deadline = time.monotonic() + timeout
        self.wait_for(lambda: self.route_state is not None, 5.0)
        ok, msg = False, 'not tried'
        while time.monotonic() < deadline:
            if self.route_state != self.RouteState.UNSET:
                ok, msg = self.call('clear')
                if not ok:
                    msg = f'clear_route: {msg}'
                self.wait_for(lambda: self.route_state == self.RouteState.UNSET, 5.0)
            req.header.stamp = self.node.get_clock().now().to_msg()
            ok, msg = self.call('set', req, timeout=20.0)
            if ok:
                self.wait_for(lambda: self.route_state == self.RouteState.SET, 5.0)
                return True, msg
            if 'ERROR_PLANNER_FAILED' in msg or 'Failed to plan' in msg:
                return False, msg       # a route the map cannot give: retrying will not help
            self.wait_for(lambda: False, 2.0)
        return False, msg

    def autonomous(self) -> bool:
        return self.op is not None and self.op.mode == self.OperationModeState.AUTONOMOUS

    def engage(self) -> Tuple[bool, str]:
        """change_to_autonomous, even when the mode reads autonomous: on arrival the
        mode drops to STOP a moment later, and a call skipped in that moment leaves the
        next route standing still."""
        self._engaged_at, self._seen_auto = time.monotonic(), False
        ok, msg = self.call('auto')
        if not ok and self.autonomous():
            return True, 'already autonomous'
        if not ok and 'in progress' in msg:
            return True, msg    # a transition already under way; keep_engaged follows it up
        return ok, msg

    def keep_engaged(self) -> bool:
        """Re-engage when the mode fell back to STOP before arrival; True if it did."""
        if self.autonomous():
            self._seen_auto = True
            return False
        # the state lags the call: wait to see autonomous, or 5 s, before calling again
        dropped = self._seen_auto or time.monotonic() - self._engaged_at > 5.0
        if (dropped and self.op is not None and not self.arrived()
                and self.op.is_autonomous_mode_available):
            return self.engage()[0]
        return False

    def arrived(self) -> bool:
        return self.route_state == self.RouteState.ARRIVED


# ------------------------------------------------------------------ CLI (the live car)

def describe(route: LoopRoute) -> str:
    f = lambda p: f'({p[0]:.2f}, {p[1]:.2f}, {math.degrees(p[2]):.0f} deg)'  # noqa: E731
    lines = [f'loop {route.length:.1f} m; start {f(route.start)}; goal {f(route.goal)}']
    for i, leg in enumerate(route.legs):
        lines.append(f'  leg {i + 1}: {len(leg.waypoints)} waypoints, goal {f(leg.goal)}')
    return '\n'.join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Set the clockwise loop route on the running stack.')
    ap.add_argument('--map', required=True, help='the map folder (lanelet2_map.osm)')
    ap.add_argument('--engage', action='store_true', help='change to autonomous after each route')
    ap.add_argument('--laps', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=600.0, help='seconds allowed per lap')
    ap.add_argument('--dry-run', action='store_true', help='print the route; no ROS')
    args = ap.parse_args(argv)
    try:
        loop = load_loop(args.map)
    except MapError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    if args.dry_run:
        print(describe(loop_route(loop)))
        return 0

    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node('lab3_route')
    api = AdApi(node)
    try:
        if not api.wait_for(lambda: api.pose() is not None, 15.0):
            print('error: no /localization/kinematic_state: is the stack running and localized?',
                  file=sys.stderr)
            return 1
        for lap in range(1, args.laps + 1):
            route = loop_route(loop, start=api.pose())
            print(f'lap {lap}/{args.laps}: ' + describe(route))
            for i, leg in enumerate(route.legs):
                ok, msg = api.set_route(leg)
                if not ok:
                    print(f'error: the route was refused: {msg}', file=sys.stderr)
                    return 1
                print(f'  leg {i + 1}/{len(route.legs)} set'
                      + ('' if args.engage else '; engage from RViz or the web UI (or pass --engage)'))
                if args.engage:
                    if not api.wait_for(lambda: api.op is not None and api.op.is_autonomous_mode_available, 30.0):
                        print('error: autonomous mode never became available', file=sys.stderr)
                        return 1
                    ok, msg = api.engage()
                    if not ok:
                        print(f'error: engage refused: {msg}', file=sys.stderr)
                        return 1

                def arrived():
                    if args.engage:
                        api.keep_engaged()
                    return api.arrived()

                if not api.wait_for(arrived, args.timeout):
                    print(f'error: leg {i + 1} of lap {lap} did not arrive in {args.timeout:.0f} s',
                          file=sys.stderr)
                    return 1
            print(f'lap {lap}: arrived')
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
