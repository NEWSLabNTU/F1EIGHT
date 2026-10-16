"""Lab 2 team-part grading: the graded route bag (lab2_uturn).

    lab2_grade_run --score BAG [BAG ...] [--reference BAG] [--course DIR]

The route is out along a straight leg, a right U-turn, and back along the
other leg to a stop line (see config/courses/lab2_uturn/course.yaml). Per bag,
from the localized pose, the wheel-speed report and the command the actuator
received (vehicle_cmd_gate's output):

  Window.   t90 = first time (engaged) the measured speed reaches 90 % of the
            target; t_ramp = the last time the commanded speed is still at the
            target (>= RAMP_FRAC of it), i.e. where the stop ramp starts. The
            steady window is [t90, t_ramp]. The start transient is never in
            a speed error.
  Segments. Each wheel-speed sample in the window is placed on the route by
            projecting the localized pose at that time onto the path
            (arc length s). The turn is the arc-length interval where path.csv
            turns (path.turn_spans: from the first vertex where the heading
            changes to the last); everything else is straight. Read from the
            course, so a different radius needs no change here.
  speed_rmse_straight / speed_rmse_turn
                RMS of (measured speed - target) over the window's samples on
                the straight legs / in the turn, m/s. speed_rmse is the same
                over the whole window (reported, not scored).
  overshoot     peak measured speed above the target between engage and t_ramp
                (0 if never above), m/s.
  stop_error    where the car came to rest (median localized pose 0.5-3 s
                after the speed report reads 0), from the stop point = the
                path's last waypoint, along the path direction there. |d| plus
                the part of d beyond the stop point (d > 0): stopping past the
                line costs double. m.
  path_dev_max  the largest distance of the localized pose from the path,
                engage to rest, m (a validity gate, not scored).

Validity: engaged, no MRM, no disengage before the finish, the run finished
(reached the stop), path_dev_max <= PATH_DEV_MAX, and a steady window. A bag
that fails any of these is INVALID and scores 0, with the reasons printed.

Points. The pass line is the car's own reference bag (--reference, a TA run on
the same route and car):

    m <= FULL_AT * ref            full points
    m >= ZERO_AT * ref            zero
    linear in between

with ref floored at FLOORS so a perfect reference does not make full marks
unreachable. Without a reference the fixed ABSOLUTE lines are used. Every
constant is at the top of this file.
"""

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from autosdv_waypoint_lab import bag_meta as bm
from autosdv_waypoint_lab import grading as g
from autosdv_waypoint_lab.path import Path

# ------------------------------------------------------------ constants

TARGET_SPEED = 0.8          # m/s, the lab's v_ref (also checked in lab2_check_bags)
START_FRAC = 0.90           # window opens at this fraction of the target
RAMP_FRAC = 0.98            # commanded speed at/above this fraction: still cruising
STOPPED_SPEED = 0.02        # m/s, "at rest"
SETTLE = (0.5, 3.0)         # s after coming to rest: the window the stop pose is the median of
LAPS_REQUIRED = 1
PATH_DEV_MAX = 0.30         # m, validity: localized distance from the path, worst sample

POINTS = {
    'uturn': {'speed_rmse_straight': 15.0, 'speed_rmse_turn': 10.0,
              'overshoot': 5.0, 'stop_error': 15.0},
}
FULL_AT = 1.5               # full points up to this multiple of the reference value
ZERO_AT = 4.0               # zero points from this multiple
FLOORS = {'speed_rmse': 0.02, 'speed_rmse_straight': 0.02, 'speed_rmse_turn': 0.02,
          'overshoot': 0.02, 'stop_error': 0.05}      # reference floors
ABSOLUTE = {                # (full up to, zero from), when there is no reference bag
    'speed_rmse': (0.04, 0.15),
    'speed_rmse_straight': (0.04, 0.15),
    'speed_rmse_turn': (0.04, 0.15),
    'overshoot': (0.04, 0.20),
    'stop_error': (0.10, 0.60),
}

ROUTE_TOPICS = [g.PROGRESS, g.CONTROL, g.VELOCITY, g.KINEMATIC, g.MRM, g.MODE, bm.META_TOPIC]


@dataclass
class RouteMetrics:
    name: str
    route: Optional[str] = None
    target: float = TARGET_SPEED
    engaged: bool = False
    t_engage: float = math.nan
    t90: float = math.nan
    t_ramp: float = math.nan
    t_rest: float = math.nan
    window_s: float = math.nan
    speed_rmse: float = math.nan
    speed_rmse_straight: float = math.nan
    speed_rmse_turn: float = math.nan
    samples_straight: int = 0
    samples_turn: int = 0
    overshoot: float = math.nan
    stop_error: float = math.nan
    stop_signed: float = math.nan       # d: > 0 past the stop point
    path_dev_max: float = math.nan
    path_dev_rms: float = math.nan
    laps: int = 0
    laps_planned: int = 0
    finished: bool = False
    mrm: int = 0
    disengages: int = 0
    gate_failures: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)


@dataclass
class Geometry:
    """The route's reference geometry: its path, where it turns, and the stop point."""
    route: str
    path: Optional['Path'] = None       # path.Path of path.csv's waypoints
    turns: List[Tuple[float, float]] = field(default_factory=list)   # arc-length intervals
    end_x: float = 0.0
    end_y: float = 0.0
    dir_x: float = 1.0
    dir_y: float = 0.0

    def in_turn(self, s: float) -> bool:
        return any(a <= s <= b for a, b in self.turns)

    def locate(self, x: float, y: float) -> Tuple[float, float]:
        """(arc length, distance) of a point's foot on the path."""
        assert self.path is not None
        p = self.path.project(x, y)
        return p.s, math.hypot(x - p.x, y - p.y)


def geometry_from_points(route: str, xs: Sequence[float], ys: Sequence[float]) -> Geometry:
    from autosdv_waypoint_lab.path import turn_spans
    xs, ys = list(xs), list(ys)
    geo = Geometry(route, Path(xs, ys, [0.0] * len(xs)), turn_spans(xs, ys))
    ex, ey = xs[-1], ys[-1]
    j = len(xs) - 2
    while j > 0 and math.hypot(ex - xs[j], ey - ys[j]) < 0.3:
        j -= 1
    d = math.hypot(ex - xs[j], ey - ys[j]) or 1.0
    geo.end_x, geo.end_y = ex, ey
    geo.dir_x, geo.dir_y = (ex - xs[j]) / d, (ey - ys[j]) / d
    return geo


def geometry_from_course(course_dir: str, route: str) -> Geometry:
    xs, ys, _ = bm.load_xy(course_dir)
    return geometry_from_points(route, xs, ys)


def template_dir(route: str) -> Optional[str]:
    """The packaged template course for a route."""
    name = {v: k for k, v in bm.ROUTE_BY_NAME.items()}.get(route)
    if not name:
        return None
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('autosdv_waypoint_lab'),
                         'config', 'courses', name)
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    d = os.path.join(os.path.dirname(__file__), '..', 'config', 'courses', name)
    return d if os.path.isdir(d) else None


# ------------------------------------------------------------ metrics

def _series(msgs, topic, getter):
    out = [(t, getter(m)) for t, m in msgs.get(topic, [])]
    return [t for t, _ in out], [v for _, v in out]


def route_metrics(name: str, msgs: Dict[str, list], route: str, geo: Optional[Geometry],
                  target: float = TARGET_SPEED) -> RouteMetrics:
    m = RouteMetrics(name, route=route, target=target)
    base = g.metrics_from_messages(name, msgs)
    m.laps, m.laps_planned, m.finished = base.laps, base.laps_planned, base.finished
    m.mrm, m.disengages = base.mrm, base.disengages
    t_all = [t for lst in msgs.values() for t, _ in lst]
    if not t_all:
        m.notes.append('empty bag')
        return m
    t_end = max(t_all)
    modes = [(t, int(x.mode)) for t, x in msgs.get(g.MODE, [])]
    eng = g.engaged_intervals(modes, t_end)
    if not eng.spans:
        m.notes.append('never AUTONOMOUS')
        return m
    m.engaged = True
    m.t_engage = eng.spans[0][0]

    vt, vv = _series(msgs, g.VELOCITY, lambda x: float(x.longitudinal_velocity))
    if not vt:
        m.notes.append('no velocity reports')
        return m
    ct, cv = _series(msgs, g.CONTROL, lambda x: float(x.longitudinal.velocity))

    # window opens when the car first gets up to speed
    t90 = next((t for t, v in zip(vt, vv) if t >= m.t_engage and v >= START_FRAC * target), None)
    if t90 is None:
        m.notes.append(f'speed never reached {START_FRAC * 100:.0f} % of {target} m/s')
        return m
    m.t90 = t90
    # ... and closes where the commanded speed leaves the plateau for the stop ramp
    t_ramp = None
    for t, v in zip(ct, cv):
        if t >= t90 and v >= RAMP_FRAC * target:
            t_ramp = t
    if t_ramp is None or t_ramp <= t90:
        m.notes.append('commanded speed never at the target after the start; no steady window')
        return m
    m.t_ramp = t_ramp
    m.window_s = t_ramp - t90

    win = [(t, v) for t, v in zip(vt, vv) if t90 <= t <= t_ramp]
    m.speed_rmse = g.rms([v - target for _, v in win])
    peak = max((v for t, v in zip(vt, vv) if m.t_engage <= t <= t_ramp), default=target)
    m.overshoot = max(0.0, peak - target)

    t_rest = next((t for t, v in zip(vt, vv) if t > t_ramp and v < STOPPED_SPEED), None)
    if t_rest is None:
        m.notes.append('the car never came to rest; stop error taken at the end of the bag')
        t_rest = vt[-1]
    m.t_rest = t_rest

    kt, kp = _series(msgs, g.KINEMATIC,
                     lambda x: (x.pose.pose.position.x, x.pose.pose.position.y))
    if geo is None or not kt:
        m.notes.append('no localized pose or route geometry: segments, stop and path '
                       'deviation not measured')
        m.gate_failures.append('no localized pose or route geometry')
        return m

    # speed error per segment: where the localized car was when each sample came
    straight, turn = [], []
    for t, v in win:
        p = g.latest_before(kt, kp, t)
        if p is None:
            continue
        s, _ = geo.locate(*p)
        (turn if geo.in_turn(s) else straight).append(v - target)
    m.speed_rmse_straight, m.speed_rmse_turn = g.rms(straight), g.rms(turn)
    m.samples_straight, m.samples_turn = len(straight), len(turn)
    if not straight:
        m.notes.append('no steady-window sample on the straight legs')
    if not turn:
        m.notes.append('no steady-window sample in the turn')

    # stop: the localized pose keeps settling for a second or two after the car
    # stops (marker fixes pull the EKF), so take the median over a settle window
    pts = [p for t, p in zip(kt, kp) if t_rest + SETTLE[0] <= t <= t_rest + SETTLE[1]]
    if not pts:
        m.notes.append('bag ends within 0.5 s of the stop: stop pose is the last sample')
        pts = [g.latest_before(kt, kp, t_rest) or kp[0]]
    p = (sorted(q[0] for q in pts)[len(pts) // 2], sorted(q[1] for q in pts)[len(pts) // 2])
    d = (p[0] - geo.end_x) * geo.dir_x + (p[1] - geo.end_y) * geo.dir_y
    m.stop_signed = d
    m.stop_error = abs(d) + max(0.0, d)

    # validity: on the path throughout, engage to rest
    dev = [geo.locate(*q)[1] for t, q in zip(kt, kp) if m.t_engage <= t <= t_rest]
    m.path_dev_max, m.path_dev_rms = g.max_abs(dev), g.rms(dev)

    if m.mrm:
        m.gate_failures.append(f'{m.mrm} MRM activation(s)')
    if m.disengages:
        m.gate_failures.append('left autonomous mode before the finish')
    if not m.finished:
        m.gate_failures.append('the run did not finish')
    if not (m.path_dev_max <= PATH_DEV_MAX):
        m.gate_failures.append(f'off the path: localized pose {m.path_dev_max:.3f} m from it, '
                               f'{PATH_DEV_MAX} m allowed')
    return m


def read_route_bag(path: str, route: Optional[str] = None, course_dir: Optional[str] = None):
    """(RouteMetrics, meta) of a bag; the route comes from its metadata unless given."""
    from autosdv_coach_lab.calibration.bagio import read_bag
    msgs = read_bag(path, ROUTE_TOPICS)
    meta = bm.meta_from_messages(msgs, path)
    route = route or bm.route_from_meta(meta)
    name = os.path.basename(os.path.normpath(path))
    if route not in bm.ROUTES:
        m = RouteMetrics(name)
        label = bm.route_label(meta)
        m.notes.append(f'route {label!r} is not a graded route (graded: {", ".join(bm.ROUTE_BY_NAME)})'
                       if label else 'route unknown: no metadata in the bag; pass --route')
        return m, meta
    cdir = course_dir or template_dir(route)
    geo = geometry_from_course(cdir, route) if cdir else None
    target = float(((meta or {}).get('route') or {}).get('v_ref', TARGET_SPEED))
    return route_metrics(name, msgs, route, geo, target), meta


# ------------------------------------------------------------ points

def fraction(value: float, ref: Optional[float], metric: str) -> float:
    """1.0 at or below the full-marks line, 0.0 at or above the zero line."""
    if value is None or math.isnan(value):
        return 0.0
    if ref is None or math.isnan(ref):
        full, zero = ABSOLUTE[metric]
    else:
        r = max(ref, FLOORS[metric])
        full, zero = FULL_AT * r, ZERO_AT * r
    if value <= full:
        return 1.0
    if value >= zero:
        return 0.0
    return (zero - value) / (zero - full)


@dataclass
class RouteScore:
    route: str
    points: Dict[str, float]
    maximum: Dict[str, float]
    lines: Dict[str, Tuple[float, float]]
    gate_failures: List[str]

    @property
    def invalid(self) -> bool:
        """A bag that fails a validity gate scores 0."""
        return bool(self.gate_failures)

    @property
    def total(self) -> float:
        return 0.0 if self.gate_failures else sum(self.points.values())

    @property
    def max_total(self) -> float:
        return sum(self.maximum.values())


def score_route(m: RouteMetrics, ref: Optional[RouteMetrics] = None) -> RouteScore:
    pts, lines = {}, {}
    for metric, mx in POINTS.get(m.route or '', {}).items():
        refv = getattr(ref, metric) if ref is not None else None
        if refv is None or math.isnan(refv):
            lines[metric] = ABSOLUTE[metric]
        else:
            r = max(refv, FLOORS[metric])
            lines[metric] = (FULL_AT * r, ZERO_AT * r)
        pts[metric] = mx * fraction(getattr(m, metric), refv, metric)
    fails = list(m.gate_failures)
    if not m.engaged:
        fails.append('never engaged')
    if math.isnan(m.speed_rmse):
        fails.append('no steady window')
    return RouteScore(m.route or '?', pts, dict(POINTS.get(m.route or '', {})), lines, fails)


def team_total(scores: Sequence[RouteScore]) -> Tuple[float, float]:
    """(points, maximum) over the graded routes; a route not submitted scores 0."""
    got = {s.route: s for s in scores}
    total = sum(got[r].total for r in bm.ROUTES if r in got)
    maximum = sum(sum(POINTS[r].values()) for r in bm.ROUTES)
    return total, maximum


def format_route(m: RouteMetrics, s: RouteScore) -> str:
    def f(x, unit='', nd=3):
        return 'n/a' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:.{nd}f}{unit}'
    rows = [('route', m.route or '?'),
            ('target speed', f'{m.target:.2f} m/s'),
            ('steady window', f'{f(m.t90 - m.t_engage if m.t90 == m.t90 else math.nan, " s", 1)} after engage, '
                              f'{f(m.window_s, " s", 1)} long'),
            ('speed RMSE straight', f'{f(m.speed_rmse_straight, " m/s")}  ({m.samples_straight} samples)'),
            ('speed RMSE turn', f'{f(m.speed_rmse_turn, " m/s")}  ({m.samples_turn} samples)'),
            ('speed RMSE window', f'{f(m.speed_rmse, " m/s")}  (not scored)'),
            ('start overshoot', f(m.overshoot, ' m/s')),
            ('stop error', f'{f(m.stop_error, " m")}  (signed {f(m.stop_signed, " m")}, + = past the stop point)'),
            ('path deviation max', f'{f(m.path_dev_max, " m")}  (valid <= {PATH_DEV_MAX:.2f} m; '
                                   f'RMS {f(m.path_dev_rms, " m")})'),
            ('laps', f'{m.laps} / {m.laps_planned}'),
            ('finished', str(m.finished)), ('MRM', str(m.mrm))]
    w = max(len(k) for k, _ in rows)
    out = [m.name] + [f'  {k:<{w}}  {v}' for k, v in rows]
    kw = max([w - 7] + [len(k) for k in s.points])
    for k, p in s.points.items():
        lo, hi = s.lines[k]
        out.append(f'  points {k:<{kw}}  {p:5.2f} / {s.maximum[k]:g}   (full <= {lo:.3f}, zero >= {hi:.3f})')
    if s.invalid:
        out.append('  INVALID -> 0 points: ' + '; '.join(s.gate_failures))
    out.append(f'  bag total  {s.total:.2f} / {s.max_total:g}')
    out += [f'  note: {n}' for n in m.notes]
    return '\n'.join(out)
