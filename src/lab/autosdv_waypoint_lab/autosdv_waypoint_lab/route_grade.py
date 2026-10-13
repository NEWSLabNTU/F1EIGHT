"""Lab 2 team-part grading: the three route bags (straight, circle CW, circle CCW).

    lab2_grade_run --score BAG [BAG ...] [--reference BAG ...] [--course DIR]

Per bag, from the localized pose, the wheel-speed report and the command the
actuator received (vehicle_cmd_gate's output):

  Window.   t90 = first time (engaged) the measured speed reaches 90 % of the
            target; t_ramp = the last time the commanded speed is still at the
            target (>= RAMP_FRAC of it), i.e. where the stop ramp starts. The
            steady window is [t90, t_ramp]. The start transient is never in
            the RMSE.
  speed_rmse    RMS of (measured speed - target) over the steady window, m/s.
  overshoot     peak measured speed above the target between engage and t_ramp
                (0 if never above), m/s.
  stop_error    (straight) where the car came to rest (median localized pose
                0.5-3 s after the speed report reads 0), from the stop point =
                the path's last waypoint, along the path direction, localized
                pose. |d| plus the part of d beyond the stop point
                (d > 0): stopping past the line costs double. m.
  radius_rms/radius_max   (circles) distance of the localized pose from the
                circle's centre, minus R, over the steady window. m.

Gates: engaged at all, no MRM, the run finished. A straight that fails is
scored 0. A circle also needs 3 laps and radius_rms/radius_max within
RADIUS_RMS_MAX / RADIUS_MAX, and a circle that fails ANY gate (radius, laps,
MRM) is INVALID: no points, no penalty, the team re-runs it. A circle with no
valid bag by the end of the slot scores 0.

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

# ------------------------------------------------------------ constants

TARGET_SPEED = 0.8          # m/s, the lab's v_ref (also checked in lab2_check_bags)
START_FRAC = 0.90           # window opens at this fraction of the target
RAMP_FRAC = 0.98            # commanded speed at/above this fraction: still cruising
STOPPED_SPEED = 0.02        # m/s, "at rest"
SETTLE = (0.5, 3.0)         # s after coming to rest: the window the stop pose is the median of
LAPS_REQUIRED = 3
RADIUS_RMS_MAX = 0.30       # m, circle gate
RADIUS_MAX = 0.30           # m, circle gate (worst single sample)

POINTS = {
    'straight': {'speed_rmse': 10.0, 'overshoot': 4.0, 'stop_error': 6.0},
    'circle_cw': {'speed_rmse': 10.0, 'overshoot': 2.5},
    'circle_ccw': {'speed_rmse': 10.0, 'overshoot': 2.5},
}
FULL_AT = 1.5               # full points up to this multiple of the reference value
ZERO_AT = 4.0               # zero points from this multiple
FLOORS = {'speed_rmse': 0.02, 'overshoot': 0.02, 'stop_error': 0.05}      # reference floors
ABSOLUTE = {                # (full up to, zero from), when there is no reference bag
    'speed_rmse': (0.04, 0.15),
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
    window_s: float = math.nan
    speed_rmse: float = math.nan
    overshoot: float = math.nan
    stop_error: float = math.nan
    stop_signed: float = math.nan       # d: > 0 past the stop point
    radius_rms: float = math.nan
    radius_max: float = math.nan
    laps: int = 0
    laps_planned: int = 0
    finished: bool = False
    mrm: int = 0
    disengages: int = 0
    gate_failures: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)


@dataclass
class Geometry:
    """The route's reference geometry: the stop point for a straight, the
    circle for a circle."""
    route: str
    end_x: float = 0.0
    end_y: float = 0.0
    dir_x: float = 1.0
    dir_y: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    radius: float = 0.0


def geometry_from_course(course_dir: str, route: str) -> Geometry:
    xs, ys, _ = bm.load_xy(course_dir)
    geo = Geometry(route)
    if route == 'straight':
        ex, ey = xs[-1], ys[-1]
        j = len(xs) - 2
        while j > 0 and math.hypot(ex - xs[j], ey - ys[j]) < 0.3:
            j -= 1
        d = math.hypot(ex - xs[j], ey - ys[j]) or 1.0
        geo.end_x, geo.end_y = ex, ey
        geo.dir_x, geo.dir_y = (ex - xs[j]) / d, (ey - ys[j]) / d
    else:
        geo.cx, geo.cy, geo.radius, _ = bm.circle_fit(xs, ys)
    return geo


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

    win = [v for t, v in zip(vt, vv) if t90 <= t <= t_ramp]
    m.speed_rmse = g.rms([v - target for v in win])
    peak = max((v for t, v in zip(vt, vv) if m.t_engage <= t <= t_ramp), default=target)
    m.overshoot = max(0.0, peak - target)

    kt, kp = _series(msgs, g.KINEMATIC,
                     lambda x: (x.pose.pose.position.x, x.pose.pose.position.y))
    if geo is None or not kt:
        m.notes.append('no localized pose or route geometry: stop / radius not measured')
        return m
    if route == 'straight':
        t_stop = next((t for t, v in zip(vt, vv) if t > t_ramp and v < STOPPED_SPEED), None)
        if t_stop is None:
            m.notes.append('the car never came to rest; stop error taken at the end of the bag')
            t_stop = vt[-1]
        # the localized pose keeps settling for a second or two after the car
        # stops (marker fixes pull the EKF), so take the median over a settle window
        pts = [p for t, p in zip(kt, kp) if t_stop + SETTLE[0] <= t <= t_stop + SETTLE[1]]
        if not pts:
            m.notes.append('bag ends within 0.5 s of the stop: stop pose is the last sample')
            pts = [g.latest_before(kt, kp, t_stop) or kp[0]]
        p = (sorted(q[0] for q in pts)[len(pts) // 2], sorted(q[1] for q in pts)[len(pts) // 2])
        d = (p[0] - geo.end_x) * geo.dir_x + (p[1] - geo.end_y) * geo.dir_y
        m.stop_signed = d
        m.stop_error = abs(d) + max(0.0, d)
    else:
        err = [math.hypot(p[0] - geo.cx, p[1] - geo.cy) - geo.radius
               for t, p in zip(kt, kp) if m.t90 <= t <= m.t_ramp]
        m.radius_rms, m.radius_max = g.rms(err), g.max_abs(err)

    # gates
    if m.mrm:
        m.gate_failures.append(f'{m.mrm} MRM activation(s)')
    if m.disengages:
        m.gate_failures.append('left autonomous mode before the finish')
    if not m.finished:
        m.gate_failures.append('the run did not finish')
    if route != 'straight':
        if m.laps < LAPS_REQUIRED:
            m.gate_failures.append(f'{m.laps} lap(s), {LAPS_REQUIRED} required')
        if not (m.radius_rms <= RADIUS_RMS_MAX):
            m.gate_failures.append(f'radius RMS {m.radius_rms:.3f} m > {RADIUS_RMS_MAX} m')
        if not (m.radius_max <= RADIUS_MAX):
            m.gate_failures.append(f'radius max {m.radius_max:.3f} m > {RADIUS_MAX} m')
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
        m.notes.append('route unknown: no metadata in the bag; pass --route')
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
        """A circle bag that fails its gate is INVALID: re-run it, no penalty
        and no points. (A straight that fails is scored 0.)"""
        return self.route in ('circle_cw', 'circle_ccw') and bool(self.gate_failures)

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
    """(points, maximum) over the three routes; a route not submitted scores 0."""
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
            ('speed RMSE', f(m.speed_rmse, ' m/s')),
            ('start overshoot', f(m.overshoot, ' m/s'))]
    if m.route == 'straight':
        rows.append(('stop error', f'{f(m.stop_error, " m")}  (signed {f(m.stop_signed, " m")}, + = past the stop point)'))
    elif m.route:
        rows += [('radius RMS / max', f'{f(m.radius_rms, " m")} / {f(m.radius_max, " m")}'),
                 ('laps', f'{m.laps} / {m.laps_planned}')]
    rows += [('finished', str(m.finished)), ('MRM', str(m.mrm))]
    w = max(len(k) for k, _ in rows)
    out = [m.name] + [f'  {k:<{w}}  {v}' for k, v in rows]
    for k, p in s.points.items():
        lo, hi = s.lines[k]
        out.append(f'  points {k:<{w - 7}}  {p:5.2f} / {s.maximum[k]:g}   (full <= {lo:.3f}, zero >= {hi:.3f})')
    if s.invalid:
        out.append('  INVALID BAG, no circle points: ' + '; '.join(s.gate_failures))
        out.append('  The team must re-run this circle; a circle with no valid bag by the end of the slot scores 0.')
    elif s.gate_failures:
        out.append('  GATE FAILED -> 0 points: ' + '; '.join(s.gate_failures))
    out.append(f'  bag total  {s.total:.2f} / {s.max_total:g}')
    out += [f'  note: {n}' for n in m.notes]
    return '\n'.join(out)
