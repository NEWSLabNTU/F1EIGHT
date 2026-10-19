"""lab2_audit: one bag, the whole verdict, and a picture of the run.

    lab2_audit BAG [--car A] [--team 3] [--reference REFBAG] [--course DIR]
               [--bench] [--show] [--out PNG]

Runs the bag checker (check_bags) and the route grade (route_grade, against
the reference bag if one is given, else the fixed lines) and prints:

    VALID / INVALID, with the reasons
    each scored metric: value, the full-marks and zero lines, the points
    the violations: every check-bags rejection, every validity gate, every
    metric below full marks, and every stretch off the path

and writes BAG/audit.png (three panels: speed vs time, the run on the floor
plan, path deviation vs time) and BAG/audit.json (the same verdict, for
scripts). Nothing here scores: the numbers are route_grade's, the acceptance
is check_bags'.

`--show` opens the PNG on $DISPLAY when an X server answers there (on the car:
`export DISPLAY=:1`, the desktop seen over RDP); otherwise it prints the path.
A bench bag is checked in bench mode automatically and marked as such: it can
pass the audit, but check_bags rejects it as a submission.
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from autosdv_waypoint_lab import bag_meta as bm
from autosdv_waypoint_lab import grading as g
from autosdv_waypoint_lab import route_grade as rg

AUDIT_TOPICS = sorted(set(rg.ROUTE_TOPICS + [bm.MARKER_FIXES, bm.TRUE_POSE]))
PNG = 'audit.png'
JSON = 'audit.json'

# Localization against the bench's true pose (a bench bag only; `just lab2
# integration-test` fails outside these). The scale is the grade's: the path
# validity line is 0.30 m, so a pose that is itself 0.2 m off at the 95th
# percentile leaves no margin for tracking. A single fix is noisier than the
# EKF that averages it: the localizer declares 0.05 m^2 per axis (sigma 0.22 m),
# whose 2-D p95 is 0.22 x 2.45 = 0.54 m, hence 0.50 m for the fixes. The
# rate is the preflight's warn line; the gap stays well inside
# localization_monitor's 5 s error_age (lab2_uturn's design gap is 1.6 s).
LOC_LIMITS = {
    'fix_rate_min': 2.0,         # accepted fixes per second, engage to rest
    'fix_err_p95_max': 0.50,     # m, a fix's position vs the true pose at its stamp
    'ekf_err_p95_max': 0.20,     # m, kinematic_state vs the true pose
    'gap_max_s': 3.0,            # s, the longest stretch without a fix
}

LABELS = {'speed_rmse_straight': ('speed RMSE straight', 'm/s'),
          'speed_rmse_turn': ('speed RMSE U-turn', 'm/s'),
          'overshoot': ('start overshoot', 'm/s'),
          'stop_error': ('stop error', 'm')}


# ------------------------------------------------------------ pure helpers

def spans_where(ts: Sequence[float], flags: Sequence[bool]) -> List[Tuple[float, float]]:
    """[(t0, t1)] over which consecutive samples are flagged."""
    out, start, prev = [], None, None
    for t, f in zip(ts, flags):
        if f and start is None:
            start = t
        elif not f and start is not None:
            out.append((start, prev))
            start = None
        prev = t
    if start is not None:
        out.append((start, prev))
    return out


def transitions(msgs: Dict[str, list], t_finish: Optional[float]):
    """(MRM onsets, disengages before the finish, disengages after it), bag times."""
    mrm, prev = [], g.MRM_NORMAL
    for t, s in msgs.get(g.MRM, []):
        st = int(s.state)
        if st != g.MRM_NORMAL and prev == g.MRM_NORMAL:
            mrm.append(t)
        prev = st
    before, after, was = [], [], False
    for t, m in msgs.get(g.MODE, []):
        now = int(m.mode) == g.AUTONOMOUS
        if was and not now:
            (after if t_finish is not None and t >= t_finish else before).append(t)
        was = now
    return mrm, before, after


@dataclass
class Series:
    """Everything the plot draws, in bag time."""
    vel_t: List[float] = field(default_factory=list)
    vel_v: List[float] = field(default_factory=list)
    cmd_t: List[float] = field(default_factory=list)
    cmd_v: List[float] = field(default_factory=list)
    kin_t: List[float] = field(default_factory=list)
    kin_xy: List[Tuple[float, float]] = field(default_factory=list)
    kin_s: List[float] = field(default_factory=list)
    kin_dev: List[float] = field(default_factory=list)
    fix_xy: List[Tuple[float, float]] = field(default_factory=list)
    true_xy: List[Tuple[float, float]] = field(default_factory=list)
    mrm: List[float] = field(default_factory=list)
    disengage: List[float] = field(default_factory=list)
    disengage_after: List[float] = field(default_factory=list)
    t_finish: Optional[float] = None


def series_from_messages(msgs: Dict[str, list], geo: Optional[rg.Geometry]) -> Series:
    s = Series()
    for t, x in msgs.get(g.VELOCITY, []):
        if rg.finite(float(x.longitudinal_velocity)):
            s.vel_t.append(t)
            s.vel_v.append(float(x.longitudinal_velocity))
    for t, x in msgs.get(g.CONTROL, []):
        s.cmd_t.append(t)
        s.cmd_v.append(float(x.longitudinal.velocity))
    for t, x in msgs.get(g.KINEMATIC, []):
        p = x.pose.pose.position
        if not rg.finite(p.x, p.y):
            continue
        s.kin_t.append(t)
        s.kin_xy.append((p.x, p.y))
        if geo is not None and geo.path is not None:
            a, d = geo.locate(p.x, p.y)
            s.kin_s.append(a)
            s.kin_dev.append(d)
    for _, x in msgs.get(bm.MARKER_FIXES, []):
        p = x.pose.pose.position
        if rg.finite(p.x, p.y):
            s.fix_xy.append((p.x, p.y))
    for _, x in msgs.get(bm.TRUE_POSE, []):
        p = x.pose.pose.position
        s.true_xy.append((p.x, p.y))
    s.t_finish = next((t for t, p in msgs.get(g.PROGRESS, []) if p.finished), None)
    s.mrm, s.disengage, s.disengage_after = transitions(msgs, s.t_finish)
    return s


def off_path_spans(s: Series, t0: float, t1: float, limit: float = rg.PATH_DEV_MAX):
    """[(t_start, t_end, worst)] where the localized pose is over `limit` from the path
    between t0 and t1 (the grade's window: engage to rest)."""
    if not s.kin_dev:
        return []
    out = []
    ts = [t for t in s.kin_t if t0 <= t <= t1]
    ds = [d for t, d in zip(s.kin_t, s.kin_dev) if t0 <= t <= t1]
    for a, b in spans_where(ts, [d > limit for d in ds]):
        out.append((a, b, max(d for t, d in zip(ts, ds) if a <= t <= b)))
    return out


def segment_spans(s: Series, geo: Optional[rg.Geometry], t0: float, t1: float):
    """[(t_start, t_end, 'straight' | 'U-turn')] from where the localized car was on the route."""
    if geo is None or not s.kin_s:
        return []
    ts, kinds = [], []
    for t, a in zip(s.kin_t, s.kin_s):
        if t0 <= t <= t1:
            ts.append(t)
            kinds.append(geo.in_turn(a))
    out = []
    for flag, name in ((True, 'U-turn'), (False, 'straight')):
        out += [(a, b, name) for a, b in spans_where(ts, [k == flag for k in kinds])]
    return sorted(out)


def speed_violations(m: rg.RouteMetrics, sc: rg.RouteScore, s: Series, geo):
    """Steady-window samples whose speed error is over its segment's full-marks line:
    [(t, v)]. These are the samples that cost points."""
    if math.isnan(m.t90) or math.isnan(m.t_ramp) or geo is None or not s.kin_s:
        return []
    lim_s = sc.lines.get('speed_rmse_straight', (math.inf,))[0]
    lim_t = sc.lines.get('speed_rmse_turn', (math.inf,))[0]
    out = []
    for t, v in zip(s.vel_t, s.vel_v):
        if not m.t90 <= t <= m.t_ramp:
            continue
        a = g.latest_before(s.kin_t, s.kin_s, t)
        if a is None:
            continue
        lim = lim_t if geo.in_turn(a) else lim_s
        if abs(v - m.target) > lim:
            out.append((t, v))
    return out


def _stamp(m) -> float:
    st = m.header.stamp
    return st.sec + st.nanosec * 1e-9


def _pct(xs, q):
    if not xs:
        return math.nan
    ys = sorted(xs)
    return ys[min(len(ys) - 1, int(round(q * (len(ys) - 1))))]


def localization_quality(msgs: Dict[str, list], t0: Optional[float] = None,
                         t1: Optional[float] = None, limits=None) -> Optional[dict]:
    """Fixes and the EKF against the bench plant's true pose, matched by header stamp,
    between bag times t0 and t1. None without a true pose (a bag from the car)."""
    limits = limits or LOC_LIMITS
    truth = msgs.get(bm.TRUE_POSE, [])
    if not truth:
        return None
    t0 = truth[0][0] if t0 is None or math.isnan(t0) else t0
    t1 = truth[-1][0] if t1 is None or math.isnan(t1) else t1
    tr = sorted((_stamp(m), m.pose.pose.position.x, m.pose.pose.position.y) for _, m in truth)
    tt = [a for a, _, _ in tr]

    def true_at(st):
        import bisect
        i = bisect.bisect_left(tt, st)
        if i <= 0 or i >= len(tt):
            return None
        a, b = tr[i - 1], tr[i]
        w = (st - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        return a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2])

    def errors(topic):
        out, times = [], []
        for t, m in msgs.get(topic, []):
            if not t0 <= t <= t1:
                continue
            q = true_at(_stamp(m))
            if q is None:
                continue
            p = m.pose.pose.position
            if not rg.finite(p.x, p.y):
                continue
            out.append(math.hypot(p.x - q[0], p.y - q[1]))
            times.append(t)
        return out, times

    fix_err, fix_t = errors(bm.MARKER_FIXES)
    ekf_err, _ = errors(g.KINEMATIC)
    span = max(t1 - t0, 1e-6)
    edges = [t0] + fix_t + [t1]
    gap = max(b - a for a, b in zip(edges, edges[1:]))
    q = {'fixes': len(fix_t), 'fix_rate': len(fix_t) / span, 'seconds': span,
         'fix_err_median': _pct(fix_err, 0.5), 'fix_err_p95': _pct(fix_err, 0.95),
         'ekf_err_median': _pct(ekf_err, 0.5), 'ekf_err_p95': _pct(ekf_err, 0.95),
         'ekf_err_max': max(ekf_err) if ekf_err else math.nan, 'gap_max_s': gap,
         'limits': dict(limits)}
    fails = []
    if not fix_t:
        fails.append('no marker fix at all')
    if q['fix_rate'] < limits['fix_rate_min']:
        fails.append(f'{q["fix_rate"]:.1f} fixes/s, at least {limits["fix_rate_min"]:g}')
    if not q['fix_err_p95'] <= limits['fix_err_p95_max']:
        fails.append(f'fix error p95 {_f(q["fix_err_p95"])} m, limit {limits["fix_err_p95_max"]:g}')
    if not q['ekf_err_p95'] <= limits['ekf_err_p95_max']:
        fails.append(f'EKF error p95 {_f(q["ekf_err_p95"])} m, limit {limits["ekf_err_p95_max"]:g}')
    if gap > limits['gap_max_s']:
        fails.append(f'{gap:.1f} s without a fix, limit {limits["gap_max_s"]:g} s')
    q['failures'] = fails
    q['ok'] = not fails
    q['summary'] = (f'{q["fixes"]} fixes ({q["fix_rate"]:.1f}/s), fix error median '
                    f'{_f(q["fix_err_median"])} / p95 {_f(q["fix_err_p95"])} m, EKF error median '
                    f'{_f(q["ekf_err_median"])} / p95 {_f(q["ekf_err_p95"])} m, longest gap '
                    f'{gap:.1f} s')
    return q


def _f(x, nd=3):
    return 'n/a' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:.{nd}f}'


@dataclass
class Audit:
    bag: str
    car: str = ''
    team: str = ''
    bench: bool = False
    check_ok: bool = False
    check_reasons: List[str] = field(default_factory=list)
    check_warnings: List[str] = field(default_factory=list)
    grade_valid: bool = False
    gate_failures: List[str] = field(default_factory=list)
    total: float = 0.0
    raw_total: float = 0.0       # the metrics' points before the validity gates
    max_total: float = 0.0
    reference: Optional[str] = None
    rows: List[dict] = field(default_factory=list)
    violations: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)
    png: Optional[str] = None
    localization: Optional[dict] = None
    graded: bool = True
    route: Optional[str] = None      # an ungraded route's name

    @property
    def valid(self) -> bool:
        """Graded: check-bags and the grade both pass. Not graded (a test route):
        check-bags passes; there is no grade."""
        return self.check_ok and (self.grade_valid or not self.graded)

    @property
    def verdict(self) -> str:
        if not self.graded:
            return 'METRICS ONLY' if self.valid else 'INVALID'
        return 'VALID' if self.valid else 'INVALID'


def build_audit(bag: str, verdict, m: rg.RouteMetrics, sc: Optional[rg.RouteScore], s: Series,
                meta: Optional[dict], reference: Optional[str] = None) -> Audit:
    """The verdict and the violation list, from check_bags' Verdict and route_grade's numbers."""
    meta = meta or {}
    a = Audit(os.path.basename(os.path.normpath(bag)), car=str(meta.get('car_id') or ''),
              team=str(meta.get('team_id') or ''), bench=bool(meta.get('bench')),
              reference=reference)
    a.check_ok, a.check_reasons, a.check_warnings = verdict.ok, list(verdict.reasons), list(verdict.warnings)
    a.notes = list(m.notes)
    if sc is None and m.ungraded:
        a.graded, a.route = False, m.ungraded
        a.violations += [f'check-bags: {r}' for r in verdict.reasons]
        _run_violations(a, m, s)
        return a
    # the cause first: the grade's gates, then what check-bags adds, then the points lost
    # and the moments on the timeline
    if sc is None:
        a.gate_failures = list(m.notes) or ['not a graded route']
        a.violations += [f'grade: {r}' for r in a.gate_failures]
        a.violations += [f'check-bags: {r}' for r in verdict.reasons]
        return a
    a.gate_failures = list(sc.gate_failures)
    a.grade_valid = not sc.invalid
    a.total, a.max_total = sc.total, sc.max_total
    a.raw_total = sum(sc.points.values())
    for r in sc.gate_failures:
        why = ''
        if r in ('no steady window', 'never engaged') and m.notes:
            why = f' ({m.notes[0]})'      # e.g. the speed never reached 90 % of the target
        a.violations.append(f'grade: {r}{why}')
    grade_text = ' '.join(sc.gate_failures)
    for r in verdict.reasons:
        if r.split(':')[0] not in grade_text and r not in grade_text:
            a.violations.append(f'check-bags: {r}')
    for k, pts in sc.points.items():
        full, zero = sc.lines[k]
        label, unit = LABELS.get(k, (k, ''))
        val = getattr(m, k)
        a.rows.append({'metric': k, 'label': label, 'value': val, 'unit': unit, 'full': full,
                       'zero': zero, 'points': pts, 'max': sc.maximum[k]})
        if pts < sc.maximum[k] - 1e-9:
            if val is None or (isinstance(val, float) and math.isnan(val)):
                a.violations.append(f'{label} not measured (0 / {sc.maximum[k]:g} points)')
            else:
                a.violations.append(f'{label} {_f(val)} {unit} over the full-marks line {full:.3f} '
                                    f'({pts:.2f} / {sc.maximum[k]:g} points)')
    _run_violations(a, m, s)
    return a


def _run_violations(a: Audit, m: rg.RouteMetrics, s: Series) -> None:
    """The moments on the timeline: off the path, MRM, disengage."""
    if not math.isnan(m.t_engage):
        t1 = m.t_rest if not math.isnan(m.t_rest) else (s.kin_t[-1] if s.kin_t else m.t_engage)
        for t0_, t1_, worst in off_path_spans(s, m.t_engage, t1):
            a.violations.append(f'off the path (> {rg.PATH_DEV_MAX:.2f} m) at '
                                f'{t0_ - m.t_engage:.1f}-{t1_ - m.t_engage:.1f} s after engage, '
                                f'worst {worst:.2f} m')
        for t in s.mrm:
            a.violations.append(f'MRM at {t - m.t_engage:.1f} s after engage')
        for t in s.disengage:
            a.violations.append(f'disengaged at {t - m.t_engage:.1f} s after engage, before the finish')


def format_audit(a: Audit, m: Optional[rg.RouteMetrics] = None) -> str:
    who = f'car {a.car or "?"}  team {a.team or "?"}' + ('   BENCH RUN (simulated car)' if a.bench else '')
    out = [f'AUDIT  {a.bag}   {who}']
    check = 'ACCEPT' if a.check_ok else 'REJECT'
    grade = 'valid' if a.grade_valid else 'INVALID'
    if not a.graded:
        out.append(f'  {a.verdict}   check-bags {check}{" (bench mode)" if a.bench else ""}; '
                   f'route {a.route}: {rg.UNGRADED}')
        if m is not None:
            out.append(f'  {"speed RMSE (steady)":<22}{_f(m.speed_rmse) + " m/s":>12}   '
                       f'window {_f(m.window_s, 1)} s')
            out.append(f'  {"overshoot":<22}{_f(m.overshoot) + " m/s":>12}')
            out.append(f'  {"peak speed":<22}{_f(m.peak_speed) + " m/s":>12}   target {m.target:g} m/s')
            out.append(f'  {"stop error":<22}{_f(m.stop_error) + " m":>12}')
            out.append(f'  {"path deviation max":<22}{_f(m.path_dev_max) + " m":>12}'
                       f'{"valid <= " + format(rg.PATH_DEV_MAX, ".2f"):>20}')
            out.append(f'  {"localization jump":<22}{_f(m.loc_jump) + " m":>12}'
                       f'{"valid <= " + format(rg.LOC_JUMP_MAX, ".2f"):>20}')
            out.append(f'  {"stop (signed)":<22}{_f(m.stop_signed) + " m":>12}   + = past the stop line')
        return '\n'.join(out + _tail(a))
    out.append(f'  {a.verdict}   check-bags {check}{" (bench mode)" if a.bench else ""}, grade {grade}'
               + ('' if not a.bench else '  -- a bench bag is never a submission'))
    lines = f'reference {a.reference}' if a.reference else 'fixed lines (no reference bag)'
    out.append(f'  {"metric":<22}{"value":>12}{"full <=":>10}{"zero >=":>10}{"points":>14}   ({lines})')
    for r in a.rows:
        out.append(f'  {r["label"]:<22}{_f(r["value"]) + " " + r["unit"]:>12}{r["full"]:>10.3f}'
                   f'{r["zero"]:>10.3f}{r["points"]:>9.2f} / {r["max"]:g}')
    if m is not None:
        out.append(f'  {"path deviation max":<22}{_f(m.path_dev_max) + " m":>12}'
                   f'{"valid <= " + format(rg.PATH_DEV_MAX, ".2f"):>20}')
        out.append(f'  {"localization jump":<22}{_f(m.loc_jump) + " m":>12}'
                   f'{"valid <= " + format(rg.LOC_JUMP_MAX, ".2f"):>20}')
        out.append(f'  {"stop (signed)":<22}{_f(m.stop_signed) + " m":>12}   + = past the stop line')
    if a.valid:
        out.append(f'  total  {a.total:.2f} / {a.max_total:g}')
    else:
        out.append(f'  total  0 / {a.max_total:g}   (INVALID; the metrics alone: {a.raw_total:.2f})')
    return '\n'.join(out + _tail(a))


def _tail(a: Audit) -> List[str]:
    out = []
    if a.violations:
        out.append('  violations:')
        out += [f'    - {v}' for v in a.violations]
    else:
        out.append('  violations: none')
    if a.localization:
        loc = a.localization
        out.append(f'  localization vs true pose (bench): {"ok" if loc["ok"] else "BROKEN"}')
        out.append(f'    {loc["summary"]}')
        out += [f'    - {x}' for x in loc['failures']]
    out += [f'  warn: {w}' for w in a.check_warnings]
    out += [f'  note: {n}' for n in a.notes]
    if a.png:
        out.append(f'  plot: {a.png}')
    return out


# ------------------------------------------------------------ the picture

C_MEAS = '#1f5fbf'      # measured speed / localized trajectory
C_TARGET = '#222222'
C_CMD = '#8a8a8a'
C_BAD = '#d62728'       # violations: off the path, MRM, disengage, costly samples
C_STEADY = '#e8eef9'
C_TURN = '#f2c14e'
C_STRAIGHT = '#9fc5a8'
C_ENGAGE = '#2a9d3a'


def _course_extras(course_dir: Optional[str]):
    """(boards [(id, x, y, yaw)], start pose (x, y, yaw)) of a course directory, or empties."""
    if not course_dir or not os.path.isfile(os.path.join(course_dir, 'course.yaml')):
        return [], None
    try:
        from autosdv_waypoint_lab.course import load_course
        c = load_course(course_dir)
        boards = [(mk.id, mk.x, mk.y, mk.yaw) for mk in c.markers]
        sp = c.start_pose
        return boards, (sp.x, sp.y, sp.yaw)
    except Exception:
        return [], None


def plot_audit(png: str, a: Audit, m: rg.RouteMetrics, sc: Optional[rg.RouteScore], s: Series,
               geo: Optional[rg.Geometry], course_dir: Optional[str] = None) -> str:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    t0 = m.t_engage if not math.isnan(m.t_engage) else (s.vel_t[0] if s.vel_t else 0.0)
    t_rest = m.t_rest if not math.isnan(m.t_rest) else (s.vel_t[-1] if s.vel_t else t0)

    def rel(ts):
        return [t - t0 for t in ts]

    fig = plt.figure(figsize=(14, 15), dpi=100)
    gs = fig.add_gridspec(3, 1, height_ratios=[1.15, 1.35, 0.8], hspace=0.32)
    ax1, ax2, ax3 = fig.add_subplot(gs[0]), fig.add_subplot(gs[1]), fig.add_subplot(gs[2])
    mode = '   BENCH (simulated car)' if a.bench else ''
    if not a.graded:
        why = a.violations[0] if a.violations else ''
        head = (f'{a.verdict}: {a.route}, {rg.UNGRADED}   speed RMSE {_f(m.speed_rmse)} m/s, '
                f'stop error {_f(m.stop_error)} m' + (f'\n{why}' if why else ''))
    elif a.valid:
        head = f'VALID   {a.total:.2f} / {a.max_total:g} points'
    else:
        why = a.violations[0] if a.violations else 'invalid'
        more = f'  (+{len(a.violations) - 1} more)' if len(a.violations) > 1 else ''
        head = f'INVALID: 0 points (the metrics alone {a.raw_total:.2f} / {a.max_total:g})\n{why}{more}'
    fig.suptitle(f'{a.bag}   car {a.car or "?"}  team {a.team or "?"}{mode}\n{head}',
                 fontsize=14, fontweight='bold', color=C_TARGET if a.valid else C_BAD, y=0.95)

    # ---- panel 1: speed vs time
    x_lo = -2.0
    x_hi = (t_rest - t0) + 4.0 if s.vel_t else 10.0
    if s.vel_t:
        x_hi = min(x_hi, s.vel_t[-1] - t0)
    if not math.isnan(m.t90) and not math.isnan(m.t_ramp):
        ax1.axvspan(m.t90 - t0, m.t_ramp - t0, color=C_STEADY, zorder=0, label='steady window (scored)')
    for a_, b_, kind in segment_spans(s, geo, t0, t_rest):
        ax1.axvspan(a_ - t0, b_ - t0, ymin=0.94, ymax=1.0, color=C_TURN if kind == 'U-turn' else C_STRAIGHT,
                    zorder=1, lw=0)
        if b_ - a_ > 2.0:
            ax1.text((a_ + b_) / 2 - t0, 0.97, kind, transform=ax1.get_xaxis_transform(), ha='center',
                     va='center', fontsize=9, color=C_TARGET)
    if s.cmd_t:
        ax1.plot(rel(s.cmd_t), s.cmd_v, color=C_CMD, lw=1.2, ls='--', label='commanded (gate)')
    ax1.plot(rel(s.vel_t), s.vel_v, color=C_MEAS, lw=2, label='measured (wheel speed)')
    ax1.axhline(m.target, color=C_TARGET, lw=1, ls=':', label=f'target {m.target:.2f} m/s')
    if sc is not None:
        bad = speed_violations(m, sc, s, geo)
        if bad:
            ax1.scatter([t - t0 for t, _ in bad], [v for _, v in bad], s=12, color=C_BAD, zorder=5,
                        label='speed error over the full-marks line')
    ax1.axvline(0.0, color=C_ENGAGE, lw=1.5)
    ax1.text(0.0, 1.01, ' engage', transform=ax1.get_xaxis_transform(), color=C_ENGAGE, fontsize=9)
    if not math.isnan(m.t_ramp):
        ax1.axvline(m.t_ramp - t0, color=C_CMD, lw=1.2, ls='-.')
        ax1.text(m.t_ramp - t0, 1.01, ' stop ramp', transform=ax1.get_xaxis_transform(),
                 color=C_CMD, fontsize=9)
        win = [(t, v) for t, v in zip(s.vel_t, s.vel_v) if m.t_engage <= t <= m.t_ramp]
        if win:
            tp, vp = max(win, key=lambda q: q[1])
            ax1.plot([tp - t0], [vp], marker='v', ms=9, color=C_BAD if m.overshoot > 0 else C_MEAS)
            ax1.annotate(f'peak {vp:.2f} m/s (overshoot +{m.overshoot:.3f})', (tp - t0, vp),
                         xytext=(8, 10), textcoords='offset points', fontsize=9)
    for t in s.mrm:
        ax1.axvline(t - t0, color=C_BAD, lw=2)
        ax1.text(t - t0, 0.5, ' MRM', transform=ax1.get_xaxis_transform(), color=C_BAD, fontsize=10,
                 fontweight='bold')
    for t in s.disengage:
        ax1.axvline(t - t0, color=C_BAD, lw=2, ls='--')
        ax1.text(t - t0, 0.4, ' disengage', transform=ax1.get_xaxis_transform(), color=C_BAD,
                 fontsize=10, fontweight='bold')
    for t in s.disengage_after:
        ax1.axvline(t - t0, color=C_CMD, lw=1, ls='--')
    if not math.isnan(m.stop_error):
        ax1.annotate(f'stop error {m.stop_error:.3f} m\n(signed {m.stop_signed:+.3f} m, + = past)',
                     (t_rest - t0, 0.0), xytext=(-150, 40), textcoords='offset points', fontsize=9,
                     arrowprops=dict(arrowstyle='->', color=C_TARGET))
    ax1.set_xlim(x_lo, max(x_hi, 1.0))
    ax1.set_ylim(-0.05, max([m.target * 1.35] + [v + 0.1 for v in s.vel_v]))
    ax1.set_xlabel('time since engage [s]')
    ax1.set_ylabel('speed [m/s]')
    ax1.set_title('Speed', loc='left', fontsize=12)
    ax1.grid(alpha=0.3)
    ax1.legend(loc='lower center', fontsize=8, ncol=3, framealpha=0.9)

    # ---- panel 2: the floor plan
    if geo is not None and geo.path is not None:
        ax2.plot(geo.path.x, geo.path.y, color='#bbbbbb', lw=7, solid_capstyle='round',
                 label='route path', zorder=1)
        for a_, b_ in geo.turns:
            px = [x for x, ss in zip(geo.path.x, geo.path.s) if a_ <= ss <= b_]
            py = [y for y, ss in zip(geo.path.y, geo.path.s) if a_ <= ss <= b_]
            ax2.plot(px, py, color=C_TURN, lw=7, alpha=0.6, zorder=1, label='U-turn section')
        # the stop line, across the path at its last waypoint
        nx, ny = -geo.dir_y, geo.dir_x
        ax2.plot([geo.end_x - 0.5 * nx, geo.end_x + 0.5 * nx], [geo.end_y - 0.5 * ny, geo.end_y + 0.5 * ny],
                 color=C_TARGET, lw=3, zorder=4)
        ax2.text(geo.end_x + 0.6 * nx, geo.end_y + 0.6 * ny, 'stop', fontsize=10, ha='center', va='center')
    if s.fix_xy:
        ax2.scatter([p[0] for p in s.fix_xy], [p[1] for p in s.fix_xy], s=5, color='#ff9f40', alpha=0.35,
                    zorder=2, label='marker fixes (raw)')
    if s.true_xy:
        ax2.plot([p[0] for p in s.true_xy], [p[1] for p in s.true_xy], color=C_ENGAGE, lw=1, ls='--',
                 zorder=3, label='true pose (bench)')
    if s.kin_xy:
        ax2.plot([p[0] for p in s.kin_xy], [p[1] for p in s.kin_xy], color=C_MEAS, lw=1.6, zorder=3,
                 label='localized trajectory')
        for a_, b_, _ in off_path_spans(s, t0, t_rest):
            seg = [p for t, p in zip(s.kin_t, s.kin_xy) if a_ <= t <= b_]
            ax2.plot([p[0] for p in seg], [p[1] for p in seg], color=C_BAD, lw=4, zorder=5)
        if off_path_spans(s, t0, t_rest):
            ax2.plot([], [], color=C_BAD, lw=4, label=f'> {rg.PATH_DEV_MAX:.1f} m off the path')
    boards, start = _course_extras(course_dir)
    for bid, bx, by, byaw in boards:
        ax2.plot([bx - 0.25 * math.sin(byaw), bx + 0.25 * math.sin(byaw)],
                 [by + 0.25 * math.cos(byaw), by - 0.25 * math.cos(byaw)], color='#6b3fa0', lw=4, zorder=4)
        ax2.annotate('', (bx + 0.5 * math.cos(byaw), by + 0.5 * math.sin(byaw)), (bx, by),
                     arrowprops=dict(arrowstyle='->', color='#6b3fa0', lw=1))
        ax2.text(bx, by - 0.45, str(bid), fontsize=8, ha='center', va='top', color='#6b3fa0')
    if boards:
        ax2.plot([], [], color='#6b3fa0', lw=4, label='boards (facing arrow)')
    if start is not None:
        sx, sy, syaw = start
        nx, ny = -math.sin(syaw), math.cos(syaw)
        ax2.plot([sx - 0.5 * nx, sx + 0.5 * nx], [sy - 0.5 * ny, sy + 0.5 * ny], color=C_ENGAGE, lw=3, zorder=4)
        ax2.text(sx - 0.7 * nx, sy - 0.7 * ny, 'start', fontsize=10, ha='center', va='center', color=C_ENGAGE)
    ax2.set_aspect('equal', adjustable='datalim')
    ax2.set_xlabel('x [m]')
    ax2.set_ylabel('y [m]')
    ax2.set_title('The run on the floor', loc='left', fontsize=12)
    ax2.grid(alpha=0.3)
    ax2.legend(loc='lower right', fontsize=8, ncol=2, framealpha=0.9)

    # ---- panel 3: deviation from the path vs time
    if s.kin_dev:
        tt = rel(s.kin_t)
        ax3.plot(tt, s.kin_dev, color=C_MEAS, lw=1.5, label='localized distance from the path')
        lim = rg.PATH_DEV_MAX
        ax3.fill_between(tt, lim, s.kin_dev, where=[d > lim for d in s.kin_dev], color=C_BAD,
                         alpha=0.6, interpolate=True, label='over the limit')
        ax3.axhline(lim, color=C_BAD, lw=1, ls='--')
        ax3.text(1.0, lim, f' {lim:.2f} m (valid limit)', transform=ax3.get_yaxis_transform(),
                 color=C_BAD, fontsize=9, va='bottom', ha='right')
        ax3.axvspan(x_lo, 0.0, color='#eeeeee', zorder=0)
        ax3.axvspan(t_rest - t0, max(x_hi, 1.0), color='#eeeeee', zorder=0, label='not judged')
        ax3.set_ylim(0, max(lim * 1.5, max(s.kin_dev) * 1.1))
    else:
        ax3.text(0.5, 0.5, 'no localized pose or route geometry', transform=ax3.transAxes, ha='center')
    ax3.set_xlim(x_lo, max(x_hi, 1.0))
    ax3.set_xlabel('time since engage [s]')
    ax3.set_ylabel('deviation [m]')
    ax3.set_title(f'Distance from the path (max {_f(m.path_dev_max)} m, judged engage to rest)',
                  loc='left', fontsize=12)
    ax3.grid(alpha=0.3)
    ax3.legend(loc='upper left', fontsize=8, framealpha=0.9)

    fig.savefig(png, bbox_inches='tight')
    plt.close(fig)
    return png


# ------------------------------------------------------------ display

def display_works(display: Optional[str]) -> bool:
    if not display:
        return False
    for tool, args in (('xdpyinfo', ['-display', display]), ('xset', ['-display', display, 'q'])):
        if shutil.which(tool):
            try:
                return subprocess.run([tool, *args], capture_output=True, timeout=5).returncode == 0
            except (OSError, subprocess.SubprocessError):
                return False
    if display.startswith(':'):
        n = display[1:].split('.')[0]
        return os.path.exists(f'/tmp/.X11-unix/X{n}')
    return False


def show_png(png: str) -> bool:
    if not display_works(os.environ.get('DISPLAY')):
        print(f'no working X display (DISPLAY={os.environ.get("DISPLAY", "")!r}); on the car '
              f'`export DISPLAY=:1` and view it over RDP. Plot: {png}')
        return False
    for viewer in ('eog', 'feh', 'display', 'xdg-open'):
        if shutil.which(viewer):
            subprocess.Popen([viewer, png], start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f'opened {png} with {viewer} on {os.environ["DISPLAY"]}')
            return True
    print(f'no image viewer found; plot: {png}')
    return False


# ------------------------------------------------------------ main

def audit_bag(bag: str, car=None, team=None, reference=None, course=None, bench=None,
              out_png=None, plot=True):
    """(Audit, RouteMetrics) for a bag; writes the PNG and the JSON beside it."""
    from autosdv_coach_lab.calibration.bagio import read_bag
    from autosdv_waypoint_lab import check_bags as cb
    msgs = read_bag(bag, AUDIT_TOPICS)
    meta = bm.meta_from_messages(msgs, bag)
    is_bench = bool((meta or {}).get('bench')) if bench is None else bench
    verdict = cb.check_bag(bag, [course] if course else [], car, team, bench_ok=is_bench)
    m, meta = rg.read_route_bag(bag, None, course, msgs=msgs)
    ref = None
    if reference and m.route is not None:
        ref, _ = rg.read_route_bag(reference, None, course)
        if ref.route is None:
            ref = None
    geo, cdir = None, None
    sc = None
    if m.route is not None:
        sc = rg.score_route(m, ref)
        tdir = course or rg.template_dir(m.route)
        geo = rg.geometry_from_course(tdir, m.route) if tdir else None
        cdir = (meta or {}).get('course_dir')
        if not cdir or not os.path.isdir(cdir):
            cdir = tdir
    elif m.ungraded:
        _, tdir = rg.ungraded_course(meta, [course] if course else [])
        geo = rg.geometry_from_course(tdir, m.ungraded)
        cdir = (meta or {}).get('course_dir')
        if not cdir or not os.path.isdir(cdir):
            cdir = tdir
    s = series_from_messages(msgs, geo)
    a = build_audit(bag, verdict, m, sc, s, meta, reference if ref is not None else None)
    a.localization = localization_quality(msgs, m.t_engage, m.t_rest)
    if plot:
        png = out_png or os.path.join(bag, PNG)
        a.png = plot_audit(png, a, m, sc, s, geo, cdir)
    d = asdict(a)
    d.update(verdict=a.verdict, valid=a.valid)
    with open(os.path.join(bag, JSON), 'w') as f:
        json.dump(d, f, indent=2, default=lambda x: None)
    return a, m


def main(argv=None):
    ap = argparse.ArgumentParser(description='Audit one Lab 2 bag: verdict, metrics, violations, plot.')
    ap.add_argument('bag')
    ap.add_argument('--car', default=None)
    ap.add_argument('--team', default=None)
    ap.add_argument('--reference', default=None, metavar='BAG',
                    help="the car's TA reference bag; default: the fixed lines")
    ap.add_argument('--course', default=None, metavar='DIR', help='a surveyed copy of the route')
    ap.add_argument('--bench', action='store_true',
                    help='check in bench mode even if the bag is not marked (TA)')
    ap.add_argument('--show', action='store_true', help='open the PNG on $DISPLAY')
    ap.add_argument('--out', default=None, help='the PNG (default BAG/audit.png)')
    args = ap.parse_args(argv)
    if not os.path.isfile(os.path.join(args.bag, 'metadata.yaml')):
        print(f'lab2_audit: {args.bag} is not a bag directory (no metadata.yaml)', file=sys.stderr)
        return 2
    a, m = audit_bag(args.bag, args.car, args.team, args.reference, args.course,
                     True if args.bench else None, args.out)
    print(format_audit(a, m))
    if args.show and a.png:
        show_png(a.png)
    return 0 if a.valid else 1


if __name__ == '__main__':
    raise SystemExit(main())
