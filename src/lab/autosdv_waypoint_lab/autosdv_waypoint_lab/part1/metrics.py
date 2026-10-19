"""Lab 2 Part 1 checks: pure functions from a 100 Hz trace and part1_profile.yaml. No ROS.

The grader node (grader_node.py) builds the trace live from the plant's true state and
calls `evaluate` as the segments complete; `part1_calibrate.py` reads the same functions
back from trace.csv. Design: docs/design/lab2-part1-sim-design.md, "Checks and marks".

Trace columns (trace.csv, in this order):

    t          sim time [s] (the plant's clock)
    s          distance along the path from the start pose [m]
    y          lateral offset from the path, left positive [m]
    target     commanded speed, /control/command/control_cmd longitudinal.velocity [m/s]
    measured   velocity_report's speed, what the controller sees [m/s]
    speed      the plant's true longitudinal speed [m/s]
    pwm        motor PWM the plant receives [counts]
    mode       the actuator's longitudinal mode (~/debug/mode: active, override_brake:emergency)
    slip_r     rear wheel surface speed minus ground speed [m/s] (spin > 0, lock < 0)
    kappa_r    rear longitudinal slip ratio (lock below about -0.2 is past the tyre's peak)
    emergency  1 while the scenario's e-stop is raised (always 0: the course has none; an
               e-stop is optional in part1_profile.yaml and no check reads it)
    segment    the profile segment the car is in, '<index>:<kind>' ('start' before it)

The course: rise 0 -> 4 m/s, hold 4 s; 4 -> 11 m/s, hold 4 s; brake 11 -> 0 onto the stop line.
Nine checks per run, one window each, all read off the profile (no positions are hard-coded):

    hold_rmse_4/11     RMSE of true speed - hold speed over the hold (0, 1), from HOLD_SETTLE s
                       after entering it to its end
    overshoot          worst over the two steps up (holds 0, 1): peak true speed in the hold
                       above the hold speed, % of the step
    reach_s            worst over the same steps: how long after the commanded target the true
                       speed comes within 10 % of the step below the hold speed (the lag behind
                       the 2.5 m/s^2 ramp; 0 when the car gets there first)
    launch_slip        max rear slip over LAUNCH_WINDOW s from the first motion (wheel or car)
    launch_accel       max true acceleration (mean over LAUNCH_ACCEL_WINDOW) in that window
    stop_lag           max (true speed - commanded target) [m/s] over the final brake segment
                       (its s0 to the stop line): a car that cannot slow down
    stop_error         |s - stop_line_s| where the final rest (true speed < REST_SPEED) began
    stop_kappa         min rear slip ratio from the final brake on (speed > LOCK_MIN_SPEED)

No undershoot check at the stop: below a target of 0 is reversing, which the ESC allows only
after the car has stopped and sat at neutral (esc.py), and stopping short or long of the line
is stop_error's.
"""
import bisect
import csv
import io
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

COLUMNS = ('t', 's', 'y', 'target', 'measured', 'speed', 'pwm', 'mode', 'slip_r', 'kappa_r',
           'emergency', 'segment')
_STR_COLUMNS = {'mode', 'segment'}
_INT_COLUMNS = {'pwm', 'emergency'}

CHECK_NAMES = (
    'hold_rmse_4', 'hold_rmse_11',
    'overshoot', 'reach_s',
    'launch_slip', 'launch_accel',
    'stop_lag', 'stop_error', 'stop_kappa',
)
HOLD_CHECKS = (('hold_rmse_4', 0), ('hold_rmse_11', 1))   # (check, hold_index)

# Limits. Calibrated 2026-10-19 by scripts/testing/lab2/part1_calibrate.py --repeats 2 on the
# simplified course (4 m/s and 11 m/s holds, the final brake; no e-stop): the reference
# (reference/speed_pid_part1.py and its parameters in config/) live on nominal, low_batt and
# low_mu, 6 runs, all valid, the repeats bit-identical. A calibrated limit
# is 1.5 x the reference's worst case (2 significant figures up), never below its floor in
# part1_calibrate.py FLOOR (hold RMSE 0.05 m/s, reach 0.2 s, stop lag 0.3 m/s): worst case and
# its variant beside each. Fixed limits are physical or the stop error, not calibrated.
LIMITS_CALIBRATED = True
LIMITS: Dict[str, float] = {
    'hold_rmse_4': 0.05,              # m/s; worst 0.0274, low_mu (floor 0.05 applies)
    'hold_rmse_11': 0.15,             # m/s; worst 0.0984, low_batt
    'overshoot': 3.8,                 # % of the step; worst 2.49, nominal
    'reach_s': 0.39,                  # s behind the target; worst 0.26, low_batt (2026-10-20)
    'launch_slip': 0.5,               # m/s, PHYSICAL (reference 0.125, low_mu)
    'launch_accel': 4.5,              # m/s^2 over 50 ms; worst 2.94, nominal
    'stop_lag': 0.88,                 # m/s above the target; worst 0.581, low_mu
    'stop_error': 0.5,                # m, FIXED (reference 0.062, low_mu)
    'stop_kappa': -0.2,               # PHYSICAL: past the tyre's peak (reference -0.075)
}
# Fixed limits: physical (spin, lock) and the stop error. The rest are calibrated.
FIXED = ('launch_slip', 'stop_error', 'stop_kappa')
CALIBRATED = tuple(n for n in CHECK_NAMES if n not in FIXED)
# A check passes when value <= limit, except these (value >= limit)
LOWER_BOUND = ('stop_kappa',)
UNITS = {'hold_rmse_4': 'm/s', 'hold_rmse_11': 'm/s', 'overshoot': '%', 'reach_s': 's',
         'launch_slip': 'm/s', 'launch_accel': 'm/s^2', 'stop_lag': 'm/s', 'stop_error': 'm',
         'stop_kappa': ''}

HOLD_SETTLE = 1.0            # s into a hold before its RMSE counts (rc_sim's hold_metrics)
REACH_BAND = 0.1             # fraction of the step: "reached" within 10 % of it
LAUNCH_WINDOW = 1.0          # s from the first motion
LAUNCH_MOVE = 0.02           # m/s of car speed or rear slip that counts as motion
LAUNCH_ACCEL_WINDOW = 0.05   # s; rc_sim's: a lurch, not the per-ms contact impulses
LOCK_MIN_SPEED = 0.3         # m/s; below it the slip ratio is a standstill artifact (< -0.2)
REST_SPEED = 0.05            # m/s of true speed: at rest (a threshold, not zero)
FINISH_REST = 2.0            # s at rest in the final brake segment or later = finished
# Decelerations are not assumed: every window comes from the profile's per-segment s0/s1/v0/v1
# (braking at decel_brake 3.5 m/s^2, the rest at accel 2.5, would be equally fine).


# ---------------------------------------------------------------- profile

@dataclass(frozen=True)
class Segment:
    index: int
    kind: str                # accel | decel | hold | brake | runout
    s0: float
    s1: float
    v0: float
    v1: float
    hold_index: Optional[int] = None

    @property
    def label(self) -> str:
        return f'{self.index}:{self.kind}'


@dataclass(frozen=True)
class Profile:
    speeds: Tuple[float, ...]
    holds: Tuple[float, ...]
    segments: Tuple[Segment, ...]
    stop_line_s: float
    estop_trigger_s: Optional[float] = None     # the optional e-stop (none on the course)
    estop_hold_index: Optional[int] = None

    def hold(self, k: int) -> Segment:
        for seg in self.segments:
            if seg.kind == 'hold' and seg.hold_index == k:
                return seg
        raise KeyError(f'part1_profile: no hold with hold_index {k}')

    def before(self, seg: Segment) -> Optional[Segment]:
        """The segment immediately before `seg` along the path."""
        return self.segments[seg.index - 1] if seg.index > 0 else None

    def first(self, kind: str) -> Segment:
        for seg in self.segments:
            if seg.kind == kind:
                return seg
        raise KeyError(f'part1_profile: no {kind} segment')

    def segment_at(self, s: float) -> str:
        if not self.segments or s < self.segments[0].s0:
            return 'start'
        for seg in self.segments:
            if s < seg.s1:
                return seg.label
        return self.segments[-1].label

    @property
    def first_up_holds(self) -> Tuple[int, ...]:
        """Holds entered by accelerating from below: the steps up."""
        out = []
        for k in range(len(self.speeds)):
            prev = self.before(self.hold(k))
            if prev is not None and prev.kind == 'accel':
                out.append(k)
        return tuple(out)


def profile_from_dict(d: dict) -> Profile:
    segs, hold_k = [], 0
    for i, raw in enumerate(d['segments']):
        kind = str(raw['kind'])
        hi = raw.get('hold_index', raw.get('index'))
        if kind == 'hold':
            if hi is None:
                hi = hold_k
            hold_k = int(hi) + 1
        else:
            hi = None
        segs.append(Segment(i, kind, float(raw['s0']), float(raw['s1']), float(raw['v0']),
                            float(raw['v1']), None if hi is None else int(hi)))
    trigger, k = d.get('estop_trigger_s'), d.get('estop_hold_index')
    return Profile(tuple(float(v) for v in d['speeds']), tuple(float(h) for h in d['holds']),
                   tuple(segs), float(d['stop_line_s']),
                   None if trigger is None else float(trigger), None if k is None else int(k))


def load_profile(path) -> Profile:
    with open(Path(path).expanduser()) as f:
        return profile_from_dict(yaml.safe_load(f))


# ---------------------------------------------------------------- trace

class Trace:
    """Column store of the 100 Hz trace; rows appended live, read back from trace.csv."""

    def __init__(self, rows: Optional[Sequence[dict]] = None):
        self.cols: Dict[str, list] = {c: [] for c in COLUMNS}
        for row in rows or ():
            self.append(row)

    def __len__(self) -> int:
        return len(self.cols['t'])

    def append(self, row: dict) -> None:
        for c in COLUMNS:
            self.cols[c].append(row[c])

    def __getitem__(self, name: str) -> list:
        return self.cols[name]

    def csv_text(self, start: int = 0) -> str:
        out = io.StringIO()
        w = csv.writer(out, lineterminator='\n')
        if start == 0:
            w.writerow(COLUMNS)
        for i in range(start, len(self)):
            w.writerow([_fmt(self.cols[c][i]) for c in COLUMNS])
        return out.getvalue()

    def write_csv(self, path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + '.part')
        tmp.write_text(self.csv_text())
        tmp.replace(path)

    @staticmethod
    def read_csv(path) -> 'Trace':
        reader = csv.reader(io.StringIO(Path(path).read_text()))
        header = next(reader)
        if tuple(header) != COLUMNS:
            raise ValueError(f'{path}: unexpected columns {header}')
        tr = Trace()
        for raw in reader:
            row = {}
            for name, text in zip(COLUMNS, raw):
                if name in _STR_COLUMNS:
                    row[name] = text
                elif name in _INT_COLUMNS:
                    row[name] = int(float(text))
                else:
                    row[name] = float(text)
            tr.append(row)
        return tr


def _fmt(v) -> str:
    if isinstance(v, bool):
        return '1' if v else '0'
    if isinstance(v, float):
        return f'{v:.9g}'
    return str(v)


# ---------------------------------------------------------------- helpers

def _result(value, idx: Optional[int], tr: Trace, t0=None, t1=None) -> dict:
    t, s = tr['t'], tr['s']
    return {'value': value,
            't': None if idx is None else t[idx], 's': None if idx is None else s[idx],
            't0': t0, 't1': t1}


def _none(t0=None, t1=None) -> dict:
    return {'value': None, 't': None, 's': None, 't0': t0, 't1': t1}


def _first(seq, pred, start: int = 0) -> Optional[int]:
    for i in range(start, len(seq)):
        if pred(i):
            return i
    return None


def _in_s(tr: Trace, s0: float, s1: float) -> List[int]:
    return [i for i, x in enumerate(tr['s']) if s0 <= x < s1]


def estop_times(tr: Trace) -> Tuple[Optional[int], Optional[int]]:
    """Row indices of the e-stop being raised and cleared (None when not seen)."""
    em = tr['emergency']
    on = _first(em, lambda i: bool(em[i]))
    if on is None:
        return None, None
    return on, _first(em, lambda i: not em[i], on)


def launch_start(tr: Trace) -> Optional[int]:
    v, slip = tr['speed'], tr['slip_r']
    return _first(v, lambda i: abs(v[i]) > LAUNCH_MOVE or abs(slip[i]) > LAUNCH_MOVE)


# ---------------------------------------------------------------- metrics

def hold_rmse(tr: Trace, hold: Segment) -> dict:
    t, v = tr['t'], tr['speed']
    idx = _in_s(tr, hold.s0, hold.s1)
    if not idx:
        return _none()
    t0 = t[idx[0]] + HOLD_SETTLE
    steady = [i for i in idx if t[i] >= t0 and not tr['emergency'][i]]
    if not steady:
        return _none(t0, t[idx[-1]])
    err = [v[i] - hold.v0 for i in steady]
    worst = max(range(len(steady)), key=lambda j: abs(err[j]))
    rmse = math.sqrt(math.fsum(e * e for e in err) / len(err))
    return _result(rmse, steady[worst], tr, t[steady[0]], t[steady[-1]])


def step_up(tr: Trace, profile: Profile, k: int) -> Tuple[dict, dict]:
    """(overshoot %, reach s) for the step into hold k."""
    t, v = tr['t'], tr['speed']
    hold = profile.hold(k)
    acc = profile.before(hold)
    prev = acc.v0 if acc is not None else 0.0
    step = hold.v0 - prev
    idx = _in_s(tr, hold.s0, hold.s1)
    if not idx or step <= 0.0:
        return _none(), _none()
    t_enter = t[idx[0]]
    peak = max(idx, key=lambda i: v[i])
    over = _result(max(0.0, (v[peak] - hold.v0) / step * 100.0), peak, tr, t_enter, t[idx[-1]])
    # reach: measured from when the target crosses the level, not from the hold's start. The
    # target ramps into the hold and crosses the level before it, so a hold-based clock reads 0
    # for any car that follows the ramp at all.
    level = hold.v0 - REACH_BAND * step
    start = _first(tr['s'], lambda i: tr['s'][i] >= (acc.s0 if acc else hold.s0)) or 0
    target = tr['target']
    eps = 1e-6               # a sample on the level counts, after trace.csv's 9 digits too
    ref = _first(target, lambda i: target[i] >= level - eps, start)
    hit = _first(v, lambda i: v[i] >= level - eps, start)
    if ref is None:
        reach = _none(t_enter, t[-1])
    elif hit is None:
        reach = _none(t[ref], t[-1])
    else:
        reach = _result(max(0.0, t[hit] - t[ref]), hit, tr, t[ref], max(t[ref], t[hit]))
    return over, reach


def worst(results: List[dict]) -> dict:
    """The largest value; a missing one (None) is the worst of all."""
    if not results:
        return _none()
    missing = [r for r in results if r['value'] is None]
    if missing:
        return missing[0]
    return max(results, key=lambda r: r['value'])


def launch(tr: Trace) -> Tuple[dict, dict]:
    t, v, slip = tr['t'], tr['speed'], tr['slip_r']
    i0 = launch_start(tr)
    if i0 is None:
        return _none(), _none()
    t0, t1 = t[i0], t[i0] + LAUNCH_WINDOW
    window = [i for i in range(i0, len(t)) if t[i] <= t1 + 1e-9]
    i_slip = max(window, key=lambda i: slip[i])
    dt = (t[-1] - t[0]) / (len(t) - 1) if len(t) > 1 else 0.01
    lag = max(1, int(round(LAUNCH_ACCEL_WINDOW / dt))) if dt > 0 else 1
    accel = [((v[i] - v[i - lag]) / (t[i] - t[i - lag]), i) for i in window
             if i >= lag and t[i] > t[i - lag]]
    a, i_a = max(accel) if accel else (0.0, i0)
    return _result(slip[i_slip], i_slip, tr, t0, t1), _result(a, i_a, tr, t0, t1)


def stop_lag(tr: Trace, profile: Profile) -> dict:
    """Max (true speed - commanded target) over the final brake segment, s0 to the stop line."""
    t, v, target, s = tr['t'], tr['speed'], tr['target'], tr['s']
    brake = profile.first('brake')
    idx = [i for i in range(len(t)) if brake.s0 <= s[i] < brake.s1]
    if not idx:
        return _none()
    worst = max(idx, key=lambda i: v[i] - target[i])
    return _result(v[worst] - target[worst], worst, tr, t[idx[0]], t[idx[-1]])


def stop(tr: Trace, profile: Profile) -> Tuple[dict, dict]:
    """(|stop error| where the final rest began, None unless at rest; min slip ratio braking)."""
    t, s, v, kappa = tr['t'], tr['s'], tr['speed'], tr['kappa_r']
    brake = profile.first('brake')
    idx = [i for i in range(len(t)) if s[i] >= brake.s0]
    if not idx:
        return _none(), _none()
    t0, t1 = t[idx[0]], t[idx[-1]]
    last = len(t) - 1
    if abs(v[last]) < REST_SPEED:
        rest = last   # where the final rest began: the stop, not a slow creep afterwards
        while rest > idx[0] and abs(v[rest - 1]) < REST_SPEED:
            rest -= 1
        err = _result(abs(s[rest] - profile.stop_line_s), rest, tr, t0, t1)
    else:
        err = _none(t0, t1)
    fast = [i for i in idx if v[i] > LOCK_MIN_SPEED]
    if not fast:
        return err, _result(0.0, None, tr, t0, t1)
    i_k = min(fast, key=lambda i: kappa[i])
    return err, _result(kappa[i_k], i_k, tr, t0, t1)


def metrics(tr: Trace, profile: Profile) -> Dict[str, dict]:
    """Every check's measured value (with where and over which window), before limits."""
    out = {}
    for name, k in HOLD_CHECKS:
        out[name] = hold_rmse(tr, profile.hold(k)) if k < len(profile.speeds) else _none()
    overs, reaches = [], []
    for k in profile.first_up_holds:
        o, r = step_up(tr, profile, k)
        overs.append(o)
        reaches.append(r)
    out['overshoot'], out['reach_s'] = worst(overs), worst(reaches)
    out['launch_slip'], out['launch_accel'] = launch(tr)
    out['stop_lag'] = stop_lag(tr, profile)
    out['stop_error'], out['stop_kappa'] = stop(tr, profile)
    for name, r in out.items():
        if not _window_finite(tr, r, NAN_COLUMNS[name]):
            out[name] = _none(r['t0'], r['t1'])
    return out


# the trace columns each check reads; a non-finite sample in its window makes it None (fail)
_SPEED = ('t', 's', 'speed')
NAN_COLUMNS = {
    'hold_rmse_4': _SPEED, 'hold_rmse_11': _SPEED, 'overshoot': _SPEED,
    'reach_s': _SPEED + ('target',),
    'launch_slip': _SPEED + ('slip_r',), 'launch_accel': _SPEED + ('slip_r',),
    'stop_lag': _SPEED + ('target',), 'stop_error': _SPEED,
    'stop_kappa': _SPEED + ('kappa_r',),
}


def _window_finite(tr: Trace, r: dict, columns) -> bool:
    """True when every sample of `columns` in the result's window [t0, t1] is finite."""
    if r['t0'] is None:
        return True
    t = tr['t']
    t1 = r['t1'] if r['t1'] is not None else t[-1]
    lo = bisect.bisect_left(t, r['t0'] - 1e-9)
    hi = bisect.bisect_right(t, t1 + 1e-9)
    for c in columns:
        col = tr[c]
        for i in range(lo, hi):
            if not math.isfinite(col[i]):
                return False
    return True


# ---------------------------------------------------------------- live windows

def closed_checks(tr: Trace, profile: Profile, finished: bool) -> set:
    """Checks whose window has closed in the trace so far (all of them once finished)."""
    if finished:
        return set(CHECK_NAMES)
    if not len(tr):
        return set()
    s_max, t_now = max(tr['s']), tr['t'][-1]
    out = set()
    for name, k in HOLD_CHECKS:
        if k < len(profile.speeds) and s_max >= profile.hold(k).s1:
            out.add(name)
    ups = profile.first_up_holds
    if ups and s_max >= profile.hold(ups[-1]).s1:
        out.update(('overshoot', 'reach_s'))
    i0 = launch_start(tr)
    if i0 is not None and t_now >= tr['t'][i0] + LAUNCH_WINDOW:
        out.update(('launch_slip', 'launch_accel'))
    return out   # stop_lag, stop_error and stop_kappa close at the finish


def passes(name: str, value, limit: float) -> bool:
    if value is None or not math.isfinite(value):
        return False
    return value >= limit if name in LOWER_BOUND else value <= limit


def evaluate(tr: Trace, profile: Profile, finished: bool = True,
             limits: Dict[str, float] = LIMITS, unclosed: str = 'pending') -> Dict[str, dict]:
    """The checks (CHECK_NAMES): {name: {value, limit, pass, unit, t, s, t0, t1}}.

    `finished`: the run reached its end, every window is closed. Otherwise a check whose window
    has not closed has value None and pass None (`unclosed` 'pending': live, or an invalid run)
    or pass False (`unclosed` 'fail': a valid run that ended early, stalled or timed out).
    """
    closed = closed_checks(tr, profile, finished)
    m = metrics(tr, profile) if len(tr) else {n: _none() for n in CHECK_NAMES}
    out = {}
    for name in CHECK_NAMES:
        r = m[name]
        c = {'value': _clean(r['value']), 'limit': limits[name], 'pass': None,
             'unit': UNITS[name], 't': r['t'], 's': r['s'], 't0': r['t0'], 't1': r['t1']}
        if name in closed:
            c['pass'] = passes(name, r['value'], limits[name])
        else:
            c['value'] = None
            c['pass'] = False if unclosed == 'fail' else None
        out[name] = c
    return out


def _clean(v):
    if v is None:
        return None
    v = float(v)
    return v if math.isfinite(v) else None


def is_finished(tr: Trace, profile: Profile, rest: float = FINISH_REST) -> bool:
    """At rest for `rest` s in the final brake segment or past it."""
    if not len(tr):
        return False
    brake_s0 = profile.first('brake').s0
    t, s, v = tr['t'], tr['s'], tr['speed']
    i = len(t) - 1
    if abs(v[i]) >= REST_SPEED or s[i] < brake_s0:
        return False
    while i > 0 and abs(v[i - 1]) < REST_SPEED and s[i - 1] >= brake_s0:
        i -= 1
    return t[-1] - t[i] >= rest


def summary(checks: Dict[str, dict], valid: bool) -> Tuple[int, int]:
    """(passed, total); an invalid run passes nothing."""
    total = len(CHECK_NAMES)
    if not valid:
        return 0, total
    return sum(1 for n in CHECK_NAMES if checks.get(n, {}).get('pass')), total


def invalid_reason(*, aborted: Optional[str] = None, mrm: Optional[str] = None,
                   fault: Optional[str] = None, lockstep: Optional[str] = None,
                   estop_missing: Optional[str] = None,
                   not_engaged: Optional[str] = None,
                   startup_stall: Optional[str] = None,
                   actuator_missing: Optional[str] = None) -> Optional[str]:
    """Why a run is not valid, the first cause in this order; None when it is valid.

    Only these invalidate (lead ruling). A car that stalls, never moves or does not finish in
    time is a VALID run whose unreached checks fail (see stall_message).
    """
    for reason in (startup_stall and STARTUP_STALL.format(startup_stall),
                   actuator_missing and f'the actuator is not running: {actuator_missing}',
                   lockstep and f'lockstep invalidated: {lockstep}',
                   fault and f'controller fault: {fault}',
                   mrm and f'MRM: {mrm}',
                   aborted and f'scenario aborted: {aborted}',
                   estop_missing and f'e-stop not raised: {estop_missing}',
                   not_engaged and f'the stack did not engage: {not_engaged}'):
        if reason:
            return reason
    return None


def actuator_missing(mode_seen: bool, t: float, t_engaged: Optional[float]) -> Optional[str]:
    """Why the actuator counts as dead: engaged ACTUATOR_GRACE s ago and no mode from it yet.

    The actuator latches its mode from its first tick, so silence after the engage means it
    never started (a crash at start-up, e.g. a speed controller that failed to load). Without
    this the car stands still and the run would end valid as "did not move", blaming a class
    that never ran.
    """
    if mode_seen or t_engaged is None or t - t_engaged <= ACTUATOR_GRACE:
        return None
    return (f'no mode from it {ACTUATOR_GRACE:g} s after the engage: it failed to start, '
            'e.g. the speed controller does not import or its __init__ raised '
            "(see the actuator's log in play_log/)")


ACTUATOR_GRACE = 2.0  # s of sim time after the engage for the actuator's first mode

# a lockstep timeout before the engage: nothing of the student's has run yet
STARTUP_STALL = ('the simulator stalled before the run started (machine too slow / '
                 'suspended?); run again [lockstep: {}]')
STALL_TIME = 10.0     # s of sim time with a target and no progress: the run ends (valid)
STALL_S = 0.5         # m: "did not move"
STALL_TARGET = 0.05   # m/s: a target above this counts as "asked to move"
ESTOP_MARGIN = 3.0    # m past estop_trigger_s with no e-stop raised: the scenario failed


def stall_message(tr: Trace, t_engaged: float) -> Optional[str]:
    """Student-facing reason to end a valid run early: the car did not move, or stopped."""
    t, s, v, target, em = tr['t'], tr['s'], tr['speed'], tr['target'], tr['emergency']
    if not len(tr):
        return None
    start = bisect.bisect_left(t, t_engaged - 1e-9)
    i_cmd = _first(target, lambda i: target[i] > STALL_TARGET, start)
    if i_cmd is None:
        return None
    if t[-1] - t[i_cmd] >= STALL_TIME and max(s[i_cmd:]) < STALL_S:
        return (f'Your car did not move: the target was {max(target[i_cmd:]):.2f} m/s for '
                f'{STALL_TIME:g} s and the car stayed within {STALL_S:g} m of the start. Is your '
                'speed_pid.py still the stub (it holds neutral)? '
                'The checks it did not reach fail.')
    i = len(t) - 1
    if not (abs(v[i]) < REST_SPEED and target[i] > STALL_TARGET and not em[i]):
        return None
    while (i > i_cmd and abs(v[i - 1]) < REST_SPEED and target[i - 1] > STALL_TARGET
           and not em[i - 1]):
        i -= 1
    if t[-1] - t[i] >= STALL_TIME:
        return (f'Your car stopped at s = {s[-1]:.1f} m and stayed at rest for {STALL_TIME:g} s '
                f'with a target of {target[-1]:.2f} m/s. Is your speed_pid.py still the stub, '
                'or does its output stay at neutral? The checks it did not reach fail.')
    return None


def estop_missing(tr: Trace, profile: Profile) -> Optional[str]:
    """The car passed the e-stop trigger and no e-stop was raised (a stack fault).

    Never on a profile without an e-stop (the course has none).
    """
    if profile.estop_trigger_s is None:
        return None
    if not len(tr) or max(tr['s']) < profile.estop_trigger_s + ESTOP_MARGIN:
        return None
    if any(tr['emergency']):
        return None
    return (f'the car passed s = {profile.estop_trigger_s + ESTOP_MARGIN:.1f} m and the '
            'scenario never raised it (a simulation stack fault, not your controller)')


def describe(name: str, c: dict) -> str:
    value = 'none' if c['value'] is None else f"{c['value']:.3g}"
    op = '>=' if name in LOWER_BOUND else '<='
    unit = f" {c['unit']}" if c.get('unit') else ''
    return f"{name}: {value} (need {op} {c['limit']:.3g}{unit})"


def violations(checks: Dict[str, dict]) -> List[dict]:
    """The failed checks, with where (t, s) and over which window (t0, t1) they failed."""
    out = []
    for name in CHECK_NAMES:
        c = checks[name]
        if c['pass'] is False:
            out.append({'name': name, 'value': c['value'], 'limit': c['limit'],
                        't': c['t'], 's': c['s'], 't0': c['t0'], 't1': c['t1'],
                        'text': describe(name, c)})
    return out
