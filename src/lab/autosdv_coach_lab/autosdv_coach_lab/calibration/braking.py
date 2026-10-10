"""Braking and reaction: what the planner's safety envelope may assume. ROS-free.

The envelope (pursuit.braking_speed) lets the car drive at most the speed from
which it stops inside the gap to the standoff floor, after `reaction_time` at
that speed and then at `envelope_decel`:

    gap = v * reaction_time + v^2 / (2 * envelope_decel)

Both numbers are guesses until measured. This module measures them from a
recording of stops from a steady speed. Each stop has a trigger:

    board_stop  the board stops (truth in the simulator, the detections on the car)
    board_drop  the board is lost (last detection, then the tracker times out)
    disengage   the operation mode leaves AUTONOMOUS
    emergency   MRM leaves NORMAL, or an emergency command is raised

and for each stop it reports

    v0          steady speed before the trigger
    reaction    trigger to the onset of deceleration, from a hinge fit
                v = v0 before t0, v0 - a (t - t0) after, on the speed down to 30 % of v0
    a_hinge     the hinge's deceleration
    a_mean      mean deceleration from 90 % to 20 % of v0
    a_peak      steepest 0.3 s slope
    distance    from the trigger to standstill
    a_eff       the deceleration the envelope formula needs to reproduce that
                distance with the measured reaction:
                v0^2 / (2 (distance - v0 * reaction))

a_eff is the number the envelope cares about: it folds the ramp-in, the jerk
limit and the creep at the end into one value consistent with how the
envelope uses envelope_decel.

The recommendation is conservative by a stated margin: reaction_time is the
worst measured reaction plus `reaction_margin` seconds, and envelope_decel is
the worst a_eff times `decel_margin`. Which stops count is a choice:
reaction_time takes every stop, since every trigger is something the
envelope must survive; envelope_decel takes only the stops the stack itself
commands at full braking (board_drop, disengage, emergency) when there are
any -- after a board_stop the headway law may choose to brake gently, which
says nothing about what the car can do -- and falls back to all of them,
with a warning, when there are none.

The speed is velocity_status. On the car it comes from the hall sensor, whose
EMA lags the wheel by roughly 0.1 s at 1 m/s and which holds its last value
for 0.5 s at a standstill. The lag rounds the corner of the speed trace, so
the hinge can put the onset a little *earlier* with a shallower slope; taken
alone, neither reaction nor a_hinge is then conservative. The pair (reaction,
a_eff) is: a_eff is defined so the envelope formula reproduces the measured
stopping distance, and the measured distance is longer than the wheel's
because the sensor lags. Read them together, as the envelope does.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

COMMANDED = ('board_drop', 'disengage', 'emergency')


@dataclass
class Trigger:
    t: float
    kind: str
    detail: str = ''


@dataclass
class StopEvent:
    trigger: Trigger
    v0: float
    reaction: float
    a_hinge: float
    a_mean: float
    a_peak: float
    t_stop: Optional[float]      # s after the trigger; None if it never stopped
    distance: float              # m from the trigger to standstill (or window end)
    a_eff: float
    steady: bool                 # speed was steady before the trigger
    followed_by: List[Trigger] = field(default_factory=list)
    cmd_accel_min: Optional[float] = None


@dataclass
class Skipped:
    trigger: Trigger
    reason: str


# ---------------------------------------------------------------------------
# Trigger detection
# ---------------------------------------------------------------------------

def edges(t: Sequence[float], x: Sequence[bool], rising: bool = True) -> List[float]:
    """Times at which the boolean series x goes False->True (or True->False)."""
    out = []
    prev = None
    for ti, xi in zip(t, x):
        xi = bool(xi)
        if prev is not None and xi != prev and xi == rising:
            out.append(float(ti))
        prev = xi
    return out


def window_speed(t: np.ndarray, x: np.ndarray, y: np.ndarray,
                 half: float = 0.25) -> np.ndarray:
    """Ground speed of a point from its positions: the slope of a line fitted
    over +-half seconds around each sample, a centred estimate that puts a
    step in speed at the step."""
    out = np.full(len(t), np.nan)
    for i in range(len(t)):
        a = np.searchsorted(t, t[i] - half)
        b = np.searchsorted(t, t[i] + half, side='right')
        if b - a >= 3:
            tt = t[a:b] - t[i]
            vx = np.polyfit(tt, x[a:b], 1)[0]
            vy = np.polyfit(tt, y[a:b], 1)[0]
            out[i] = math.hypot(vx, vy)
    return out


def board_stop_times(t: np.ndarray, speed: np.ndarray, moving: float = 0.4,
                     stopped: float = 0.15, hold: float = 1.0,
                     slowdown: float = 3.0) -> List[float]:
    """When a board that had been moving at >= `moving` m/s for `hold` s
    comes to a stop (below `stopped`) within `slowdown` s of last moving:
    the time its speed crossed half of its speed over that hold
    (interpolated) -- the stop instant for a dead stop, the middle of the
    slowdown for a gradual one."""
    out = []
    ok = np.isfinite(speed)
    t, speed = t[ok], speed[ok]
    i = 0
    while i < len(t):
        if speed[i] >= stopped:
            i += 1
            continue
        j = i - 1
        while j >= 0 and speed[j] < moving:
            j -= 1
        if j < 0 or t[i] - t[j] > slowdown:
            i += 1
            continue
        a = int(np.searchsorted(t, t[j] - hold))
        pre = speed[a:j + 1]
        if t[j] - t[a] < 0.8 * hold or pre.min() < moving:
            i += 1
            continue
        level = 0.5 * float(np.mean(pre))
        k = j + 1
        while k < i and speed[k] > level:
            k += 1
        s0, s1 = speed[k - 1], speed[k]
        tc = t[k - 1] + (s0 - level) / (s0 - s1) * (t[k] - t[k - 1]) if s0 != s1 else t[k]
        out.append(float(tc))
        while i < len(t) and speed[i] < moving:
            i += 1
    return out


def board_in_world(det_t, det_xy, odom_t, odom_xy, odom_yaw):
    """Detections (board in base_link) placed in the odometry frame with the
    ego pose at each detection's stamp (nearest odometry sample)."""
    det_t = np.asarray(det_t)
    odom_t = np.asarray(odom_t)
    k = np.clip(np.searchsorted(odom_t, det_t), 0, len(odom_t) - 1)
    k0 = np.clip(k - 1, 0, len(odom_t) - 1)
    k = np.where(np.abs(odom_t[k0] - det_t) < np.abs(odom_t[k] - det_t), k0, k)
    ex, ey = np.asarray(odom_xy)[k].T
    yaw = np.asarray(odom_yaw)[k]
    bx, by = np.asarray(det_xy).T
    c, s = np.cos(yaw), np.sin(yaw)
    return ex + c * bx - s * by, ey + s * bx + c * by


# ---------------------------------------------------------------------------
# One stop
# ---------------------------------------------------------------------------

def _crossing(t: np.ndarray, v: np.ndarray, level: float, after: float) -> Optional[float]:
    """First time after `after` at which v is at or below `level` (interpolated)."""
    i = int(np.searchsorted(t, after))
    if i >= len(t):
        return None
    if v[i] <= level:
        return float(t[i])
    for k in range(i + 1, len(t)):
        if v[k] <= level:
            if v[k - 1] == v[k]:
                return float(t[k])
            return float(t[k - 1] + (v[k - 1] - level) / (v[k - 1] - v[k]) * (t[k] - t[k - 1]))
    return None


def hinge_fit(t: np.ndarray, v: np.ndarray, v0: float, t_lo: float,
              t_hi: float, step: float = 0.005) -> Tuple[float, float]:
    """Best (t0, a) for v = v0 (t < t0), v0 - a (t - t0) (t >= t0) on [t_lo, t_hi]
    by a scan over t0 with a least squares in a."""
    m = (t >= t_lo) & (t <= t_hi)
    tt, vv = t[m], v[m]
    best = (t_lo, 0.0, math.inf)
    for t0 in np.arange(t_lo, t_hi, step):
        after = tt >= t0
        dt = tt[after] - t0
        if len(dt) < 2:
            continue
        den = float(np.sum(dt * dt))
        a = float(np.sum((v0 - vv[after]) * dt) / den) if den > 0 else 0.0
        a = max(a, 0.0)
        sse = float(np.sum((vv[~after] - v0) ** 2) + np.sum((vv[after] - (v0 - a * dt)) ** 2))
        if sse < best[2]:
            best = (float(t0), a, sse)
    return best[0], best[1]


def peak_decel(t: np.ndarray, v: np.ndarray, t_lo: float, t_hi: float,
               span: float = 0.3) -> float:
    best = 0.0
    i = int(np.searchsorted(t, t_lo))
    while i < len(t) and t[i] < t_hi:
        j = int(np.searchsorted(t, t[i] + span, side='right'))
        if j - i >= 3:
            s = np.polyfit(t[i:j] - t[i], v[i:j], 1)[0]
            best = max(best, -float(s))
        i += 1
    return best


def analyse_stop(t: np.ndarray, v: np.ndarray, trig: Trigger, min_speed: float = 0.3,
                 pre: float = 1.0, max_window: float = 10.0,
                 stopped: float = 0.05) -> Tuple[Optional[StopEvent], Optional[str]]:
    """Measure one stop; (event, None) or (None, why it was skipped)."""
    a = np.searchsorted(t, trig.t - pre)
    b = np.searchsorted(t, trig.t)
    if b - a < 3:
        return None, 'no speed data before the trigger'
    before = v[a:b]
    v0 = float(np.mean(before))
    if v0 < min_speed:
        return None, f'not moving ({v0:.2f} m/s < {min_speed:.2f})'
    steady = float(np.std(before)) <= 0.05 + 0.1 * v0

    t30 = _crossing(t, v, 0.3 * v0, trig.t - 0.3)
    if t30 is None or t30 > trig.t + max_window:
        return None, f'did not slow below 30 % of {v0:.2f} m/s within {max_window:.0f} s'
    t0, a_h = hinge_fit(t, v, v0, trig.t - 0.5, t30)
    reaction = t0 - trig.t

    t90 = _crossing(t, v, 0.9 * v0, trig.t - 0.3)
    t20 = _crossing(t, v, 0.2 * v0, trig.t - 0.3)
    a_mean = (0.7 * v0 / (t20 - t90)) if (t90 is not None and t20 is not None
                                          and t20 > t90) else float('nan')
    t_stop = _crossing(t, v, stopped, trig.t)
    end = t_stop if t_stop is not None else min(trig.t + max_window, float(t[-1]))
    m = (t >= trig.t) & (t <= end)
    tt = np.concatenate(([trig.t], t[m]))
    vv = np.concatenate(([float(np.interp(trig.t, t, v))], v[m]))
    distance = float(np.sum(0.5 * (vv[1:] + vv[:-1]) * np.diff(tt)))
    brake_dist = distance - v0 * max(reaction, 0.0)
    a_eff = v0 * v0 / (2.0 * brake_dist) if brake_dist > 1e-6 else float('inf')
    return StopEvent(
        trigger=trig, v0=v0, reaction=reaction, a_hinge=a_h, a_mean=a_mean,
        a_peak=peak_decel(t, v, t0, end), t_stop=(t_stop - trig.t) if t_stop else None,
        distance=distance, a_eff=a_eff, steady=steady), None


def analyse(t_v: Sequence[float], v: Sequence[float], triggers: Sequence[Trigger],
            cmd: Optional[Tuple[Sequence[float], Sequence[float]]] = None,
            min_speed: float = 0.3) -> Tuple[List[StopEvent], List[Skipped]]:
    """Every trigger, in time order. A trigger inside a stop already being
    measured (before that vehicle stopped) is attached to it, not measured
    again: the first trigger of a stop is the one it reacted to."""
    t = np.asarray(t_v, dtype=float)
    vv = np.asarray(v, dtype=float)
    o = np.argsort(t, kind='stable')
    t, vv = t[o], vv[o]
    events: List[StopEvent] = []
    skipped: List[Skipped] = []
    for trig in sorted(triggers, key=lambda x: x.t):
        if events:
            last = events[-1]
            end = last.trigger.t + (last.t_stop if last.t_stop is not None else 10.0)
            if trig.t <= end:
                last.followed_by.append(trig)
                continue
        ev, why = analyse_stop(t, vv, trig, min_speed=min_speed)
        if ev is None:
            skipped.append(Skipped(trig, why))
            continue
        if cmd is not None and len(cmd[0]):
            ct, ca = np.asarray(cmd[0]), np.asarray(cmd[1])
            e = ev.trigger.t + (ev.t_stop if ev.t_stop is not None else 5.0)
            m = (ct >= ev.trigger.t) & (ct <= e)
            if m.any():
                ev.cmd_accel_min = float(ca[m].min())
        events.append(ev)
    return events, skipped


# ---------------------------------------------------------------------------
# Recommendation
# ---------------------------------------------------------------------------

@dataclass
class Recommendation:
    reaction_time: float
    envelope_decel: float
    reaction_from: List[str]
    decel_from: List[str]
    worst_reaction: float
    worst_a_eff: float
    reaction_margin: float
    decel_margin: float
    notes: List[str] = field(default_factory=list)


def _ceil(x: float, q: float) -> float:
    return math.ceil(x / q - 1e-9) * q


def _floor(x: float, q: float) -> float:
    return math.floor(x / q + 1e-9) * q


def recommend(events: Sequence[StopEvent], reaction_margin: float = 0.2,
              decel_margin: float = 0.75) -> Optional[Recommendation]:
    if not events:
        return None
    notes = []
    commanded = [e for e in events if e.trigger.kind in COMMANDED]
    decel_set = commanded
    if not commanded:
        decel_set = list(events)
        notes.append('no commanded stop (board_drop / disengage / emergency) in the '
                     'recording: envelope_decel is taken from board stops, where the '
                     'headway law chose the braking; record a disengage or a board drop '
                     'at speed for a measurement of the car itself')
    if any(not e.steady for e in events):
        notes.append('some stops did not start from a steady speed (marked *)')
    worst_r = max(e.reaction for e in events)
    worst_a = min(e.a_eff for e in decel_set)
    rec_r = _ceil(max(worst_r, 0.0) + reaction_margin, 0.05)
    rec_a = max(_floor(worst_a * decel_margin, 0.05), 0.05)
    return Recommendation(
        reaction_time=round(rec_r, 3), envelope_decel=round(rec_a, 3),
        reaction_from=sorted({e.trigger.kind for e in events}),
        decel_from=sorted({e.trigger.kind for e in decel_set}),
        worst_reaction=worst_r, worst_a_eff=worst_a,
        reaction_margin=reaction_margin, decel_margin=decel_margin, notes=notes)


def braking_speed(gap: float, envelope_decel: float, reaction_time: float) -> float:
    """pursuit.braking_speed with explicit numbers."""
    from autosdv_coach_lab.pursuit import PlannerParams, braking_speed as bs
    return bs(gap, PlannerParams(envelope_decel=envelope_decel, reaction_time=reaction_time))


def range_for_speed(v: float, envelope_decel: float, reaction_time: float,
                    standoff_min: float) -> float:
    """The board range from which the envelope allows speed v."""
    return standoff_min + v * reaction_time + v * v / (2.0 * envelope_decel)


def format_report(events: Sequence[StopEvent], skipped: Sequence[Skipped],
                  rec: Optional[Recommendation], current: Dict[str, float],
                  at_range: float = 2.0) -> str:
    lines = []
    if events:
        lines.append(f'{"#":>2} {"t [s]":>8} {"trigger":<11}{"v0":>6}{"react":>7}{"a_hinge":>8}'
                     f'{"a_mean":>8}{"a_peak":>8}{"t_stop":>7}{"dist":>7}{"a_eff":>7}'
                     f'{"cmd_a":>7}')
        for i, e in enumerate(events, 1):
            ts = f'{e.t_stop:.2f}' if e.t_stop is not None else '-'
            ca = f'{e.cmd_accel_min:.2f}' if e.cmd_accel_min is not None else '-'
            lines.append(
                f'{i:>2} {e.trigger.t:>8.2f} {e.trigger.kind:<11}{e.v0:>6.2f}{e.reaction:>7.2f}'
                f'{e.a_hinge:>8.2f}{e.a_mean:>8.2f}{e.a_peak:>8.2f}{ts:>7}{e.distance:>7.2f}'
                f'{e.a_eff:>7.2f}{ca:>7}' + ('' if e.steady else ' *'))
            for f in e.followed_by:
                lines.append(f'{"":>12}then {f.kind} at +{f.t - e.trigger.t:.2f} s'
                             + (f' ({f.detail})' if f.detail else ''))
        lines.append('   (speeds m/s, decelerations m/s^2, times s, distances m; '
                     't is from the start of the bag)')
    else:
        lines.append('no stops measured')
    for s in skipped:
        lines.append(f'skipped {s.trigger.kind} at {s.trigger.t:.2f} s: {s.reason}')
    lines.append('')
    sm = current.get('standoff_min', 1.0)
    cur_a, cur_r = current.get('envelope_decel'), current.get('reaction_time')
    if rec is None:
        lines.append('no recommendation: no stop from a steady speed in the recording')
        return '\n'.join(lines)
    lines += [
        f'worst reaction      {rec.worst_reaction:.2f} s   (from {", ".join(rec.reaction_from)})',
        f'worst a_eff         {rec.worst_a_eff:.2f} m/s^2 (from {", ".join(rec.decel_from)})',
        '',
        'recommended, for board_pursuit_planner.param.yaml:',
        f'  reaction_time:  {rec.reaction_time:.2f}   # worst {rec.worst_reaction:.2f} s '
        f'+ {rec.reaction_margin:.2f} s margin',
        f'  envelope_decel: {rec.envelope_decel:.2f}   # worst a_eff {rec.worst_a_eff:.2f} '
        f'x {rec.decel_margin:.2f}',
    ]
    gap = at_range - sm
    v_rec = braking_speed(gap, rec.envelope_decel, rec.reaction_time)
    lines.append('')
    lines.append(f'speed cap at {at_range:.1f} m range (standoff_min {sm:.2f} m):')
    lines.append(f'  recommended  {v_rec:.2f} m/s')
    if cur_a and cur_r:
        v_cur = braking_speed(gap, cur_a, cur_r)
        lines.append(f'  current      {v_cur:.2f} m/s  (envelope_decel {cur_a:.2f}, '
                     f'reaction_time {cur_r:.2f})')
    v_max = current.get('v_max')
    if v_max:
        r = range_for_speed(v_max, rec.envelope_decel, rec.reaction_time, sm)
        lines.append(f'  v_max {v_max:.2f} m/s is allowed from {r:.2f} m')
    for n in rec.notes:
        lines.append(f'NOTE: {n}')
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Bags and the command line
# ---------------------------------------------------------------------------

TOPICS = dict(
    speed='/vehicle/status/velocity_status',
    mode='/api/operation_mode/state',
    mrm='/system/fail_safe/mrm_state',
    emergency='/control/command/emergency_cmd',
    state='/perception/coach/state',
    board='/perception/coach/board',
    truth='/virtual_coach/truth',
    odom='/localization/kinematic_state',
    cmd='/control/command/control_cmd',
)

AUTONOMOUS = 2   # autoware_adapi_v1_msgs/OperationModeState
MRM_NORMAL = 1   # autoware_adapi_v1_msgs/MrmState


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def triggers_from_bag(msgs: Dict[str, list], board_source: str = 'auto',
                      scan_period: float = 0.1,
                      board_time: str = 'stamp') -> Tuple[List[Trigger], str]:
    """Every trigger the bag shows; and which board-stop source was used."""
    out: List[Trigger] = []
    mode = msgs.get(TOPICS['mode'], [])
    if mode:
        for t in edges([t for t, _ in mode], [m.mode == AUTONOMOUS for _, m in mode], False):
            out.append(Trigger(t, 'disengage'))
    mrm = msgs.get(TOPICS['mrm'], [])
    if mrm:
        for t in edges([t for t, _ in mrm], [m.state != MRM_NORMAL and m.state != 0
                                             for _, m in mrm], True):
            out.append(Trigger(t, 'emergency', 'MRM'))
    em = msgs.get(TOPICS['emergency'], [])
    if em:
        for t in edges([t for t, _ in em], [m.emergency for _, m in em], True):
            out.append(Trigger(t, 'emergency', 'emergency_cmd'))
    state = msgs.get(TOPICS['state'], [])
    board = msgs.get(TOPICS['board'], [])
    if state:
        bt = np.array([t for t, _ in board]) if board else np.zeros(0)
        for t in edges([t for t, _ in state], [m.valid for _, m in state], False):
            # The board was lost one scan after its last detection; the
            # tracker's timeout is part of the reaction being measured.
            k = np.searchsorted(bt, t) - 1
            if k >= 0 and t - bt[k] < 2.0:
                out.append(Trigger(float(bt[k] + scan_period), 'board_drop',
                                   f'tracker invalid +{t - bt[k] - scan_period:.2f} s later'))
            else:
                out.append(Trigger(t, 'board_drop', 'tracker invalid'))

    def stamped(t_rx, header):
        if board_time == 'receive':
            return t_rx
        s = header.stamp.sec + 1e-9 * header.stamp.nanosec
        return s if s > 0.0 else t_rx

    # Board positions are placed at their sample time (the header stamp: the
    # scan's, for a detection), not their arrival, so the detector's latency
    # is not mistaken for the board stopping late. That needs the LiDAR's
    # clock to be the host's; board_time='receive' is the fallback when it is
    # not (and then the reaction reads short by the detector's latency).
    used = 'none'
    truth = msgs.get(TOPICS['truth'], [])
    odom = msgs.get(TOPICS['odom'], [])
    if truth and board_source in ('auto', 'truth'):
        tt = np.array([stamped(t, m.header) for t, m in truth])
        x = np.array([m.pose.position.x for _, m in truth])
        y = np.array([m.pose.position.y for _, m in truth])
        sp = window_speed(tt, x, y)
        used = 'truth (/virtual_coach/truth)'
    elif board and odom and board_source in ('auto', 'detections'):
        bt = np.array([stamped(t, m.header) for t, m in board])
        bxy = np.array([[m.pose.position.x, m.pose.position.y] for _, m in board])
        ot = np.array([stamped(t, m.header) for t, m in odom])
        oxy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in odom])
        oyaw = np.array([_yaw(m.pose.pose.orientation) for _, m in odom])
        x, y = board_in_world(bt, bxy, ot, oxy, oyaw)
        sp = window_speed(bt, x, y, half=0.4)
        tt = bt
        used = 'detections (/perception/coach/board + odometry)'
    else:
        tt, sp = None, None
    if tt is not None:
        for t in board_stop_times(tt, sp):
            out.append(Trigger(t, 'board_stop'))
    return sorted(out, key=lambda x: x.t), used


def _planner_params(path: Optional[str]) -> Dict[str, float]:
    import yaml

    from autosdv_coach_lab.pursuit import PlannerParams
    d = PlannerParams()
    cur = dict(standoff_min=d.standoff_min, v_max=d.v_max,
               envelope_decel=d.envelope_decel, reaction_time=d.reaction_time,
               brake_decel=d.brake_decel)
    if path is None:
        try:
            import os

            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('autosdv_coach_lab'),
                                'config', 'board_pursuit_planner.param.yaml')
        except Exception:  # noqa: BLE001 -- no ament index: defaults
            return cur
    with open(path) as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    cur.update({k: float(p[k]) for k in cur if k in p})
    return cur


def main(argv=None) -> int:
    import argparse

    from .bagio import read_bag

    ap = argparse.ArgumentParser(
        prog='lab2_braking_report',
        description='Reaction time and deceleration per stop, and the envelope_decel / '
                    'reaction_time they support (board_pursuit_planner).')
    ap.add_argument('bag', nargs='+', help='rosbag2 directories (`just lab2 record`)')
    ap.add_argument('--reaction-margin', type=float, default=0.2,
                    help='seconds added to the worst reaction [0.2]')
    ap.add_argument('--decel-margin', type=float, default=0.75,
                    help='factor on the worst effective deceleration [0.75]')
    ap.add_argument('--min-speed', type=float, default=0.3,
                    help='ignore triggers below this steady speed [0.3 m/s]')
    ap.add_argument('--board-source', choices=('auto', 'truth', 'detections', 'none'),
                    default='auto', help='where board stops come from [auto: truth if recorded]')
    ap.add_argument('--board-time', choices=('stamp', 'receive'), default='stamp',
                    help='time board positions by their header stamp (needs the LiDAR '
                         'clock synced to the host) or by bag receive time [stamp]')
    ap.add_argument('--range', type=float, default=2.0, dest='at_range',
                    help='board range for the speed-cap figure [2.0 m]')
    ap.add_argument('--planner-params', help='board_pursuit_planner.param.yaml for the '
                    'current values [the installed one]')
    args = ap.parse_args(argv)

    all_events: List[StopEvent] = []
    all_skipped: List[Skipped] = []
    for path in args.bag:
        msgs = read_bag(path, list(TOPICS.values()))
        speed = msgs[TOPICS['speed']]
        if not speed:
            raise SystemExit(f'{path}: no {TOPICS["speed"]}')
        t0 = speed[0][0]
        trigs, used = triggers_from_bag(msgs, args.board_source,
                                        board_time=args.board_time)
        cmd = msgs[TOPICS['cmd']]
        events, skipped = analyse(
            [t - t0 for t, _ in speed], [m.longitudinal_velocity for _, m in speed],
            [Trigger(x.t - t0, x.kind, x.detail) for x in trigs],
            cmd=([t - t0 for t, _ in cmd], [m.longitudinal.acceleration for _, m in cmd]),
            min_speed=args.min_speed)
        print(f'{path}: {len(trigs)} triggers, board stops from {used}')
        all_events += events
        all_skipped += skipped
    rec = recommend(all_events, args.reaction_margin, args.decel_margin)
    print()
    print(format_report(all_events, all_skipped, rec, _planner_params(args.planner_params),
                        args.at_range))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
