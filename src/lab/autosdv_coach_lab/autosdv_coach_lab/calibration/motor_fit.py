"""Fit the motor plant model (MotorPlantParams) to a recording. ROS-free.

What is fitted, and against what
--------------------------------
autosdv_vehicle_interface.motor_plant.MotorPlant is a first-order response to
the motor PWM beyond a deadband around init_pwm, a brake below it and an
exponential coast inside it:

    deadband             PWM counts either side of init_pwm with no drive
    gain                 steady speed per count beyond the deadband [m/s/count]
    time_constant        first-order lag while driven [s]
    coast_time_constant  speed decay inside the deadband [s]
    brake_decel          deceleration below the deadband [m/s^2]

The input is the PWM the actuator actually wrote
(/autosdv/actuator_node/debug/pwm_values, data[0]) and the output is what
velocity_report published (/vehicle/status/velocity_status). That output is not
the vehicle's speed: velocity_report times edges of 12 wheel markers, runs an
EMA (alpha 0.3) over the edge-to-edge speeds, publishes at 20 Hz and drops to
zero 0.5 s after the last edge. A fit that compared the plant's speed with that
signal directly would put the sensor's lag into time_constant and its
quantisation into the residual. So the fit is a *simulation-error* fit: for a
candidate parameter set it replays the recorded PWM through MotorPlant and
through HallSpeedSensor -- the same classes speed_bench and the course handout
run -- and compares the simulated sensor output with the recorded one at the
recorded report times. The sensor model is in the loop, so its lag and its
quantisation are explained by the model rather than absorbed into the
parameters.

How
---
1. A first pass on the raw signals gives starting values and says which
   regimes the recording excites: steady speed under steady drive gives gain
   and deadband (v_ss = gain * (pwm - init_pwm - deadband)), decay inside the
   deadband gives the coast constant, decay below it the brake.
2. Nelder-Mead on the simulation error refines (gain, time_constant,
   coast_time_constant, brake_decel) in log space, for each integer deadband
   near the first-pass estimate -- PWM is an integer, so the deadband is one
   too -- and keeps the best. A parameter whose regime the recording does not
   excite is held at its prior and reported as not identified.

The result loads straight into MotorPlantParams(**mapping): speed_bench
--plant, the handout's bench, and motor_plant.py's defaults all take it.
"""

import math
from dataclasses import asdict, dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from autosdv_vehicle_interface.motor_plant import (
    HallSpeedSensor, MotorPlant, MotorPlantParams)

FIT_FIELDS = ('deadband', 'gain', 'time_constant', 'coast_time_constant', 'brake_decel')
CONTINUOUS = ('gain', 'time_constant', 'coast_time_constant', 'brake_decel')

#: velocity_report's sensor, as configured on the car (params/velocity_report.yaml).
SENSOR_DEFAULTS = dict(wheel_diameter=0.105, markers_per_rotation=12, alpha=0.3, timeout=0.5)

MOVING = 0.05  # m/s; below this a sample counts as standing still

#: Physical bounds the search is kept inside [lower, upper].
BOUNDS = {
    'gain': (1e-3, 0.2),
    'time_constant': (0.02, 10.0),
    'coast_time_constant': (0.1, 100.0),
    'brake_decel': (0.1, 20.0),
}


@dataclass
class Recording:
    """PWM samples (zero-order hold until the next one) and speed reports."""

    pwm_t: np.ndarray
    pwm: np.ndarray
    v_t: np.ndarray
    v: np.ndarray
    source: str = ''

    def __post_init__(self):
        self.pwm_t = np.asarray(self.pwm_t, dtype=float)
        self.pwm = np.asarray(self.pwm, dtype=float)
        self.v_t = np.asarray(self.v_t, dtype=float)
        self.v = np.asarray(self.v, dtype=float)
        o = np.argsort(self.pwm_t, kind='stable')
        self.pwm_t, self.pwm = self.pwm_t[o], self.pwm[o]
        o = np.argsort(self.v_t, kind='stable')
        self.v_t, self.v = self.v_t[o], self.v[o]
        if len(self.pwm_t) < 2 or len(self.v_t) < 2:
            raise ValueError('a recording needs at least two PWM samples and two speed reports')

    def window(self, start: Optional[float] = None, end: Optional[float] = None) -> 'Recording':
        """The part between start and end, in seconds from the first PWM sample."""
        t0 = self.pwm_t[0]
        lo = -math.inf if start is None else t0 + start
        hi = math.inf if end is None else t0 + end
        a = (self.pwm_t >= lo) & (self.pwm_t <= hi)
        b = (self.v_t >= lo) & (self.v_t <= hi)
        return Recording(self.pwm_t[a], self.pwm[a], self.v_t[b], self.v[b], self.source)

    @property
    def duration(self) -> float:
        return float(self.pwm_t[-1] - self.pwm_t[0])


# ---------------------------------------------------------------------------
# Forward model
# ---------------------------------------------------------------------------

def simulate(params: MotorPlantParams, rec: Recording, substep: float = 0.01,
             sensor: Optional[Dict] = None) -> Tuple[np.ndarray, np.ndarray]:
    """Replay the recorded PWM through MotorPlant and HallSpeedSensor.

    Returns (reported, true) at rec.v_t: what velocity_report would have
    published and the plant's own speed. Reports before the first PWM sample
    are NaN. The plant starts at the first report's speed.
    """
    cfg = {**SENSOR_DEFAULTS, **(sensor or {})}
    sens = HallSpeedSensor(**cfg)
    t = float(rec.pwm_t[0])
    first = int(np.searchsorted(rec.v_t, t))
    v_init = float(rec.v[first]) if first < len(rec.v) else 0.0
    plant = MotorPlant(params, speed=v_init)
    if v_init > 0.0:
        sens.current_velocity = v_init
        sens.last_edge_time = t
    sens._last_time = t

    reported = np.full(len(rec.v_t), np.nan)
    true = np.full(len(rec.v_t), np.nan)
    pwm_t, pwm_v, v_t = rec.pwm_t, rec.pwm, rec.v_t
    n_pwm, n_v = len(pwm_t), len(v_t)
    j = first
    step, upd = plant.step, sens.update

    for k in range(n_pwm):
        pwm = pwm_v[k]
        t_end = pwm_t[k + 1] if k + 1 < n_pwm else pwm_t[k]
        while True:
            t_next = v_t[j] if j < n_v and v_t[j] <= t_end else t_end
            while t < t_next - 1e-12:
                h = min(substep, t_next - t)
                step(pwm, h)
                t += h
                upd(t, plant.distance)
            if j < n_v and v_t[j] <= t_end:
                reported[j] = sens.report(v_t[j])
                true[j] = plant.speed
                j += 1
            else:
                break
    return reported, true


def residuals(params: MotorPlantParams, rec: Recording, **kw) -> np.ndarray:
    pred, _ = simulate(params, rec, **kw)
    r = pred - rec.v
    return r[np.isfinite(r)]


# ---------------------------------------------------------------------------
# First pass: starting values, and which regimes are excited
# ---------------------------------------------------------------------------

def pwm_at(rec: Recording, t: np.ndarray) -> np.ndarray:
    """The PWM in force at each time t (zero-order hold)."""
    i = np.searchsorted(rec.pwm_t, t, side='right') - 1
    return rec.pwm[np.clip(i, 0, len(rec.pwm) - 1)]


@dataclass
class Excitation:
    """How much of each regime the recording contains, by the first pass."""

    drive_s: float = 0.0
    coast_s: float = 0.0
    brake_s: float = 0.0
    steady_levels: List[Tuple[float, float]] = field(default_factory=list)  # (pwm, v)
    drive_pwm_span: float = 0.0   # counts between the 5th and 95th percentile under drive

    def identified(self) -> Dict[str, bool]:
        drive = self.drive_s >= 2.0
        levels = len({round(p) for p, _ in self.steady_levels})
        return {
            # Two settled levels pin the line v = gain * (pwm - init - deadband);
            # so does a transient that sweeps the PWM widely enough.
            'deadband': drive and (levels >= 2 or self.drive_pwm_span >= 15.0),
            'gain': drive,
            'time_constant': drive,
            'coast_time_constant': self.coast_s >= 0.5,
            'brake_decel': self.brake_s >= 0.3,
        }


def _runs(mask: np.ndarray) -> List[Tuple[int, int]]:
    """[start, end) index pairs of the True runs of mask."""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(mask)))
    return out


def steady_levels(rec: Recording, init_pwm: int, deadband: float,
                  window: float = 1.5, v_std_max: float = 0.04) -> List[Tuple[float, float]]:
    """(mean PWM, mean speed) of windows where both have settled under drive.

    Windows are taken back to back, so one long hold yields several samples
    of the same level; they are merged by rounding the PWM.
    """
    t = rec.v_t
    out = []
    i = 0
    while i < len(t):
        j = int(np.searchsorted(t, t[i] + window))
        if j - i >= 5:
            v = rec.v[i:j]
            tt = t[i:j]
            p = pwm_at(rec, tt)
            slope = np.polyfit(tt - tt[0], v, 1)[0] if len(v) > 2 else 0.0
            if (v.min() > 0.1 and v.std() < v_std_max and abs(slope) * window < 0.75 * v_std_max
                    and np.mean(p > init_pwm + deadband) > 0.95):
                out.append((float(p.mean()), float(v.mean())))
        i = j if j > i else i + 1
    merged: Dict[int, List[Tuple[float, float]]] = {}
    for p, v in out:
        merged.setdefault(int(round(p)), []).append((p, v))
    return [(float(np.mean([a for a, _ in g])), float(np.mean([b for _, b in g])))
            for g in merged.values()]


def first_pass(rec: Recording, prior: MotorPlantParams) -> Tuple[MotorPlantParams, Excitation]:
    """Starting values from the raw signals, and the excitation summary."""
    init = prior.init_pwm
    est = replace(prior)
    exc = Excitation()

    levels = steady_levels(rec, init, prior.deadband)
    exc.steady_levels = levels
    if len({round(p) for p, _ in levels}) >= 2:
        p = np.array([a for a, _ in levels]) - init
        v = np.array([b for _, b in levels])
        g, c = np.polyfit(p, v, 1)
        if g > 1e-4:
            est.gain = float(g)
            est.deadband = int(min(max(round(-c / g), 0), 30))
    elif len(levels) == 1:
        p, v = levels[0]
        if p - init - prior.deadband > 0.5:
            est.gain = float(v / (p - init - prior.deadband))

    db = est.deadband
    dt = np.diff(rec.v_t, append=rec.v_t[-1])
    p_v = pwm_at(rec, rec.v_t) - init
    moving = rec.v > 0.1
    drive = p_v > db
    coast = (np.abs(p_v) <= db) & moving
    # A report equal to the previous one saw no new wheel edge: the sensor is
    # holding, typically for 0.5 s at the end of a stop. It says nothing
    # about the deceleration.
    fresh = np.append(True, np.diff(rec.v) != 0.0)
    brake = (p_v < -db) & (rec.v > 0.2) & fresh
    exc.drive_s = float(dt[drive].sum())
    if drive.any():
        lo, hi = np.percentile(p_v[drive], [5, 95])
        exc.drive_pwm_span = float(hi - lo)
    exc.coast_s = float(dt[coast].sum())
    exc.brake_s = float(dt[brake].sum())

    # Coast: log-linear decay over runs inside the deadband.
    rates = []
    for a, b in _runs(coast):
        if b - a >= 4:
            s = np.polyfit(rec.v_t[a:b] - rec.v_t[a], np.log(rec.v[a:b]), 1)[0]
            if s < -1e-3:
                rates.append((b - a, -s))
    if rates:
        w = np.array([n for n, _ in rates], dtype=float)
        est.coast_time_constant = float(1.0 / np.average([r for _, r in rates], weights=w))

    # Brake: linear decay over runs below the deadband.
    slopes = []
    for a, b in _runs(brake):
        if b - a >= 3:
            s = np.polyfit(rec.v_t[a:b] - rec.v_t[a], rec.v[a:b], 1)[0]
            if s < -0.05:
                slopes.append((b - a, -s))
    if slopes:
        w = np.array([n for n, _ in slopes], dtype=float)
        est.brake_decel = float(np.average([s for _, s in slopes], weights=w))
    return est, exc


# ---------------------------------------------------------------------------
# Refinement
# ---------------------------------------------------------------------------

def nelder_mead(f, x0: Sequence[float], step: float = 0.2, max_iter: int = 400,
                ftol: float = 1e-7, xtol: float = 1e-4) -> Tuple[np.ndarray, float]:
    """Minimise f from x0. Plain Nelder-Mead; scipy is avoided on purpose (a
    user-site numpy can break the system scipy, and this must run on the car)."""
    n = len(x0)
    pts = [np.asarray(x0, dtype=float)]
    for i in range(n):
        x = pts[0].copy()
        x[i] += step
        pts.append(x)
    vals = [f(x) for x in pts]
    for _ in range(max_iter):
        order = np.argsort(vals)
        pts = [pts[i] for i in order]
        vals = [vals[i] for i in order]
        if (abs(vals[-1] - vals[0]) <= ftol * (abs(vals[0]) + 1e-12)
                and max(np.max(np.abs(p - pts[0])) for p in pts[1:]) <= xtol):
            break
        centroid = np.mean(pts[:-1], axis=0)
        xr = centroid + (centroid - pts[-1])
        fr = f(xr)
        if fr < vals[0]:
            xe = centroid + 2.0 * (centroid - pts[-1])
            fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = centroid + 0.5 * (pts[-1] - centroid)
            fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                for i in range(1, len(pts)):
                    pts[i] = pts[0] + 0.5 * (pts[i] - pts[0])
                    vals[i] = f(pts[i])
    i = int(np.argmin(vals))
    return pts[i], float(vals[i])


@dataclass
class FitResult:
    params: MotorPlantParams
    prior: MotorPlantParams
    start: MotorPlantParams
    identified: Dict[str, bool]
    excitation: Excitation
    rmse_fit: float           # the objective: stop tails faded out [m/s]
    rmse: float               # all reports [m/s]
    rmse_moving: float        # reports where either signal is above MOVING
    rmse_by_regime: Dict[str, float]
    rmse_prior: float         # the prior's moving RMSE, for comparison
    deadband_profile: Dict[int, float]  # objective RMSE per deadband tried
    # Per fitted parameter, the range over which the objective RMSE stays
    # within PROFILE_RISE of its minimum with the others held: how tightly the
    # recording pins it down. A wide range is a weakly determined parameter.
    intervals: Dict[str, Tuple[float, float]]
    # Fitted but weakly determined: the interval spans more than WEAK_RATIO,
    # or the value sits on a search bound. Use with care, or record more of
    # that regime.
    weak: List[str]
    n_reports: int
    duration: float
    evaluations: int

    def plant_mapping(self) -> Dict:
        """The MotorPlantParams fields, rounded for a config file."""
        p = asdict(self.params)
        p['deadband'] = int(p['deadband'])
        for k in CONTINUOUS:
            p[k] = float(f'{p[k]:.4g}')
        return p


#: An interval wider than this ratio (hi / lo) marks a parameter weak.
WEAK_RATIO = 1.5

#: Rise of the objective RMSE that bounds a parameter's interval: relative,
#: with an absolute floor [m/s] for a recording with almost no noise.
PROFILE_RISE = 0.05
PROFILE_FLOOR = 0.002


#: Below this simulated speed a report's weight in the objective fades to zero.
STOP_FADE = (0.1, 0.2)


def objective(pred: np.ndarray, true: np.ndarray, meas: np.ndarray) -> float:
    """Weighted mean square simulation error.

    Reports taken while the simulated vehicle is (nearly) stopped are faded
    out. Coming to a stop, velocity_report holds the EMA of the last edges for
    0.5 s and then drops to zero; which value it holds depends on the phase of
    the last few edges, so the stopping tail jumps by a decimetre per second
    for a parameter change of a fraction of a percent. The weight fades with
    the *simulated* speed, continuously, so no report enters or leaves the
    objective with a jump. Above STOP_FADE[1] every report counts in full.
    """
    lo, hi = STOP_FADE
    w = np.clip((np.nan_to_num(true) - lo) / (hi - lo), 0.0, 1.0)
    r = pred - meas
    ok = np.isfinite(r)
    if not ok.any() or w[ok].sum() <= 0:
        return float('inf')
    return float(np.sum(w[ok] * r[ok] ** 2) / np.sum(w[ok]))


def _moving_rmse(r: np.ndarray, pred: np.ndarray, meas: np.ndarray) -> float:
    m = np.isfinite(r) & ((pred > MOVING) | (meas > MOVING))
    return float(np.sqrt(np.mean(r[m] ** 2))) if m.any() else float('nan')


def fit(rec: Recording, prior: Optional[MotorPlantParams] = None,
        deadband_max: int = 15, substep: float = 0.01, sensor: Optional[Dict] = None,
        max_iter: int = 300, stage_a_iter: int = 80) -> FitResult:
    """Fit MotorPlantParams to rec; see the module docstring.

    Two stages. A: for every integer deadband 0..deadband_max, a short
    search of the continuous parameters -- gain and deadband trade off along
    a ridge, and a scan finds the right end of it where a local search would
    not. B: a longer search from the best of A and its two neighbours.
    """
    prior = prior or MotorPlantParams()
    start, exc = first_pass(rec, prior)
    ident = exc.identified()
    evals = [0]

    def make(db: int, names, x: np.ndarray, base: MotorPlantParams) -> MotorPlantParams:
        values = {k: float(min(max(math.exp(min(v, 5.0)), BOUNDS[k][0]), BOUNDS[k][1]))
                  for k, v in zip(names, x)}
        return replace(base, deadband=int(db), **values)

    def cost(pars: MotorPlantParams) -> float:
        evals[0] += 1
        pred, true = simulate(pars, rec, substep=substep, sensor=sensor)
        return objective(pred, true, rec.v)

    def search(db: int, names, base: MotorPlantParams, iters: int):
        if not names:
            return base, cost(replace(base, deadband=int(db)))
        x0 = np.log([getattr(base, k) for k in names])
        x, c = nelder_mead(lambda x: cost(make(db, names, x, base)), x0, max_iter=iters)
        # One restart from the result: Nelder-Mead stalls on a collapsed simplex.
        x, c = nelder_mead(lambda x: cost(make(db, names, x, base)), x, step=0.05,
                           max_iter=iters)
        return make(db, names, x, base), c

    free = [k for k in CONTINUOUS if ident[k]]
    candidates = range(0, deadband_max + 1) if ident['deadband'] else [start.deadband]

    stage_a: Dict[int, Tuple[MotorPlantParams, float]] = {}
    for db in candidates:
        stage_a[db] = search(db, free, start, stage_a_iter)
    ranked = sorted(stage_a, key=lambda db: stage_a[db][1])
    best_db = ranked[0]
    finalists = [db for db in (best_db - 1, best_db, best_db + 1) if db in stage_a]

    profile: Dict[int, float] = {db: math.sqrt(c) for db, (_, c) in stage_a.items()}
    best = None
    for db in finalists:
        pars, c = search(db, free, stage_a[db][0], max_iter)
        profile[db] = math.sqrt(c)
        if best is None or c < best[1]:
            best = (pars, c)
    params, c_best = best

    # The brake and coast constants act only in their own short regimes,
    # where the hall sensor's quantisation ripples the objective at the mm/s
    # level; a local search stalls in a ripple. A grid over each, then one
    # more local pass from the best point, finds the floor.
    def grid(k, base, lo=1.0 / 3.0, hi=3.0, n=41):
        x = getattr(base, k)
        values = x * np.exp(np.linspace(math.log(lo), math.log(hi), n))
        values = np.clip(values, *BOUNDS[k])
        return [(float(v), cost(replace(base, **{k: float(v)}))) for v in values]

    rescanned = False
    for k in ('brake_decel', 'coast_time_constant'):
        if k in free:
            v, c = min(grid(k, params), key=lambda vc: vc[1])
            if c < c_best:
                params, c_best, rescanned = replace(params, **{k: v}), c, True
    if rescanned:
        polished, c = search(params.deadband, free, params, max_iter)
        if c < c_best:
            params, c_best = polished, c
    profile[params.deadband] = math.sqrt(c_best)

    limit = max(math.sqrt(c_best) * (1.0 + PROFILE_RISE), math.sqrt(c_best) + PROFILE_FLOOR)
    intervals: Dict[str, Tuple[float, float]] = {}
    for k in free:
        samples = (grid(k, params) + grid(k, params, 0.9, 1.0 / 0.9, 21)
                   + grid(k, params, 0.97, 1.0 / 0.97, 31))
        inside = [getattr(params, k)] + [v for v, c in samples if math.sqrt(c) <= limit]
        intervals[k] = (float(min(inside)), float(max(inside)))
    dbs = [db for db, v in profile.items() if v <= limit] or [params.deadband]
    intervals['deadband'] = (float(min(dbs)), float(max(dbs)))
    weak = [k for k in free
            if intervals[k][1] > WEAK_RATIO * intervals[k][0]
            or any(abs(getattr(params, k) / b - 1.0) < 0.02 for b in BOUNDS[k])]
    if ident['deadband'] and intervals['deadband'][1] - intervals['deadband'][0] > 2:
        weak.append('deadband')

    pred, _ = simulate(params, rec, substep=substep, sensor=sensor)
    r = pred - rec.v
    ok = np.isfinite(r)
    p_v = pwm_at(rec, rec.v_t) - params.init_pwm
    regimes = {
        'drive': (p_v > params.deadband),
        'coast': (np.abs(p_v) <= params.deadband) & (rec.v > MOVING),
        'brake': (p_v < -params.deadband) & (rec.v > MOVING),
    }
    by_regime = {k: (float(np.sqrt(np.mean(r[m & ok] ** 2))) if (m & ok).any() else float('nan'))
                 for k, m in regimes.items()}
    pred0, _ = simulate(prior, rec, substep=substep, sensor=sensor)
    return FitResult(
        params=params, prior=prior, start=start, identified=ident, excitation=exc,
        rmse_fit=math.sqrt(c_best),
        rmse=float(np.sqrt(np.mean(r[ok] ** 2))),
        rmse_moving=_moving_rmse(r, pred, rec.v),
        rmse_by_regime=by_regime,
        rmse_prior=_moving_rmse(pred0 - rec.v, pred0, rec.v),
        deadband_profile=profile, intervals=intervals, weak=weak,
        n_reports=int(ok.sum()), duration=rec.duration, evaluations=evals[0])


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def to_yaml(res: FitResult, source: str = '') -> str:
    """A YAML mapping MotorPlantParams(**yaml.safe_load(text)) accepts, with the
    fit quality as comments."""
    m = res.plant_mapping()
    lines = [
        '# MotorPlantParams fitted by lab2_fit_motor'
        + (f' from {source}' if source else ''),
        f'# {res.n_reports} speed reports over {res.duration:.1f} s; residual RMSE '
        f'{res.rmse_fit:.3f} m/s (stop tails faded), {res.rmse_moving:.3f} m/s over every '
        f'moving report; the provisional prior: {res.rmse_prior:.3f} m/s',
    ]
    not_ident = [k for k in FIT_FIELDS if not res.identified[k]]
    if not_ident:
        lines.append('# NOT identified by this recording, held at the prior: '
                     + ', '.join(not_ident))
    if res.weak:
        lines.append('# WEAKLY determined (see the +5% RMSE range in the report): '
                     + ', '.join(res.weak))
    for k in ('init_pwm',) + FIT_FIELDS + ('max_speed', 'allow_reverse'):
        v = m[k]
        if isinstance(v, bool):
            v = 'true' if v else 'false'
        note = ''
        if k in FIT_FIELDS and not res.identified[k]:
            note = '  # prior, not fitted'
        elif k in res.weak:
            lo, hi = res.intervals[k]
            note = f'  # weak: {lo:.3g} .. {hi:.3g}'
        lines.append(f'{k}: {v}{note}')
    return '\n'.join(lines) + '\n'


def flow_mapping(res: FitResult) -> str:
    """The one-line form for speed_bench --plant '{...}'."""
    m = res.plant_mapping()
    return '{' + ', '.join(f'{k}: {m[k]}' for k in FIT_FIELDS) + '}'


def _status(res: FitResult, k: str) -> str:
    if not res.identified[k]:
        return 'NO (prior)'
    return 'weak' if k in res.weak else 'yes'


def report(res: FitResult) -> str:
    p, s, pr = res.params, res.start, res.prior
    rows = [f'{"parameter":<21}{"prior":>9}{"first pass":>12}{"fit":>9}'
            f'{"+%d%% RMSE range" % round(100 * PROFILE_RISE):>21}  identified']
    for k in FIT_FIELDS:
        fmt = (lambda v: f'{int(v)}') if k == 'deadband' else (lambda v: f'{v:.4g}')
        iv = res.intervals.get(k)
        span = f'{fmt(iv[0])} .. {fmt(iv[1])}' if iv and res.identified[k] else '-'
        rows.append(f'{k:<21}{fmt(getattr(pr, k)):>9}{fmt(getattr(s, k)):>12}'
                    f'{fmt(getattr(p, k)):>9}{span:>21}  '
                    f'{_status(res, k)}')
    e = res.excitation
    lv = ', '.join(f'{pw:.1f}->{v:.2f}' for pw, v in sorted(e.steady_levels)) or 'none'
    rows += [
        '',
        f'recording        {res.duration:.1f} s, {res.n_reports} speed reports',
        f'excitation       drive {e.drive_s:.1f} s, coast {e.coast_s:.1f} s, '
        f'brake {e.brake_s:.1f} s while moving',
        f'steady levels    {lv}  (PWM -> m/s)',
        f'residual RMSE    {res.rmse_fit:.4f} m/s fit objective (stop tails faded); '
        f'{res.rmse_moving:.4f} m/s over moving reports, {res.rmse:.4f} m/s overall '
        f'(prior: {res.rmse_prior:.4f} m/s moving)',
        '  by regime      ' + ', '.join(f'{k} {v:.4f}' for k, v in res.rmse_by_regime.items()),
        '  per deadband   ' + ', '.join(
            f'{k}: {v:.4f}' for k, v in sorted(res.deadband_profile.items())),
        f'evaluations      {res.evaluations}',
    ]
    return '\n'.join(rows)


# ---------------------------------------------------------------------------
# Synthetic recordings: the vehicle loop with a known plant
# ---------------------------------------------------------------------------

class JitteryHallSensor(HallSpeedSensor):
    """HallSpeedSensor with unevenly spaced markers: the edges fall at
    irregular wheel angles but velocity_report still divides by the nominal
    spacing, as on a real wheel with hand-placed magnets."""

    def __init__(self, jitter: float = 0.0, seed: int = 0, **kw):
        super().__init__(**kw)
        rng = np.random.default_rng(seed)
        n = int(kw.get('markers_per_rotation', 12))
        offsets = rng.uniform(-jitter, jitter, n) * self.distance_per_marker
        offsets -= offsets.mean()
        self._spacing = self.distance_per_marker + np.diff(np.append(offsets, offsets[0]))
        self._k = 0
        self._next_edge_distance = float(self._spacing[0])

    def update(self, now: float, distance: float) -> None:
        while distance >= self._next_edge_distance:
            span = distance - self._last_distance
            frac = (self._next_edge_distance - self._last_distance) / span if span > 0 else 1.0
            self._on_edge(self._last_time + frac * (now - self._last_time))
            self._k = (self._k + 1) % len(self._spacing)
            self._next_edge_distance += float(self._spacing[self._k])
        self._last_time = now
        self._last_distance = distance


@dataclass
class SynthNoise:
    report_noise: float = 0.0     # m/s, Gaussian on each non-zero report
    process_noise: float = 0.0    # m/s/sqrt(s), random-walk disturbance on the speed
    marker_jitter: float = 0.0    # fraction of the marker spacing
    report_jitter: float = 0.0    # s, uniform jitter of the 20 Hz report times
    seed: int = 0


def synthesize(params: MotorPlantParams, speeds: Sequence[float], hold: float = 4.0,
               noise: Optional[SynthNoise] = None, accel: float = 0.5,
               controller_class: str = '', controller_overrides: Optional[Dict] = None,
               lead_in: float = 1.0) -> Tuple[Recording, Dict[str, np.ndarray]]:
    """The speed step test on a known plant, through the actuator's own loop.

    The target steps through `speeds` (each held `hold` s, rate-limited at
    `accel`, as the trajectory follower would), the actuator's
    LongitudinalShell and speed controller close the loop at 100 Hz on a 20 Hz
    velocity report, and MotorPlant is the vehicle. Returns the Recording the
    car would produce and the truth trace (t, speed, target, pwm, mode).
    """
    from autosdv_vehicle_interface.speed_bench import (
        DEFAULT_ACTUATOR_PARAMS, controller_params, make_shell)
    from autosdv_vehicle_interface.speed_controller import load_speed_controller

    from .speed_steps import StepSchedule

    noise = noise or SynthNoise()
    rng = np.random.default_rng(noise.seed)
    act = dict(DEFAULT_ACTUATOR_PARAMS)
    act['init_pwm'] = params.init_pwm
    ctrl = load_speed_controller(controller_class, controller_params(act, controller_overrides))
    shell = make_shell(ctrl, act)
    plant = MotorPlant(params)
    sensor = JitteryHallSensor(jitter=noise.marker_jitter, seed=noise.seed, **SENSOR_DEFAULTS)
    sched = StepSchedule.build(speeds, hold)

    tick = 1.0 / act['rate']
    total = lead_in + sched.duration + 3.0
    n = int(round(total / tick))
    n_sub = 10
    report_period = 0.05
    next_report = 0.0
    measured = None
    last_vt = 0.0
    target = 0.0
    pwm = act['init_pwm']
    pwm_t, pwm_v, v_t, v_v = [], [], [], []
    truth = {k: [] for k in ('t', 'speed', 'target', 'pwm', 'mode')}
    for k in range(n):
        t = k * tick
        if k > 0:
            h = tick / n_sub
            for i in range(n_sub):
                plant.step(pwm, h)
                if noise.process_noise > 0.0 and plant.speed > 0.0:
                    plant.speed = max(0.0, plant.speed
                                      + noise.process_noise * math.sqrt(h) * rng.normal())
                sensor.update(t - tick + (i + 1) * h, plant.distance)
        if t + 1e-9 >= next_report:
            tr = t + (rng.uniform(-noise.report_jitter, noise.report_jitter)
                      if noise.report_jitter > 0 else 0.0)
            v = sensor.report(t)
            if v > 0.0 and noise.report_noise > 0.0:
                v = max(0.0, v + noise.report_noise * rng.normal())
            measured, last_vt = v, t
            v_t.append(tr)
            v_v.append(v)
            next_report += report_period
        goal, _ = sched.target(t - lead_in)
        if goal > target:
            target = min(goal, target + accel * tick)
        else:
            target = max(goal, target - 2.0 * accel * tick)
        pwm = shell.step(t, target, measured, last_vt)
        pwm_t.append(t)
        pwm_v.append(pwm)
        for key, val in (('t', t), ('speed', plant.speed), ('target', target),
                         ('pwm', pwm), ('mode', shell.mode.value)):
            truth[key].append(val)
    rec = Recording(np.array(pwm_t), np.array(pwm_v), np.array(v_t), np.array(v_v), 'synthetic')
    return rec, {k: np.array(v) for k, v in truth.items()}


def write_csv(path: str, rec: Recording) -> None:
    """t,pwm,speed rows; speed empty where a row is a PWM sample only."""
    rows = [(t, f'{p:.0f}', '') for t, p in zip(rec.pwm_t, rec.pwm)]
    rows += [(t, '', f'{v:.6f}') for t, v in zip(rec.v_t, rec.v)]
    rows.sort(key=lambda r: r[0])
    with open(path, 'w') as f:
        f.write('t,pwm,speed\n')
        for t, p, v in rows:
            f.write(f'{t:.6f},{p},{v}\n')


def read_csv(path: str) -> Recording:
    pwm_t, pwm, v_t, v = [], [], [], []
    with open(path) as f:
        header = f.readline().strip().split(',')
        ti, pi, si = header.index('t'), header.index('pwm'), header.index('speed')
        for line in f:
            cols = line.strip().split(',')
            if len(cols) < len(header):
                continue
            t = float(cols[ti])
            if cols[pi]:
                pwm_t.append(t)
                pwm.append(float(cols[pi]))
            if cols[si]:
                v_t.append(t)
                v.append(float(cols[si]))
    return Recording(np.array(pwm_t), np.array(pwm), np.array(v_t), np.array(v), path)


# ---------------------------------------------------------------------------
# Bags and the command line
# ---------------------------------------------------------------------------

PWM_TOPIC = '/autosdv/actuator_node/debug/pwm_values'
SPEED_TOPIC = '/vehicle/status/velocity_status'
MODE_TOPIC = '/autosdv/actuator_node/debug/mode'


def recording_from_bag(path: str, pwm_topic: str = PWM_TOPIC,
                       speed_topic: str = SPEED_TOPIC) -> Recording:
    """PWM from the actuator's debug array (data[0], its own stamp) and speed
    from velocity_status (header stamp); the bag's receive time where a stamp
    is zero."""
    from .bagio import read_bag, stamp_sec
    msgs = read_bag(path, [pwm_topic, speed_topic])
    if not msgs[pwm_topic]:
        raise ValueError(
            f'{path} has no {pwm_topic}: record with `just lab2 record-cal` and '
            'enable_debug_publishing: true in actuator.yaml')
    if not msgs[speed_topic]:
        raise ValueError(f'{path} has no {speed_topic}')

    def when(t_rx, stamp):
        s = stamp_sec(stamp)
        return s if s > 0.0 else t_rx

    pwm_t = [when(t, m.stamp) for t, m in msgs[pwm_topic]]
    pwm = [m.data[0] for _, m in msgs[pwm_topic]]
    v_t = [when(t, m.header.stamp) for t, m in msgs[speed_topic]]
    v = [m.longitudinal_velocity for _, m in msgs[speed_topic]]
    return Recording(np.array(pwm_t), np.array(pwm), np.array(v_t), np.array(v), path)


def write_synthetic_bag(path: str, rec: Recording, t0: float = 1.7e9) -> None:
    """The recording as the car would have logged it: the actuator's debug
    array and velocity_status, stamped and received at the same times."""
    from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped
    from autoware_vehicle_msgs.msg import VelocityReport
    from builtin_interfaces.msg import Time

    from .bagio import write_bag

    def stamp(t):
        s = Time()
        s.sec = int(math.floor(t))
        s.nanosec = int(round((t - math.floor(t)) * 1e9)) % 1000000000
        return s

    out = []
    for t, p in zip(rec.pwm_t, rec.pwm):
        m = Float32MultiArrayStamped()
        m.stamp = stamp(t0 + t)
        m.data = [float(p), 489.0]
        out.append((PWM_TOPIC, t0 + t, m))
    for t, v in zip(rec.v_t, rec.v):
        m = VelocityReport()
        m.header.stamp = stamp(t0 + t)
        m.header.frame_id = 'base_link'
        m.longitudinal_velocity = float(v)
        out.append((SPEED_TOPIC, t0 + t, m))
    out.sort(key=lambda x: x[1])
    write_bag(path, {
        PWM_TOPIC: 'autoware_internal_debug_msgs/msg/Float32MultiArrayStamped',
        SPEED_TOPIC: 'autoware_vehicle_msgs/msg/VelocityReport',
    }, out)


def _mapping(text: str) -> Dict:
    import yaml
    value = yaml.safe_load(text) if text else {}
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"expected a mapping such as '{{gain: 0.025}}', got {text!r}")
    return value


def main(argv=None) -> int:
    import argparse
    import os
    import sys
    import time

    ap = argparse.ArgumentParser(
        prog='lab2_fit_motor',
        description='Fit MotorPlantParams (motor_plant.py) to a recording of the motor '
                    'PWM and the wheel speed: a bag from `just lab2 record-cal` during '
                    '`just lab2 speed-steps`, or a t,pwm,speed CSV.')
    ap.add_argument('recording', nargs='?', help='rosbag2 directory or .csv')
    ap.add_argument('-o', '--out', help='write the fitted MotorPlantParams YAML here')
    ap.add_argument('--pwm-topic', default=PWM_TOPIC)
    ap.add_argument('--speed-topic', default=SPEED_TOPIC)
    ap.add_argument('--start', type=float, help='ignore the first START s')
    ap.add_argument('--end', type=float, help='ignore everything after END s')
    ap.add_argument('--prior', type=_mapping, default={},
                    help="MotorPlantParams to start from and to hold unidentified "
                         "parameters at, e.g. '{init_pwm: 370}'")
    ap.add_argument('--deadband-max', type=int, default=15)
    ap.add_argument('--trace', help='write t,measured,fitted,plant_speed,pwm CSV here')
    ap.add_argument('--synthesize', metavar='OUT',
                    help='instead of fitting: run the speed step test on a known plant '
                         '(--truth) through the actuator loop and write OUT (a bag '
                         'directory, or .csv), for validating the fit')
    ap.add_argument('--truth', type=_mapping, default={},
                    help='plant for --synthesize, MotorPlantParams overrides')
    ap.add_argument('--steps', default='0,0.5,1.0,0.5,1.5,0',
                    help='speed steps for --synthesize, comma separated')
    ap.add_argument('--hold', type=float, default=4.0)
    ap.add_argument('--noise', type=_mapping,
                    default={'report_noise': 0.02, 'process_noise': 0.02,
                             'marker_jitter': 0.05, 'report_jitter': 0.002},
                    help='SynthNoise for --synthesize')
    args = ap.parse_args(argv)

    if args.synthesize:
        truth = MotorPlantParams(**args.truth)
        speeds = [float(x) for x in args.steps.split(',') if x.strip()]
        rec, _ = synthesize(truth, speeds, args.hold, SynthNoise(**args.noise))
        if args.synthesize.endswith('.csv'):
            write_csv(args.synthesize, rec)
        else:
            write_synthetic_bag(args.synthesize, rec)
        print(f'wrote {args.synthesize}: {rec.duration:.1f} s, truth {asdict(truth)}')
        return 0

    if not args.recording:
        ap.error('a recording is required (or --synthesize OUT)')
    if args.recording.endswith('.csv'):
        rec = read_csv(args.recording)
    else:
        rec = recording_from_bag(args.recording, args.pwm_topic, args.speed_topic)
    if args.start is not None or args.end is not None:
        rec = rec.window(args.start, args.end)
    prior = MotorPlantParams(**args.prior)

    t = time.monotonic()
    res = fit(rec, prior, deadband_max=args.deadband_max)
    print(report(res))
    print(f'fit time         {time.monotonic() - t:.1f} s')
    print()
    text = to_yaml(res, os.path.basename(os.path.normpath(args.recording)))
    print(text, end='')
    print()
    print('one line, for speed_bench --plant (AutoSDV or the course handout):')
    print(f"  --plant '{flow_mapping(res)}'")
    if args.out:
        with open(args.out, 'w') as f:
            f.write(text)
        print(f'wrote {args.out}')
    if args.trace:
        pred, true = simulate(res.params, rec)
        p = pwm_at(rec, rec.v_t)
        with open(args.trace, 'w') as f:
            f.write('t,measured,fitted,plant_speed,pwm\n')
            for row in zip(rec.v_t - rec.pwm_t[0], rec.v, pred, true, p):
                f.write(','.join(f'{x:.5f}' for x in row) + '\n')
        print(f'wrote {args.trace}')
    bad = [k for k in FIT_FIELDS if not res.identified[k]]
    if bad:
        print(f'NOTE: not identified, held at the prior: {", ".join(bad)}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
