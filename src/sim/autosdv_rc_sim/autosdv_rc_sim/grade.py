"""Grade a Lab 2 Part 1 submission in the lockstep simulator: deterministic and headless.

    rc_sim_grade DIR                       every model variant, both scenarios; marks out of 15
    rc_sim_grade DIR --variant nominal     one variant (repeatable)
    rc_sim_grade DIR --jobs 4              variants in parallel (same result)
    rc_sim_grade DIR --print-params        the speed_controller_params mapping (for just sim rc)

DIR holds speed_controller_params.yaml (or config/speed_controller_params.yaml):
a flat mapping of the coefficients in ALLOWED_KEYS, and nothing else. The
coefficients go to the car's AckermannSpeedPID exactly as the actuator's
speed_controller_params would pass them. See docs/guides/rc-sim.md.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
from typing import Dict

import yaml

from autosdv_rc_sim.detmath import sqrt
from autosdv_rc_sim.lockstep import (
    EstopScenario, load_actuator_params, Lockstep, ProfileScenario, Trace)
from autosdv_rc_sim.params import load_vehicle, variant_names
from autosdv_rc_sim.track import default_track_dir, load_track, TrackProfile

SUBMISSION_FILE = 'speed_controller_params.yaml'
ALLOWED_KEYS = {
    'kp_speed': float,
    'ki_speed': float,
    'kd_speed': float,
    'integral_limit': float,
    'enable_conditional_integration': bool,
    'derivative_filter_alpha': float,
    'pwm_output_alpha': float,   # read by the actuator but not applied (docs/guides/rc-sim.md)
    'max_pwm_offset': int,
}
# (low, high]: low exclusive for the alphas and the limits, inclusive elsewhere
RANGES = {
    'kp_speed': (0.0, 1000.0), 'ki_speed': (0.0, 1000.0), 'kd_speed': (0.0, 100.0),
    'integral_limit': (0.0, 1000.0), 'derivative_filter_alpha': (0.0, 1.0),
    'pwm_output_alpha': (0.0, 1.0), 'max_pwm_offset': (1, 100),
}
_EXCLUSIVE_LOW = {'integral_limit', 'derivative_filter_alpha', 'pwm_output_alpha'}
SCENARIOS = ('profile', 'estop')
RESUME_SPEED = 6.0
LIMITS = {
    'hold_rmse_rel': 0.05,         # m/s per m/s, floored at hold_rmse_floor
    'hold_rmse_floor': 0.10,       # m/s
    'overshoot_pct': 11.0,         # % of the step (calibrated: Task 16)
    'reach_s': 1.0,                # s
    'launch_slip': 0.5,            # m/s, physical
    'launch_accel': 4.8,           # m/s^2, over 50 ms (calibrated: Task 16)
    'stop_error': 0.5,             # m from the stop line, not calibrated
    'brake_kappa': -0.2,           # physical: past the tyre's peak
    'reverse_speed': -0.05,        # m/s, physical
    'resume_overshoot_pct': 10.0,  # % of the resume speed
    'resume_slip': 0.5,            # m/s, physical
}
PHYSICAL_LIMITS = ('launch_slip', 'brake_kappa', 'reverse_speed', 'resume_slip')
# The check each physical limit decides (calibrate.py refuses a reference that fails one)
PHYSICAL_CHECKS = {'launch_slip': 'launch_slip', 'brake_kappa': 'brake_lock',
                   'reverse_speed': 'estop_no_reverse', 'resume_slip': 'estop_resume_slip'}
# A lurch is the acceleration over this window, not the per-millisecond contact impulses
LAUNCH_ACCEL_WINDOW = 0.05  # [s]


class SubmissionError(ValueError):
    """A submission the grader refuses."""


def load_submission(path) -> dict:
    path = Path(path)
    if path.is_dir():
        for candidate in (path / SUBMISSION_FILE, path / 'config' / SUBMISSION_FILE):
            if candidate.is_file():
                path = candidate
                break
        else:
            raise SubmissionError(f'{path}: no {SUBMISSION_FILE} (or config/{SUBMISSION_FILE})')
    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise SubmissionError(f'{path}: expected a mapping of coefficients')
    unknown = sorted(set(data) - set(ALLOWED_KEYS))
    if unknown:
        raise SubmissionError(f'{path}: not tunable: {unknown}. Allowed: {sorted(ALLOWED_KEYS)}')
    params = {}
    for key, value in data.items():
        kind = ALLOWED_KEYS[key]
        if kind is bool:
            if not isinstance(value, bool):
                raise SubmissionError(f'{key} must be true or false, got {value!r}')
        elif kind is int:
            if isinstance(value, bool) or not isinstance(value, int):
                raise SubmissionError(f'{key} must be an integer, got {value!r}')
        else:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise SubmissionError(f'{key} must be a finite number, got {value!r}')
            value = float(value)
        if key in RANGES:
            low, high = RANGES[key]
            too_low = value <= low if key in _EXCLUSIVE_LOW else value < low
            if too_low or value > high:
                raise SubmissionError(f'{key} = {value} is outside its range ({low}, {high}]')
        params[key] = value
    return params


def flow_mapping(params: dict) -> str:
    return '{' + ', '.join(f'{k}: {v}' for k, v in params.items()) + '}'


# ---------------------------------------------------------------- metrics

def _segments(profile: TrackProfile, kind: str):
    return [seg for seg in profile.segments if seg.kind == kind]


def hold_metrics(trace: Trace, profile: TrackProfile) -> Dict[str, dict]:
    t, s, v = trace.column('t'), trace.column('s'), trace.column('speed')
    out, previous = {}, 0.0
    for acc, hold in zip(_segments(profile, 'accel'), _segments(profile, 'hold')):
        speed = hold.v0
        key = f'{speed:g}'
        idx = [i for i, x in enumerate(s) if hold.s0 <= x < hold.s1]
        if not idx:
            out[key] = {'rmse': None, 'overshoot_pct': None, 'reach_s': None}
            previous = speed
            continue
        t0 = t[idx[0]]
        steady = [v[i] - speed for i in idx if t[i] >= t0 + 1.0]
        rmse = sqrt(sum(e * e for e in steady) / len(steady)) if steady else None
        overshoot = max(0.0, (max(v[i] for i in idx) - speed) / (speed - previous) * 100.0)
        level = speed - 0.1 * (speed - previous)
        start = next(i for i, x in enumerate(s) if x >= acc.s0)
        hit = next((i for i in range(start, len(t)) if v[i] >= level), None)
        out[key] = {'rmse': rmse, 'overshoot_pct': overshoot,
                    'reach_s': None if hit is None else max(0.0, t[hit] - t0)}
        previous = speed
    return out


def launch_metrics(trace: Trace, profile: TrackProfile) -> dict:
    t, s = trace.column('t'), trace.column('s')
    first_hold = _segments(profile, 'hold')[0]
    t_hold = next((ti for ti, x in zip(t, s) if x >= first_hold.s0), t[-1])
    window = [i for i, ti in enumerate(t) if ti <= t_hold + 1.0]
    slip, v = trace.column('slip_r'), trace.column('speed')
    # Mean acceleration over LAUNCH_ACCEL_WINDOW (a box filter on the true speed)
    lag = max(1, int(round(LAUNCH_ACCEL_WINDOW / (t[1] - t[0])))) if len(t) > 1 else 1
    accel = [(v[i] - v[i - lag]) / (t[i] - t[i - lag]) for i in window if i >= lag]
    return {'slip': max(slip[i] for i in window), 'accel': max(accel) if accel else 0.0}


def brake_metrics(trace: Trace, profile: TrackProfile) -> dict:
    s, v, kappa = trace.column('s'), trace.column('speed'), trace.column('kappa_r')
    brake = _segments(profile, 'brake')[0]
    kappas = [k for k, x, vi in zip(kappa, s, v) if x >= brake.s0 and vi > 1.0]
    return {'stop_error': s[-1] - profile.stop_line_s,
            'stopped': abs(v[-1]) < 0.01, 'min_kappa': min(kappas) if kappas else 0.0}


def estop_metrics(trace: Trace, resume_speed: float = RESUME_SPEED) -> dict:
    t, v, slip, phase = (trace.column('t'), trace.column('speed'), trace.column('slip_r'),
                         trace.column('phase'))
    resume = [i for i, ph in enumerate(phase) if ph == 'resume']
    if not resume:
        return {'min_speed': min(v), 'resume_overshoot_pct': None, 'resume_slip': None}
    t0 = t[resume[0]]
    peak = max(v[i] for i in resume)
    return {'min_speed': min(v),
            'resume_overshoot_pct': max(0.0, (peak - resume_speed) / resume_speed * 100.0),
            'resume_slip': max(slip[i] for i in resume if t[i] <= t0 + 1.5)}


def _upper(value, limit) -> dict:
    return {'value': value, 'limit': limit, 'pass': value is not None and value <= limit}


def _lower(value, limit) -> dict:
    return {'value': value, 'limit': limit, 'pass': value is not None and value >= limit}


def checks(metrics: dict, limits: Dict[str, float] = LIMITS) -> Dict[str, dict]:
    """The 16 checks of one variant: 9 holds, launch slip and accel, stop error, brake lock, 3 e-stop."""
    out = {}
    for key, m in metrics['holds'].items():
        rmse_limit = max(limits['hold_rmse_floor'], limits['hold_rmse_rel'] * float(key))
        out[f'hold_rmse_{key}'] = _upper(m['rmse'], rmse_limit)
        out[f'overshoot_{key}'] = _upper(m['overshoot_pct'], limits['overshoot_pct'])
        out[f'reach_{key}'] = _upper(m['reach_s'], limits['reach_s'])
    out['launch_slip'] = _upper(metrics['launch']['slip'], limits['launch_slip'])
    out['launch_accel'] = _upper(metrics['launch']['accel'], limits['launch_accel'])
    b = metrics['brake']
    out['stop_error'] = _upper(abs(b['stop_error']) if b['stopped'] else None, limits['stop_error'])
    out['brake_lock'] = _lower(b['min_kappa'], limits['brake_kappa'])
    e = metrics['estop']
    out['estop_no_reverse'] = _lower(e['min_speed'], limits['reverse_speed'])
    out['estop_resume_overshoot'] = _upper(e['resume_overshoot_pct'], limits['resume_overshoot_pct'])
    out['estop_resume_slip'] = _upper(e['resume_slip'], limits['resume_slip'])
    return out


def _round(x):
    if isinstance(x, float):
        return round(x, 6)
    if isinstance(x, dict):
        return {k: _round(v) for k, v in x.items()}
    return x


# --------------------------------------------------------------- grading

def run_one(job) -> Trace:
    variant, scenario, params, track_dir = job
    profile = load_track(track_dir)
    sc = ProfileScenario(profile) if scenario == 'profile' else EstopScenario(profile, RESUME_SPEED)
    return Lockstep(load_vehicle(variant=variant), load_actuator_params(), params, profile,
                    variant).run(sc)


def evaluate(params: dict, variants=None, jobs: int = 1, out_dir=None, track_dir=None,
             points: float = 15.0, limits: Dict[str, float] = LIMITS) -> dict:
    track_dir = str(track_dir or default_track_dir())
    variants = list(variants or variant_names())
    work = [(v, sc, dict(params), track_dir) for v in variants for sc in SCENARIOS]
    if jobs > 1:
        import multiprocessing
        with multiprocessing.get_context('spawn').Pool(jobs) as pool:
            traces = pool.map(run_one, work)
    else:
        traces = [run_one(job) for job in work]
    profile = load_track(track_dir)
    report = {'params': dict(params),
              'profile': {'speeds': list(profile.speeds), 'hold': profile.hold,
                          'accel': profile.accel, 'decel': profile.decel},
              'variants': {}, 'traces': {}}
    passed = total = 0
    by_key = {(tr.variant, tr.scenario): tr for tr in traces}
    for v in variants:
        prof_t, estop_t = by_key[(v, 'profile')], by_key[(v, 'estop')]
        metrics = {'holds': hold_metrics(prof_t, profile), 'launch': launch_metrics(prof_t, profile),
                   'brake': brake_metrics(prof_t, profile), 'estop': estop_metrics(estop_t)}
        result = checks(metrics, limits)
        report['variants'][v] = {'metrics': _round(metrics), 'checks': _round(result)}
        passed += sum(1 for c in result.values() if c['pass'])
        total += len(result)
        for tr in (prof_t, estop_t):
            report['traces'][f'{v}_{tr.scenario}'] = tr.digest()
            if out_dir:
                tr.write_csv(Path(out_dir) / 'traces' / f'{v}_{tr.scenario}.csv')
    report.update(passed=passed, total=total, points=points,
                  score=round(passed / total, 6), marks=round(points * passed / total, 2))
    if out_dir:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        (Path(out_dir) / 'marks.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    return report


def _failures(result: Dict[str, dict]) -> str:
    out = []
    for name, c in result.items():
        if not c['pass']:
            value = 'none' if c['value'] is None else f"{c['value']:.3g}"
            out.append(f"{name} ({value} vs {c['limit']:.3g})")
    return ', '.join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Grade a speed-controller submission (Lab 2 Part 1).')
    ap.add_argument('submission', help=f'directory holding {SUBMISSION_FILE}, or the file')
    ap.add_argument('--variant', action='append', help='grade only this variant (repeatable)')
    ap.add_argument('--jobs', type=int, default=1)
    ap.add_argument('--out', default='', help='traces and marks.json (default tmp/rc_sim/grade/<name>)')
    ap.add_argument('--track', default='', help='track directory (default: the packaged straight)')
    ap.add_argument('--points', type=float, default=15.0)
    ap.add_argument('--print-params', action='store_true', help='print the mapping and exit')
    args = ap.parse_args(argv)
    try:
        params = load_submission(args.submission)
    except (SubmissionError, OSError, yaml.YAMLError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    if args.print_params:
        print(flow_mapping(params))
        return 0
    name = Path(args.submission).resolve().name
    out = Path(args.out or os.path.join('tmp', 'rc_sim', 'grade', name))
    report = evaluate(params, args.variant, args.jobs, out, args.track or None, args.points)
    print(f'rc_sim grade: {name} {flow_mapping(params)}')
    for v, r in report['variants'].items():
        n_pass = sum(1 for c in r['checks'].values() if c['pass'])
        print(f'  {v:<14} {n_pass:>2}/{len(r["checks"])}  {_failures(r["checks"])}')
    print(f"score {report['score']:.3f}  marks {report['marks']:.2f} / {report['points']:g}")
    print(f'traces and marks.json: {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
