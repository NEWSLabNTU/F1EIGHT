#!/usr/bin/env python3
"""Find reference coefficients for the rc_sim grader, and suggest its limits from them.

    python3 scripts/testing/rc_sim/calibrate.py --jobs 8

1. Grid-search the coefficients, every variant graded under the provisional
   limits; keep the set that passes most checks and, among those, the one whose
   worst stop-error or physical check sits furthest inside its limit (the
   smallest value/limit). Those limits never move, so their margin is the one
   that has to come from the coefficients.
2. If the winner fails a physical check anywhere, exit 3: the model or a
   variant cannot be driven on this profile, and no limit should be loosened
   to hide that.
3. Print, for each calibrated metric, the worst value across variants and the
   suggested limit = max(provisional, 1.5 x worst), 2 significant figures up.
   The stop error and the physical limits are not calibrated.
4. Regrade under the suggested limits; exit 4 if any check sits within 10 % of
   its limit (value/limit above MARGIN), exit 0 otherwise.
5. Write src/sim/autosdv_rc_sim/config/reference_speed_controller.yaml.
"""
import argparse
import itertools
import math
import multiprocessing
from pathlib import Path
import sys

import yaml

REPO = Path(__file__).resolve().parents[3]
for p in (REPO / 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface', REPO / 'src/sim/autosdv_rc_sim'):
    sys.path.insert(0, str(p))

from autosdv_rc_sim import grade  # noqa: E402

GRID = {'kp_speed': (5.0, 6.0, 7.0), 'ki_speed': (42.0, 43.0, 44.0, 45.0), 'kd_speed': (1.25, 1.5, 1.75),
        'integral_limit': (100.0,)}
FIXED = {'max_pwm_offset': 100, 'enable_conditional_integration': True,
         'derivative_filter_alpha': 0.15}
OUT = REPO / 'src/sim/autosdv_rc_sim/config/reference_speed_controller.yaml'
MARGIN = 0.9      # every check of the reference within 90 % of its limit
UNCALIBRATED = ('stop_error',) + tuple(grade.PHYSICAL_CHECKS.values())   # their limits do not move


def candidates():
    keys = list(GRID)
    for values in itertools.product(*(GRID[k] for k in keys)):
        yield dict(FIXED, **dict(zip(keys, values)))


def real(x):
    """A metric as a number; None (not measured: the run never got there) is infinitely bad."""
    return math.inf if x is None else x


def ratio(check):
    """value / limit: below 1 passes, for upper and lower limits alike (both lower limits are <= 0)."""
    if check['value'] is None:
        return math.inf
    return check['value'] / check['limit']


def margin_of(report, names=None):
    """The largest value/limit over the variants, for the named checks (default: all)."""
    return max(ratio(c) for r in report['variants'].values() for n, c in r['checks'].items()
               if names is None or n in names)


def score(params):
    """(checks passed, minus the worst value/limit among the checks whose limits do not move)."""
    report = grade.evaluate(params)
    return report['passed'], -margin_of(report, UNCALIBRATED), params


def up2(x):
    if x <= 0.0:
        return x
    exponent = math.floor(math.log10(x)) - 1
    return float(f'{math.ceil(x / 10 ** exponent) * 10 ** exponent:.2g}')


def worst(report):
    """Worst value per calibrated metric across variants."""
    out = {}
    for v in report['variants'].values():
        c = v['checks']
        speeds = [k[len('hold_rmse_'):] for k in c if k.startswith('hold_rmse_')]
        values = {
            'hold_rmse_rel': max(real(c[f'hold_rmse_{s}']['value']) / float(s) for s in speeds),
            'overshoot_pct': max(real(c[f'overshoot_{s}']['value']) for s in speeds),
            'reach_s': max(real(c[f'reach_{s}']['value']) for s in speeds),
            'launch_accel': real(c['launch_accel']['value']),
            'resume_overshoot_pct': real(c['estop_resume_overshoot']['value']),
        }
        for k, x in values.items():
            out[k] = max(out.get(k, -math.inf), x)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--jobs', type=int, default=multiprocessing.cpu_count())
    args = ap.parse_args(argv)
    with multiprocessing.get_context('spawn').Pool(args.jobs) as pool:
        scored = pool.map(score, list(candidates()))
    passed, neg_margin, best = max(scored, key=lambda r: (r[0], r[1]))
    print(f'reference ({passed} checks passed over all variants; worst value/limit among the '
          f'uncalibrated checks {-neg_margin:.2f}): {grade.flow_mapping(best)}')
    report = grade.evaluate(best, jobs=args.jobs)
    physical_failures = [(v, name) for v, r in report['variants'].items()
                         for name in grade.PHYSICAL_CHECKS.values() if not r['checks'][name]['pass']]
    if physical_failures:
        print(f'STOP: the reference fails physical checks {physical_failures}; '
              'fix the model or the variant (and tell the user), do not loosen these.')
        return 3
    print('metric                 worst     provisional  suggested')
    suggested = dict(grade.LIMITS)
    for key, value in worst(report).items():
        suggested[key] = max(grade.LIMITS[key], up2(1.5 * value))
        print(f'{key:<22} {value:>9.4f} {grade.LIMITS[key]:>12.4g} {suggested[key]:>10.4g}')
    for key in grade.PHYSICAL_LIMITS:
        assert suggested[key] == grade.LIMITS[key]          # physical limits are never moved
    stop = max(real(r['checks']['stop_error']['value']) for r in report['variants'].values())
    print(f"stop_error (not calibrated)  worst {stop:.4f}  limit {grade.LIMITS['stop_error']:g}")
    print('LIMITS = ' + repr(suggested))
    final = grade.evaluate(best, jobs=args.jobs, limits=suggested)
    print(f"under the suggested limits: {final['passed']}/{final['total']} checks, "
          f'worst value/limit {margin_of(final):.2f} (must stay under {MARGIN:g})')
    if final['passed'] != final['total'] or margin_of(final) > MARGIN:
        print('STOP: the reference has too little margin under the suggested limits; '
              'the model, the variants or the search grid need work.')
        return 4
    OUT.write_text('# Reference coefficients for the rc_sim grader (scripts/testing/rc_sim/calibrate.py).\n'
                   + yaml.safe_dump(best, sort_keys=True))
    print(f'wrote {OUT}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
