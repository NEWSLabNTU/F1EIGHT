"""Calibration hooks: refit the guessed resistances and friction from car data.

    rc_sim_fit coastdown run.csv           columns t,v: a coast-down at neutral on the flat

Coast-down: m_eff (-dv/dt) = a0 + a2 v^2, least squares on central-difference
decelerations. a0 is rolling resistance plus the drivetrain's Coulomb drag at
the ground; a2 is the aerodynamic coefficient (the small viscous term folds
into both). Skidpad: on a circle of radius R at the speed where the car starts
to slide, mu = v^2 / (g R). The motor and gear: `just coach fit-motor`.
"""
import argparse
import csv
import sys
from typing import Sequence, Tuple

from autosdv_rc_sim.params import GRAVITY, load_vehicle


def fit_coastdown(t: Sequence[float], v: Sequence[float], effective_mass: float) -> Tuple[float, float]:
    xs, ys = [], []
    for i in range(1, len(t) - 1):
        decel = -(v[i + 1] - v[i - 1]) / (t[i + 1] - t[i - 1])
        xs.append(v[i] * v[i])
        ys.append(effective_mass * decel)
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    a2 = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    a0 = (sy - a2 * sx) / n
    return a0, a2


def skidpad_mu(radius: float, speed: float) -> float:
    return speed * speed / (GRAVITY * radius)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Refit rc_sim parameters from car data.')
    sub = ap.add_subparsers(dest='cmd', required=True)
    cd = sub.add_parser('coastdown', help='CSV with columns t,v')
    cd.add_argument('csv')
    cd.add_argument('--variant', default='nominal')
    args = ap.parse_args(argv)
    p = load_vehicle(variant=args.variant)
    with open(args.csv) as f:
        rows = [(float(r['t']), float(r['v'])) for r in csv.DictReader(f)]
    a0, a2 = fit_coastdown([r[0] for r in rows], [r[1] for r in rows], p.effective_mass)
    coulomb = (p.drivetrain_coulomb + p.front_bearing_coulomb) / p.wheel_radius
    print(f'a0 = {a0:.3f} N, a2 = {a2:.4f} N/(m/s)^2')
    print(f'rolling_resistance ~ {(a0 - coulomb) / (p.mass * GRAVITY):.4f} '
          f'(model {p.rolling_resistance}); aero_drag ~ {a2:.4f} (model {p.aero_drag})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
