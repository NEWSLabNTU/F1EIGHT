"""Calibration hooks: refit the guessed resistances and friction from car data.

    rc_sim_fit coastdown run.csv [--variant nominal] [--v-min 0.5]

The CSV has columns t,v: a coast-down at neutral on the flat.

Coast-down: m_eff (-dv/dt) = a0 + a1 v + a2 v^2, least squares on
central-difference decelerations. These are LUMPED estimates: a0 is rolling
resistance plus the drivetrain's Coulomb drag at the ground, a1 is the
drivetrain's viscous drag at the ground (model: drivetrain_viscous / r^2), a2 is
the aerodynamic coefficient. rolling_resistance is (a0 - Coulomb/r) / (m g),
which subtracts the model's Coulomb term (the only model value subtracted).
Samples below --v-min, and those next to one, are dropped; the samples left must
span at least MIN_SPAN m/s, or the three terms cannot be told apart. For a
usable fit coast from top speed to below 1 m/s on the flat, in both directions
averaged, and smooth the 20 Hz hall speed (low-pass, or fit over windows of at
least 0.2 s) before differencing.
Skidpad: on a circle of radius R at the speed where the car starts to slide,
mu = v^2 / (g R). The motor and gear: `just coach fit-motor`.
"""
import argparse
import csv
import math
import sys
from typing import Sequence, Tuple

from autosdv_rc_sim.params import GRAVITY, load_vehicle

MIN_SAMPLES = 5
# Below this speed span a near-constant, noisy trace still solves, but the three terms
# trade against each other freely and the fit means nothing.
MIN_SPAN = 1.0  # m/s


def _solve3(a, b):
    """Gaussian elimination with partial pivoting; ValueError if singular."""
    m = [list(a[i]) + [b[i]] for i in range(3)]
    scale = max(abs(m[i][i]) for i in range(3))
    for c in range(3):
        piv = max(range(c, 3), key=lambda r: abs(m[r][c]))
        if abs(m[piv][c]) <= 1e-12 * scale:
            raise ValueError('coast-down data cannot separate a0, a1 and a2: '
                             'the speed range is too narrow (or v is constant)')
        m[c], m[piv] = m[piv], m[c]
        for r in range(c + 1, 3):
            f = m[r][c] / m[c][c]
            for k in range(c, 4):
                m[r][k] -= f * m[c][k]
    x = [0.0] * 3
    for r in (2, 1, 0):
        x[r] = (m[r][3] - sum(m[r][k] * x[k] for k in range(r + 1, 3))) / m[r][r]
    return x


def fit_coastdown(t: Sequence[float], v: Sequence[float], effective_mass: float,
                  v_min: float = 0.5) -> Tuple[float, float, float]:
    """Return (a0, a1, a2) of m_eff (-dv/dt) = a0 + a1 v + a2 v^2.

    Raises ValueError on bad data.
    """
    if len(t) != len(v):
        raise ValueError('t and v have different lengths')
    for i in range(len(t)):
        if not (math.isfinite(t[i]) and math.isfinite(v[i])):
            raise ValueError(f't and v must be finite (row {i + 1})')
    for i in range(1, len(t)):
        if not t[i] > t[i - 1]:
            raise ValueError(f'time must be strictly increasing (row {i + 1})')
    rows = []
    for i in range(1, len(t) - 1):
        if min(v[i - 1], v[i], v[i + 1]) < v_min:
            continue
        decel = -(v[i + 1] - v[i - 1]) / (t[i + 1] - t[i - 1])
        rows.append((v[i], effective_mass * decel))
    if len(rows) < MIN_SAMPLES:
        raise ValueError(f'only {len(rows)} usable samples with v >= {v_min} m/s; '
                         f'need at least {MIN_SAMPLES}')
    span = max(r[0] for r in rows) - min(r[0] for r in rows)
    if span < MIN_SPAN:
        raise ValueError(f'coast-down data cannot separate a0, a1 and a2: the usable samples '
                         f'span {span:.2f} m/s, need at least {MIN_SPAN} m/s')
    # normal equations for the basis 1, v, v^2
    pw = [0.0] * 5
    rhs = [0.0, 0.0, 0.0]
    for x, y in rows:
        p = 1.0
        for k in range(5):
            pw[k] += p
            if k < 3:
                rhs[k] += p * y
            p *= x
    a = [[pw[i + j] for j in range(3)] for i in range(3)]
    a0, a1, a2 = _solve3(a, rhs)
    return a0, a1, a2


def skidpad_mu(radius: float, speed: float) -> float:
    return speed * speed / (GRAVITY * radius)


def _read_csv(path):
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        names = reader.fieldnames or ()
        if 't' not in names or 'v' not in names:
            raise ValueError(f'{path}: the CSV needs columns t and v')
        out = []
        for n, r in enumerate(reader, start=2):
            try:
                out.append((float(r['t']), float(r['v'])))
            except (TypeError, ValueError):
                raise ValueError(f'{path}: line {n}: t and v must be numbers')
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Refit rc_sim parameters from car data.')
    sub = ap.add_subparsers(dest='cmd', required=True)
    cd = sub.add_parser('coastdown', help='CSV with columns t,v')
    cd.add_argument('csv')
    cd.add_argument('--variant', default='nominal')
    cd.add_argument('--v-min', type=float, default=0.5,
                    help='drop samples slower than this [m/s] (default 0.5)')
    args = ap.parse_args(argv)
    try:
        p = load_vehicle(variant=args.variant)
        rows = _read_csv(args.csv)
        a0, a1, a2 = fit_coastdown([r[0] for r in rows], [r[1] for r in rows],
                                   p.effective_mass, v_min=args.v_min)
    except (ValueError, OSError) as e:
        print(f'rc_sim_fit: {e}', file=sys.stderr)
        return 2
    r = p.wheel_radius
    coulomb = (p.drivetrain_coulomb + p.front_bearing_coulomb) / r
    print('lumped estimates (m_eff * decel = a0 + a1 v + a2 v^2):')
    print(f'a0 = {a0:.3f} N, a1 = {a1:.4f} N/(m/s) (model drivetrain_viscous/r^2 = '
          f'{p.drivetrain_viscous / (r * r):.4f}), a2 = {a2:.4f} N/(m/s)^2')
    print(f'rolling_resistance ~ {(a0 - coulomb) / (p.mass * GRAVITY):.4f} '
          f'(model {p.rolling_resistance}); aero_drag ~ {a2:.4f} (model {p.aero_drag})')
    for name, val in (('a0', a0), ('a1', a1), ('a2', a2)):
        if val < 0:
            print(f'warning: {name} is negative; the data is too noisy or too narrow '
                  f'in speed to trust', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
