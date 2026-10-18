#!/usr/bin/env python3
"""Suggest Lab 2 Part 1's calibrated limits from live runs of the reference controller.

    python3 scripts/testing/lab2/part1_calibrate.py                    run, then suggest
    python3 scripts/testing/lab2/part1_calibrate.py --repeats 3        worst over 3 runs each
    python3 scripts/testing/lab2/part1_calibrate.py --from-dir DIR     re-read runs already made

1. Run the reference (default autosdv_waypoint_lab.reference.speed_pid_part1:SpeedPID with
   config/speed_pid_part1_reference.yaml) through the live runner, once per graded variant
   (nominal, low_batt, low_mu) and repeat: scripts/testing/lab2/part1_run.py --class C --params P
   --variant V --out DIR[/rN] --no-rviz --no-gui, read back from DIR[/rN]/V/ (a --params
   file is passed as the one-line mapping the runner takes). A run is refused without reading
   its trace when marks.json says invalid, or valid but ended early (it carries a message).
2. Re-evaluate every trace.csv with autosdv_waypoint_lab.part1.metrics (so the numbers
   do not depend on the limits the grader ran with) and refuse a run that is invalid
   (exit 3) or fails a FIXED check (physical or the stop error; exit 3): no limit is
   loosened to hide a reference that cannot drive the course.
3. Print, per calibrated metric, the worst value across variants and repeats and the
   suggested limit = max(1.5 x worst, FLOOR), 2 significant figures up, with a LIMITS
   snippet to paste into part1/metrics.py (then set LIMITS_CALIBRATED = True).
4. Write DIR/calibration.json. Exit 4 if a calibrated check of the reference sits within
   10 % of its suggested limit (cannot happen with 1.5 x unless a floor applies; reported).

The rule is the RC simulator's (docs/design/lab2-part1-sim-design.md, "Checks and marks");
floors keep a limit from collapsing to 0 where the reference is perfect (e.g. no overshoot).
"""
import argparse
import json
import math
from pathlib import Path
import shlex
import subprocess
import sys

import yaml

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'src/lab/autosdv_waypoint_lab'))

from autosdv_waypoint_lab.part1 import metrics as M  # noqa: E402

RUNNER = REPO / 'scripts/testing/lab2/part1_run.py'
VARIANTS = ('nominal', 'low_batt', 'low_mu')
DEFAULT_CLASS = 'autosdv_waypoint_lab.reference.speed_pid_part1:SpeedPID'
_CONFIG = REPO / 'src/lab/autosdv_waypoint_lab/config'
# the reference's parameters (renamed from speed_pid_part1.yaml when the handout default split off)
DEFAULT_PARAMS = next((p for p in (_CONFIG / 'speed_pid_part1_reference.yaml',
                                   _CONFIG / 'speed_pid_part1.yaml') if p.is_file()),
                      _CONFIG / 'speed_pid_part1_reference.yaml')
DEFAULT_PROFILE = (REPO / 'src/lab/autosdv_waypoint_lab/config/courses/lab2_part1'
                   / 'part1_profile.yaml')
FACTOR = 1.5
MARGIN = 0.9
# the least each calibrated limit may be, in its own unit
FLOOR = {'hold_rmse_2': 0.05, 'hold_rmse_6': 0.05, 'hold_rmse_11': 0.05, 'hold_rmse_11b': 0.05,
         'overshoot': 2.0, 'reach_s': 0.2, 'launch_accel': 1.0, 'stepdown_undershoot': 2.0,
         'estop_resume_overshoot': 2.0, 'stepdown_lag': 0.3}


def up2(x: float) -> float:
    """Round up to 2 significant figures."""
    if x <= 0.0:
        return x
    exponent = math.floor(math.log10(x)) - 1
    return float(f'{math.ceil(x / 10 ** exponent - 1e-9) * 10 ** exponent:.2g}')


def params_mapping(params: str) -> str:
    """The runner's --params (a one-line YAML mapping): a file is read and flattened."""
    path = Path(params).expanduser()
    if not path.is_file():
        return params
    data = yaml.safe_load(path.read_text()) or {}
    return '{' + ', '.join(f'{k}: {json.dumps(v)}' for k, v in data.items()) + '}'


def run_one(args, variant: str, out: Path) -> int:
    cmd = [sys.executable, str(args.runner), '--class', args.cls, '--params',
           params_mapping(args.params), '--variant', variant, '--out', str(out),
           '--no-rviz', '--no-gui'] + (['--web-addr', args.web_addr] if args.web_addr else [])
    print('$ ' + ' '.join(shlex.quote(c) for c in cmd), flush=True)
    try:
        return subprocess.run(cmd, timeout=args.timeout).returncode
    except subprocess.TimeoutExpired:
        print(f'  {variant}: the runner did not return within {args.timeout:g} s', file=sys.stderr)
        return 124


def read_run(run_dir: Path, profile: M.Profile) -> dict:
    """marks.json first: an invalid run (or the runner's stand-in for a dead stack, which has
    no trace.csv) is refused without reading the trace."""
    marks = json.loads((run_dir / 'marks.json').read_text())
    run = {'dir': str(run_dir), 'valid': bool(marks.get('valid', False)),
           'invalid_reason': marks.get('invalid_reason'), 'message': marks.get('message'),
           'checks': None}
    if run['valid'] and marks.get('message'):
        run['valid'] = False   # ended early (stalled / timed out): not a reference run
        run['invalid_reason'] = marks['message']
    if run['valid']:
        if not (run_dir / 'trace.csv').is_file():
            run['valid'], run['invalid_reason'] = False, 'no trace.csv beside marks.json'
        else:
            run['checks'] = M.evaluate(M.Trace.read_csv(run_dir / 'trace.csv'), profile,
                                       finished=True)
    return run


def suggest(runs: list) -> dict:
    out = {}
    for name in M.CALIBRATED:
        values = [r['checks'][name]['value'] for r in runs]
        if any(v is None for v in values):
            out[name] = {'worst': None, 'suggested': None, 'floor': FLOOR.get(name)}
            continue
        worst = max(values)
        limit = max(up2(FACTOR * worst), FLOOR.get(name, 0.0))
        out[name] = {'worst': worst, 'suggested': limit,
                     'floor_applied': limit > up2(FACTOR * worst), 'current': M.LIMITS[name]}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--class', dest='cls', default=DEFAULT_CLASS)
    ap.add_argument('--params', default=str(DEFAULT_PARAMS),
                    help='a YAML file or a one-line mapping (default: the reference yaml)')
    ap.add_argument('--variants', default=','.join(VARIANTS))
    ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--out', default=str(REPO / 'tmp/lab2/part1/calibrate'))
    ap.add_argument('--profile', default=str(DEFAULT_PROFILE))
    ap.add_argument('--runner', default=str(RUNNER))
    ap.add_argument('--timeout', type=float, default=900.0, help='wall s per run')
    ap.add_argument('--web-addr', default='', help="passed to the runner (play_launch's web UI)")
    ap.add_argument('--from-dir', default='', help='read DIR/*/marks.json; run nothing')
    args = ap.parse_args(argv)
    profile = M.load_profile(args.profile)
    variants = [v for v in args.variants.split(',') if v]

    if args.from_dir:
        base = Path(args.from_dir).expanduser()
        dirs = sorted({p.parent for pat in ('*/marks.json', 'r*/*/marks.json')
                       for p in base.glob(pat)})
        if not dirs:
            print(f'error: no */marks.json under {args.from_dir}', file=sys.stderr)
            return 2
    else:
        if not Path(args.runner).is_file():
            print(f'error: the runner {args.runner} does not exist yet', file=sys.stderr)
            return 2
        dirs = []
        for v in variants:
            for k in range(args.repeats):
                # the runner writes <--out>/<variant>/marks.json
                root = Path(args.out) if args.repeats == 1 else Path(args.out) / f'r{k + 1}'
                code = run_one(args, v, root)
                d = root / v
                if not (d / 'marks.json').is_file():
                    print(f'error: {v}: no marks.json in {d} (runner exit {code})',
                          file=sys.stderr)
                    return 3
                dirs.append(d)

    runs = [read_run(d, profile) for d in dirs]
    bad = False
    for r in runs:
        if not r['valid']:
            print(f"REFUSED {r['dir']}: invalid run ({r['invalid_reason']})", file=sys.stderr)
            bad = True
            continue
        failed_fixed = [n for n in M.FIXED if not r['checks'][n]['pass']]
        if failed_fixed:
            text = ', '.join(M.describe(n, r['checks'][n]) for n in failed_fixed)
            print(f"REFUSED {r['dir']}: the reference fails a fixed check: {text}",
                  file=sys.stderr)
            bad = True
    if bad:
        return 3

    table = suggest(runs)
    print(f'\n{len(runs)} runs: ' + ', '.join(Path(r['dir']).name for r in runs))
    print('metric'.ljust(24) + 'worst'.rjust(10) + 'x1.5 / floor'.rjust(14) + 'current'.rjust(10))
    for name, row in table.items():
        worst = 'none' if row['worst'] is None else f"{row['worst']:.4g}"
        sug = 'none' if row['suggested'] is None else (
            f"{row['suggested']:.3g}" + (' (floor)' if row.get('floor_applied') else ''))
        print(f"{name:<24}{worst:>10}{sug:>14}{M.LIMITS[name]:>10.3g}")
    print('\nLIMITS snippet for autosdv_waypoint_lab/part1/metrics.py (fixed limits unchanged):')
    for name in M.CHECK_NAMES:
        row = table.get(name)
        value = M.LIMITS[name] if row is None or row['suggested'] is None else row['suggested']
        worst = row and row['worst']
        tag = 'fixed' if name in M.FIXED else (
            'calibrated: not measured' if worst is None else f'calibrated: 1.5 x {worst:.4g}')
        print(f"    '{name}': {value!r},  # {tag}")

    close = []
    for r in runs:
        for name, row in table.items():
            v, lim = r['checks'][name]['value'], row['suggested']
            if v is not None and lim and v / lim > MARGIN:
                close.append(f"{Path(r['dir']).name}:{name} {v:.3g}/{lim:.3g}")
    out = Path(args.from_dir or args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    (out / 'calibration.json').write_text(json.dumps(
        {'factor': FACTOR, 'floor': FLOOR, 'runs': [r['dir'] for r in runs], 'limits': table,
         'within_margin': close}, indent=2) + '\n')
    print(f"\nwrote {out / 'calibration.json'}")
    missing = [n for n, row in table.items() if row['suggested'] is None]
    if missing:
        print(f"error: not measured in some run: {', '.join(missing)}", file=sys.stderr)
        return 3
    if close:
        print('WARNING: within 10 % of the suggested limit: ' + '; '.join(close), file=sys.stderr)
        return 4
    return 0


if __name__ == '__main__':
    sys.exit(main())
