#!/usr/bin/env python3
"""Lab 2 Part 1 load tolerance: the same controller graded alike on a quiet and a loaded host.

    python3 scripts/testing/lab2/part1_tolerance.py --submission DIR [--submission DIR2 ...]
    python3 scripts/testing/lab2/part1_tolerance.py --class pkg.mod:Class [--params '{kp: 30}']
        [--variant nominal] [-n 3] [--busy-mult 1] [--out DIR] [--web-addr HOST:PORT]
    python3 scripts/testing/lab2/part1_tolerance.py --analyze DIR      (re-read finished runs)

The acceptance test of the design's "Clock synchronization" section: each controller is run
N times on the host as it is and N times with a busy loop on every core (busy-mult loops per
core, started in their own session and killed by process group afterwards), each run through
the Part 1 runner (scripts/testing/lab2/part1_run.py --out ... rviz:=false gui:=false), and
the grader's marks.json of every run is compared check by check:

  verdict    every run gives the same pass/fail, wherever every run's value is more than
             MARGIN (3 %) of the limit away from it (a value that close may flip either way,
             so it is listed and exempt)
  spread     max - min over all 2N runs is within max(REL_SPREAD (2 %) of the limit, the
             check's absolute floor, FLOORS below)

Every run must be valid (an invalid run fails the test, with its reason). Prints a table per
controller, writes <out>/tolerance.json, and exits 0 when both hold for every controller,
1 when not, 2 on a usage error. Outputs default to tmp/lab2/part1/tolerance/<name>/, one
directory per run (quiet-1 .. loaded-N), each the runner's --out.

Run it alone on the host (flock tmp/bench.lock), with its own ROS_DOMAIN_ID: the busy loops
load every core, which is the point, and a second simulation would load it further.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[3]
RUNNER = REPO / 'scripts/testing/lab2/part1_run.py'
MARGIN = 0.03           # of the limit: closer than this, a verdict may flip and is exempt
REL_SPREAD = 0.02       # of the limit: the allowed spread, unless the floor is larger
# Absolute floors of the spread, in each check's unit: the resolution below which a spread is
# noise of the measurement itself (one 10 ms control tick at the speed involved, one hall
# marker, the slip estimator's quantum), not a load effect.
FLOORS = {
    'hold_rmse_2': 0.01, 'hold_rmse_6': 0.01, 'hold_rmse_11': 0.01, 'hold_rmse_11b': 0.01,   # m/s
    'overshoot': 0.5, 'stepdown_undershoot': 0.5, 'estop_resume_overshoot': 0.5,            # %
    'reach_s': 0.02,                                                                        # s
    'launch_slip': 0.02, 'estop_resume_slip': 0.02, 'estop_reverse': 0.01,                  # m/s
    'launch_accel': 0.1,                                                                    # m/s^2
    'stepdown_kappa': 0.01, 'stop_kappa': 0.01,                                             # slip
    'stop_error': 0.03,                                                                     # m
}
DEFAULT_FLOOR = 0.01

_busy = []              # the busy-loop Popen, for the signal handler


# ------------------------------------------------------------------ load
def start_busy(mult: int) -> subprocess.Popen:
    """A busy loop per core x mult, in a new session (its own process group)."""
    n = (os.cpu_count() or 1) * mult
    script = f'for i in $(seq {n}); do (while :; do :; done) & done; wait'
    proc = subprocess.Popen(['bash', '-c', script], start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    _busy.append(proc)
    time.sleep(1.0)
    return proc


def stop_busy(proc) -> None:
    if proc is None:
        return
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.wait(timeout=3.0)
            break
        except subprocess.TimeoutExpired:
            continue
    try:                # the loops are children of bash: gone only when the group is empty
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    if proc in _busy:
        _busy.remove(proc)


def on_signal(signum, frame):
    for proc in list(_busy):
        stop_busy(proc)
    sys.exit(130)


# ------------------------------------------------------------------ runs
def read_marks(run_dir: Path, variant: str):
    for path in (run_dir / variant / 'marks.json', run_dir / 'marks.json'):
        if path.is_file():
            return json.loads(path.read_text())
    return None


def run_once(args, controller, run_dir: Path) -> dict:
    cmd = [sys.executable, str(args.runner)] + controller + [
        '--variant', args.variant, '--out', str(run_dir), '--timeout', str(args.timeout),
        '--web-addr', args.web_addr, 'rviz:=false', 'gui:=false']
    run_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.monotonic()
    with open(run_dir / 'runner.log', 'w') as log:
        rc = subprocess.run(cmd, cwd=REPO, stdout=log, stderr=subprocess.STDOUT).returncode
    marks = read_marks(run_dir, args.variant)
    return {'dir': str(run_dir), 'rc': rc, 'wall': round(time.monotonic() - t0, 1), 'marks': marks}


def fingerprint(paths) -> dict:
    """sha256 (12 hex) of each watched file: the inputs that must not change between runs."""
    out = {}
    for p in paths:
        try:
            out[p] = hashlib.sha256(Path(p).read_bytes()).hexdigest()[:12]
        except OSError:
            out[p] = None
    return out


# ------------------------------------------------------------------ analysis
def margin_of(value, limit):
    if value is None or limit in (None, 0):
        return None
    return abs(value - limit) / abs(limit)


def analyze(runs: list) -> dict:
    """runs: [{'load': 'quiet'|'loaded', 'marks': {...} | None, ...}] -> the verdict."""
    problems = []
    for r in runs:
        m = r['marks']
        if m is None:
            problems.append(f"{r['name']}: no marks.json (runner exit {r.get('rc')})")
        elif not m.get('valid', False):
            problems.append(f"{r['name']}: invalid run: {m.get('invalid_reason')}")
    graded = [r for r in runs if r['marks'] is not None and r['marks'].get('valid', False)]
    names = []
    for r in graded:
        for name in r['marks'].get('checks', {}):
            if name not in names:
                names.append(name)
    rows = []
    for name in names:
        cells = [r['marks']['checks'].get(name, {}) for r in graded]
        values = [c.get('value') for c in cells]
        verdicts = [c.get('pass') for c in cells]
        limit = next((c.get('limit') for c in cells if c.get('limit') is not None), None)
        finite = [v for v in values if isinstance(v, (int, float))]
        spread = (max(finite) - min(finite)) if len(finite) == len(values) and finite else None
        allowed = max(REL_SPREAD * abs(limit) if limit else 0.0, FLOORS.get(name, DEFAULT_FLOOR))
        margins = [margin_of(v, limit) for v in values]
        near = any(mg is None or mg <= MARGIN for mg in margins)
        agree = len(set(verdicts)) <= 1
        row = {'check': name, 'limit': limit, 'values': values, 'verdicts': verdicts,
               'spread': spread, 'allowed': allowed, 'near_limit': near, 'agree': agree,
               'spread_ok': spread is not None and spread <= allowed + 1e-12,
               'quiet': [c.get('value') for c, r in zip(cells, graded) if r['load'] == 'quiet'],
               'loaded': [c.get('value') for c, r in zip(cells, graded) if r['load'] == 'loaded']}
        if not agree and not near:
            problems.append(f'{name}: verdicts differ {verdicts} (values {values}, limit {limit})')
        if not row['spread_ok']:
            problems.append(f'{name}: spread {spread} > allowed {allowed:g} (values {values})')
        rows.append(row)
    return {'ok': not problems and len(graded) == len(runs) and bool(runs),
            'problems': problems, 'checks': rows,
            'passed': [r['marks'].get('passed') for r in graded],
            'total': [r['marks'].get('total') for r in graded]}


def fmt(v, width=8):
    if v is None:
        return '-'.rjust(width)
    if isinstance(v, float):
        return f'{v:.4g}'.rjust(width)
    return str(v).rjust(width)


def print_table(label: str, runs: list, result: dict) -> None:
    print(f'\n== {label}: {len(runs)} runs ({sum(r["load"] == "quiet" for r in runs)} quiet, '
          f'{sum(r["load"] == "loaded" for r in runs)} loaded); passed {result["passed"]} of '
          f'{result["total"]}')
    print(f'{"check":24s}{"limit":>9s}{"quiet min..max":>24s}{"loaded min..max":>24s}'
          f'{"spread":>10s}{"allowed":>10s}  verdicts')
    for row in result['checks']:
        q = [v for v in row['quiet'] if v is not None]
        lo = [v for v in row['loaded'] if v is not None]
        qs = f'{fmt(min(q), 0)}..{fmt(max(q), 0)}' if q else '-'
        ls = f'{fmt(min(lo), 0)}..{fmt(max(lo), 0)}' if lo else '-'
        qs, ls = ' ' + qs, ' ' + ls
        verdicts = ''.join({True: 'P', False: 'F', None: '?'}[v] for v in row['verdicts'])
        flag = '' if row['agree'] else (' (near limit: exempt)' if row['near_limit'] else ' DISAGREE')
        flag += '' if row['spread_ok'] else ' SPREAD'
        print(f'{row["check"]:24s}{fmt(row["limit"], 9)}{qs:>24s}{ls:>24s} {fmt(row["spread"], 9)}'
              f' {fmt(row["allowed"], 9)}  {verdicts}{flag}')
    for r in runs:
        m = r['marks'] or {}
        print(f'  {r["name"]:10s} wall {r.get("wall", "-")} s  valid {m.get("valid")}  '
              f'passed {m.get("passed")}/{m.get("total")}  {m.get("invalid_reason") or ""}')
    print('PASS' if result['ok'] else 'FAIL: ' + '; '.join(result['problems']))


# ------------------------------------------------------------------ main
def controllers(args):
    """[(label, runner args)] from the command line."""
    if args.submission:
        return [(Path(s).expanduser().resolve().name, ['--submission', str(Path(s).expanduser())])
                for s in args.submission]
    if args.cls:
        ctrl = ['--class', args.cls] + (['--params', args.params] if args.params else [])
        return [(args.label or args.cls.rsplit(':', 1)[-1], ctrl)]
    return [(args.label or 'default', [])]


def load_existing(base: Path) -> list:
    runs = []
    for d in sorted(base.iterdir()):
        load = d.name.split('-')[0]
        if d.is_dir() and load in ('quiet', 'loaded'):
            variant = next((p.parent.name for p in d.glob('*/marks.json')), 'nominal')
            runs.append({'name': d.name, 'load': load, 'dir': str(d), 'marks': read_marks(d, variant)})
    return runs


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Lab 2 Part 1: same verdicts quiet and under load?')
    ap.add_argument('--submission', action='append', metavar='DIR', help='repeatable')
    ap.add_argument('--class', dest='cls', metavar='PKG.MOD:CLASS')
    ap.add_argument('--params', default=None, help='with --class: a YAML mapping')
    ap.add_argument('--label', default=None, help='output name with --class')
    ap.add_argument('--variant', default='nominal')
    ap.add_argument('-n', type=int, default=3, help='runs per condition (default 3)')
    ap.add_argument('--busy-mult', type=int, default=1, help='busy loops per core (default 1)')
    ap.add_argument('--out', default=None, help='default tmp/lab2/part1/tolerance')
    ap.add_argument('--timeout', type=float, default=480.0, help='[s wall] per run, for the runner')
    ap.add_argument('--web-addr', default=os.environ.get('LAB2_WEB_ADDR', '127.0.0.1:8081'))
    ap.add_argument('--runner', default=str(RUNNER))
    ap.add_argument('--analyze', metavar='DIR', help='only analyze the runs already in DIR')
    ap.add_argument('--interleave', action='store_true',
                    help='quiet-1, loaded-1, quiet-2, ... (back-to-back pairs) instead of all quiet first')
    ap.add_argument('--watch', action='append', default=[], metavar='FILE',
                    help='repeatable: files (model, controller) whose content must not change '
                         'between runs; a change is reported and fails the test')
    args = ap.parse_args(argv)
    if args.submission and args.cls:
        ap.error('--submission or --class, not both')
    if os.environ.setdefault('ROS_DOMAIN_ID', '73') == '0':
        print('refusing ROS_DOMAIN_ID 0 (shared host)')
        return 2

    if args.analyze:
        base = Path(args.analyze)
        runs = load_existing(base)
        result = analyze(runs)
        print_table(base.name, runs, result)
        return 0 if result['ok'] else 1

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    out = Path(args.out).expanduser() if args.out else REPO / 'tmp/lab2/part1/tolerance'
    summary, all_ok = {}, True
    for label, ctrl in controllers(args):
        base = out / label
        runs = []
        if args.interleave:
            order = [(load, i) for i in range(1, args.n + 1) for load in ('quiet', 'loaded')]
        else:
            order = [(load, i) for load in ('quiet', 'loaded') for i in range(1, args.n + 1)]
        for load, i in order:
            name = f'{load}-{i}'
            busy = start_busy(args.busy_mult) if load == 'loaded' else None
            try:
                print(f'{label}: {name} ...', flush=True)
                before = fingerprint(args.watch)
                r = run_once(args, ctrl, base / name)
                r.update(name=name, load=load, inputs=before)
                if fingerprint(args.watch) != before:
                    r['inputs_changed'] = True
                runs.append(r)
                m = r['marks'] or {}
                print(f'  valid {m.get("valid")} passed {m.get("passed")}/{m.get("total")} '
                      f'({r["wall"]} s wall)', flush=True)
            finally:
                stop_busy(busy)
        result = analyze(runs)
        changed = [r['name'] for r in runs if r.get('inputs_changed')]
        if len({json.dumps(r.get('inputs'), sort_keys=True) for r in runs}) > 1 or changed:
            result['ok'] = False
            result['problems'].append(f'watched inputs changed between runs (during: {changed})')
        print_table(label, runs, result)
        summary[label] = {'runs': [{k: r.get(k) for k in ('name', 'load', 'dir', 'rc', 'wall',
                                                          'inputs', 'inputs_changed')}
                                   for r in runs], **result}
        all_ok &= result['ok']
    out.mkdir(parents=True, exist_ok=True)
    meta = {'variant': args.variant, 'n': args.n, 'busy_mult': args.busy_mult,
            'cpus': os.cpu_count(), 'margin': MARGIN, 'rel_spread': REL_SPREAD, 'floors': FLOORS}
    (out / 'tolerance.json').write_text(json.dumps({'meta': meta, 'controllers': summary},
                                                   indent=2) + '\n')
    print(f'\n{"PASS" if all_ok else "FAIL"}  ({out / "tolerance.json"})')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
