#!/usr/bin/env python3
"""Lab 2 vehicle gate, safety half: does a vehicle model stop each car?

    python3 scripts/testing/lab2/vehicle_gate.py [vehicle_model:=team3_vehicle] [--cars car_a,car_c]

For each car in individual_params (car_a, car_b, car_c by default) it starts
lab2_vehicle_gate.launch.xml: the model's vehicle interface on hardware:=sim and the RC
simulator's plant with THAT car's ESC and servo points, driven by lab2_vehicle_gate_probe
(autosdv_waypoint_lab/vehicle_gate.py says what it commands and checks). One line per check
and car, PASS or FAIL; exit status is the number of cars that failed. `just lab2 gate` runs
this first and the route on the vehicle bench after it.

Every car, not one: the cars' neutrals differ (A 380, B 390, C 385), so a model that
hard-codes one car's stop PWM passes on that car and drives the others. Rules it keeps on a
shared host, as bench_gate.py does: its own ROS_DOMAIN_ID (default 73, refuses 0), each run
started with setsid and stopped by process group.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
DEFAULT_MODEL = 'autosdv_vehicle'


def cars():
    try:
        from autosdv_vehicle_interface import car_profile
    except ImportError:
        sys.path.insert(0, str(REPO / 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface'))
        from autosdv_vehicle_interface import car_profile
    return [n for n in car_profile.names() if n.startswith('car_')]


def split(argv):
    model, rest = DEFAULT_MODEL, []
    for a in argv:
        k, sep, v = a.partition(':=')
        if sep and k == 'vehicle_model':
            model = v
        else:
            rest.append(a)
    return model, rest


def run_car(model, car, workdir: Path, env, timeout, rtf, extra):
    out = workdir / f'{car}.json'
    logdir = workdir / car
    out.unlink(missing_ok=True)
    logdir.mkdir(parents=True, exist_ok=True)
    cmd = ['play_launch', 'launch', '--web-addr', env.get('LAB2_WEB_ADDR', '127.0.0.1:8092'),
           '--log-dir', str(logdir / 'play_log'), 'autosdv_launch', 'lab2_vehicle_gate.launch.xml',
           f'vehicle_model:={model}', f'vehicle_id:={car}', f'out_file:={out}',
           f'real_time_factor:={rtf}', *extra]
    with open(logdir / 'launch.log', 'w') as log:
        proc = subprocess.Popen(cmd, cwd=REPO, env=env, start_new_session=True,
                                stdout=log, stderr=subprocess.STDOUT)
        t0 = time.monotonic()
        try:
            while not out.is_file() and proc.poll() is None and time.monotonic() - t0 < timeout:
                time.sleep(0.5)
        finally:
            for sig, wait in ((signal.SIGINT, 15), (signal.SIGKILL, 5)):
                try:
                    os.killpg(proc.pid, sig)
                    proc.wait(timeout=wait)
                    break
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    continue
    if out.is_file():
        return json.loads(out.read_text())
    tail = (logdir / 'launch.log').read_text(errors='replace').strip().splitlines()[-3:]
    why = ('the stack exited before the probe finished' if proc.returncode is not None
           and time.monotonic() - t0 < timeout else f'no verdict in {timeout:.0f} s')
    return {'ok': False, 'error': f'{why}; {logdir}/launch.log ends: ' + ' | '.join(tail),
            'checks': [], 'vehicle_id': car, 'vehicle_model': model}


def run_safety(model, targets, workdir: Path, env, timeout=150.0, rtf=1.0, extra=(), say=print):
    """(every car passed, [per-car result]): the per-car lines printed as they come."""
    workdir = Path(workdir) / model
    workdir.mkdir(parents=True, exist_ok=True)
    say(f'vehicle gate (safety): {model} on {", ".join(targets)}')
    results = []
    for car in targets:
        res = run_car(model, car, workdir, env, timeout, rtf, list(extra))
        results.append(res)
        if res.get('error'):
            say(f'FAIL  {car}: {res["error"]}')
        for c in res.get('checks', []):
            say(f"{'PASS' if c['ok'] else 'FAIL'}  {car} {c['check']}: {c['detail']}")
    (workdir / 'result.json').write_text(json.dumps(
        [{k: v for k, v in r.items() if k != 'samples'} for r in results], indent=2))
    failed = sum(0 if r.get('ok') else 1 for r in results)
    say(f'{len(targets) - failed}/{len(targets)} cars passed'
        + ('  -> SAFETY PASSED' if not failed else '  -> SAFETY FAILED'))
    return not failed, results


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 usage='%(prog)s [vehicle_model:=MODEL] [--cars LIST] [launch args]')
    ap.add_argument('--cars', default='', help='comma list (default: every car_* instance)')
    ap.add_argument('--timeout', type=float, default=150.0, help='per car [s wall]')
    ap.add_argument('--rtf', type=float, default=1.0, help="the plant's real-time factor")
    ap.add_argument('--workdir', default=str(REPO / 'tmp/lab2_vehicle_gate'))
    args, rest = ap.parse_known_args(argv)
    model, extra = split(rest)
    targets = [c for c in args.cars.split(',') if c] or cars()
    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', '73')
    if env['ROS_DOMAIN_ID'] == '0':
        print('refusing to run on ROS_DOMAIN_ID 0 (shared host)')
        return 1
    ok, results = run_safety(model, targets, Path(args.workdir), env, args.timeout, args.rtf,
                             extra, say=lambda m: print(m, flush=True))
    return sum(0 if r.get('ok') else 1 for r in results)


if __name__ == '__main__':
    sys.exit(main())
