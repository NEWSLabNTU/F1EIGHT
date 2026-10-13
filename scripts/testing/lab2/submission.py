#!/usr/bin/env python3
"""Turn a student's Lab 2 submission directory into the launch arguments that run it.

    python3 scripts/testing/lab2/submission.py DIR              # what would be used
    python3 scripts/testing/lab2/submission.py DIR --pythonpath
    python3 scripts/testing/lab2/submission.py DIR --launch-args   # one per line

DIR is the `<student_id>_lab_2_1/` directory (holding `lab2_waypoint/`), or the
`lab2_waypoint/` package itself (the one with `package.xml`). It must hold

    lab2_waypoint/speed_pid.py          the SpeedPID (and any helper module beside it)
    config/speed_pid.yaml               the gains
    config/calibration.yaml             deadband and gain from the car's speed-step CSV

and the result is

    PYTHONPATH                  the package directory, so `lab2_waypoint.speed_pid` imports
    speed_controller_class      lab2_waypoint.speed_pid:SpeedPID
    speed_controller_params     speed_pid.yaml, with ff_deadband / ff_gain from the
                                calibration unless speed_pid.yaml sets them itself

The merge is the handout's `params.speed_pid_params()`, re-implemented here on
purpose: the gate must not import code out of the student's copy of the bench.

`just lab2 gate|bench|vehicle submission:=DIR` calls this; see
scripts/testing/lab2/submission.sh.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import yaml

CLASS = 'lab2_waypoint.speed_pid:SpeedPID'
CARS = ('A', 'B', 'C')


class SubmissionError(ValueError):
    pass


def package_dir(path) -> Path:
    root = Path(path).expanduser().resolve()
    for cand in (root, root / 'lab2_waypoint'):
        if (cand / 'lab2_waypoint' / 'speed_pid.py').is_file():
            return cand
    raise SubmissionError(
        f'{root}: no lab2_waypoint/lab2_waypoint/speed_pid.py inside; give the '
        '<student_id>_lab_2_1 directory or the lab2_waypoint package directory')


def _load(path: Path) -> dict:
    if not path.is_file():
        raise SubmissionError(f'{path}: missing')
    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise SubmissionError(f'{path}: expected a mapping')
    return data


def params(pkg: Path):
    """(merged speed_controller_params mapping, [warnings])."""
    warnings = []
    data = dict(_load(pkg / 'config' / 'speed_pid.yaml'))
    cal = _load(pkg / 'config' / 'calibration.yaml')
    try:
        deadband, gain = float(cal['deadband']), float(cal['gain'])
    except (KeyError, TypeError, ValueError):
        raise SubmissionError(f'{pkg}/config/calibration.yaml: deadband and gain must be numbers') \
            from None
    if not (math.isfinite(deadband) and math.isfinite(gain)):
        raise SubmissionError(f'{pkg}/config/calibration.yaml: deadband and gain must be finite')
    if str(cal.get('car', '')).strip().upper() not in CARS:
        warnings.append(f'calibration.yaml: car is {cal.get("car")!r}, expected one of A, B, C')
    if gain <= 0.0:
        warnings.append('calibration.yaml: gain is 0, the calibration is not done')
    data.setdefault('ff_deadband', deadband)
    data.setdefault('ff_gain', gain)
    return data, warnings


def flow_mapping(d: dict) -> str:
    """{kp: 15.0, ki: 5.0}: one launch-argument word, valid YAML (values as JSON)."""
    return '{' + ', '.join(f'{k}: {json.dumps(v)}' for k, v in d.items()) + '}'


def resolve(path):
    """(package dir, [launch args], [warnings])."""
    pkg = package_dir(path)
    mapping, warnings = params(pkg)
    return pkg, [f'speed_controller_class:={CLASS}',
                 f'speed_controller_params:={flow_mapping(mapping)}'], warnings


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('dir')
    what = ap.add_mutually_exclusive_group()
    what.add_argument('--pythonpath', action='store_true', help='print the PYTHONPATH entry only')
    what.add_argument('--launch-args', action='store_true', help='print the launch args, one per line')
    args = ap.parse_args(argv)
    try:
        pkg, launch_args, warnings = resolve(args.dir)
    except SubmissionError as e:
        print(f'submission error: {e}', file=sys.stderr)
        return 2
    for w in warnings:
        print(f'warning: {w}', file=sys.stderr)
    if args.pythonpath:
        print(pkg)
    elif args.launch_args:
        print('\n'.join(launch_args))
    else:
        print(f'PYTHONPATH={pkg}')
        print('\n'.join(launch_args))
    return 0


if __name__ == '__main__':
    sys.exit(main())
