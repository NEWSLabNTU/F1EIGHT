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

With --team, DIR is a Part 2 team directory instead, `~/lab2/team<N>/`:

    <student_id>_lab_2_1/               the one controller the team drives with
    path_tracker.param.yaml             the team's lookahead (optional)

The tracker file may be the package's whole `path_tracker.param.yaml` or just
the three settings a team may change, flat or under `tracker:`. Those three are
merged into the package's file, written to tmp/lab2/, and passed as
tracker_param_file:=. Any other setting that differs from the package's is an
error: the handout allows only the lookahead to change.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import yaml

CLASS = 'lab2_waypoint.speed_pid:SpeedPID'
CARS = ('A', 'B', 'C')
REPO = Path(__file__).resolve().parents[3]
TRACKER_DEFAULT = REPO / 'src/lab/autosdv_waypoint_lab/config/path_tracker.param.yaml'
TRACKER_FILE = 'path_tracker.param.yaml'
TUNABLE = ('k_lookahead', 'lookahead_min', 'lookahead_max')


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


def team_controller(team: Path) -> Path:
    """The single controller directory inside a team directory."""
    found = []
    for d in sorted(p for p in team.iterdir() if p.is_dir()):
        try:
            package_dir(d)
            found.append(d)
        except SubmissionError:
            pass
    if len(found) != 1:
        names = ', '.join(d.name for d in found) or 'none'
        raise SubmissionError(
            f'{team}: expected exactly one <student_id>_lab_2_1 directory holding '
            f'lab2_waypoint/, found {names}')
    return found[0]


def _ros_params(doc: dict) -> dict:
    """The ros__parameters mapping of a param file, or the document itself."""
    for v in doc.values():
        if isinstance(v, dict) and isinstance(v.get('ros__parameters'), dict):
            return v['ros__parameters']
    return doc


def merge_tracker(team_file: Path, default_file: Path = TRACKER_DEFAULT):
    """(merged param document, {setting: value}) for a team's tracker file."""
    base = yaml.safe_load(default_file.read_text())
    base_params = _ros_params(base)
    theirs = _ros_params(_load(team_file))
    # the three settings may sit at the top level or under tracker:
    tracker = theirs.get('tracker', {}) if isinstance(theirs.get('tracker'), dict) else {}
    flat = {k: v for k, v in theirs.items() if k != 'tracker'}
    chosen = {}
    for scope, values, ref in (('', flat, base_params), ('tracker.', tracker, base_params['tracker'])):
        for k, v in values.items():
            if k in TUNABLE:
                chosen[k] = v
            elif ref.get(k) != v:
                raise SubmissionError(
                    f'{team_file}: {scope}{k} is {v!r}, the package has {ref.get(k)!r}; '
                    f'only {", ".join(TUNABLE)} may change')
    for k, v in chosen.items():
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
            raise SubmissionError(f'{team_file}: {k} must be a number >= 0, not {v!r}')
    merged_tracker = dict(base_params['tracker'], **{k: float(v) for k, v in chosen.items()})
    if merged_tracker['lookahead_min'] > merged_tracker['lookahead_max']:
        raise SubmissionError(f'{team_file}: lookahead_min is above lookahead_max')
    base_params['tracker'] = merged_tracker
    return base, {k: merged_tracker[k] for k in TUNABLE}


def resolve_team(path, out_dir: Path = REPO / 'tmp' / 'lab2'):
    """(package dir, [launch args], [warnings]) for a Part 2 team directory."""
    team = Path(path).expanduser().resolve()
    if not team.is_dir():
        raise SubmissionError(f'{team}: no such directory')
    pkg, args, warnings = resolve(team_controller(team))
    tf = team / TRACKER_FILE
    if not tf.is_file():
        warnings.append(f'{tf} missing: the default lookahead is used')
        return pkg, args, warnings
    doc, chosen = merge_tracker(tf)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f'tracker_{team.name}.param.yaml'
    out.write_text('# GENERATED by scripts/testing/lab2/submission.py from\n'
                   f'# {tf}\n' + yaml.safe_dump(doc, sort_keys=False))
    warnings.append('lookahead ' + ', '.join(f'{k} {v:g}' for k, v in chosen.items()))
    return pkg, args + [f'tracker_param_file:={out}'], warnings


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('dir')
    ap.add_argument('--team', action='store_true',
                    help='DIR is a Part 2 team directory (controller + path_tracker.param.yaml)')
    what = ap.add_mutually_exclusive_group()
    what.add_argument('--pythonpath', action='store_true', help='print the PYTHONPATH entry only')
    what.add_argument('--launch-args', action='store_true', help='print the launch args, one per line')
    args = ap.parse_args(argv)
    try:
        pkg, launch_args, warnings = (resolve_team if args.team else resolve)(args.dir)
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
