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

With --part1, DIR is a Lab 2 Part 1 submission for the simulated car
(`just lab2 sim`), `<student_id>_lab_2_1/` holding `lab2_speed/`, or the
`lab2_speed/` package itself. It must hold

    lab2_speed/speed_pid.py             the SpeedPID (and any helper module beside it)
    config/speed_pid.yaml               every parameter, feed-forward included

and the result is

    PYTHONPATH                  the package directory, so `lab2_speed.speed_pid` imports
    speed_controller_class      lab2_speed.speed_pid:SpeedPID
    speed_controller_params     speed_pid.yaml, checked: only the keys in PART1_KEYS, each
                                of its type and in its range (no calibration file: the
                                feed-forward constants are given in speed_pid.yaml)

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


# ---- Part 1 on the simulated car: lab2_speed/ (just lab2 sim)

PART1_PACKAGE = 'lab2_speed'
PART1_CLASS = 'lab2_speed.speed_pid:SpeedPID'
ACTUATOR_YAML = REPO / 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/actuator.yaml'
_PWM_FALLBACK = {'init_pwm': 370, 'min_pwm': 360, 'max_pwm': 470}


def actuator_pwm(path: Path = ACTUATOR_YAML) -> dict:
    """init_pwm, min_pwm and max_pwm from the actuator's file (the limits of the offsets)."""
    try:
        doc = yaml.safe_load(path.read_text())['/**']['ros__parameters']
        return {k: int(doc[k]) for k in _PWM_FALLBACK}
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return dict(_PWM_FALLBACK)


def part1_keys(pwm: dict = None) -> dict:
    """{key: (type, low, high, low_inclusive)}: what config/speed_pid.yaml may hold.

    The course handout's lab2_speed/selfcheck.py carries a copy of this table (KEYS); a change
    here must be made there too.
    """
    pwm = pwm or actuator_pwm()
    return {
        'kp': (float, 0.0, 1000.0, True),
        'ki': (float, 0.0, 1000.0, True),
        'kd': (float, 0.0, 100.0, True),
        'integral_limit': (float, 0.0, 1000.0, False),
        'error_limit': (float, 0.0, 20.0, False),
        'integral_band': (float, 0.0, 20.0, False),
        'enable_conditional_integration': (bool, None, None, True),
        'derivative_filter_alpha': (float, 0.0, 1.0, False),
        'max_pwm_offset': (int, 1, pwm['max_pwm'] - pwm['init_pwm'], True),
        'max_brake_offset': (int, 0, pwm['init_pwm'] - pwm['min_pwm'], True),
        'kp_brake': (float, 0.0, 1000.0, True),
        'brake_enter': (float, 0.0, 5.0, False),
        'brake_exit': (float, 0.0, 5.0, True),
        'brake_deadband': (int, 0, pwm['init_pwm'] - pwm['min_pwm'], True),
        'ff_deadband': (float, 0.0, 50.0, True),
        'ff_gain': (float, 0.0, 1.0, False),
    }


class _UniqueKeyLoader(yaml.SafeLoader):
    """safe_load that refuses a key given twice (YAML keeps the last one silently)."""

    def construct_mapping(self, node, deep=False):
        seen = set()
        for key_node, _ in node.value:
            key = self.construct_object(key_node, deep=deep)
            if key in seen:
                raise SubmissionError(f'line {key_node.start_mark.line + 1}: {key} is given twice')
            seen.add(key)
        return super().construct_mapping(node, deep)


def part1_package_dir(path) -> Path:
    root = Path(path).expanduser().resolve()
    for cand in (root, root / PART1_PACKAGE):
        if (cand / PART1_PACKAGE / 'speed_pid.py').is_file():
            return cand
    raise SubmissionError(
        f'{root}: no {PART1_PACKAGE}/{PART1_PACKAGE}/speed_pid.py inside; give the '
        f'<student_id>_lab_2_1 directory or the {PART1_PACKAGE} package directory')


def part1_params(pkg: Path, pwm: dict = None):
    """(checked speed_controller_params mapping, [warnings]) from config/speed_pid.yaml."""
    return check_part1_yaml(pkg / 'config' / 'speed_pid.yaml', pwm)


def check_part1_yaml(path: Path, pwm: dict = None):
    """(checked mapping, [warnings]) for one Part 1 speed_pid.yaml."""
    path = Path(path)
    if not path.is_file():
        raise SubmissionError(f'{path}: missing')
    try:
        data = yaml.load(path.read_text(), Loader=_UniqueKeyLoader)
    except SubmissionError as e:
        raise SubmissionError(f'{path}: {e}') from None
    except yaml.YAMLError as e:
        raise SubmissionError(f'{path}: not valid YAML ({e})') from None
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise SubmissionError(f'{path}: expected a mapping of parameters, got {type(data).__name__}')
    keys = part1_keys(pwm)
    unknown = sorted(str(k) for k in data if k not in keys)
    if unknown:
        raise SubmissionError(f'{path}: unknown key(s) {", ".join(unknown)}; '
                              f'allowed: {", ".join(keys)}')
    out, warnings = {}, []
    for key, (kind, lo, hi, lo_incl) in keys.items():
        if key not in data:
            warnings.append(f'speed_pid.yaml: {key} is not set; the class falls back to its '
                            "default (or the actuator's value of the same name)")
            continue
        v = data[key]
        if kind is bool:
            if not isinstance(v, bool):
                raise SubmissionError(f'{path}: {key} must be true or false, not {v!r}')
            out[key] = v
            continue
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise SubmissionError(f'{path}: {key} must be a number, not {v!r}')
        if kind is int:
            if isinstance(v, float) and not v.is_integer():
                raise SubmissionError(f'{path}: {key} must be a whole number of counts, not {v!r}')
            v = int(v)
        elif not math.isfinite(v):
            raise SubmissionError(f'{path}: {key} must be finite, not {v!r}')
        else:
            v = float(v)
        if (v < lo if lo_incl else v <= lo) or v > hi:
            raise SubmissionError(f'{path}: {key} is {v!r}, outside '
                                  f'{"[" if lo_incl else "("}{lo:g}, {hi:g}]')
        out[key] = v
    if 'brake_enter' in out and 'brake_exit' in out and out['brake_exit'] >= out['brake_enter']:
        raise SubmissionError(f'{path}: brake_exit ({out["brake_exit"]:g}) must be below '
                              f'brake_enter ({out["brake_enter"]:g}): the gap is the hysteresis')
    return out, warnings


def resolve_part1(path):
    """(package dir, [launch args], [warnings]) for a Part 1 (simulated car) submission."""
    pkg = part1_package_dir(path)
    mapping, warnings = part1_params(pkg)
    return pkg, [f'speed_controller_class:={PART1_CLASS}',
                 f'speed_controller_params:={flow_mapping(mapping)}'], warnings


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
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--team', action='store_true',
                      help='DIR is a Part 2 team directory (controller + path_tracker.param.yaml)')
    mode.add_argument('--part1', action='store_true',
                      help='DIR is a Part 1 submission for the simulated car (lab2_speed/)')
    what = ap.add_mutually_exclusive_group()
    what.add_argument('--pythonpath', action='store_true', help='print the PYTHONPATH entry only')
    what.add_argument('--launch-args', action='store_true', help='print the launch args, one per line')
    args = ap.parse_args(argv)
    try:
        resolver = resolve_team if args.team else resolve_part1 if args.part1 else resolve
        pkg, launch_args, warnings = resolver(args.dir)
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
