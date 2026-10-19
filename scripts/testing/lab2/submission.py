#!/usr/bin/env python3
"""Check a team's vehicle model before Lab 2 Part 1 runs it on the simulated car.

    python3 scripts/testing/lab2/submission.py --part1 DIR            # model, source
    python3 scripts/testing/lab2/submission.py --part1 DIR --model    # the model name only

DIR is the Part 1 simulation vehicle model's source, lab2_sim_vehicle (one shared name for
everyone; the car models, teamN_vehicle, are Part 2's): `lab2_sim_vehicle_launch/` as the
handout ships it (the directory holding the three packages), or a directory holding
it, e.g. a student's `<student_id>_lab_2_1/` submission (the model plus a report). It must
hold

    lab2_sim_vehicle_description/       a package (package.xml)
    lab2_sim_vehicle_interface/         a package, with lab2_sim_vehicle_interface/speed_pid.py
                                        defining class SpeedPID
    lab2_sim_vehicle_launch/            a package, with config/sim/speed_controller.param.yaml:
        /**:
          ros__parameters:
            speed_controller_params: {kp: ..., ...}   only the keys in part1_keys(), each of
                                                      its type and in its range

and nothing in it may import autosdv_waypoint_lab.reference (the TA's reference controllers)
or name a class there. Nothing is run or imported: the checks read the files. It prints the
model name and its source directory (--model: the name alone; --name NAME checks another
model), warnings on stderr; errors
exit 2. `just lab2 sim` (scripts/testing/lab2/part1_run.py) runs the same check in-process.

--part1 is the only mode (the old lab2_speed/ package checks are gone with the
SpeedPID-only lab); it is accepted, and implied when left out.
"""
import argparse
import ast
import json
import math
import os
import re
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
DEFAULT_VEHICLE_ID = 'sim'       # where no car is known: the sims and the bench run sim, the nominal car
PWM_KEYS = ('init_pwm', 'min_pwm', 'max_pwm')


class SubmissionError(ValueError):
    pass


def _instances():
    try:
        from autosdv_vehicle_interface import car_profile
    except ImportError:     # no ROS environment sourced: the source tree
        sys.path.insert(0, str(REPO / 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface'))
        from autosdv_vehicle_interface import car_profile
    return car_profile


def actuator_pwm(vehicle_id: str = None) -> dict:
    """init_pwm, min_pwm and max_pwm of the car (the limits of the offsets): actuator.yaml
    with individual_params/config/<vehicle_id>/autosdv_vehicle/actuator.param.yaml over it.
    No vehicle_id: sim, the simulated nominal car."""
    try:
        p = _instances().load(vehicle_id or DEFAULT_VEHICLE_ID)
    except ValueError as e:
        raise SubmissionError(str(e)) from None
    return {k: int(p[k]) for k in PWM_KEYS}


def part1_keys(pwm: dict = None) -> dict:
    """{key: (type, low, high, low_inclusive)}: what config/speed_pid.yaml may hold. pwm: the
    car's actuator_pwm() (default sim's): forward headroom max_pwm - init_pwm, brake room
    init_pwm - min_pwm.

    The course handout's selfcheck.py carries a copy of this table (KEYS); a change
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


def _load_unique(path: Path):
    """The YAML document at path, refusing a key given twice."""
    if not path.is_file():
        raise SubmissionError(f'{path}: missing')
    try:
        return yaml.load(path.read_text(), Loader=_UniqueKeyLoader)
    except SubmissionError as e:
        raise SubmissionError(f'{path}: {e}') from None
    except yaml.YAMLError as e:
        raise SubmissionError(f'{path}: not valid YAML ({e})') from None


def check_part1_yaml(path: Path, pwm: dict = None):
    """(checked mapping, [warnings]) for one flat SpeedPID parameter file (speed_pid_part1.yaml,
    a Part 2 car file)."""
    path = Path(path)
    return check_part1_mapping(_load_unique(path), path, pwm)


def check_part1_mapping(data, path: Path, pwm: dict = None, where: str = ''):
    """(checked mapping, [warnings]) for the SpeedPID parameters `data` read from path (under
    `where`, e.g. 'speed_controller_params', for the messages)."""
    path = Path(path)
    label = f'{path}' + (f' ({where})' if where else '')
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise SubmissionError(f'{label}: expected a mapping of parameters, got {type(data).__name__}')
    keys = part1_keys(pwm)
    unknown = sorted(str(k) for k in data if k not in keys)
    if unknown:
        raise SubmissionError(f'{label}: unknown key(s) {", ".join(unknown)}; '
                              f'allowed: {", ".join(keys)}')
    out, warnings = {}, []
    for key, (kind, lo, hi, lo_incl) in keys.items():
        if key not in data:
            warnings.append(f'{path.name}: {key} is not set; the class falls back to its '
                            "default (or the actuator's value of the same name)")
            continue
        v = data[key]
        if kind is bool:
            if not isinstance(v, bool):
                raise SubmissionError(f'{label}: {key} must be true or false, not {v!r}')
            out[key] = v
            continue
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise SubmissionError(f'{label}: {key} must be a number, not {v!r}')
        if kind is int:
            if isinstance(v, float) and not v.is_integer():
                raise SubmissionError(f'{label}: {key} must be a whole number of counts, not {v!r}')
            v = int(v)
        elif not math.isfinite(v):
            raise SubmissionError(f'{label}: {key} must be finite, not {v!r}')
        else:
            v = float(v)
        if (v < lo if lo_incl else v <= lo) or v > hi:
            raise SubmissionError(f'{label}: {key} is {v!r}, outside '
                                  f'{"[" if lo_incl else "("}{lo:g}, {hi:g}]')
        out[key] = v
    if 'brake_enter' in out and 'brake_exit' in out and out['brake_exit'] >= out['brake_enter']:
        raise SubmissionError(f'{label}: brake_exit ({out["brake_exit"]:g}) must be below '
                              f'brake_enter ({out["brake_enter"]:g}): the gap is the hysteresis')
    return out, warnings


def flow_mapping(d: dict) -> str:
    """{kp: 15.0, ki: 5.0}: one launch-argument word, valid YAML (values as JSON)."""
    return '{' + ', '.join(f'{k}: {json.dumps(v)}' for k, v in d.items()) + '}'


# ---- Part 1 on the simulated car: the team's vehicle model (just lab2 sim)

try:
    from lab2_ws import SIM_MODEL                   # the one name; lab2_ws owns it
except ImportError:
    SIM_MODEL = 'lab2_sim_vehicle'
#: Part 1's model: ONE simulation vehicle model, the same name for everyone (the handout's
#: lab2_sim_vehicle_launch/); the car models, teamN_vehicle, are Part 2's
SIM_TUNING = Path('config') / 'sim' / 'speed_controller.param.yaml'
NESTED_UNDER = 'speed_controller_params'
REFERENCE_PKG = 'autosdv_waypoint_lab.reference'
_REFERENCE_NAME = re.compile(r'autosdv_waypoint_lab\s*[./]\s*reference\b')
_SKIP_DIRS = {'__pycache__', '.git', 'build', 'install', 'log', '.pytest_cache'}
_TEXT_SUFFIXES = {'.py', '.xml', '.yaml', '.yml', '.cfg', '.txt', '.json', '.launch'}


def _has_model(root: Path, model: str) -> bool:
    """Does <model>_launch (a package) sit directly in root?"""
    return (Path(root) / f'{model}_launch' / 'package.xml').is_file()


def part1_model_dir(path, model: str = SIM_MODEL) -> Path:
    """The source root of `model` (default the Part 1 simulation model): the directory
    holding <model>_launch/, <model>_interface/ and <model>_description/ -- `path` itself,
    or the one such directory inside it (a student's <student_id>_lab_2_1/, say)."""
    root = Path(path).expanduser().resolve()
    if not root.is_dir():
        raise SubmissionError(f'{root}: no such directory')
    if _has_model(root, model):
        return root
    found = [d for d in sorted(p for p in root.iterdir() if p.is_dir() and p.name not in _SKIP_DIRS)
             if _has_model(d, model)]
    if len(found) == 1:
        return found[0]
    if found:
        raise SubmissionError(f'{root}: {model} in several places inside ('
                              + ', '.join(str(d) for d in found) + '); keep one')
    raise SubmissionError(
        f'{root}: no {model}_launch/package.xml in it or one level below; give the '
        f'{model}_launch directory (the handout ships it), or a directory holding it')


def _defines_class(path: Path, name: str) -> bool:
    try:
        tree = ast.parse(path.read_text(), filename=str(path))
    except SyntaxError as e:
        raise SubmissionError(f'{path}: does not parse (line {e.lineno}: {e.msg})') from None
    return any(isinstance(n, ast.ClassDef) and n.name == name for n in tree.body)


def _py_reference_use(path: Path):
    """The first line of path that imports autosdv_waypoint_lab.reference or names it in a
    string (an import_module, a class path), docstrings aside; None when there is none."""
    try:
        tree = ast.parse(path.read_text(errors='replace'), filename=str(path))
    except SyntaxError:
        return None     # a speed_pid.py that does not parse is reported on its own
    docstrings = {id(n.value) for n in ast.walk(tree)
                  if isinstance(n, ast.Expr) and isinstance(n.value, ast.Constant)}
    for n in ast.walk(tree):
        if isinstance(n, ast.Import) and any(_REFERENCE_NAME.match(a.name) for a in n.names):
            return n.lineno
        if isinstance(n, ast.ImportFrom) and n.module:
            if _REFERENCE_NAME.match(n.module):
                return n.lineno
            if n.module == 'autosdv_waypoint_lab' and any(a.name == 'reference' for a in n.names):
                return n.lineno
        if (isinstance(n, ast.Constant) and isinstance(n.value, str) and id(n) not in docstrings
                and _REFERENCE_NAME.search(n.value)):
            return n.lineno
    return None


def _text_reference_use(path: Path):
    """The first line naming autosdv_waypoint_lab.reference outside a comment, or None."""
    text = path.read_text(errors='replace')
    if path.suffix == '.xml' or path.name.endswith('.launch'):
        text = re.sub(r'<!--.*?-->', lambda m: '\n' * m.group(0).count('\n'), text, flags=re.S)
        lines = text.splitlines()
    else:
        lines = [ln.split('#', 1)[0] for ln in text.splitlines()]
    for i, ln in enumerate(lines, 1):
        if _REFERENCE_NAME.search(ln):
            return i
    return None


def reference_uses(root: Path):
    """['file:line', ...]: every place under root that imports or names the TA's reference."""
    hits = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(d for d in dirnames if d not in _SKIP_DIRS)
        for f in sorted(filenames):
            p = Path(dirpath) / f
            if p.suffix not in _TEXT_SUFFIXES or not p.is_file():
                continue
            line = _py_reference_use(p) if p.suffix == '.py' else _text_reference_use(p)
            if line is not None:
                hits.append(f'{p.relative_to(root)}:{line}')
    return hits


def sim_tuning(path: Path, pwm: dict = None):
    """(checked speed_controller_params mapping, [warnings]) from a model's
    config/sim/speed_controller.param.yaml."""
    doc = _load_unique(path)
    shape = (f'{path}: expected `/**: {{ros__parameters: {{{NESTED_UNDER}: {{kp: ..., ...}}}}}}` '
             '(the SpeedPID parameters nested under speed_controller_params)')
    if not isinstance(doc, dict) or len(doc) != 1:
        raise SubmissionError(shape)
    (node, body), = doc.items()
    if not isinstance(body, dict) or not isinstance(body.get('ros__parameters'), dict):
        raise SubmissionError(shape)
    ros = body['ros__parameters']
    if NESTED_UNDER not in ros:
        flat = sorted(k for k in ros if k in part1_keys(pwm))
        raise SubmissionError(shape + (f'; {", ".join(flat)} sit directly under ros__parameters'
                                       if flat else ''))
    mapping, warnings = check_part1_mapping(ros[NESTED_UNDER], path, pwm,
                                            f'{node}.ros__parameters.{NESTED_UNDER}')
    others = sorted(str(k) for k in ros if k != NESTED_UNDER)
    if others:
        warnings.append(f'{path.name}: {", ".join(others)} under ros__parameters as well: loaded '
                        'into the actuator as they are, not checked here')
    return mapping, warnings


def resolve_part1(path, pwm: dict = None, model: str = SIM_MODEL):
    """(model name, source root, checked sim tuning, [warnings]) for the simulation model
    source at path, before Part 1 runs it on the simulated car. SubmissionError names the
    first problem."""
    root = part1_model_dir(path, model)
    for suffix in ('_description', '_interface', '_launch'):
        pkg = root / f'{model}{suffix}'
        if not (pkg / 'package.xml').is_file():
            raise SubmissionError(f'{root}: no {model}{suffix}/package.xml (a model is three '
                                  f'packages: {model}_description, _interface, _launch)')
    hits = reference_uses(root)
    if hits:
        raise SubmissionError(f'{root}: uses the TA reference ({REFERENCE_PKG}) at '
                              + ', '.join(hits) + '; the controller must be your own')
    pid = root / f'{model}_interface' / f'{model}_interface' / 'speed_pid.py'
    if not pid.is_file():
        raise SubmissionError(f"{pid}: missing (the model's SpeedPID)")
    if not _defines_class(pid, 'SpeedPID'):
        raise SubmissionError(f'{pid}: defines no class SpeedPID')
    mapping, warnings = sim_tuning(root / f'{model}_launch' / SIM_TUNING, pwm)
    return model, root, mapping, warnings


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('dir')
    ap.add_argument('--part1', action='store_true',
                    help='DIR is the Part 1 simulation model (lab2_sim_vehicle_launch/, or a '
                    'directory holding it): the default and only mode')
    ap.add_argument('--vehicle-id', default=None,
                    help='the car whose PWM bounds the offsets (default sim, the simulated '
                    'nominal car)')
    ap.add_argument('--model', action='store_true', help='print the model name only')
    ap.add_argument('--name', default=SIM_MODEL,
                    help=f'the model to look for (default {SIM_MODEL}, Part 1\'s)')
    args = ap.parse_args(argv)
    try:
        model, root, _, warnings = resolve_part1(args.dir, pwm=actuator_pwm(args.vehicle_id),
                                                 model=args.name)
    except SubmissionError as e:
        print(f'submission error: {e}', file=sys.stderr)
        return 2
    for w in warnings:
        print(f'warning: {w}', file=sys.stderr)
    print(model if args.model else f'model {model}\nsource {root}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
