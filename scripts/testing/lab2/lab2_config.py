#!/usr/bin/env python3
"""The `just lab2` settings: built-in defaults, the user's file, the command line.

    python3 scripts/testing/lab2/lab2_config.py RECIPE [ARGS...]

prints the arguments RECIPE should run with, one per line, and one dim line on
stderr with the values it resolved. The recipes in just/lab2.just call it
through scripts/testing/lab2/lab2_config.sh and stay thin.

Precedence, highest first:

    key:=value on the command line      (and the tools' own flags, e.g. --car)
    the user's file                     ~/lab2/lab2.yaml, or config:=FILE / --config FILE
    the built-in defaults               src/lab/autosdv_waypoint_lab/config/lab2_defaults.yaml

Keys: car, team, vehicle_model, part1_submission, route, course_dir, reference, bag_dir (and
team_dir, submission: read from an old file and ignored, refused on the command line). A missing
user file is fine as long as every key a recipe needs has a value; a missing
required key is one error line naming the key and the file to put it in.

Relative paths resolve against the file that sets them: the user's file's
directory, the repository root for the defaults and for the command line (the
recipes run from the repository root).

Recipes:

    vehicle     course_dir:= (below), vehicle_model:= (below) and vehicle_id:=car_<car>
                (on the car `car` is required; the bench takes sim, the simulated nominal
                car, when it is not set, hardware:=sim passes vehicle_id:=sim)
    loc-check   vehicle_id:=car_<car> when `car` is set, else vehicle_id:=sim, the nominal
                car (nothing drives: the actuator is a dry run; the car only picks its
                calibration and IMU bias, which no car has measured yet); every other
                argument passes through
    survey      the same as loc-check (the survey's URDF is that car's calibration)
    gate        vehicle_model:= (below)
    sim         submission:=PART1_SUBMISSION when set (no default: without one the runner
                warns and runs the stub), unless --class / speed_controller_class:= is given;
                on its command line submission:= means part1_submission
    record      --car --team --vehicle-model --out-dir, and --name ROUTE_N (the next free one)
    grade       no bag: --score LATEST --reference REFERENCE
    check-bags  no bag: LATEST; --car --team when set
    plot        no bag: LATEST
    audit       no bag: LATEST; --car --team when set; --reference REFERENCE if it is a bag
    config      print every value and where it came from
    init        write a commented template to the user's file; on an existing file, set
                only the key:=value keys given (other lines and comments kept), and
                refuse when none is given

`car` (A, B or C) names the car in the bags AND picks its instance: car:=B passes
vehicle_id:=car_b to the stack, which loads individual_params/config/car_b/ (its
autosdv_vehicle/actuator.param.yaml PWM, its calibration, its autosdv.preset.yaml).
A vehicle_id:= given on the command line must agree with `car`.

The vehicle model (Autoware's vehicle_model: <model>_launch, _interface, _description) is
what a team submits (docs/design/lab2-team-vehicle-models.md): `vehicle_model` when set, else
team<team>_vehicle when `team` is set, else autosdv_vehicle, the reference. `team:=` is the
team NUMBER everywhere (team:=3 is team3_vehicle). The team_dir:= and submission:= of the
SpeedPID-only lab are gone from vehicle and gate: the controller lives in the model.

Which course the car runs (`course_dir`): `auto` (the default) is the aligned
survey surveys/latest when `just lab2 survey-align` aligned it for `route`,
else the route's packaged course (its PLANNED boards, with a warning); `package`
is always the packaged course; a directory is that course. The bench
(`just lab2 run --bench`) takes `auto` as `package`: a survey is of the field.

A survey aligned for one route serves every route on the same FIELD (same start
box, boards and start-box frame; a route's course.yaml names its field with
`field: lab2_uturn`, and a route without one is its own field). When `route` is
not the one surveys/latest is aligned for but shares its field, `auto` builds a
sibling surveys/<latest dir>.<route>/ once, re-solved for `route` from latest's
recorded drive (align_poses.csv, align_pnp.csv; the lab2_survey_align_solve
solver), and runs that. surveys/latest itself is never written; the sibling is
rebuilt when latest's alignment inputs change (a hash in its .reused_from). One
stderr line names the survey a run uses; on any failure, the packaged course
with the warning, as before.
"""
import json
import os
import re
import sys
from pathlib import Path
from typing import Optional, Tuple

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lab2_env  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
try:
    from autosdv_vehicle_interface import car_profile as _instances   # vehicle_ids: names(), load()
except ImportError:     # no ROS environment sourced: the source tree
    sys.path.insert(0, str(REPO / 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface'))
    from autosdv_vehicle_interface import car_profile as _instances   # vehicle_ids: names(), load()
DEFAULTS = REPO / 'src/lab/autosdv_waypoint_lab/config/lab2_defaults.yaml'
USER_FILE = Path('~/lab2/lab2.yaml')     # ~/lab2 is $LAB2_HOME when that is set


def user_file() -> Path:
    return lab2_env.lab2_home() / 'lab2.yaml'
KEYS = ('car', 'team', 'vehicle_model', 'team_dir', 'submission', 'part1_submission', 'route',
        'course_dir', 'reference', 'bag_dir')
#: the SpeedPID-only lab's keys: an old file that sets them still loads (they are ignored, with
#: a warning); on the command line of vehicle and gate they are refused
RETIRED_KEYS = ('team_dir', 'submission')
REFERENCE_MODEL = 'autosdv_vehicle'
PATH_KEYS = ('team_dir', 'submission', 'part1_submission', 'reference', 'bag_dir')
LAUNCH_RECIPES = ('vehicle', 'gate')
COURSE_CHOICES = ('auto', 'package')
SURVEY_LATEST = REPO / 'surveys' / 'latest'
COURSES = REPO / 'src/lab/autosdv_waypoint_lab/config/courses'

TEMPLATE_HEAD = """\
# Lab 2 settings for the `just lab2` commands.
#
# Any key can also be given on the command line as key:=value, which wins over
# this file; a key left out uses the built-in default. `just lab2 config` shows
# what every command would use and where each value came from.
"""
TEMPLATE_KEYS = [
    ('car', 'A', "the CAR's letter, A, B or C: the TA sets it once per car (`just lab2 init car:=A`); "
     "it is the bags' label and picks the car's vehicle_id (car_a, car_b, car_c)"),
    ('team', '3', 'optional: `just lab2 run team:=3` names the team per run (record, audit, ...) '
     'and runs its vehicle model, team3_vehicle'),
    ('vehicle_model', 'team3_vehicle',
     'optional: the vehicle model to run; default team<team>_vehicle, else autosdv_vehicle '
     '(the reference)'),
    ('part1_submission', '~/lab2/B12345678_lab_2_1',
     'optional: `just lab2 sim` finds the one ~/lab2/*_lab_2_1 by itself; this picks one'),
]
INIT_KEYS_HINT = (
    'Nothing here is needed for the student commands:\n'
    '  Part 1, any machine    just lab2 sim             finds ~/lab2/<id>_lab_2_1 itself\n'
    '  Part 2                 just lab2 run team:=3     team3_vehicle; the car or the bench\n'
    'On a car the TA sets its letter once: `just lab2 init car:=A`.\n'
    '`just lab2 config` shows what every command will use.')
TEMPLATE_DEFAULTS = [
    ('route', 'the graded course'),
    ('course_dir', 'auto: surveys/latest once survey-align aligned it, else the route\'s package course'),
    ('reference', "the car's TA reference bag, for `just lab2 grade`"),
    ('bag_dir', 'where bags are recorded and looked for'),
]


class ConfigError(Exception):
    pass


def _load_yaml(path: Path) -> dict:
    try:
        data = yaml.safe_load(path.read_text())
    except yaml.YAMLError as e:
        raise ConfigError(f'{path}: not valid YAML ({e})') from None
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ConfigError(f'{path}: expected key: value lines')
    unknown = sorted(set(data) - set(KEYS))
    if unknown:
        raise ConfigError(f'{path}: unknown key {unknown[0]!r}; the keys are {", ".join(KEYS)}')
    return data


def _path(value, base: Path) -> str:
    v = str(value)
    if v == '~/lab2' or v.startswith('~/lab2/'):
        v = str(lab2_env.lab2_home()) + v[len('~/lab2'):]
    p = Path(os.path.expanduser(v))
    return str(p if p.is_absolute() else (base / p).resolve())


class Config:
    """The resolved settings: get(key), source(key), require(key)."""

    def __init__(self, cli=None, config_file=None, defaults_file=DEFAULTS, cwd=REPO):
        cli = dict(cli or {})
        self.explicit_file = config_file is not None
        self.file = Path(os.path.expanduser(str(config_file or user_file())))
        if not self.file.is_absolute():
            self.file = (Path(cwd) / self.file).resolve()
        self.file_exists = self.file.is_file()
        if self.explicit_file and not self.file_exists:
            raise ConfigError(f'config file {self.file} not found (`just lab2 init config:={self.file}` '
                              'writes one)')
        self.values, self.sources = {}, {}
        layers = [(_load_yaml(Path(defaults_file)), 'default', REPO)]
        if self.file_exists:
            layers.append((_load_yaml(self.file), str(self.file), self.file.parent))
        layers.append((cli, 'command line', Path(cwd)))
        for data, source, base in layers:
            for k, v in data.items():
                if k not in KEYS:
                    raise ConfigError(f'unknown key {k!r}; the keys are {", ".join(KEYS)}')
                if v is None or v == '':
                    continue
                if k in PATH_KEYS:
                    v = _path(v, base)
                elif k == 'course_dir' and str(v) not in COURSE_CHOICES:
                    v = _path(v, base)
                elif k == 'route' and '/' in str(v):
                    v = _path(v, base)
                else:
                    v = str(v)
                self.values[k], self.sources[k] = v, source

    def get(self, key, default=None):
        return self.values.get(key, default)

    def source(self, key):
        return self.sources.get(key, 'not set')

    def require(self, key, why=''):
        if key not in self.values:
            example = {k: ex for k, ex, _ in TEMPLATE_KEYS}.get(key, '...')
            hint = '' if self.file_exists else ' (`just lab2 init` writes it)'
            raise ConfigError(
                f'no {key} set{" (" + why + ")" if why else ""}: add `{key}: {example}` to '
                f'{self.file}{hint}, or pass {key}:=...')
        return self.values[key]


# ---- the course -----------------------------------------------------------------------------

def _route_fields(course_dir: Path):
    d = yaml.safe_load((course_dir / 'course.yaml').read_text()) or {}
    sp = d.get('start_pose') or {}
    name = str(d.get('path', 'path.csv'))
    return (name, (course_dir / name).read_bytes(), bool(d.get('closed', True)), int(d.get('laps', 1)),
            round(float(d.get('v_ref', 0.8)), 6),
            tuple(round(float(sp.get(k, 0.0)), 6) for k in ('x', 'y', 'yaw'))), d


def aligned_for(course_dir, route: str):
    """(True, '') if course_dir is a survey that `just lab2 survey-align` aligned for the
    route `route` (a packaged course name), with that route's path, start pose, laps
    and v_ref unchanged since; else (False, why)."""
    course_dir = Path(course_dir)
    if not (course_dir / 'course.yaml').is_file():
        return False, f'{course_dir} has no course.yaml'
    template = Path(route) if '/' in str(route) else COURSES / str(route)
    try:
        mine, d = _route_fields(course_dir)
    except (OSError, ValueError, yaml.YAMLError) as e:
        return False, f'{course_dir}: {e}'
    al = d.get('alignment')
    if not isinstance(al, dict):
        return False, f'{course_dir} is not aligned (run `just lab2 survey-align`)'
    if str(al.get('route')) != template.name:
        return False, f'{course_dir} is aligned for route {al.get("route")!r}, not {template.name!r}'
    try:
        theirs, _ = _route_fields(template)
    except (OSError, ValueError, yaml.YAMLError) as e:
        return False, f'route {template}: {e}'
    if mine != theirs:
        return False, (f'{course_dir} was aligned for an older {template.name} (its path, start pose, '
                       'laps or v_ref differ): run `just lab2 survey-align` again')
    return True, ''


def _template(route: str) -> Path:
    return Path(route) if '/' in str(route) else COURSES / str(route)


def field_of(route: str) -> str:
    """The field a route stands on: its course.yaml's `field` (a straight test route says
    `field: lab2_uturn`), else the route itself. Routes on one field share its start box,
    its boards and its START-BOX frame, so one aligned survey serves them all."""
    t = _template(route)
    try:
        d = yaml.safe_load((t / 'course.yaml').read_text()) or {}
    except (OSError, yaml.YAMLError):
        d = {}
    return str(d.get('field') or t.name)


def same_field(a: str, b: str) -> bool:
    return field_of(a) == field_of(b)


REUSE_KEY = '.reused_from'


def _reuse_key(latest: Path, route: str) -> str:
    """What the sibling is derived from: latest's alignment inputs (stage 1, the recorded
    drive, the multi-board poses, whether it was forced) and the route template."""
    import hashlib
    h = hashlib.sha256()
    t = _template(route)
    d = yaml.safe_load((latest / 'course.yaml').read_text()) or {}
    h.update(repr(bool((d.get('alignment') or {}).get('forced'))).encode())
    for p in (latest / 'stage1' / 'course.yaml', latest / 'align_poses.csv', latest / 'align_pnp.csv',
              t / 'course.yaml', t / str((yaml.safe_load((t / 'course.yaml').read_text()) or {})
                                          .get('path', 'path.csv'))):
        h.update(p.name.encode())
        h.update(p.read_bytes() if p.is_file() else b'-')
    return h.hexdigest()


def _solve_sibling(sib: Path, route: str, forced: bool) -> Tuple[bool, str]:
    """Align `sib` (stage1/ and the drive already copied in) for `route`: the solver of
    `lab2_survey_align_solve`, in this process, its report into sib/align_report.txt."""
    for pkg in ('autosdv_marker_survey', 'autosdv_waypoint_lab'):
        p = str(REPO / 'src/lab' / pkg)
        try:
            __import__(pkg)
        except ImportError:
            sys.path.insert(0, p)
    from autosdv_marker_survey import align as al
    samples = al.read_samples(str(sib / al.POSES_FILE))
    pnp_file = sib / al.PNP_FILE
    pnp = al.read_pnp(str(pnp_file)) if pnp_file.is_file() else None
    a, _ = al.solve_dir(str(sib), str(_template(route)), samples, force=forced, log=lambda *x: None,
                        pnp=pnp)
    return a.ok, '; '.join(a.problems)


def reuse_survey(latest: Path, route: str) -> Tuple[Optional[Path], str]:
    """(surveys/<latest name>.<route>, how) when `latest` is aligned for another route on
    the same field: a sibling copy re-solved for `route` from latest's recorded drive,
    built once and rebuilt when latest's alignment inputs change. latest itself is never
    written. (None, why not) otherwise, or on any failure."""
    import shutil
    try:
        d = yaml.safe_load((latest / 'course.yaml').read_text()) or {}
    except (OSError, yaml.YAMLError) as e:
        return None, str(e)
    al_ = d.get('alignment')
    if not isinstance(al_, dict):
        return None, f'{latest} is not aligned (run `just lab2 survey-align`)'
    other = str(al_.get('route'))
    if other == _template(route).name:
        return None, 'aligned for this route'
    if not (COURSES / other).is_dir() or not same_field(other, route):
        return None, (f'{latest} is aligned for route {other!r}, not {_template(route).name!r}, '
                      'and on another field')
    if not (latest / 'align_poses.csv').is_file():
        return None, (f'{latest} is aligned for {other!r} but has no align_poses.csv to re-solve '
                      f'for {_template(route).name!r} from')
    if not (latest / 'stage1' / 'course.yaml').is_file():
        return None, f'{latest} has no stage1/course.yaml'
    real = latest.resolve()
    sib = real.parent / f'{real.name}.{_template(route).name}'
    try:
        key = _reuse_key(real, route)
        if (sib / REUSE_KEY).is_file() and (sib / REUSE_KEY).read_text().split('\n')[0] == key \
                and aligned_for(sib, route)[0]:
            return sib, f'cached, re-solved from {real.name} (aligned for {other})'
        if sib.exists():
            if not (sib / REUSE_KEY).is_file():
                return None, f'{sib} exists and was not made here; not overwriting it'
            shutil.rmtree(sib)
        sib.mkdir()
        shutil.copytree(real / 'stage1', sib / 'stage1')
        for name in ('align_poses.csv', 'align_pnp.csv'):
            if (real / name).is_file():
                shutil.copy2(real / name, sib / name)
        shutil.copy2(real / 'stage1' / 'course.yaml', sib / 'course.yaml')
        ok, why = _solve_sibling(sib, route, bool(al_.get('forced')))
        if not ok:
            shutil.rmtree(sib, ignore_errors=True)
            return None, f're-solving {real.name} for {_template(route).name} refused: {why}'
        good, why = aligned_for(sib, route)
        if not good:
            shutil.rmtree(sib, ignore_errors=True)
            return None, why
        (sib / REUSE_KEY).write_text(f'{key}\n{real}\n')
        return sib, f're-solved from {real.name} (aligned for {other}) for {_template(route).name}'
    except Exception as e:     # anything: the packaged course, with the warning
        shutil.rmtree(sib, ignore_errors=True)
        return None, f'reusing {real.name} for {_template(route).name} failed: {e}'


def course_for(c: 'Config', bench: bool = False):
    """(the launch argument naming the course, the summary entry, a warning or '')."""
    route = c.require('route')
    choice = c.get('course_dir', 'auto')
    if choice not in COURSE_CHOICES:
        if not Path(choice, 'course.yaml').is_file():
            raise ConfigError(f'course_dir {choice} ({c.source("course_dir")}) has no course.yaml')
        return f'course_dir:={choice}', ('course_dir', choice), ''
    warn = ''
    if choice == 'auto' and not bench:
        latest = SURVEY_LATEST
        if latest.is_dir():
            ok, why = aligned_for(latest, route)
            if ok:
                d = str(latest.resolve())
                print(f'survey: {d} (aligned for {_template(route).name})', file=sys.stderr)
                return f'course_dir:={d}', ('course_dir', d), ''
            sib, how = reuse_survey(latest, route)
            if sib is not None:
                print(f'survey: {sib} ({how})', file=sys.stderr)
                return f'course_dir:={sib}', ('course_dir', str(sib)), ''
            if how != 'aligned for this route' and 'another field' not in how and \
                    'is not aligned' not in how:
                why = how
            warn = (f'surveys/latest not used: {why}. Running the PACKAGED {route}, whose boards are '
                    'where the plan put them, not where they stand')
        else:
            warn = (f'no surveys/latest: running the PACKAGED {route}, whose boards are where the plan '
                    'put them (survey, then `just lab2 survey-align`)')
    arg = f'course_dir:={route}' if '/' in route else f'route:={route}'
    return arg, ('route', route), warn


# ---- the car's instance (vehicle_id) ----------------------------------------------------

#: what hardware:=sim runs when no car is set: the plant models actuator.yaml's car, the
#: simulated nominal car individual_params/config/sim (never a physical car)
SIM_VEHICLE_ID = 'sim'


def vehicle_id_of(car: str) -> str:
    """'B' (or 'b', 'car_b') -> 'car_b', one of individual_params' config/<vehicle_id>/."""
    v = str(car).strip().lower()
    name = v if v.startswith('car_') else f'car_{v}'
    known = [n for n in _instances.names() if n.startswith('car_')]   # not sim: no car
    if name not in known:
        raise ConfigError(f'car {car!r}: not a known car; one of '
                          + ', '.join(n[len('car_'):].upper() for n in known))
    return name


def vehicle_id_for(c: 'Config', rest, required: bool, fallback: Optional[str] = None):
    """(the vehicle_id:= argument to append or None, the vehicle_id or None).

    `car` picks the instance; a vehicle_id:= already among `rest` must agree with it (and
    is then kept, not repeated). required: on the car, where neither is a refusal;
    elsewhere `fallback` is passed when given, else the launch file's default (sim)
    stands."""
    given = [a.split(':=', 1)[1] for a in rest if a.startswith('vehicle_id:=')]
    explicit = given[-1] if given else None
    if explicit is not None and explicit not in _instances.names():
        raise ConfigError(f'vehicle_id:={explicit}: one of {", ".join(_instances.names())}')
    car = c.get('car')
    if car and str(car).strip().lower() == SIM_VEHICLE_ID:
        # SIM: the bags' label for the simulated nominal car (the bench, integration-test)
        if required:
            raise ConfigError(f'car {car!r} is the simulated car; on a car set its letter, A, B or C '
                              '(`just lab2 init car:=A`)')
        mine = SIM_VEHICLE_ID
    else:
        mine = vehicle_id_of(car) if car else None
    if explicit and mine and explicit != mine:
        raise ConfigError(f'vehicle_id:={explicit} disagrees with car {car} ({c.source("car")}), '
                          f'which is {mine}: drop vehicle_id:= or fix car')
    if explicit:
        return None, explicit
    if mine:
        return f'vehicle_id:={mine}', mine
    if required:
        raise ConfigError(
            'no car set (the stack loads that car\'s instance, individual_params/config/car_<car>; '
            f'the cars\' neutral PWMs differ): `just lab2 init car:=A|B|C` sets it in {c.file}, '
            'or pass car:=A|B|C')
    if fallback:
        return f'vehicle_id:={fallback}', fallback
    return None, None


# ---- the vehicle model ------------------------------------------------------------------

def team_model(team) -> str:
    """3 -> team3_vehicle (just lab2 new-team 3 generates it); 0 -> the reference, the TA's
    runs (the integration test records as team 0)."""
    t = str(team).strip()
    if not t.isdigit():
        raise ConfigError(f'team {team!r}: a team NUMBER (team:=3 is team3_vehicle); a team '
                          'directory is no longer how a team is named')
    return f'team{int(t)}_vehicle' if int(t) else REFERENCE_MODEL


def vehicle_model_for(c: 'Config') -> str:
    """`vehicle_model` when set, else team<team>_vehicle when `team` is, else the reference.
    A retired key (team_dir, submission) on the command line is refused, from a file
    ignored with a warning."""
    for k in RETIRED_KEYS:
        if c.get(k) is None:
            continue
        if c.source(k) == 'command line':
            raise ConfigError(
                f'{k}:= is gone: a team now submits its vehicle model, selected with '
                'vehicle_model:=teamN_vehicle or team:=N (docs/design/lab2-team-vehicle-models.md)')
        print(f'warning: {k} in {c.source(k)} is no longer used (the vehicle model is the '
              'submission); remove it', file=sys.stderr)
    model = c.get('vehicle_model')
    if model:
        if not re.fullmatch(r'[a-z][a-z0-9_]*', model) or model.endswith(('_launch', '_interface')):
            raise ConfigError(f'vehicle_model {model!r} ({c.source("vehicle_model")}): a model name '
                              'such as team3_vehicle, without _launch')
        return model
    if c.get('team'):
        return team_model(c.get('team'))
    return REFERENCE_MODEL


# ---- bags -------------------------------------------------------------------------------

def route_stem(route: str) -> str:
    """lab2_uturn -> uturn: the bag name's stem."""
    name = os.path.basename(os.path.normpath(str(route)))
    return name[len('lab2_'):] if name.startswith('lab2_') else name


def next_bag_name(bag_dir, route) -> str:
    """uturn_N, one past the highest N already under bag_dir (as lab2_uturn_N)."""
    stem = route_stem(route)
    pat = re.compile(rf'^lab2_{re.escape(stem)}_(\d+)$')
    taken = [int(m.group(1)) for p in (Path(bag_dir).iterdir() if Path(bag_dir).is_dir() else [])
             if (m := pat.match(p.name))]
    return f'{stem}_{max(taken, default=0) + 1}'


def latest_bag(bag_dir) -> Path:
    """The most recently finished bag under bag_dir/lab2_* (by its metadata.yaml)."""
    bags = [p for p in Path(bag_dir).glob('lab2_*') if (p / 'metadata.yaml').is_file()]
    if not bags:
        raise ConfigError(f'no bag under {bag_dir}/lab2_*: record one with `just lab2 record`, '
                          'or name it')
    return max(bags, key=lambda p: ((p / 'metadata.yaml').stat().st_mtime, p.name))


# ---- the command line -------------------------------------------------------------------

def split_args(recipe, argv):
    """(config_file, {key: value} from the command line, [the remaining args])."""
    cfg_file, cli, rest = None, {}, []
    it = iter(argv)
    launch = recipe in LAUNCH_RECIPES
    own_flags = {'--car': 'car', '--team': 'team', '--out-dir': 'bag_dir'} \
        if recipe in ('record', 'check-bags', 'audit') else {}
    if launch:
        own_flags = {'--team': 'team'}
    elif recipe == 'sim':
        own_flags = {'--submission': 'part1_submission'}
    for a in it:
        key, sep, val = a.partition(':=')
        flag, eq, fval = a.partition('=')
        if a == '--config' or a in own_flags:
            try:
                v = next(it)
            except StopIteration:
                raise ConfigError(f'{a} needs a value') from None
            if a == '--config':
                cfg_file = v
            else:
                cli[own_flags[a]] = v
        elif eq and flag == '--config':
            cfg_file = fval
        elif eq and flag in own_flags:
            cli[own_flags[flag]] = fval
        elif sep and key == 'config':
            cfg_file = val
        elif sep and recipe == 'sim' and key == 'submission':
            cli['part1_submission'] = val
        elif sep and key in KEYS and (recipe not in ('loc-check', 'survey') or key == 'car'):
            cli[key] = val
        else:
            rest.append(a)
    return cfg_file, cli, rest


def _positionals(args, value_flags):
    out, skip = [], False
    for a in args:
        if skip:
            skip = False
        elif a in value_flags:
            skip = True
        elif not a.startswith('-'):
            out.append(a)
    return out


def _has(args, *prefixes):
    return any(a == p or a.startswith(p) for a in args for p in prefixes)


def _home(p) -> str:
    h = str(Path.home())
    s = str(p)
    return '~' + s[len(h):] if s == h or s.startswith(h + '/') else s


def plan(recipe, argv, cwd=REPO, bench=False):
    """(args for the recipe's command, the dim summary line). bench: the vehicle bench
    (`course_dir: auto` is the packaged course there)."""
    cfg_file, cli, rest = split_args(recipe, argv)
    c = Config(cli, cfg_file, cwd=cwd)
    used = []

    def use(k, v=None):
        used.append(f'{k} {_home(v if v is not None else c.get(k))}')

    if recipe == 'vehicle':
        out = list(rest)
        if not _has(rest, 'course:=', 'route:='):
            arg, (k, v), warn = course_for(c, bench)
            out.append(arg)
            use(k, v)
            if warn:
                print(f'warning: {warn}', file=sys.stderr)
        model = vehicle_model_for(c)
        out.append(f'vehicle_model:={model}')
        use('vehicle_model', model)
        sim = _has(rest, 'hardware:=sim', 'hardware_sim:=true', 'hardware_sim:=True')
        on_car = not bench and not sim
        arg, vid = vehicle_id_for(c, rest, required=on_car,
                                  fallback=SIM_VEHICLE_ID if sim and not bench else None)
        if arg:
            out.append(arg)
        if vid:
            use('vehicle_id', vid)
    elif recipe in ('loc-check', 'survey'):
        out = list(rest)
        # nothing here drives, so no car is not a refusal: the nominal car's calibration
        arg, vid = vehicle_id_for(c, rest, required=False, fallback=SIM_VEHICLE_ID)
        if arg:
            out.append(arg)
        use('vehicle_id', vid)
    elif recipe == 'gate':
        out = list(rest)
        model = vehicle_model_for(c)
        out.append(f'vehicle_model:={model}')
        use('vehicle_model', model)
    elif recipe == 'sim':
        out = list(rest)
        if c.get('part1_submission') and not _has(rest, '--class', 'speed_controller_class:='):
            out.append(f'submission:={c.get("part1_submission")}')
            use('part1_submission')
    elif recipe == 'record':
        car = c.require('car', 'recorded into the bag')
        team = c.require('team', 'recorded into the bag')
        bag_dir = c.require('bag_dir')
        name_given = _has(rest, '--name')
        model = vehicle_model_for(c)
        out = ['--car', car, '--team', team, '--vehicle-model', model, '--out-dir', bag_dir]
        use('car'), use('team'), use('vehicle_model', model), use('bag_dir')
        if not name_given:
            name = next_bag_name(bag_dir, c.require('route'))
            out += ['--name', name]
            use('name', name)
        out += rest
    elif recipe == 'grade':
        out = list(rest)
        bags = _positionals(rest, ('--course', '--reference', '--route'))
        if not bags:
            if '--score' not in out:
                out.insert(0, '--score')
            bag = latest_bag(c.require('bag_dir'))
            out.append(str(bag))
            use('bag', bag)
        if '--score' in out and not _has(rest, '--reference'):
            ref = c.get('reference')
            if ref and Path(ref, 'metadata.yaml').is_file():
                out[out.index('--score') + 1:out.index('--score') + 1] = ['--reference', ref]
                use('reference')
            else:
                print(f'warning: reference {_home(ref)} ({c.source("reference")}) is not a bag; '
                      'scored against the fixed absolute pass lines', file=sys.stderr)
    elif recipe == 'check-bags':
        out = []
        for k in ('car', 'team'):
            if c.get(k):
                out += [f'--{k}', c.get(k)]
                use(k)
        out += rest
        if not _positionals(rest, ('--course', '--car', '--team')):
            bag = latest_bag(c.require('bag_dir'))
            out.append(str(bag))
            use('bag', bag)
    elif recipe == 'audit':
        out = []
        for k in ('car', 'team'):
            if c.get(k):
                out += [f'--{k}', c.get(k)]
                use(k)
        out += rest
        if not _positionals(rest, ('--car', '--team', '--reference', '--course', '--out')):
            bag = latest_bag(c.require('bag_dir'))
            out.append(str(bag))
            use('bag', bag)
        if not _has(rest, '--reference'):
            ref = c.get('reference')
            if ref and Path(ref, 'metadata.yaml').is_file():
                out += ['--reference', ref]
                use('reference')
    elif recipe == 'plot':
        out = list(rest)
        if not _positionals(rest, ()):
            bag = latest_bag(c.require('bag_dir'))
            out.append(str(bag))
            use('bag', bag)
    else:
        raise ConfigError(f'unknown recipe {recipe!r}')
    src = _home(c.file) + ('' if c.file_exists else ' (none)')
    return out, f'config {src}: ' + (', '.join(used) or 'nothing from it')


def show(argv, cwd=REPO) -> str:
    cfg_file, cli, rest = split_args('config', argv)
    if rest:
        raise ConfigError(f'unexpected argument {rest[0]!r}')
    c = Config(cli, cfg_file, cwd=cwd)
    lines = [f'# user file: {c.file}' + ('' if c.file_exists else '  (not found)'),
             f'# defaults:  {DEFAULTS.relative_to(REPO)}']
    w = max(len(k) for k in KEYS)
    for k in KEYS:
        v = c.get(k)
        lines.append(f'{k + ":":<{w + 1}} {_home(v) if v is not None else "-":<34}  # {c.source(k)}')
    return '\n'.join(lines)


def template(values=None) -> str:
    values = values or {}
    defaults = _load_yaml(DEFAULTS)
    w = 38
    out = [TEMPLATE_HEAD]
    for k, ex, why in TEMPLATE_KEYS:
        if k in values:
            out.append(f'{k + ": " + str(values[k]):<{w}}  # {why}')
        else:
            out.append(f'# {k + ": " + ex:<{w - 2}}  # {why}')
    out.append('')
    out.append('# The defaults, shown for reference; uncomment to change one.')
    for k, why in TEMPLATE_DEFAULTS:
        if k in values:
            out.append(f'{k + ": " + str(values[k]):<{w}}  # {why}')
        else:
            out.append(f'# {k + ": " + str(defaults.get(k, "")):<{w - 2}}  # {why}')
    return '\n'.join(out) + '\n'


def _yaml_scalar(value) -> str:
    """value as it would be written after `key: `, quoted only when YAML would misread it."""
    v = str(value)
    try:
        ok = v.strip() == v and v != '' and yaml.safe_load(f'k: {v}') == {'k': yaml.safe_load(v)} \
            and type(yaml.safe_load(v)) in (str, int, float) and ' #' not in v
    except yaml.YAMLError:
        ok = False
    return v if ok else json.dumps(v)


def _with_comment(head: str, line: str, comment: Optional[str]) -> str:
    """head, then the trailing comment of line kept at its column where it fits."""
    if not comment:
        return head
    col = line.index(comment)
    return head + ' ' * max(2, col - len(head)) + comment


_ENTRY = r'^{prefix}({key})\s*:(?P<value>.*?)(?:\s+(?P<comment>#.*))?$'


def set_keys(text: str, values: dict) -> Tuple[str, list]:
    """(text with each key set to its value, [what changed]). A key already set is
    rewritten in place (its comment kept); else its commented template line
    (`# key: example  # why`) is uncommented; else it is appended. Every other line is
    kept as it is."""
    lines = text.splitlines()
    changes = []
    for key, value in values.items():
        new = f'{key}: {_yaml_scalar(value)}'
        live = re.compile(_ENTRY.format(prefix='', key=re.escape(key)))
        hits = [i for i, ln in enumerate(lines) if live.match(ln)]
        if hits:
            for i in hits:
                m = live.match(lines[i])
                old = m.group('value').strip()
                if old == _yaml_scalar(value) or old == str(value):
                    changes.append(f'{key}: {old} (unchanged)')
                    continue
                lines[i] = _with_comment(new, lines[i], m.group('comment'))
                changes.append(f'{key}: {old or "(empty)"} -> {_yaml_scalar(value)}')
            continue
        dead = re.compile(_ENTRY.format(prefix=r'#\s*', key=re.escape(key)))
        hit = next((i for i, ln in enumerate(lines) if dead.match(ln)), None)
        if hit is not None:
            lines[hit] = _with_comment(new, lines[hit], dead.match(lines[hit]).group('comment'))
            changes.append(f'{key}: {_yaml_scalar(value)} (was commented out)')
        else:
            lines.append(new)
            changes.append(f'{key}: {_yaml_scalar(value)} (added at the end)')
    return '\n'.join(lines) + '\n', changes


def init(argv, cwd=REPO) -> str:
    cfg_file, cli, rest = split_args('init', argv)
    if rest:
        raise ConfigError(f'unexpected argument {rest[0]!r}')
    path = Path(os.path.expanduser(str(cfg_file or user_file())))
    if not path.is_absolute():
        path = (Path(cwd) / path).resolve()
    if path.exists():
        if not cli:
            raise ConfigError(f'{path} exists; init never overwrites it. Give the keys to set '
                              f'(`just lab2 init car:=A`), or edit it')
        _load_yaml(path)                    # refuse to edit a file that does not load
        text, changes = set_keys(path.read_text(), cli)
        try:
            data = yaml.safe_load(text) or {}
        except yaml.YAMLError as e:
            raise ConfigError(f'{path}: setting {", ".join(cli)} would break it ({e}); '
                              f'edit it by hand') from None
        wrong = [k for k, v in cli.items() if data.get(k) != yaml.safe_load(_yaml_scalar(v))]
        if wrong:
            raise ConfigError(f'{path}: could not set {", ".join(wrong)} in place; edit it by hand')
        if text != path.read_text():
            tmp = path.with_name(path.name + '.tmp')
            tmp.write_text(text)
            os.chmod(tmp, path.stat().st_mode & 0o7777)
            os.replace(tmp, path)
            head = f'updated {path}'
        else:
            head = f'{path}: nothing to change'
        return head + '\n' + '\n'.join('  ' + c for c in changes)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(template(cli))
    return f"wrote {path}\n{INIT_KEYS_HINT}"


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ('-h', '--help'):
        print(__doc__)
        return 0 if argv else 2
    recipe, rest = argv[0], argv[1:]
    try:
        if recipe == 'config':
            print(show(rest))
        elif recipe == 'init':
            print(init(rest))
        else:
            args, summary = plan(recipe, rest)
            print(f'\033[2m{summary}\033[0m', file=sys.stderr)
            for a in args:
                print(a)
    except ConfigError as e:
        print(f'lab2: {e}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
