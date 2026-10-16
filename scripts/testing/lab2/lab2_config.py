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

Keys: car, team, team_dir, submission, route, course_dir, reference, bag_dir. A missing
user file is fine as long as every key a recipe needs has a value; a missing
required key is one error line naming the key and the file to put it in.

Relative paths resolve against the file that sets them: the user's file's
directory, the repository root for the defaults and for the command line (the
recipes run from the repository root).

Recipes:

    vehicle     course_dir:= (below) and team:=TEAM_DIR, unless given
    gate        submission:=SUBMISSION, unless a controller is given
    record      --car --team --out-dir, and --name ROUTE_N (the next free one)
    grade       no bag: --score LATEST --reference REFERENCE
    check-bags  no bag: LATEST; --car --team when set
    plot        no bag: LATEST
    audit       no bag: LATEST; --car --team when set; --reference REFERENCE if it is a bag
    config      print every value and where it came from
    init        write a commented template to the user's file (never overwrites)

`team:=` means the team DIRECTORY in vehicle and gate (as it always has) and
the team NUMBER elsewhere.

Which course the car runs (`course_dir`): `auto` (the default) is the aligned
survey surveys/latest when `just lab2 survey-align` aligned it for `route`,
else the route's packaged course (its PLANNED boards, with a warning); `package`
is always the packaged course; a directory is that course. The bench
(`just lab2 run --bench`) takes `auto` as `package`: a survey is of the field.
"""
import os
import re
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
DEFAULTS = REPO / 'src/lab/autosdv_waypoint_lab/config/lab2_defaults.yaml'
USER_FILE = Path('~/lab2/lab2.yaml')
KEYS = ('car', 'team', 'team_dir', 'submission', 'route', 'course_dir', 'reference', 'bag_dir')
PATH_KEYS = ('team_dir', 'submission', 'reference', 'bag_dir')
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
    ('car', 'A', 'the car you drive: A, B or C'),
    ('team', '3', 'your team number'),
    ('team_dir', '~/lab2/team3', "Part 2: the team's controller directory and its path_tracker.param.yaml"),
    ('submission', '~/lab2/B12345678_lab_2_1',
     'Part 1: your controller, for `just lab2 gate` on a workstation'),
]
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
    p = Path(os.path.expanduser(str(value)))
    return str(p if p.is_absolute() else (base / p).resolve())


class Config:
    """The resolved settings: get(key), source(key), require(key)."""

    def __init__(self, cli=None, config_file=None, defaults_file=DEFAULTS, cwd=REPO):
        cli = dict(cli or {})
        self.explicit_file = config_file is not None
        self.file = Path(os.path.expanduser(str(config_file or USER_FILE)))
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
                return f'course_dir:={d}', ('course_dir', d), ''
            warn = (f'surveys/latest not used: {why}. Running the PACKAGED {route}, whose boards are '
                    'where the plan put them, not where they stand')
        else:
            warn = (f'no surveys/latest: running the PACKAGED {route}, whose boards are where the plan '
                    'put them (survey, then `just lab2 survey-align`)')
    arg = f'course_dir:={route}' if '/' in route else f'route:={route}'
    return arg, ('route', route), warn


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
        own_flags = {'--team': 'team_dir', '--submission': 'submission'}
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
        elif sep and launch and key == 'team':
            cli['team_dir'] = val
        elif sep and key in KEYS:
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
        if c.source('submission') == 'command line' and c.source('team_dir') != 'command line':
            out.append(f'submission:={c.get("submission")}')
            use('submission')
        elif c.source('team_dir') == 'command line' or not _has(rest, 'speed_controller_class:='):
            out.append(f'team:={c.require("team_dir", "the team controller the car drives")}')
            use('team_dir')
    elif recipe == 'gate':
        out = list(rest)
        if 'team_dir' in c.sources and c.source('team_dir') == 'command line':
            out.append(f'team:={c.get("team_dir")}')
            use('team_dir')
        elif not _has(rest, 'speed_controller_class:='):
            out.append(f'submission:={c.require("submission", "the controller the gate runs")}')
            use('submission')
    elif recipe == 'record':
        car = c.require('car', 'recorded into the bag')
        team = c.require('team', 'recorded into the bag')
        bag_dir = c.require('bag_dir')
        name_given = _has(rest, '--name')
        out = ['--car', car, '--team', team, '--out-dir', bag_dir]
        use('car'), use('team'), use('bag_dir')
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


def init(argv, cwd=REPO) -> str:
    cfg_file, cli, rest = split_args('init', argv)
    if rest:
        raise ConfigError(f'unexpected argument {rest[0]!r}')
    path = Path(os.path.expanduser(str(cfg_file or USER_FILE)))
    if not path.is_absolute():
        path = (Path(cwd) / path).resolve()
    if path.exists():
        raise ConfigError(f'{path} exists; edit it, or remove it first (init never overwrites)')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(template(cli))
    return f'wrote {path}; fill in car, team, team_dir and submission'


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
