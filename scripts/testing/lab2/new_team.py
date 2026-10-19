#!/usr/bin/env python3
"""Generate Lab 2's Autoware vehicle models from the reference, autosdv_vehicle. TA tool:
the course's make_student_zip.sh puts its output in the handout.

    python3 scripts/testing/lab2/new_team.py --sim --dest DIR        lab2_sim_vehicle_launch/
    python3 scripts/testing/lab2/new_team.py --template --dest DIR   teamN_vehicle_launch/
    python3 scripts/testing/lab2/new_team.py N --dest DIR            team<N>_vehicle_launch/ (tests,
                                                                     the TA's own team models)

(--from-worktree: the working trees instead of HEAD; DIR defaults to ~/lab2/ws/src.) Two
models of the same shape:

    lab2_sim_vehicle_launch/    Part 1: the RC simulator's car, vehicle_id:=sim only. ONE
                                package, the same name for every student
    teamN_vehicle_launch/       Part 2: the template of a team's model for the real cars,
                                vehicle_id:=car_a|car_b|car_c, with "N" literally in its names.
                                Students rename it by hand: every "teamN" in directory and file
                                names and in file contents becomes e.g. "team3"
                                (test_new_team.py proves find/mv plus sed does it).

Students put both under AutoSDV's src/lab2_vehicles/ (gitignored) and build them with
colcon there, so the models resolve from AutoSDV's own install.

Each holds the three packages Autoware resolves for vehicle_model:=<model>, here for
teamN_vehicle:

    teamN_vehicle_description   vehicle_info.param.yaml, the URDF and the meshes of
                                autosdv_vehicle_description, installed BY REFERENCE at
                                build time (the car's geometry is the car's, not a team's)
    teamN_vehicle_interface     a renamed copy of autosdv_vehicle_interface: the actuator,
                                its safety shell, velocity_report, the gear, mode and
                                steering nodes; plus speed_pid.py, the Lab 2 student stub
                                (autosdv_waypoint_lab/stub/speed_pid_part1.py, byte for byte),
                                which is the model's speed controller. The team's to change.
    teamN_vehicle_launch        launch/vehicle_interface.launch.xml, renamed, its controller
                                class defaulting to teamN_vehicle_interface.speed_pid:SpeedPID,
                                and config/<vehicle_id>/speed_controller.param.yaml, the
                                team's tuning per vehicle (the SpeedPID parameters under
                                speed_controller_params): the sim model has config/sim/
                                only, seeded from config/speed_pid_part1.yaml (the
                                simulated car's, with its feed-forward); the car model
                                config/car_*/ only, from config/speed_pid_car.yaml

The source is the autosdv_vehicle_launch commit this repository PINS (the submodule entry
in HEAD), read with `git archive`, and the stub and seeds as HEAD has them, so a template
never carries an uncommitted edit; --from-worktree takes the working trees instead (testing
only). Nothing TA-only is read: the reference SpeedPID (autosdv_waypoint_lab/reference/) is
not among the sources.

What is NOT copied, on purpose (docs/design/lab2-team-vehicle-models.md):

  - the car's PWM points: the launch file keeps loading
    individual_params/config/<vehicle_id>/autosdv_vehicle/actuator.param.yaml, a fixed path
    whatever vehicle_model is. The cars differ in their stop PWM, and a copy that is wrong
    for a car drives it instead of stopping it.
  - the reference model's per-car tuning (individual_params' speed_controller.param.yaml):
    the team's launch loads its own config/speed_controller.param.yaml instead.

The node names and the /autosdv namespace stay as they are: the bench, the gate and the
preflight find the actuator as /autosdv/actuator_node.
"""
import argparse
import io
import os
import re
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import lab2_env  # noqa: E402
import lab2_ws  # noqa: E402

REPO = HERE.parents[2]
REF_SUBMODULE = 'src/vehicle/autosdv_vehicle_launch'
REF = 'autosdv_vehicle'
REF_PACKAGES = (f'{REF}_interface', f'{REF}_launch')   # copied; the description is generated
LAB = 'src/lab/autosdv_waypoint_lab'
STUB = f'{LAB}/autosdv_waypoint_lab/stub/speed_pid_part1.py'      # the student SpeedPID
SIM_SEED = f'{LAB}/config/speed_pid_part1.yaml'                    # the simulated car's
CAR_SEED = f'{LAB}/config/speed_pid_car.yaml'                      # a real car's
INSTANCES = 'src/param/autoware_individual_params/individual_params/config'
SIM_ID = 'sim'
NOT_A_VEHICLE = ('default',)
# what the copy leaves behind: the reference's tests of its own golden files are kept
# (they test the copy), the repository furniture is not
SKIP = re.compile(r'(^|/)(__pycache__|\.pytest_cache)(/|$)|\.pyc$')
TEXT_SUFFIXES = {'.py', '.xml', '.yaml', '.yml', '.cfg', '.txt', '.md', '.json', '.sh', ''}

TUNING_LET = re.compile(
    r'(<let name="car_tuning_file"\s+value=")[^"]*(autosdv_vehicle/speed_controller\.param\.yaml)("/>)')
TEAM_TUNING = '$(find-pkg-share {model}_launch)/config/$(var vehicle_id)/speed_controller.param.yaml'
CLASS_ARG = re.compile(r'(<arg name="speed_controller_class"\s+default=")[^"]*(")')
TUNING_COMMENT = re.compile(r"<!-- the car's tuning of the default speed controller.*?-->", re.S)


class NewTeamError(Exception):
    pass


def model_name(team) -> str:
    """3 -> team3_vehicle; refuses anything but a positive integer."""
    try:
        return lab2_ws.model_name(team)
    except ValueError as e:
        raise NewTeamError(str(e)) from None


def default_dest(env=os.environ) -> Path:
    return lab2_env.lab2_home(env) / 'ws' / 'src'


def rename_text(text: str, model: str) -> str:
    """The reference's three package names -> the team's. The car's parameter directory,
    individual_params/config/<vehicle_id>/autosdv_vehicle/, is not a package name and is
    left alone (the suffix after `autosdv_vehicle` is what tells them apart)."""
    for suffix in ('_interface', '_launch', '_description'):
        text = text.replace(REF + suffix, model + suffix)
    return text


def retarget_tuning(launch_xml: str, model: str) -> str:
    """The launch file's car_tuning_file -> the team's config/<vehicle_id>/ tuning."""
    new, n = TUNING_LET.subn(lambda m: f'<let name="car_tuning_file"\n       value="'
                             + TEAM_TUNING.format(model=model) + '"/>', launch_xml)
    if n != 1:
        raise NewTeamError('the reference vehicle_interface.launch.xml has no car_tuning_file '
                           'let to retarget; update new_team.py for it')
    return TUNING_COMMENT.sub(
        "<!-- this model's speed controller tuning for this car (config/<vehicle_id>/\n"
        "       speed_controller.param.yaml of this package), loaded after the car's PWM points -->",
        new, count=1)


def retarget_class(launch_xml: str, model: str) -> str:
    """The launch file's speed_controller_class default -> the model's own SpeedPID."""
    new, n = CLASS_ARG.subn(lambda m: m.group(1) + f'{model}_interface.speed_pid:SpeedPID'
                            + m.group(2), launch_xml)
    if n != 1:
        raise NewTeamError('the reference vehicle_interface.launch.xml has no '
                           'speed_controller_class argument; update new_team.py for it')
    return new


def check_launch(launch_xml: str, model: str) -> None:
    """What the generated launch must still do, or the template is refused."""
    car = 'config/$(var vehicle_id)/autosdv_vehicle/actuator.param.yaml'
    if car not in launch_xml:
        raise NewTeamError('the generated launch file no longer loads the car\'s '
                           f'individual_params {car}; refusing to write a model without it')
    if f'{model}_interface' not in launch_xml:
        raise NewTeamError(f'the generated launch file never starts {model}_interface')


def pinned_commit(repo: Path = REPO) -> str:
    out = subprocess.run(['git', '-C', str(repo), 'ls-tree', 'HEAD', REF_SUBMODULE],
                         capture_output=True, text=True)
    parts = out.stdout.split()
    if out.returncode != 0 or len(parts) < 3 or parts[1] != 'commit':
        raise NewTeamError(f'{REF_SUBMODULE} is not a submodule pinned in HEAD')
    return parts[2]


def read_reference(repo: Path = REPO, worktree: bool = False) -> Tuple[Dict[str, bytes], str]:
    """({path relative to the submodule: bytes} for the two copied packages, the source)."""
    sub = repo / REF_SUBMODULE
    files: Dict[str, bytes] = {}
    if worktree:
        for pkg in REF_PACKAGES:
            for p in sorted((sub / pkg).rglob('*')):
                rel = p.relative_to(sub).as_posix()
                if p.is_file() and not SKIP.search(rel):
                    files[rel] = p.read_bytes()
        return files, f'{REF_SUBMODULE} working tree'
    sha = pinned_commit(repo)
    out = subprocess.run(['git', '-C', str(sub), 'archive', '--format=tar', sha, *REF_PACKAGES],
                         capture_output=True)
    if out.returncode != 0:
        raise NewTeamError(f'git archive of {REF_SUBMODULE} at {sha[:7]} failed: '
                           f'{out.stderr.decode(errors="replace").strip()} '
                           '(is the submodule checked out? `just checkout`)')
    with tarfile.open(fileobj=io.BytesIO(out.stdout)) as tar:
        for m in tar.getmembers():
            if m.isfile() and not SKIP.search(m.name):
                files[m.name] = tar.extractfile(m).read()
    return files, f'{REF_SUBMODULE} at {sha[:7]}'


def description_files(model: str) -> Dict[str, str]:
    pkg = f'{model}_description'
    return {
        f'{pkg}/package.xml': f'''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{pkg}</name>
  <version>0.1.0</version>
  <description>{model}'s vehicle description: autosdv_vehicle_description's, by reference</description>
  <maintainer email="team@example.com">{model.split('_')[0]}</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>autosdv_vehicle_description</build_depend>
  <exec_depend>autosdv_vehicle_description</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
''',
        f'{pkg}/CMakeLists.txt': f'''cmake_minimum_required(VERSION 3.5)
project({pkg})

find_package(ament_cmake REQUIRED)
find_package(autosdv_vehicle_description REQUIRED)

# The car's geometry (wheelbase, steering limit) and its URDF are the CAR's, not a
# team's: they are installed from autosdv_vehicle_description, not copied into this
# package, so a team model can never disagree with the planner and the tracker about
# the car. A copy drifted once (0.5 rad and 0.340 m against 0.349 and 0.319).
set(REFERENCE_SHARE "${{autosdv_vehicle_description_DIR}}/..")
install(DIRECTORY
  "${{REFERENCE_SHARE}}/config"
  "${{REFERENCE_SHARE}}/urdf"
  "${{REFERENCE_SHARE}}/mesh"
  DESTINATION share/${{PROJECT_NAME}})

ament_package()
''',
    }


def tuning_file(model: str, vehicle_id: str, seed: str, seed_name: str) -> str:
    """config/<vehicle_id>/speed_controller.param.yaml: the seed's SpeedPID mapping, nested
    under speed_controller_params (the actuator merges speed_controller_params.<key> into the
    controller's parameters), comments kept."""
    lines = seed.splitlines()
    if lines and lines[0].startswith('# A copy of the Lab 2 handout'):   # our provenance note
        lines = lines[lines.index('#') + 1:] if '#' in lines else lines
    body = ''.join(('      ' + ln if ln.strip() else '') + '\n' for ln in lines)
    return f'''# {model}'s speed controller tuning on {vehicle_id}: YOURS. The vehicle interface loads
# it after the car's individual_params actuator.param.yaml (the car's PWM points, which are
# not yours to change), so these values win over both actuator.yaml and the car's file.
# One file per vehicle_id: sim is the RC simulator's car (Part 1), car_a/b/c the real cars.
# Seeded from autosdv_waypoint_lab's {seed_name}.
/**:
  ros__parameters:
    speed_controller_params:
{body}'''


def vehicle_ids(repo: Path = REPO, worktree: bool = False):
    """The individual_params instances a model needs a tuning file for (sim and car_*)."""
    if worktree:
        names = [p.name for p in (repo / INSTANCES).iterdir() if p.is_dir()]
    else:
        sub = 'src/param/autoware_individual_params'
        out = subprocess.run(['git', '-C', str(repo), 'ls-tree', 'HEAD', sub],
                             capture_output=True, text=True)
        parts = out.stdout.split()
        if out.returncode != 0 or len(parts) < 3:
            raise NewTeamError(f'{sub} is not a submodule pinned in HEAD')
        ls = subprocess.run(['git', '-C', str(repo / sub), 'ls-tree', '--name-only', parts[2],
                             'individual_params/config/'], capture_output=True, text=True)
        if ls.returncode != 0:
            raise NewTeamError(f'cannot list {sub} at {parts[2][:7]} (`just checkout`?)')
        names = [Path(n).name for n in ls.stdout.split()]
    return sorted(n for n in names if n not in NOT_A_VEHICLE)


def read_lab_files(repo: Path = REPO, worktree: bool = False) -> Dict[str, bytes]:
    """The stub and the two seeds, as HEAD has them (or the working tree)."""
    out = {}
    for rel in (STUB, SIM_SEED, CAR_SEED):
        if worktree:
            out[rel] = (repo / rel).read_bytes()
            continue
        r = subprocess.run(['git', '-C', str(repo), 'show', f'HEAD:{rel}'], capture_output=True)
        if r.returncode != 0:
            raise NewTeamError(f'{rel} is not in HEAD: {r.stderr.decode(errors="replace").strip()}')
        out[rel] = r.stdout
    return out


def readme(model: str, n: int, source: str, kind: str = 'car') -> str:
    if kind == 'sim':
        use = f'''Part 1: the RC simulator's car, the one package every student completes
(`just lab2 sim` runs it, as `vehicle_model:={model}`). In AutoSDV's src/lab2_vehicles/:

    colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \\
        --packages-up-to {model}_launch
    just lab2 sim'''
        tuning = f'''Its parameters are in `{model}_launch/config/sim/speed_controller.param.yaml` under
`speed_controller_params:`, with the simulated car's calibration (ff_deadband, ff_gain).'''
        gate = ''
    else:
        rename = ('''First give it your team number (here 3): every "teamN" becomes "team3", in
directory and file names and inside the files, this directory's own name included.

''' if n == 'N' else '')
        use = f'''Part 2: the real cars. Select it with `vehicle_model:={model}` (`team:={n}` does).
{rename}In AutoSDV's src/lab2_vehicles/:

    colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \\
        --packages-up-to {model}_launch
    just lab2 gate team:={n}                       the vehicle gate; pass it before any car
    just lab2 run team:={n}'''
        tuning = f'''Its parameters are per car, in
`{model}_launch/config/<car>/speed_controller.param.yaml` (car_a, car_b, car_c) under
`speed_controller_params:`. Put each car's calibration (ff_deadband, ff_gain) in its own
file.'''
        gate = f'''

## The gate

`just lab2 gate team:={n}` runs the model in the RC simulator against each car's PWM points
and requires that it

- holds the car's neutral PWM at rest and after a stop,
- stops when `control_cmd` stops arriving, on an MRM, and on an emergency,
- never pulls the ESC into reverse (a command at or below the car's `reverse_arm_pwm`),

and then drives the graded route on the vehicle bench. A model that fails does not go
on a car.
'''
    whose = "Lab 2 Part 1's" if kind == 'sim' else f"team {n}'s"
    made = 'new_team.py --sim' if kind == 'sim' else (
        'new_team.py --template' if n == 'N' else f'new_team.py {n}')
    return f'''# {model}: {whose} Autoware vehicle model

Generated by `{made}` from the reference model, autosdv_vehicle
({source}). {use}

## What is yours

Everything in `{model}_interface` and `{model}_launch`: the actuator node, its safety
shell, velocity_report, the gear, control-mode and steering nodes, and the launch file.

The speed controller is `{model}_interface/{model}_interface/speed_pid.py`, the Lab 2
SpeedPID stub, until you complete it: it holds the motor in neutral. {tuning}

The contract with the rest of the stack is Autoware's vehicle interface:

- in:  `/control/command/control_cmd`, `/control/command/emergency_cmd`,
       `/system/fail_safe/mrm_state`
- out: `/vehicle/status/velocity_status`, `steering_status`, `gear_status`,
       `control_mode`

and two names the lab's tools use: the actuator is `/autosdv/actuator_node`, and the
bench's car model reads the motor PWM from its `debug/pwm_values`.

## What is not

- `{model}_description` installs autosdv_vehicle_description's files by reference: the
  wheelbase and steering limit are the car's.
- The launch file loads the vehicle's PWM points from
  `individual_params/config/<vehicle_id>/autosdv_vehicle/actuator.param.yaml`. Keep that
  line: the cars' stop PWMs differ, and a copied number drives a car instead of stopping it.
{gate}'''


KINDS = ('sim', 'car', 'template')
PLACEHOLDER = 'teamN_vehicle'      # the template's model name; students rename teamN by hand



def generate(n, dest: Path, files: Dict[str, bytes], source: str,
             lab: Optional[Dict[str, bytes]] = None, ids=('sim', 'car_a', 'car_b', 'car_c'),
             kind: str = 'car') -> Path:
    """One model: kind 'car' (team<n>_vehicle, the car_* tuning files), 'template' (the
    same as teamN_vehicle; n is not used) or 'sim' (lab2_sim_vehicle,
    config/sim/ only; n is not used)."""
    if kind not in KINDS:
        raise NewTeamError(f'kind {kind!r}: one of {", ".join(KINDS)}')
    model = {'car': model_name(n) if kind == 'car' else None, 'template': PLACEHOLDER,
             'sim': lab2_ws.SIM_MODEL}[kind]
    ids = [i for i in ids if (i == SIM_ID) == (kind == 'sim')]
    if not ids:
        raise NewTeamError(f'no vehicle_id for a {kind} model among the individual_params instances')
    root = Path(dest) / f'{model}_launch'
    if root.exists():
        raise NewTeamError(f'{root} exists; never overwriting a team\'s package')
    out: Dict[str, bytes] = {}
    for rel, data in files.items():
        new_rel = rename_text(rel, model)
        if Path(rel).suffix in TEXT_SUFFIXES:
            try:
                text = data.decode('utf-8')
            except UnicodeDecodeError:
                out[new_rel] = data
                continue
            text = rename_text(text, model)
            if rel.endswith('launch/vehicle_interface.launch.xml'):
                text = retarget_class(retarget_tuning(text, model), model)
                check_launch(text, model)
            out[new_rel] = text.encode()
        else:
            out[new_rel] = data
    launch_cmake = f'{model}_launch/CMakeLists.txt'
    if launch_cmake not in out:
        raise NewTeamError(f'{REF}_launch has no CMakeLists.txt')
    cm = out[launch_cmake].decode()
    if 'INSTALL_TO_SHARE' in cm and 'config' not in cm:
        cm = cm.replace('INSTALL_TO_SHARE\n  launch', 'INSTALL_TO_SHARE\n  launch\n  config')
        if 'config' not in cm:
            raise NewTeamError(f'cannot add config/ to {launch_cmake}')
    out[launch_cmake] = cm.encode()
    for rel, text in description_files(model).items():
        out[rel] = text.encode()
    lab = read_lab_files() if lab is None else lab
    out[f'{model}_interface/{model}_interface/speed_pid.py'] = lab[STUB]   # byte for byte
    for vid in ids:
        seed = SIM_SEED if vid == SIM_ID else CAR_SEED
        out[f'{model}_launch/config/{vid}/speed_controller.param.yaml'] = tuning_file(
            model, vid, lab[seed].decode(), Path(seed).name).encode()
    out['README.md'] = readme(model, {'car': n, 'template': 'N', 'sim': 0}[kind], source,
                              'sim' if kind == 'sim' else 'car').encode()
    tmp = root.with_name(root.name + '.tmp')
    shutil.rmtree(tmp, ignore_errors=True)
    for rel, data in sorted(out.items()):
        p = tmp / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
        if rel.startswith(f'{model}_interface/') and rel.endswith('.py') and data.startswith(b'#!'):
            p.chmod(0o755)
    tmp.rename(root)
    return root


def main(argv: Optional[Iterable[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('team', nargs='?', help='a team number, e.g. 3 -> team3_vehicle')
    which = ap.add_mutually_exclusive_group()
    which.add_argument('--sim', action='store_true', help=f'Part 1\'s model, {lab2_ws.SIM_MODEL}')
    which.add_argument('--template', action='store_true',
                       help=f'the Part 2 template, {PLACEHOLDER}')
    ap.add_argument('--dest', default=None, help='default: ~/lab2/ws/src ($LAB2_HOME/ws/src)')
    ap.add_argument('--from-worktree', action='store_true',
                    help='copy the working trees, not the pinned commits (testing)')
    args = ap.parse_args(None if argv is None else list(argv))
    named = args.sim or args.template
    if named == (args.team is not None):
        ap.error('one of: a team number, --sim, --template')
    kind = 'sim' if args.sim else 'template' if args.template else 'car'
    try:
        dest = Path(os.path.expanduser(args.dest)) if args.dest else default_dest()
        files, source = read_reference(worktree=args.from_worktree)
        root = generate(args.team or 1, dest, files, source,
                        read_lab_files(worktree=args.from_worktree),
                        vehicle_ids(worktree=args.from_worktree), kind)
    except NewTeamError as e:
        print(f'new_team: {e}', file=sys.stderr)
        return 2
    print(f'wrote {root} (from {source})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
