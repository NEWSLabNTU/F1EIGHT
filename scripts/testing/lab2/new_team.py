#!/usr/bin/env python3
"""Generate a team's Autoware vehicle model from the reference, autosdv_vehicle.

    python3 scripts/testing/lab2/new_team.py N [--dest DIR] [--from-worktree]

writes DIR/teamN_vehicle_launch/ (DIR: ~/lab2/ws/src, or $LAB2_HOME/ws/src), the three
packages Autoware resolves for vehicle_model:=teamN_vehicle:

    teamN_vehicle_description   vehicle_info.param.yaml, the URDF and the meshes of
                                autosdv_vehicle_description, installed BY REFERENCE at
                                build time (the car's geometry is the car's, not a team's)
    teamN_vehicle_interface     a renamed copy of autosdv_vehicle_interface: the actuator,
                                its speed controller and safety shell, velocity_report,
                                the gear, mode and steering nodes. The team's to change.
    teamN_vehicle_launch        launch/vehicle_interface.launch.xml, renamed, and
                                config/speed_controller.param.yaml, the team's tuning

The source is the autosdv_vehicle_launch commit this repository PINS (the submodule entry
in HEAD), read with `git archive`, so a template never carries an uncommitted edit of the
reference; --from-worktree takes the submodule's working tree instead (testing only).

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

REPO = HERE.parents[2]
REF_SUBMODULE = 'src/vehicle/autosdv_vehicle_launch'
REF = 'autosdv_vehicle'
REF_PACKAGES = (f'{REF}_interface', f'{REF}_launch')   # copied; the description is generated
# what the copy leaves behind: the reference's tests of its own golden files are kept
# (they test the copy), the repository furniture is not
SKIP = re.compile(r'(^|/)(__pycache__|\.pytest_cache)(/|$)|\.pyc$')
TEXT_SUFFIXES = {'.py', '.xml', '.yaml', '.yml', '.cfg', '.txt', '.md', '.json', '.sh', ''}

TUNING_LET = re.compile(
    r'(<let name="car_tuning_file"\s+value=")[^"]*(autosdv_vehicle/speed_controller\.param\.yaml)("/>)')
TEAM_TUNING = '$(find-pkg-share {model}_launch)/config/speed_controller.param.yaml'
TUNING_COMMENT = re.compile(r"<!-- the car's tuning of the default speed controller.*?-->", re.S)


class NewTeamError(Exception):
    pass


def model_name(team) -> str:
    """3 -> team3_vehicle; refuses anything but a positive integer."""
    s = str(team).strip()
    if not re.fullmatch(r'[1-9][0-9]{0,2}', s):
        raise NewTeamError(f'team {team!r}: a team number, 1 to 999')
    return f'team{int(s)}_vehicle'


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
    """The launch file's car_tuning_file -> the team's config/speed_controller.param.yaml."""
    new, n = TUNING_LET.subn(lambda m: f'<let name="car_tuning_file"\n       value="'
                             + TEAM_TUNING.format(model=model) + '"/>', launch_xml)
    if n != 1:
        raise NewTeamError('the reference vehicle_interface.launch.xml has no car_tuning_file '
                           'let to retarget; update new_team.py for it')
    return TUNING_COMMENT.sub(
        "<!-- this model's speed controller tuning (config/speed_controller.param.yaml\n"
        "       of this package), loaded after the car's PWM points -->", new, count=1)


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


def tuning_file(model: str) -> str:
    return f'''# {model}'s speed controller tuning: YOURS. The vehicle interface loads it after the
# car's individual_params actuator.param.yaml (the car's PWM points, which are not
# yours to change), so these values win over both actuator.yaml and the car's file.
#
# Seeded with actuator.yaml's own defaults, which every car runs. One file serves
# all three cars, so keep car-specific numbers out of it: the cars' neutral and ESC
# thresholds differ (A 380, B 390, C 385), and a PWM written here would be the same
# number on each. esc_deadzone_compensation needs the car's measured
# forward_hold_pwm, which car A does not have yet: turned on here, the actuator
# refuses to start on car A, and the gate fails it.
/**:
  ros__parameters:
    kp_speed: 12.0
    ki_speed: 10.0
    kd_speed: 0.001
    esc_deadzone_compensation: false
'''


def readme(model: str, n: int, source: str) -> str:
    return f'''# {model}: team {n}'s Autoware vehicle model

Generated by `just lab2 new-team {n}` from the reference model, autosdv_vehicle
({source}). Select it with `vehicle_model:={model}`:

    just lab2 build-teams                          build ~/lab2/ws (every team)
    just lab2 gate vehicle_model:={model}          the bench gate; required before any car
    just lab2 run vehicle_model:={model} team:={n}

## What is yours

Everything in `{model}_interface` and `{model}_launch`: the actuator node, its speed
controller and safety shell, velocity_report, the gear, control-mode and steering
nodes, the launch file and `{model}_launch/config/speed_controller.param.yaml`.

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
- The launch file loads the car's PWM points from
  `individual_params/config/<vehicle_id>/autosdv_vehicle/actuator.param.yaml`. Keep that
  line: the cars' stop PWMs differ, and a copied number drives a car instead of stopping it.

## The gate

`just lab2 gate vehicle_model:={model}` runs the model in the RC simulator against each
car's PWM points and requires that it

- holds the car's neutral PWM at rest and after a stop,
- stops when `control_cmd` stops arriving,
- never pulls the ESC into reverse (a command at or below the car's `reverse_arm_pwm`),
- stops on an MRM,

and then drives the graded route on the vehicle bench. A model that fails does not go
on a car.
'''


def generate(n, dest: Path, files: Dict[str, bytes], source: str) -> Path:
    model = model_name(n)
    root = Path(dest) / f'{model}_launch'
    if root.exists():
        raise NewTeamError(f'{root} exists; new-team never overwrites a team\'s package')
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
                text = retarget_tuning(text, model)
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
    out[f'{model}_launch/config/speed_controller.param.yaml'] = tuning_file(model).encode()
    out['README.md'] = readme(model, int(n), source).encode()
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
    ap.add_argument('team', help='the team number, e.g. 3 -> team3_vehicle')
    ap.add_argument('--dest', default=None, help='default: ~/lab2/ws/src ($LAB2_HOME/ws/src)')
    ap.add_argument('--from-worktree', action='store_true',
                    help='copy the submodule working tree, not the pinned commit (testing)')
    args = ap.parse_args(None if argv is None else list(argv))
    try:
        dest = Path(os.path.expanduser(args.dest)) if args.dest else default_dest()
        files, source = read_reference(worktree=args.from_worktree)
        root = generate(args.team, dest, files, source)
    except NewTeamError as e:
        print(f'new-team: {e}', file=sys.stderr)
        return 2
    model = model_name(args.team)
    print(f'wrote {root} ({model}, from {source})')
    print(f'next: just lab2 build-teams, then just lab2 gate vehicle_model:={model}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
