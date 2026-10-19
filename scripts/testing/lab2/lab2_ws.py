"""The teams' vehicle models: their names and their workspace, for every Lab 2 tool.

    SIM_MODEL               'lab2_sim_vehicle': Part 1's model, the RC simulator's car. ONE
                            package, the same name for every student
    model_name(team)        3 -> 'team3_vehicle': a team's car model, Part 2 (ValueError for
                            anything but 1 .. 999)
    team_of(model)          'team3_vehicle' -> 3, None for another name
    ws_dir(env)             ~/lab2/ws, or $LAB2_HOME/ws: a workspace for the TA's grading of
                            submissions (students build theirs in src/lab2_vehicles/)
    ws_install(env)         its install/
    model_installed(model)  are <model>_launch and the rest in the ament index of this process
    model_source_root(model) the directory holding the model's three packages (its installed
                            launch file's link, under --symlink-install), or None
    controller_problems(model) why a TEAM model's controller is not its own speed_pid.py
                            SpeedPID (submission.builtin_controller_uses on its source); []
                            for the reference
    calibrated_cars(root, model) the cars ('A', 'B', 'C') whose config/car_<x>/ tuning file
                            in the model's source sets ff_gain: the bench drives that car

lab2_run (Part 2) and part1_run (Part 1) both resolve team:=N with these, so the two parts
name and find a team's model the same way. A student's models live under this repository's
src/lab2_vehicles/ (gitignored) and resolve from its own install.
"""
import os
import re
from pathlib import Path
from typing import Mapping, Optional

from lab2_env import lab2_home

REFERENCE_MODEL = 'autosdv_vehicle'
SIM_MODEL = 'lab2_sim_vehicle'


def model_name(team) -> str:
    s = str(team).strip()
    if not re.fullmatch(r'[1-9][0-9]{0,2}', s):
        raise ValueError(f'team {team!r}: a team number, 1 to 999')
    return f'team{int(s)}_vehicle'


def team_of(model: str) -> Optional[int]:
    m = re.fullmatch(r'team([1-9][0-9]{0,2})_vehicle', str(model))
    return int(m.group(1)) if m else None


def ws_dir(env: Mapping[str, str] = os.environ) -> Path:
    return lab2_home(env) / 'ws'


def ws_install(env: Mapping[str, str] = os.environ) -> Path:
    return ws_dir(env) / 'install'


def model_source_root(model: str) -> Optional[Path]:
    try:
        from ament_index_python.packages import get_package_share_directory
        launch = Path(get_package_share_directory(f'{model}_launch'),
                      'launch', 'vehicle_interface.launch.xml')
    except Exception:
        return None
    root = launch.resolve().parents[2]
    return root if (root / f'{model}_launch').is_dir() else None


def controller_problems(model: str) -> list:
    if team_of(model) is None:
        return []
    root = model_source_root(model)
    if root is None:
        return [f'{model}: cannot find its source (built without --symlink-install?), so its '
                'controller cannot be checked']
    import submission
    return submission.builtin_controller_uses(root, model)


def model_installed(model: str) -> bool:
    """Whether this process (its sourced workspaces) can launch the model."""
    try:
        from ament_index_python.packages import get_package_share_directory
        for suffix in ('_launch', '_interface', '_description'):
            get_package_share_directory(f'{model}{suffix}')
        return True
    except Exception:
        return False


def calibrated_cars(root: Optional[Path], model: str) -> list:
    """The car letters whose <model>_launch/config/car_<x>/speed_controller.param.yaml sets
    ff_gain > 0 (the team's motor fit), sorted; [] without a source root.

    The car tools of the handout's lab2_speed pick their car by the same rule, so the bench
    of `just lab2 run` drives the car whose file the team calibrated, not the nominal car."""
    if root is None:
        return []
    import yaml
    cars = []
    for f in sorted(Path(root, f'{model}_launch', 'config').glob('car_*/speed_controller.param.yaml')):
        try:
            doc = yaml.safe_load(f.read_text()) or {}
            params = doc['/**']['ros__parameters']['speed_controller_params'] or {}
            gain = float(params.get('ff_gain') or 0.0)
        except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError, AttributeError):
            continue
        letter = f.parent.name[len('car_'):]
        if gain > 0.0 and re.fullmatch(r'[a-z]', letter):
            cars.append(letter.upper())
    return cars
