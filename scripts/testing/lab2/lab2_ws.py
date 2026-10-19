"""The teams' vehicle models: their names and their workspace, for every Lab 2 tool.

    model_name(team)        3 -> 'team3_vehicle' (ValueError for anything but 1 .. 999)
    team_of(model)          'team3_vehicle' -> 3, None for another name
    ws_dir(env)             ~/lab2/ws, or $LAB2_HOME/ws: where `just lab2 new-team` writes and
                            `just lab2 build-teams` builds
    ws_install(env)         its install/ (lab2_ws.sh sources install/local_setup.bash from it)
    model_installed(model)  are <model>_launch and the rest in the ament index of this process

lab2_run (Part 2) and part1_run (Part 1) both resolve team:=N with these, so the two parts
name and find a team's model the same way.
"""
import os
import re
from pathlib import Path
from typing import Mapping, Optional

from lab2_env import lab2_home

REFERENCE_MODEL = 'autosdv_vehicle'


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


def model_installed(model: str) -> bool:
    """Whether this process (its sourced workspaces) can launch the model."""
    try:
        from ament_index_python.packages import get_package_share_directory
        for suffix in ('_launch', '_interface', '_description'):
            get_package_share_directory(f'{model}{suffix}')
        return True
    except Exception:
        return False
