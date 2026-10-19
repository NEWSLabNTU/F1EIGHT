"""The vehicle gate's record of what passed: ~/lab2/gate/<model>.json.

`just lab2 gate` writes it after a full gate (the safety half on every car, then the
route), keyed by bag_meta.model_digest: a sha256 over the model's three packages' source.
`just lab2 run` on a car reads it and refuses a team model whose source is not the one
that passed, so a model edited after its gate goes back through the gate before it drives
a car. The reference model (autosdv_vehicle) is the TA's and is not held to it.
"""
import json
import os
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import lab2_env  # noqa: E402

REFERENCE_MODEL = 'autosdv_vehicle'


def _digest(model: str) -> str:
    try:
        from autosdv_waypoint_lab import bag_meta
    except ImportError:
        sys.path.insert(0, str(HERE.parents[2] / 'src/lab/autosdv_waypoint_lab'))
        from autosdv_waypoint_lab import bag_meta
    return bag_meta.model_digest(model)


def stamp_path(model: str, home: Optional[Path] = None) -> Path:
    return Path(home or lab2_env.lab2_home()) / 'gate' / f'{model}.json'


def write(model: str, ok: bool, detail: dict, home: Optional[Path] = None,
          digest: Optional[str] = None) -> Path:
    p = stamp_path(model, home)
    p.parent.mkdir(parents=True, exist_ok=True)
    data = dict(detail, vehicle_model=model, ok=bool(ok),
                digest=_digest(model) if digest is None else digest, at=time.time())
    tmp = p.with_name(p.name + '.tmp')
    tmp.write_text(json.dumps(data, indent=2))
    os.replace(tmp, p)
    return p


def check(model: str, home: Optional[Path] = None,
          digest: Optional[str] = None) -> Tuple[bool, str]:
    """(may this model drive a car, why not)."""
    if model == REFERENCE_MODEL:
        return True, ''
    cmd = f'`just lab2 gate vehicle_model:={model}`'
    p = stamp_path(model, home)
    if not p.is_file():
        return False, f'{model} has not passed the vehicle gate on this machine: run {cmd} first'
    try:
        d = json.loads(p.read_text())
    except (OSError, ValueError):
        return False, f'{p} is unreadable: run {cmd} again'
    if not d.get('ok'):
        return False, f'{model} FAILED its last vehicle gate ({p}): fix it, then {cmd}'
    now = _digest(model) if digest is None else digest
    if not now:
        return False, f'{model} is not installed (just lab2 build-teams)'
    if d.get('digest') != now:
        return False, (f'{model} changed since it passed the vehicle gate '
                       f'({time.strftime("%Y-%m-%d %H:%M", time.localtime(d.get("at", 0)))}): '
                       f'run {cmd} again')
    return True, ''
