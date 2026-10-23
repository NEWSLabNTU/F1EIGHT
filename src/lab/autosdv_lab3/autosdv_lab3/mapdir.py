"""The Lab 3 map folder and its provenance file.

    team<N>_map/
        anchored_full.pcd         the GLIM cloud in the board frame, full resolution
                                  (kept for re-downsampling, not handed in)
        pointcloud_map.pcd        anchored_full.pcd downsampled; x y z intensity
        lanelet2_map.osm          the clockwise loop, local coordinates
        map_projector_info.yaml   projector_type: Local
        board_anchor.yaml         T(map <- slam) and the board detection
        board_profile.yaml        the detector profile the anchor used; the week-2
                                  board initializer loads the same file
        lab3_map.yaml             how the folder was made, one section per step

Every tool that writes into the folder records its step in lab3_map.yaml with
`record_step`, so the folder says which bag, which GLIM config, which board profile
and which voxel size made it, and at which AutoSDV commit.
"""

import hashlib
import os
import subprocess
import time
from typing import Optional

import yaml

MAP_META = 'lab3_map.yaml'
PCD = 'pointcloud_map.pcd'
FULL = 'anchored_full.pcd'
BOARD_PROFILE = 'board_profile.yaml'
LANELET = 'lanelet2_map.osm'
PROJECTOR = 'map_projector_info.yaml'
ANCHOR = 'board_anchor.yaml'


def sha256(path: str, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(chunk), b''):
            h.update(block)
    return h.hexdigest()


def git_rev(cwd: Optional[str] = None) -> str:
    """HEAD of the AutoSDV checkout, with +dirty when tracked files are modified."""
    cwd = cwd or os.environ.get('AUTOSDV_ROOT') or os.getcwd()
    try:
        def run(*a):
            return subprocess.run(['git', *a], cwd=cwd, capture_output=True, text=True,
                                  timeout=10).stdout.strip()
        rev = run('rev-parse', 'HEAD')
        if not rev:
            return 'unknown'
        return rev + ('+dirty' if run('status', '--porcelain', '--untracked-files=no') else '')
    except (OSError, subprocess.SubprocessError):
        return 'unknown'


def load_meta(map_dir: str) -> dict:
    path = os.path.join(map_dir, MAP_META)
    if not os.path.isfile(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def record_step(map_dir: str, step: str, data: dict) -> dict:
    """Write `data` as section `step` of lab3_map.yaml, stamped; returns the whole file."""
    meta = load_meta(map_dir)
    meta[step] = dict(data, time=time.strftime('%Y-%m-%dT%H:%M:%S%z'), autosdv=git_rev())
    os.makedirs(map_dir, exist_ok=True)
    with open(os.path.join(map_dir, MAP_META), 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return meta


def folder_hash(map_dir: str) -> str:
    """One hash over the files that define the map, for the week-2 bag's metadata."""
    h = hashlib.sha256()
    for name in (PCD, LANELET, PROJECTOR):
        path = os.path.join(map_dir, name)
        h.update(name.encode())
        h.update(sha256(path).encode() if os.path.isfile(path) else b'missing')
    return h.hexdigest()
