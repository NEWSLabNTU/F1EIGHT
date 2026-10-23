"""`just lab3 anchor`: the GLIM cloud into the board frame.

    lab3_anchor RAW_MAP MAP_DIR (--car car_b | --sensor robin_w | --config FILE)
                [--dump-debug] [--floor-band M ...]

Runs reflective_pose_cli's anchor-map-to-board on the full-resolution cloud with the
Lab 3 board profile for the car's LiDAR, and writes into MAP_DIR:

    anchored_full.pcd         the cloud in the board frame (origin on the floor below
                              the board centre, x out of the wall, z up)
    board_anchor.yaml         the transform and the detection
    board_profile.yaml        a copy of the profile used; the week-2 initializer loads it
    map_projector_info.yaml   projector_type: Local
    lab3_map.yaml             the `anchor` step

The anchor runs before downsampling on purpose: the board is 0.6 m wide, and a coarse
voxel grid leaves it too few points to be found. `just lab3 downsample` comes next.
"""

import argparse
import os
import shutil
import subprocess
import sys

import yaml

from autosdv_lab3 import mapdir
from autosdv_lab3.cars import CAR_SENSOR, SENSORS

# anchor-map-to-board's own advice is for the golf cart's workflow, not this one
_SKIP = ('next:', 'tile the cloud', 'wrote map_projector_info')


def profile_path(sensor: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('autosdv_lab3'), 'config', 'board',
                        f'lab3_{sensor}.yaml')


def sensor_from_glim_report(raw_map: str):
    """The sensor family `just lab3 map` recorded beside the cloud, if it did."""
    report = os.path.join(os.path.dirname(os.path.abspath(raw_map)), 'glim_report.yaml')
    if not os.path.isfile(report):
        return None
    with open(report) as f:
        data = yaml.safe_load(f) or {}
    sensor = data.get('sensor') or data.get('sensor_family')
    return sensor if sensor in SENSORS else None


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='lab3_anchor', description='Anchor the GLIM cloud to the reflective board.')
    ap.add_argument('raw_map', help='the GLIM cloud, full resolution (raw_map.pcd)')
    ap.add_argument('map_dir', help='the map folder to write')
    who = ap.add_mutually_exclusive_group()
    who.add_argument('--car', choices=sorted(CAR_SENSOR), help='pick the profile by car')
    who.add_argument('--sensor', choices=SENSORS, help='pick the profile by LiDAR')
    who.add_argument('--config', help='a board profile file instead of the packaged one')
    ap.add_argument('--dump-debug', action='store_true',
                    help='also write MAP_DIR/anchor_debug.npz (replay with '
                         'ros2 run reflective_pose_ros anchor_debug_viewer)')
    ap.add_argument('--dry-run', action='store_true', help='report, write nothing')
    args, floor_flags = ap.parse_known_args(argv)

    if not os.path.isfile(args.raw_map):
        print(f'lab3_anchor: no such cloud: {args.raw_map}', file=sys.stderr)
        return 2
    config = args.config
    sensor = args.sensor or CAR_SENSOR.get(args.car or '') or sensor_from_glim_report(args.raw_map)
    if not config:
        if not sensor:
            print('lab3_anchor: which LiDAR? give --car car_a|car_b|car_c (or --sensor)',
                  file=sys.stderr)
            return 2
        config = profile_path(sensor)
    print(f'board profile: {config}')

    # the module rather than `ros2 run`: same code, and it runs from a test without
    # an installed workspace
    cmd = [sys.executable, '-m', 'reflective_pose_cli.anchor_cli', args.raw_map,
           '-o', args.map_dir, '--name', mapdir.FULL, '--config', config, *floor_flags]
    if args.dry_run:
        cmd.append('--dry-run')
    debug = os.path.join(args.map_dir, 'anchor_debug.npz')
    if args.dump_debug:
        os.makedirs(args.map_dir, exist_ok=True)
        cmd += ['--dump-debug', debug]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, text=True)
    for line in proc.stdout.splitlines():
        if not line.strip().startswith(_SKIP):
            print(line)
    if proc.returncode != 0:
        print('\nlab3_anchor: the board was not anchored (reason above).', file=sys.stderr)
        if not args.dump_debug:
            print('  run again with --dump-debug, then look at the clusters with\n'
                  f'  ros2 run reflective_pose_ros anchor_debug_viewer {debug}', file=sys.stderr)
        return proc.returncode
    if args.dry_run:
        return 0

    # The extra polygon is the golf cart's; the students draw lanelet2_map.osm themselves.
    stray = os.path.join(args.map_dir, 'board_polygon.osm')
    if os.path.isfile(stray):
        os.remove(stray)
    shutil.copyfile(config, os.path.join(args.map_dir, mapdir.BOARD_PROFILE))
    with open(os.path.join(args.map_dir, mapdir.ANCHOR)) as f:
        anchor = yaml.safe_load(f) or {}
    with open(config) as f:
        board = (yaml.safe_load(f) or {}).get('board', {})
    mapdir.record_step(args.map_dir, 'anchor', {
        'raw_map': os.path.abspath(args.raw_map),
        'raw_map_sha256': mapdir.sha256(args.raw_map),
        'sensor': sensor or 'custom',
        'board_pose_in_map': board.get('pose_in_map'),
        'floor_tilt_deg': anchor.get('floor_tilt_deg'),
        'detection': anchor.get('detection'),
    })
    print(f'\nanchored: {os.path.join(args.map_dir, mapdir.FULL)}')
    print('next: just lab3 downsample', args.map_dir, '--voxel V')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
