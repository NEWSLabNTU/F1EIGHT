"""`just lab3 downsample`: the anchored cloud down to the size limit.

    lab3_downsample MAP_DIR --voxel V [--max-mb 50]

Reads MAP_DIR/anchored_full.pcd (from `just lab3 anchor`), keeps one point per
V-metre voxel — the mean position and the mean intensity of the points in it — and
writes MAP_DIR/pointcloud_map.pcd, the file NDT and Vector Map Builder load. Run it
again with another V as often as you like; it always starts from the full cloud.

The choice of V is yours. Larger is smaller and faster to load, but every voxel
averages away detail NDT matches against, and the 0.6 m board shrinks to a handful
of points: the report counts them.
"""

import argparse
import os
import sys

import numpy as np
import yaml

from autosdv_lab3 import mapdir

DEFAULT_MAX_MB = 50.0    # provisional, until Vector Map Builder's upload limit is measured
BOARD_WARN_VOXEL = 0.15  # above this the board is four voxels wide or fewer


def voxel_downsample(points: np.ndarray, intensity, voxel: float):
    """One point per occupied voxel: the mean of its points, and of their intensity.

    The voxel index is floor(p / voxel), so the grid is fixed in the map frame and
    the same cloud always gives the same answer.
    """
    if voxel <= 0:
        raise ValueError('voxel must be positive')
    keys = np.floor(points / voxel).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.reshape(-1)
    n = len(counts)
    out = np.zeros((n, 3))
    for axis in range(3):
        out[:, axis] = np.bincount(inverse, weights=points[:, axis], minlength=n) / counts
    out_i = None
    if intensity is not None:
        out_i = np.bincount(inverse, weights=intensity, minlength=n) / counts
    return out, out_i


def board_box(map_dir: str):
    """(centre, half-extents) of the board in the map frame, from the profile copied
    beside the cloud; the board lies in the y-z plane at x = pose_in_map x."""
    path = os.path.join(map_dir, mapdir.BOARD_PROFILE)
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        board = (yaml.safe_load(f) or {}).get('board', {})
    pose = board.get('pose_in_map')
    if not pose:
        return None
    centre = np.array(pose[:3], dtype=float)
    half = np.array([0.05, board.get('width', 0.6) / 2, board.get('height', 0.6) / 2])
    return centre, half


def count_in_box(points: np.ndarray, box) -> int:
    centre, half = box
    return int(np.count_nonzero(np.all(np.abs(points - centre) <= half, axis=1)))


def main(argv=None):
    ap = argparse.ArgumentParser(prog='lab3_downsample',
                                 description='Downsample the anchored map to the size limit.')
    ap.add_argument('map_dir', help='the map folder `just lab3 anchor` wrote')
    ap.add_argument('--voxel', type=float, required=True, help='voxel size, metres')
    ap.add_argument('--max-mb', type=float, default=DEFAULT_MAX_MB,
                    help=f'the size limit of pointcloud_map.pcd (default {DEFAULT_MAX_MB:g} MB)')
    args = ap.parse_args(argv)

    from reflective_pose_core.pointcloud_io import PointCloud, read_cloud, write_pcd

    full = os.path.join(args.map_dir, mapdir.FULL)
    if not os.path.isfile(os.path.join(args.map_dir, mapdir.ANCHOR)) or not os.path.isfile(full):
        print(f'lab3_downsample: {args.map_dir} is not anchored (no {mapdir.ANCHOR} or '
              f'{mapdir.FULL}); run just lab3 anchor first', file=sys.stderr)
        return 2
    if args.voxel <= 0:
        print('lab3_downsample: --voxel must be positive', file=sys.stderr)
        return 2

    cloud = read_cloud(full)
    if not cloud.has_intensity:
        print(f'lab3_downsample: {full} has no intensity field', file=sys.stderr)
        return 2
    pts, inten = voxel_downsample(np.asarray(cloud.points, float),
                                  np.asarray(cloud.intensity, float), args.voxel)
    out = os.path.join(args.map_dir, mapdir.PCD)
    write_pcd(out, PointCloud(points=pts, intensity=inten))
    size_mb = os.path.getsize(out) / 1e6
    ok = size_mb <= args.max_mb

    box = board_box(args.map_dir)
    board_before = count_in_box(np.asarray(cloud.points, float), box) if box else None
    board_after = count_in_box(pts, box) if box else None

    print(f'{len(cloud)} -> {len(pts)} points at {args.voxel:g} m '
          f'({100.0 * len(pts) / max(len(cloud), 1):.1f} %)')
    print(f'{out}: {size_mb:.1f} MB  (limit {args.max_mb:g} MB)  '
          f'{"PASS" if ok else "FAIL: over the limit, use a larger voxel"}')
    if box:
        print(f'board: {board_after} points (was {board_before})')
    if args.voxel > BOARD_WARN_VOXEL:
        print(f'warning: at {args.voxel:g} m the 0.6 m board is only '
              f'{0.6 / args.voxel:.1f} voxels wide, and NDT loses the detail it matches on')

    mapdir.record_step(args.map_dir, 'downsample', {
        'voxel_m': args.voxel,
        'points_in': len(cloud),
        'points_out': len(pts),
        'size_mb': round(size_mb, 2),
        'max_mb': args.max_mb,
        'board_points': board_after,
        'pcd_sha256': mapdir.sha256(out),
    })
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
