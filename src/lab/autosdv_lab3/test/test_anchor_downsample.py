"""The anchor and the downsampler on a synthetic classroom.

The room is a GLIM-like cloud: walls, floor, two desks and the low board, then moved
by an arbitrary rigid transform with a slight tilt, as a SLAM frame would be. The
anchor must bring the board back to pose_in_map, whichever LiDAR's profile it uses.
"""

import os

import numpy as np
import pytest
import yaml

from autosdv_lab3 import anchor, downsample, mapdir

pc = pytest.importorskip('reflective_pose_core.pointcloud_io')

PROFILES = os.path.join(os.path.dirname(__file__), '..', 'config', 'board')
BOARD_CENTRE = np.array([0.0, 0.0, 0.40])  # in the room frame, on the wall x = 0


def _plane(origin, u, v, step, rng):
    """Points on the rectangle origin + a*u + b*v, a and b in [0, 1], about `step` apart."""
    u, v = np.asarray(u, float), np.asarray(v, float)
    nu = max(int(np.linalg.norm(u) / step), 2)
    nv = max(int(np.linalg.norm(v) / step), 2)
    a, b = np.meshgrid(np.linspace(0, 1, nu), np.linspace(0, 1, nv))
    pts = np.asarray(origin, float) + a.reshape(-1, 1) * u + b.reshape(-1, 1) * v
    return pts + rng.normal(0, 0.005, pts.shape)


def classroom(rng):
    """(points, intensity) in the room frame: the board on the wall x = 0, facing +x."""
    parts, inten = [], []

    def add(p, i):
        parts.append(p)
        inten.append(np.full(len(p), float(i)) + rng.normal(0, 3, len(p)))

    w, d, h = 12.0, 8.0, 2.6        # x into the room, y along the board wall
    add(_plane([0, -d / 2, 0], [w, 0, 0], [0, d, 0], 0.08, rng), 15)        # floor
    add(_plane([0, -d / 2, 0], [0, d, 0], [0, 0, h], 0.05, rng), 40)        # board wall
    add(_plane([w, -d / 2, 0], [0, d, 0], [0, 0, h], 0.05, rng), 40)
    add(_plane([0, -d / 2, 0], [w, 0, 0], [0, 0, h], 0.05, rng), 40)
    add(_plane([0, d / 2, 0], [w, 0, 0], [0, 0, h], 0.05, rng), 40)
    for x0, y0 in ((5.0, -2.0), (7.0, 1.5)):                              # desks
        add(_plane([x0, y0, 0.75], [1.2, 0, 0], [0, 0.6, 0], 0.04, rng), 50)
    # the board, 1 cm proud of the wall, centre 0.4 m up
    board = _plane([0.01, -0.3, 0.1], [0, 0.6, 0], [0, 0, 0.6], 0.02, rng)
    add(board, 220)
    return np.vstack(parts), np.concatenate(inten)


def slam_frame(points):
    """An arbitrary SLAM frame: yawed, shifted, tilted 0.4 deg, origin at sensor height."""
    yaw, tilt = np.radians(37.0), np.radians(0.4)
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, np.cos(tilt), -np.sin(tilt)], [0, np.sin(tilt), np.cos(tilt)]])
    return points @ (rx @ rz).T + np.array([3.2, -1.7, -0.30])


@pytest.fixture(scope='module')
def raw_map(tmp_path_factory):
    rng = np.random.default_rng(3)
    pts, inten = classroom(rng)
    path = tmp_path_factory.mktemp('glim') / 'raw_map.pcd'
    pc.write_pcd(str(path), pc.PointCloud(points=slam_frame(pts), intensity=inten))
    return str(path)


@pytest.mark.parametrize('sensor', ['robin_w', 'vlp16'])
def test_anchor_puts_board_at_pose_in_map(raw_map, tmp_path, sensor, monkeypatch):
    monkeypatch.setattr(anchor, 'profile_path',
                        lambda s: os.path.join(PROFILES, f'lab3_{s}.yaml'))
    out = tmp_path / 'map'
    assert anchor.main([raw_map, str(out), '--sensor', sensor]) == 0
    for name in (mapdir.FULL, mapdir.ANCHOR, mapdir.BOARD_PROFILE, mapdir.PROJECTOR,
                 mapdir.MAP_META):
        assert (out / name).is_file(), name
    assert not (out / 'board_polygon.osm').exists()

    cloud = pc.read_cloud(str(out / mapdir.FULL))
    bright = cloud.points[cloud.intensity > 150]
    centre = 0.5 * (bright.min(axis=0) + bright.max(axis=0))
    assert np.allclose(centre, BOARD_CENTRE, atol=0.03), centre
    # the board lies in the wall plane x = 0 (to its 1 cm offset), so the normal is +-x
    assert np.ptp(bright[:, 0]) < 0.05
    # the floor is level at z = 0
    floor = cloud.points[cloud.intensity < 25]
    assert abs(np.median(floor[:, 2])) < 0.02
    # the room is on the +x side: x points out of the wall
    assert np.median(cloud.points[:, 0]) > 1.0

    meta = mapdir.load_meta(str(out))
    assert meta['anchor']['sensor'] == sensor
    assert meta['anchor']['board_pose_in_map'][2] == pytest.approx(0.40)


def test_anchor_refuses_without_board(tmp_path, monkeypatch):
    monkeypatch.setattr(anchor, 'profile_path',
                        lambda s: os.path.join(PROFILES, f'lab3_{s}.yaml'))
    rng = np.random.default_rng(4)
    pts, inten = classroom(rng)
    keep = inten < 150
    raw = tmp_path / 'raw_map.pcd'
    pc.write_pcd(str(raw), pc.PointCloud(points=slam_frame(pts[keep]), intensity=inten[keep]))
    assert anchor.main([str(raw), str(tmp_path / 'map'), '--sensor', 'robin_w']) != 0
    assert not (tmp_path / 'map' / mapdir.FULL).exists()


def test_anchor_needs_a_lidar(raw_map, tmp_path):
    assert anchor.main([raw_map, str(tmp_path / 'map')]) == 2


def test_voxel_downsample_means():
    pts = np.array([[0.01, 0.01, 0.01], [0.03, 0.03, 0.03], [0.51, 0.0, 0.0]])
    out, inten = downsample.voxel_downsample(pts, np.array([10.0, 30.0, 7.0]), 0.1)
    order = np.argsort(out[:, 0])
    assert np.allclose(out[order], [[0.02, 0.02, 0.02], [0.51, 0.0, 0.0]])
    assert np.allclose(inten[order], [20.0, 7.0])


def test_voxel_downsample_fixed_grid_is_idempotent():
    rng = np.random.default_rng(0)
    pts = rng.uniform(0, 2, (5000, 3))
    once, i1 = downsample.voxel_downsample(pts, np.ones(len(pts)), 0.2)
    twice, _ = downsample.voxel_downsample(once, i1, 0.2)
    assert len(twice) == len(once)
    with pytest.raises(ValueError):
        downsample.voxel_downsample(pts, None, 0.0)


def test_downsample_needs_an_anchored_folder(tmp_path):
    assert downsample.main([str(tmp_path), '--voxel', '0.1']) == 2


def test_downsample_after_anchor(raw_map, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(anchor, 'profile_path',
                        lambda s: os.path.join(PROFILES, f'lab3_{s}.yaml'))
    out = tmp_path / 'map'
    assert anchor.main([raw_map, str(out), '--car', 'car_b']) == 0
    capsys.readouterr()
    assert downsample.main([str(out), '--voxel', '0.2', '--max-mb', '50']) == 0
    text = capsys.readouterr().out
    assert 'PASS' in text and 'warning' in text and 'board:' in text
    small = pc.read_cloud(str(out / mapdir.PCD))
    full = pc.read_cloud(str(out / mapdir.FULL))
    assert small.has_intensity and len(small) < len(full) / 5
    meta = mapdir.load_meta(str(out))
    assert meta['downsample']['voxel_m'] == 0.2
    assert meta['downsample']['board_points'] > 0
    # a tiny limit fails, and says so
    assert downsample.main([str(out), '--voxel', '0.2', '--max-mb', '0.001']) == 1
    with open(out / mapdir.MAP_META) as f:
        assert yaml.safe_load(f)['downsample']['max_mb'] == 0.001
