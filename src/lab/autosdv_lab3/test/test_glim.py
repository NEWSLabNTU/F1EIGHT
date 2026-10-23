"""lab3_map without GLIM or a ROS runtime: the pieces that decide what GLIM is told,
and the exporter on a dump written the way GLIM 1.2.2 writes one."""

import json
import os

import numpy as np
import pytest

from autosdv_lab3 import glim

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATES = os.path.join(HERE, '..', 'config', 'glim')


def yaw_q(a):
    return (0.0, 0.0, np.sin(a / 2), np.cos(a / 2))


# ---------------------------------------------------------------- transforms

def test_quaternion_round_trip():
    for q in [(0, 0, 0, 1), yaw_q(0.7), (0.5, -0.5, 0.5, 0.5), (1, 0, 0, 0)]:
        q = np.asarray(q, float) / np.linalg.norm(q)
        back = glim.rot_to_quat(glim.quat_to_rot(q))
        assert np.allclose(back, q if q[3] >= 0 else -q, atol=1e-9)


def test_chain_through_common_parent():
    # base_link -> sensor_kit -> {lidar (yawed -0.221), camera -> imu}
    tfs = [
        ('base_link', 'sensor_kit_base_link', (0.1, 0, 0.2), (0, 0, 0, 1)),
        ('sensor_kit_base_link', 'lidar', (0, 0, 0), yaw_q(-0.221)),
        ('sensor_kit_base_link', 'camera_link', (0.05, 0, 0.1), (0, 0, 0, 1)),
        ('camera_link', 'imu_link', (0, 0.03, 0), (0, 0, 0, 1)),
    ]
    T = glim.chain_transform(tfs, 'lidar', 'imu_link')
    # the IMU sits at (0.05, 0.03, 0.1) in the kit frame; seen from the lidar,
    # which is yawed -0.221, that point is rotated by +0.221
    c, s = np.cos(0.221), np.sin(0.221)
    assert np.allclose(T[:3, 3], [c * 0.05 - s * 0.03, s * 0.05 + c * 0.03, 0.1], atol=1e-9)
    assert np.allclose(glim.T_to_tum(T)[3:], yaw_q(0.221), atol=1e-9)
    # GLIM's convention: p_lidar = T_lidar_imu * p_imu
    p_imu = np.array([1.0, 0, 0, 1])
    T_kit_lidar = glim.make_T((0, 0, 0), yaw_q(-0.221))
    T_kit_imu = glim.make_T((0.05, 0.03, 0.1), (0, 0, 0, 1))
    assert np.allclose(T @ p_imu, np.linalg.inv(T_kit_lidar) @ T_kit_imu @ p_imu)


def test_chain_direct_child_and_leading_slash():
    tfs = [('/imu', 'lidar', (0, 0, 0.3), (0, 0, 0, 1))]
    # lidar is 0.3 above the imu, so the imu is 0.3 below the lidar
    assert np.allclose(glim.chain_transform(tfs, 'lidar', 'imu')[:3, 3], [0, 0, -0.3])


def test_chain_robin_w_rotation():
    # Robin-W's own axes are X up, Y right, Z forward (sensor_kit_calibration.yaml)
    r = 0.5 * np.array([1, -1, 1, 1])   # any proper rotation will do
    tfs = [('kit', 'robin_w', (0, 0, 0), r), ('kit', 'imu', (0.2, 0, 0), (0, 0, 0, 1))]
    T = glim.chain_transform(tfs, 'robin_w', 'imu')
    assert np.allclose(T[:3, :3], glim.quat_to_rot(r).T)
    assert np.allclose(T[:3, 3], glim.quat_to_rot(r).T @ [0.2, 0, 0])


def test_chain_disconnected_raises():
    tfs = [('a', 'lidar', (0, 0, 0), (0, 0, 0, 1)), ('b', 'imu', (0, 0, 0), (0, 0, 0, 1))]
    with pytest.raises(glim.Lab3MapError, match='does not connect'):
        glim.chain_transform(tfs, 'lidar', 'imu')


def test_parse_tum():
    T = glim.parse_tum('1 2 3 0 0 0 1')
    expect = np.eye(4)
    expect[:3, 3] = [1, 2, 3]
    assert np.allclose(T, expect)
    with pytest.raises(glim.Lab3MapError):
        glim.parse_tum('1 2 3')


# ---------------------------------------------------------------- topics

IMU = ('sensor_msgs/msg/Imu', 100)
PC = ('sensor_msgs/msg/PointCloud2', 10)


def test_detect_vlp16():
    topics = {'/sensing/lidar/velodyne_points': PC, '/sensing/camera/zedxm/imu/data': IMU,
              '/sensing/imu/imu_data': IMU}
    assert glim.detect_topics(topics) == (
        '/sensing/lidar/velodyne_points', '/sensing/camera/zedxm/imu/data', 'vlp16')


def test_detect_robin_w_and_imu_fallback():
    topics = {'/sensing/lidar/iv_points': PC, '/sensing/imu/imu_data': IMU}
    assert glim.detect_topics(topics) == (
        '/sensing/lidar/iv_points', '/sensing/imu/imu_data', 'robin_w')


def test_detect_skips_empty_and_wrong_type():
    topics = {'/sensing/lidar/velodyne_points': ('sensor_msgs/msg/PointCloud2', 0),
              '/sensing/lidar/iv_points': PC,
              '/sensing/camera/zedxm/imu/data': ('std_msgs/msg/String', 5),
              '/sensing/imu/imu_data': IMU}
    assert glim.detect_topics(topics)[::2] == ('/sensing/lidar/iv_points', 'robin_w')
    assert glim.detect_topics(topics)[1] == '/sensing/imu/imu_data'


def test_detect_errors():
    with pytest.raises(glim.Lab3MapError, match='no LiDAR'):
        glim.detect_topics({'/sensing/imu/imu_data': IMU})
    with pytest.raises(glim.Lab3MapError, match='no IMU'):
        glim.detect_topics({'/sensing/lidar/iv_points': PC})
    custom = {'/my/points': PC, '/sensing/imu/imu_data': IMU}
    with pytest.raises(glim.Lab3MapError, match='--sensor'):
        glim.detect_topics(custom, lidar_topic='/my/points')
    assert glim.detect_topics(custom, 'robin_w', '/my/points')[2] == 'robin_w'
    with pytest.raises(glim.Lab3MapError, match='not a PointCloud2'):
        glim.detect_topics(custom, lidar_topic='/absent')


def test_read_metadata(tmp_path):
    (tmp_path / 'metadata.yaml').write_text("""
rosbag2_bagfile_information:
  version: 5
  storage_identifier: mcap
  duration: {nanoseconds: 2500000000}
  relative_file_paths: [b_0.mcap]
  topics_with_message_count:
    - topic_metadata: {name: /sensing/lidar/iv_points, type: sensor_msgs/msg/PointCloud2}
      message_count: 25
""")
    (tmp_path / 'b_0.mcap').write_bytes(b'x')
    assert glim.bag_dir(str(tmp_path / 'b_0.mcap')) == str(tmp_path)
    m = glim.read_metadata(str(tmp_path))
    assert m['storage'] == 'mcap' and m['duration'] == 2.5
    assert m['topics'] == {'/sensing/lidar/iv_points': ('sensor_msgs/msg/PointCloud2', 25)}
    assert m['files'] == [str(tmp_path / 'b_0.mcap')]


# ---------------------------------------------------------------- config

def fake_defaults(d):
    """A miniature of GLIM's share/glim/config, comments and all."""
    d.mkdir()
    (d / 'config.json').write_text('{\n  /*** Global ***/\n  "global": {\n'
                                   '    "config_odometry": "config_odometry_gpu.json",\n'
                                   '    "config_viewer": "config_viewer.json"\n  }\n}')
    (d / 'config_ros.json').write_text(
        '{\n  // topics\n  "glim_ros": {\n    "extension_modules": [\n'
        '      "libstandard_viewer.so",\n      "librviz_viewer.so" // ,"x"\n    ],\n'
        '    "imu_topic": "/os/imu", // the IMU\n    "url": "http://a//b",\n'
        '    "points_topic": "/os/points",\n  }\n}')
    (d / 'config_sensors.json').write_text(
        '{"sensors": {"T_lidar_imu": [0,0,0,0,0,0,1], "imu_acc_noise": 0.05}}')
    (d / 'config_sub_mapping_gpu.json').write_text(
        '{"sub_mapping": {"submap_downsample_resolution": 0.1, "enable_imu": true}}')
    (d / 'config_logging.json').write_text('{"logging": {"log_dir": "/tmp"}}')
    return str(d)


def test_load_glim_json_strips_comments_not_strings(tmp_path):
    cfg = glim.load_glim_json(os.path.join(fake_defaults(tmp_path / 'd'), 'config_ros.json'))
    assert cfg['glim_ros']['url'] == 'http://a//b'
    assert cfg['glim_ros']['extension_modules'] == ['libstandard_viewer.so', 'librviz_viewer.so']


def test_generate_config(tmp_path):
    defaults = fake_defaults(tmp_path / 'defaults')
    out = tmp_path / 'cfg'
    T = glim.make_T((0.1, 0.2, 0.3), yaw_q(0.5))
    merged = glim.generate_config(str(out), 'vlp16', '/sensing/lidar/velodyne_points',
                                  '/sensing/camera/zedxm/imu/data', T, cpu=False,
                                  log_dir=str(tmp_path / 'log'), defaults=defaults,
                                  templates=TEMPLATES)
    # every default file is there, as plain JSON GLIM and json both read
    assert sorted(os.listdir(out)) == sorted(os.listdir(defaults))
    ros = json.loads((out / 'config_ros.json').read_text())['glim_ros']
    assert ros['points_topic'] == '/sensing/lidar/velodyne_points'
    assert ros['imu_topic'] == '/sensing/camera/zedxm/imu/data'
    assert ros['extension_modules'] == []          # no viewer
    sensors = merged['config_sensors.json']['sensors']
    assert np.allclose(sensors['T_lidar_imu'], [0.1, 0.2, 0.3, *yaw_q(0.5)])
    assert sensors['imu_acc_noise'] == 0.05        # untouched default survives
    assert sensors['intensity_field'] == 'intensity'   # family layer
    sub = merged['config_sub_mapping_gpu.json']['sub_mapping']
    assert sub['submap_downsample_resolution'] <= 0.05  # the board stays sampled
    assert sub['enable_imu'] is True               # untouched default survives
    g = merged['config.json']['global']
    assert g['config_odometry'] == 'config_odometry_gpu.json'
    assert g['config_viewer'] == 'config_viewer.json'
    assert merged['config_logging.json']['logging']['log_dir'] == str(tmp_path / 'log')

    merged = glim.generate_config(str(out), 'robin_w', '/sensing/lidar/iv_points',
                                  '/sensing/imu/imu_data', np.eye(4), cpu=True,
                                  log_dir='/x', defaults=defaults, templates=TEMPLATES)
    g = merged['config.json']['global']
    for key in ('config_odometry', 'config_sub_mapping', 'config_global_mapping'):
        assert g[key].endswith('_cpu.json')


def test_templates_are_plain_json_and_classroom_sized():
    for d, _, files in os.walk(TEMPLATES):
        for f in files:
            if f.endswith('.json'):
                with open(os.path.join(d, f)) as fh:
                    json.load(fh)
    for kind in ('gpu', 'cpu'):
        with open(os.path.join(TEMPLATES, 'common', f'config_global_mapping_{kind}.json')) as f:
            assert json.load(f)['global_mapping']['max_implicit_loop_distance'] <= 20.0
    with open(os.path.join(TEMPLATES, 'common', 'config_global_mapping_pose_graph.json')) as f:
        assert json.load(f)['global_mapping']['min_travel_dist'] <= 10.0


@pytest.mark.skipif(not os.path.isdir('/opt/ros/humble/share/glim/config'),
                    reason='GLIM not installed')
def test_generate_config_against_installed_glim(tmp_path):
    merged = glim.generate_config(str(tmp_path / 'cfg'), 'vlp16', '/p', '/i', np.eye(4),
                                  cpu=False, log_dir=str(tmp_path),
                                  defaults='/opt/ros/humble/share/glim/config',
                                  templates=TEMPLATES)
    assert merged['config_ros.json']['glim_ros']['extension_modules'] == []
    assert 'config_preprocess.json' in merged


# ---------------------------------------------------------------- exporter

def write_submap(path, T_world_origin, pts, inten=None, compact=True):
    """A submap directory as SubMap::save + PointCloud::save_compact write it."""
    os.makedirs(path)
    T = np.asarray(T_world_origin)
    eye = np.eye(4)

    def mat(m):
        return '\n'.join(' '.join(f'{v:g}' for v in row) for row in m)
    with open(os.path.join(path, 'data.txt'), 'w') as f:
        f.write(f'id: 0\nT_world_origin: \n{mat(T)}\nT_origin_endpoint_L: \n{mat(eye)}\n'
                f'T_origin_endpoint_R: \n{mat(eye)}\nT_lidar_imu: \n{mat(eye)}\n'
                'imu_bias: 0 0 0 0 0 0\nframe_id: 0\nnum_frames: 0\n')
    pts = np.asarray(pts, float)
    if compact:
        pts.astype(np.float32).tofile(os.path.join(path, 'points_compact.bin'))
        if inten is not None:
            np.asarray(inten, np.float32).tofile(os.path.join(path, 'intensities_compact.bin'))
    else:
        np.hstack([pts, np.ones((len(pts), 1))]).tofile(os.path.join(path, 'points.bin'))
        if inten is not None:
            np.asarray(inten, np.float64).tofile(os.path.join(path, 'intensities.bin'))
    # covariances are written too; the exporter must ignore them
    np.zeros((len(pts), 6), np.float32).tofile(os.path.join(path, 'covs_compact.bin'))


def test_export_dump(tmp_path):
    dump = tmp_path / 'dump'
    T0 = np.eye(4)
    T1 = glim.make_T((10.0, 0, 0), yaw_q(np.pi / 2))
    write_submap(str(dump / '000000'), T0, [[1, 2, 3], [0, 0, 0]], [5, 200])
    write_submap(str(dump / '000001'), T1, [[1, 0, 0]], [120], compact=False)
    (dump / 'graph.txt').write_text('num_submaps: 2\nnum_all_frames: 0\n')
    (dump / 'odom_lidar.txt').write_text('')

    pts, inten, n = glim.export_dump(str(dump))
    assert n == 2
    # the second submap's point (1,0,0) yawed 90 deg about its origin at x=10
    assert np.allclose(pts, [[1, 2, 3], [0, 0, 0], [10, 1, 0]], atol=1e-6)
    assert np.allclose(inten, [5, 200, 120])


def test_export_dump_without_intensity_and_mismatch(tmp_path):
    dump = tmp_path / 'dump'
    write_submap(str(dump / '000000'), np.eye(4), [[1, 2, 3]])
    pts, inten, n = glim.export_dump(str(dump))
    assert inten is None and n == 1

    (dump / 'graph.txt').write_text('num_submaps: 3\n')
    with pytest.raises(glim.Lab3MapError, match='stale'):
        glim.export_dump(str(dump))


def test_export_dump_empty(tmp_path):
    (tmp_path / 'dump').mkdir()
    with pytest.raises(glim.Lab3MapError, match='no submaps'):
        glim.export_dump(str(tmp_path / 'dump'))


def test_export_writes_pcd_with_intensity(tmp_path):
    pio = pytest.importorskip('reflective_pose_core.pointcloud_io')
    dump = tmp_path / 'dump'
    write_submap(str(dump / '000000'), np.eye(4), [[1, 2, 3], [4, 5, 6]], [7, 250])
    pts, inten, _ = glim.export_dump(str(dump))
    path = str(tmp_path / 'raw_map.pcd')
    pio.write_pcd(path, pio.PointCloud(points=pts, intensity=inten))
    with open(path, 'rb') as f:
        head = f.read(300)
    assert b'FIELDS x y z intensity' in head and b'DATA binary' in head
    back = pio.read_cloud(path)
    assert np.allclose(back.points, pts) and np.allclose(back.intensity, [7, 250])
