"""The mapping-bag checker on small synthetic bags written with rosbag2_py."""

import json

import pytest

rosbag2_py = pytest.importorskip('rosbag2_py')
from rclpy.serialization import serialize_message  # noqa: E402

from autosdv_lab3 import cars, check_bag  # noqa: E402

LIDAR = cars.LIDAR_TOPIC['robin_w']


def _msgs():
    from autoware_vehicle_msgs.msg import VelocityReport
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from tf2_msgs.msg import TFMessage
    return VelocityReport, PoseWithCovarianceStamped, TransformStamped, Imu, PointCloud2, \
        PointField, TFMessage


def write_bag(path, *, duration=70.0, vmax=0.6, board_ends=True, time_field=True,
              tf_chain=True, imu_hz=200.0, still=10.0):
    VelocityReport, Pose, TransformStamped, Imu, PointCloud2, PointField, TFMessage = _msgs()
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for i, (name, typ) in enumerate([
            (LIDAR, 'sensor_msgs/msg/PointCloud2'), (cars.IMU_RAW, 'sensor_msgs/msg/Imu'),
            ('/tf_static', 'tf2_msgs/msg/TFMessage'),
            (cars.VELOCITY, 'autoware_vehicle_msgs/msg/VelocityReport'),
            (cars.BOARD_POSE_MAPPING, 'geometry_msgs/msg/PoseWithCovarianceStamped')]):
        w.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ,
                                                serialization_format='cdr'))
    ns = lambda t: int((1000.0 + t) * 1e9)  # noqa: E731

    tf = TFMessage()
    for parent, child in (('base_link', 'sensor_kit_base_link'),
                          ('sensor_kit_base_link', 'robin_w'),
                          ('sensor_kit_base_link', 'zed_imu_link')):
        if not tf_chain and child == 'zed_imu_link':
            continue
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        tf.transforms.append(t)
    w.write('/tf_static', serialize_message(tf), ns(0.0))

    cloud = PointCloud2()
    cloud.header.frame_id = 'robin_w'
    names = ['x', 'y', 'z', 'intensity'] + (['timestamp'] if time_field else [])
    cloud.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                    for k, n in enumerate(names)]
    imu = Imu()
    imu.header.frame_id = 'zed_imu_link'
    t = 0.0
    while t <= duration:
        if abs(t * 10 - round(t * 10)) < 1e-6:
            w.write(LIDAR, serialize_message(cloud), ns(t))
            v = VelocityReport()
            moving = still < t < duration - still
            v.longitudinal_velocity = vmax if moving else 0.0
            w.write(cars.VELOCITY, serialize_message(v), ns(t))
            near_end = t < 10.0 or t > duration - 10.0
            if board_ends and near_end and abs(t - round(t)) < 1e-6:
                w.write(cars.BOARD_POSE_MAPPING, serialize_message(Pose()), ns(t))
        w.write(cars.IMU_RAW, serialize_message(imu), ns(t))
        t = round(t + 1.0 / imu_hz, 6)
    del w
    (path / 'lab3_meta.json').write_text(json.dumps({'lidar_topic': LIDAR}))
    return str(path)


def verdict(path, **kw):
    return {name: ok for name, ok, _ in check_bag.check(check_bag.scan_bag(path), 0.8, 60.0)}


def test_good_bag_passes(tmp_path):
    rows = verdict(write_bag(tmp_path / 'good'))
    assert all(rows.values()), rows
    assert check_bag.main([str(tmp_path / 'good')]) == 0


@pytest.mark.parametrize('kw, failing', [
    (dict(vmax=1.2), 'speed <= 0.8 m/s'),
    (dict(board_ends=False), 'board detected in the first and last 15 s'),
    (dict(time_field=False), 'LiDAR intensity and per-point time'),
    (dict(tf_chain=False), '/tf_static links LiDAR and IMU'),
    (dict(duration=40.0), 'duration >= 60 s'),
    (dict(imu_hz=50.0), 'IMU >= 100 Hz'),
    (dict(still=3.0), 'still >= 8 s at start and end'),
])
def test_each_requirement_fails_alone(tmp_path, kw, failing):
    rows = verdict(write_bag(tmp_path / 'bag', **kw))
    assert [n for n, ok in rows.items() if not ok] == [failing], rows


def test_connected_either_direction():
    edges = [('a', 'b'), ('c', 'b')]
    assert check_bag.connected(edges, 'a', 'c')
    assert not check_bag.connected(edges, 'a', 'd')
