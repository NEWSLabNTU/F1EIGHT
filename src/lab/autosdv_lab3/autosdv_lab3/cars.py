"""What Lab 3 needs to know about each car's sensors, in one place.

The topics are what the drivers actually publish (pointcloud_preprocessor.launch.py's
LIDAR_TOPICS, imu.launch.xml), not the launch-argument defaults that override them.
"""

# the LiDAR family per car, from each car's autosdv.preset.yaml sensor_suite
# (vlp16_zed on A, robin_zed on B and C)
CAR_SENSOR = {'car_a': 'vlp16', 'car_b': 'robin_w', 'car_c': 'robin_w'}
SENSORS = ('vlp16', 'robin_w')

LIDAR_TOPIC = {
    'vlp16': '/sensing/lidar/velodyne_points',
    'robin_w': '/sensing/lidar/iv_points',
}
LIDAR_FRAME = {'vlp16': 'vlp16', 'robin_w': 'robin_w'}

IMU_RAW = '/sensing/camera/zedxm/imu/data'
IMU = '/sensing/imu/imu_data'
VELOCITY = '/vehicle/status/velocity_status'
TWIST = '/sensing/vehicle_velocity_converter/twist_with_covariance'

# the week-1 mapping stack runs the detector under /lab3, the week-2 initializer
# under /localization (board_pose_initializer.launch.xml's default namespace)
BOARD_POSE_MAPPING = '/lab3/board_detector/board_pose'
BOARD_POSE_RUN = '/localization/board_detector/board_pose'


def sensor_of(car: str) -> str:
    if car not in CAR_SENSOR:
        raise ValueError(f'unknown car {car!r}: one of {", ".join(sorted(CAR_SENSOR))}')
    return CAR_SENSOR[car]
