"""virtual_marker_localizer: the ZED + ar_tag_based_localizer, for the bench.

Reads the plant's TRUE pose and the course map (the same lanelet2 OSM the
real localizer loads), and publishes on the real localizer's output topic
what it would publish: one PoseWithCovarianceStamped per camera frame in which
a marker is detected and the fix passes the EKF gate. The model, and how it
follows the stock node, is in marker_sim.py.

  in   ~/input/true_pose   nav_msgs/Odometry   (vehicle_bench_plant/odometry)
       ~/input/ekf_pose    PoseWithCovarianceStamped (the EKF's biased pose,
                           what the stock node gates against)
  out  ~/output/pose_with_covariance
"""

import math
import random

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from autosdv_waypoint_lab.course import parse_osm_landmarks
from autosdv_waypoint_lab.marker_sim import CameraModel, NoiseModel, simulate_fix


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VirtualMarkerLocalizer(Node):
    def __init__(self):
        super().__init__('virtual_marker_localizer')
        declare = self.declare_parameter
        declare('map_file', '')
        declare('rate', 15.0)                     # camera frame rate
        declare('seed', -1)
        declare('camera_x', 0.20)
        declare('camera_y', 0.0)
        declare('hfov_deg', 100.0)
        declare('range_min', 0.4)
        declare('range_max', 5.0)
        declare('max_incidence_deg', 60.0)
        declare('pos_std', 0.02)
        declare('pos_std_per_m', 0.01)
        declare('yaw_std_deg', 1.5)
        declare('dropout', 0.1)
        # the stock localizer's own parameters, from the same param file
        declare('ekf_time_tolerance', 5.0)
        declare('ekf_position_tolerance', 1.5)
        declare('base_covariance', [0.0] * 36)
        declare('marker_size', 0.25)
        declare('target_tag_ids', [''])
        declare('distance_threshold', 5.0)
        declare('consider_orientation', True)
        declare('detection_mode', 'DM_NORMAL')
        declare('min_marker_size', 0.02)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        with open(gp('map_file')) as f:
            self.landmarks = parse_osm_landmarks(f.read())
        ids = [i for i in gp('target_tag_ids') if i]
        if ids:
            self.landmarks = {k: v for k, v in self.landmarks.items() if k in ids}
        if not self.landmarks:
            raise RuntimeError(f'no markers in {gp("map_file")}')
        self.cam = CameraModel(
            x=float(gp('camera_x')), y=float(gp('camera_y')),
            hfov=math.radians(float(gp('hfov_deg'))), range_min=float(gp('range_min')),
            range_max=min(float(gp('range_max')), float(gp('distance_threshold'))),
            max_incidence=math.radians(float(gp('max_incidence_deg'))),
            marker_size=float(gp('marker_size')))
        self.noise = NoiseModel(
            pos_std=float(gp('pos_std')), pos_std_per_m=float(gp('pos_std_per_m')),
            yaw_std=math.radians(float(gp('yaw_std_deg'))), dropout=float(gp('dropout')))
        seed = int(gp('seed'))
        self.rng = random.Random(None if seed < 0 else seed)
        self.ekf_time_tolerance = float(gp('ekf_time_tolerance'))
        self.ekf_position_tolerance = float(gp('ekf_position_tolerance'))
        self.base_cov = [float(c) for c in gp('base_covariance')]
        self.get_logger().info(
            f'{sum(len(v) for v in self.landmarks.values())} markers; {self.cam}; {self.noise}')

        self.true_pose = None
        self.ekf_pose = None
        self.ekf_rx = None
        self.published = 0
        self.gated = 0
        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         '~/output/pose_with_covariance', 10)
        self.create_subscription(Odometry, '~/input/true_pose', self.on_true, 10)
        self.create_subscription(PoseWithCovarianceStamped, '~/input/ekf_pose', self.on_ekf,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / float(gp('rate')), self.on_frame)
        self.create_timer(10.0, self.on_report)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_true(self, msg):
        p = msg.pose.pose
        self.true_pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_ekf(self, msg):
        if msg.header.frame_id != 'map':
            return
        p = msg.pose.pose
        self.ekf_pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.ekf_rx = self.now_sec()

    def on_frame(self):
        if self.true_pose is None:
            return
        ekf = self.ekf_pose
        if ekf is not None and self.now_sec() - self.ekf_rx > self.ekf_time_tolerance:
            ekf = None
        fix = simulate_fix(self.cam, self.noise, self.rng, self.true_pose, ekf,
                           self.landmarks, self.ekf_position_tolerance)
        if fix is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = fix.x
        msg.pose.pose.position.y = fix.y
        msg.pose.pose.orientation.z = math.sin(fix.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(fix.yaw / 2.0)
        msg.pose.covariance = [c * fix.covariance_scale for c in self.base_cov]
        self.pub.publish(msg)
        self.published += 1

    def on_report(self):
        self.get_logger().info(f'{self.published} fixes published so far')


def main(args=None):
    rclpy.init(args=args)
    node = VirtualMarkerLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
