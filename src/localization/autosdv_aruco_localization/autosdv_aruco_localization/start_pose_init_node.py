"""start_pose_init: seed the EKF with the start box pose, once.

The stock ar_tag_based_localizer publishes nothing until the EKF has a pose
to gate its fixes against, and the EKF has no pose until something
initializes it. On the car Autoware's pose_initializer would ask NDT; this lab
has none. So the car starts in a taped box whose pose is in course.yaml, and
this node hands that pose to the EKF:

    /localization/pose_twist_fusion_filter/initialpose3d   (the EKF's initialpose input)
    /localization/pose_twist_fusion_filter/trigger_node    (std_srvs/SetBool true: start)

then exits its work: the first marker fix within ekf_position_tolerance takes
over. ~/reinit (std_srvs/Trigger) does it again, for a car put back in the box.
"""

import math
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger

from autosdv_aruco_localization.marker_map import load_marker_map


class StartPoseInit(Node):
    def __init__(self):
        super().__init__('start_pose_init')
        self.declare_parameter('course_dir', '')
        # position and yaw uncertainty of a car placed in a taped box
        self.declare_parameter('position_std', 0.2)
        self.declare_parameter('yaw_std', math.radians(5.0))
        self.declare_parameter('z', 0.0)
        self.declare_parameter('initialpose_topic',
                               '/localization/pose_twist_fusion_filter/initialpose3d')
        self.declare_parameter('trigger_service',
                               '/localization/pose_twist_fusion_filter/trigger_node')
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        course_dir = gp('course_dir')
        if not course_dir or not os.path.isdir(course_dir):
            raise RuntimeError(f'course_dir {course_dir!r} is not a directory')
        self.start = load_marker_map(course_dir).start_pose
        self.position_std = float(gp('position_std'))
        self.yaw_std = float(gp('yaw_std'))
        self.z = float(gp('z'))

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, gp('initialpose_topic'), qos)
        self.trigger = self.create_client(SetBool, gp('trigger_service'))
        self.create_service(Trigger, '~/reinit', self.on_reinit)
        self.done = False
        self.pending = None
        self.timer = self.create_timer(0.5, self.on_timer)

    def pose_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.start.x
        msg.pose.pose.position.y = self.start.y
        msg.pose.pose.position.z = self.z
        msg.pose.pose.orientation.z = math.sin(self.start.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.start.yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = self.position_std ** 2
        cov[14] = 0.01
        cov[21] = cov[28] = 0.01
        cov[35] = self.yaw_std ** 2
        msg.pose.covariance = cov
        return msg

    def on_timer(self):
        if self.done or self.pending is not None:
            return
        if not self.trigger.service_is_ready():
            self.get_logger().info('waiting for the EKF trigger service',
                                   throttle_duration_sec=5.0)
            return
        if self.pub.get_subscription_count() == 0:
            self.get_logger().info('waiting for the EKF initialpose subscriber',
                                   throttle_duration_sec=5.0)
            return
        self.pub.publish(self.pose_msg())
        self.pending = self.trigger.call_async(SetBool.Request(data=True))
        self.pending.add_done_callback(self.on_triggered)

    def on_triggered(self, fut):
        self.pending = None
        try:
            res = fut.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'EKF trigger failed: {e}')
            return
        if not res.success:
            self.get_logger().warn(f'EKF trigger refused: {res.message}; retrying')
            return
        # the EKF resets its state on activation; send the pose again after it
        self.pub.publish(self.pose_msg())
        self.done = True
        self.get_logger().info(
            f'EKF seeded at the start box ({self.start.x:.2f}, {self.start.y:.2f}, '
            f'{math.degrees(self.start.yaw):.1f} deg); waiting for the first marker')

    def on_reinit(self, request, response):
        self.pub.publish(self.pose_msg())
        response.success = True
        response.message = 'EKF re-seeded at the start box'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StartPoseInit()
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
