"""localization_monitor: is the car localized, and is it on the path?

Two statuses on /diagnostics at `rate`, both read by the lab diagnostic graph
(autosdv-waypoint-main.yaml), so either one in ERROR makes autonomous
unavailable and, if engaged, triggers the MRM stop:

  localization_monitor: marker_fix
      age of the last accepted marker fix (the AR-tag localizer's output, the
      EKF's pose input). OK up to error_age, ERROR beyond it or before the
      first fix. So the car cannot be engaged before it has
      seen a marker, and stops if it drives error_age seconds blind: between
      markers the EKF coasts on odometry, and a long coast is a stale pose.

  localization_monitor: path_deviation
      the car's distance from the course (waypoint_path_publisher's
      ~/progress). ERROR beyond error_distance: a car that far off the path is
      either mislocalized or out of control, and either way should stop.

There is no WARN level: the graph's mode availability treats WARN like ERROR
(measured on the bench: a WARN at 2 s triggered the MRM), so a WARN tier
would only move the limit. The age and distance are in the message instead.
"""

import rclpy
from autosdv_lab_msgs.msg import WaypointProgress
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

OK, ERROR = DiagnosticStatus.OK, DiagnosticStatus.ERROR


def fix_level(age, error_age):
    if age is None:
        return ERROR, 'no marker fix yet'
    if age <= error_age:
        return OK, f'last marker fix {age:.1f} s ago'
    return ERROR, f'no marker fix for {age:.1f} s (limit {error_age} s)'


def deviation_level(dist, age, error_distance, timeout):
    if dist is None or age > timeout:
        return ERROR, 'no course progress'
    if dist <= error_distance:
        return OK, f'{dist:.2f} m from the path'
    return ERROR, f'{dist:.2f} m from the path (limit {error_distance} m)'


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')
        declare = self.declare_parameter
        declare('fix_topic', '/localization/pose_estimator/pose_with_covariance')
        declare('progress_topic', '/planning/waypoint_path_publisher/progress')
        declare('rate', 10.0)
        declare('error_age', 5.0)
        declare('error_distance', 0.8)
        declare('progress_timeout', 1.0)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.error_age = float(gp('error_age'))
        self.error_distance = float(gp('error_distance'))
        self.progress_timeout = float(gp('progress_timeout'))
        self.last_fix = None
        self.fixes = 0
        self.dist = None
        self.dist_rx = None
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        # the stock localizer publishes with the default (reliable) QoS; a
        # sensor-data (best-effort) subscription matches it and the bench's
        self.create_subscription(PoseWithCovarianceStamped, gp('fix_topic'), self.on_fix,
                                 qos_profile_sensor_data)
        self.create_subscription(WaypointProgress, gp('progress_topic'), self.on_progress, 10)
        self.create_timer(1.0 / float(gp('rate')), self.on_timer)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_fix(self, msg):
        self.last_fix = self.now_sec()
        self.fixes += 1

    def on_progress(self, msg):
        self.dist = float(msg.distance_to_path)
        self.dist_rx = self.now_sec()

    def on_timer(self):
        now = self.now_sec()
        age = None if self.last_fix is None else now - self.last_fix
        lvl, text = fix_level(age, self.error_age)
        fix = DiagnosticStatus(level=lvl, name='localization_monitor: marker_fix',
                               hardware_id='localization_monitor', message=text)
        fix.values = [KeyValue(key='fixes', value=str(self.fixes)),
                      KeyValue(key='age', value='' if age is None else f'{age:.2f}')]
        dist_age = float('inf') if self.dist_rx is None else now - self.dist_rx
        lvl, text = deviation_level(self.dist, dist_age,
                                    self.error_distance, self.progress_timeout)
        dev = DiagnosticStatus(level=lvl, name='localization_monitor: path_deviation',
                               hardware_id='localization_monitor', message=text)
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [fix, dev]
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitor()
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
