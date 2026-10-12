"""waypoint_path_publisher: the TA's course -> /planning/trajectory.

Replaces Autoware planning for the waypoint lab. Loads courses/<name>/
course.yaml and its path CSV, tracks the car's progress along the course from
/localization/kinematic_state, and publishes the window of the course around
the car (window_behind metres back, window_ahead metres ahead) as an Autoware
Trajectory at `rate`. Each point's velocity is the waypoint speed, capped so
the car stops exactly at the end of the run (CoursePath.window).

Progress and laps go out on ~/progress (WaypointProgress), which is what the
grader counts laps from. ~/restart (std_srvs/Trigger) starts a new run from
wherever the car is, without relaunching: the TA's "again".
"""

import math
import os

import rclpy
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autosdv_lab_msgs.msg import WaypointProgress
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

from autosdv_waypoint_lab.course import load_course
from autosdv_waypoint_lab.path import CoursePath, load_csv


class WaypointPathPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_path_publisher')
        declare = self.declare_parameter
        declare('course_dir', '')
        declare('laps', 0)               # 0: the course's own
        declare('v_max', 0.8)
        declare('decel', 0.3)
        declare('spacing', 0.1)
        declare('window_behind', 1.0)
        declare('window_ahead', 6.0)
        declare('rate', 10.0)
        declare('odom_timeout', 0.5)
        declare('odom_topic', '/localization/kinematic_state')
        declare('trajectory_topic', '/planning/trajectory')
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        course_dir = gp('course_dir')
        if not course_dir or not os.path.isdir(course_dir):
            raise RuntimeError(f'course_dir {course_dir!r} is not a directory')
        self.course = load_course(course_dir)
        laps = int(gp('laps')) or self.course.laps
        self.v_max = float(gp('v_max'))
        self.decel = float(gp('decel'))
        self.spacing = float(gp('spacing'))
        self.laps = laps
        self.window_behind = float(gp('window_behind'))
        self.window_ahead = float(gp('window_ahead'))
        self.odom_timeout = float(gp('odom_timeout'))
        self.path = self._new_path()
        self.get_logger().info(
            f'course {course_dir}: {"closed" if self.course.closed else "open"}, '
            f'{self.path.lap_length:.2f} m a lap, {self.path.laps} lap(s), '
            f'v_max {self.v_max} m/s')

        self.odom = None
        self.odom_rx = None
        self.pub_traj = self.create_publisher(Trajectory, gp('trajectory_topic'), 1)
        self.pub_progress = self.create_publisher(WaypointProgress, '~/progress', 10)
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 1)
        self.create_service(Trigger, '~/restart', self.on_restart)
        self.create_timer(1.0 / float(gp('rate')), self.on_timer)

    def _new_path(self):
        x, y, v = load_csv(self.course.path_file, self.course.v_ref)
        return CoursePath(x, y, v, closed=self.course.closed, spacing=self.spacing,
                          v_max=self.v_max, decel=self.decel, laps=self.laps)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        self.odom = msg
        self.odom_rx = self.now_sec()

    def on_restart(self, request, response):
        self.path = self._new_path()
        response.success = True
        response.message = 'progress reset; the next run starts from the car'
        self.get_logger().info(response.message)
        return response

    def on_timer(self):
        now = self.now_sec()
        if self.odom is None or now - self.odom_rx > self.odom_timeout:
            self.get_logger().warn(
                'no fresh /localization/kinematic_state; not publishing a trajectory',
                throttle_duration_sec=5.0)
            return
        p = self.odom.pose.pose.position
        was_finished = self.path.finished
        progress, dist = self.path.update_progress(p.x, p.y)
        if self.path.finished and not was_finished:
            self.get_logger().info(f'run finished: {self.path.laps_done} lap(s)')
        win = self.path.window(self.window_behind, self.window_ahead)

        stamp = self.get_clock().now().to_msg()
        traj = Trajectory()
        traj.header.stamp = stamp
        traj.header.frame_id = 'map'
        for x, y, yaw, v in zip(win.x, win.y, win.yaw, win.v):
            tp = TrajectoryPoint()
            tp.pose.position.x = x
            tp.pose.position.y = y
            tp.pose.position.z = p.z
            tp.pose.orientation.z = math.sin(yaw / 2.0)
            tp.pose.orientation.w = math.cos(yaw / 2.0)
            tp.longitudinal_velocity_mps = float(v)
            traj.points.append(tp)
        self.pub_traj.publish(traj)

        msg = WaypointProgress()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.progress = float(progress)
        msg.lap_length = float(self.path.lap_length)
        msg.run_length = float(self.path.run_length)
        msg.laps_done = int(self.path.laps_done)
        msg.laps = int(self.path.laps)
        msg.finished = bool(self.path.finished)
        msg.distance_to_path = float(dist)
        self.pub_progress.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPathPublisher()
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
