"""loc_check: is the course's localization good? Drive the car by hand and watch.

Publishes, for RViz (`just lab2 loc-check` opens it):

  /lab2/course_path   nav_msgs/Path, the whole course from path.csv (latched)
  /lab2/trail         nav_msgs/Path, where the localizer has put the car so far
  /lab2/fixes         geometry_msgs/PoseArray, the recent marker fixes

and logs a summary every 10 s and on exit:

  fixes        accepted marker fixes per second while driving
  longest gap  the longest stretch (seconds and metres) with no fix: where the
               EKF coasts on odometry alone
  disagreement how far each fix lands from the EKF's pose at the fix's stamp --
               what odometry and the marker disagree by. Small (a few cm) on a
               good survey; a fix that is consistently far off points at a
               marker surveyed wrong, or knocked since.
  off path     the trail's distance from the course path, if you drive along it
"""

import bisect
import math
import os
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from autosdv_waypoint_lab.course import load_course
from autosdv_waypoint_lab.path import CoursePath, Path, load_csv


def stamp_sec(s) -> float:
    return s.sec + 1e-9 * s.nanosec


class PoseHistory:
    """The EKF's recent (t, x, y), to read its pose at a fix's own stamp: a fix arrives
    a camera frame or more after the image it was computed from, and at 0.8 m/s 100 ms
    of latency alone is 8 cm of apparent disagreement."""

    def __init__(self, horizon: float = 2.0, slack: float = 0.1):
        self.horizon = horizon
        self.slack = slack      # a fix stamped this far past the newest pose reads the newest
        self.t, self.x, self.y = deque(), deque(), deque()

    def add(self, t: float, x: float, y: float):
        if self.t and t < self.t[-1]:          # clock jumped back (bag loop): start over
            self.t.clear(); self.x.clear(); self.y.clear()
        self.t.append(t); self.x.append(x); self.y.append(y)
        while self.t and self.t[0] < t - self.horizon:
            self.t.popleft(); self.x.popleft(); self.y.popleft()

    def at(self, t: float):
        """(x, y) at time t, interpolated; None when t is outside the history.

        A fix stamped at its arrival rather than its image (the vehicle bench's) is newer
        than any EKF pose yet; up to `slack` past the newest, that newest pose is the one
        the fix corrects."""
        if not self.t or t < self.t[0] or t > self.t[-1] + self.slack:
            return None
        if t >= self.t[-1]:
            return self.x[-1], self.y[-1]
        i = bisect.bisect_left(self.t, t)
        if self.t[i] == t or i == 0:
            return self.x[i], self.y[i]
        t0, t1 = self.t[i - 1], self.t[i]
        a = (t - t0) / (t1 - t0)
        return (self.x[i - 1] + a * (self.x[i] - self.x[i - 1]),
                self.y[i - 1] + a * (self.y[i] - self.y[i - 1]))


class LocCheck(Node):
    def __init__(self):
        super().__init__('loc_check')
        self.declare_parameter('course_dir', '')
        self.declare_parameter('trail_spacing', 0.05)   # [m]
        self.declare_parameter('fixes_shown', 30)
        self.declare_parameter('report_period', 10.0)   # [s]
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        course_dir = gp('course_dir')
        if not course_dir or not os.path.isdir(course_dir):
            raise RuntimeError(f'course_dir {course_dir!r} is not a directory')
        c = load_course(course_dir)
        self.path = None
        if os.path.exists(c.path_file):
            x, y, v = load_csv(c.path_file, c.v_ref)
            cp = CoursePath(x, y, v, closed=c.closed)
            xs, ys = list(cp.x), list(cp.y)
            if c.closed:
                xs.append(xs[0]); ys.append(ys[0])
            self.path = Path(xs, ys, [0.0] * len(xs))
        else:
            self.get_logger().warn(f'{c.path_file} not found: no course path drawn, no off-path check')
        self.spacing = float(gp('trail_spacing'))
        self.fixes_shown = int(gp('fixes_shown'))

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_path = self.create_publisher(PathMsg, '/lab2/course_path', latched)
        self.pub_trail = self.create_publisher(PathMsg, '/lab2/trail', 1)
        self.pub_fixes = self.create_publisher(PoseArray, '/lab2/fixes', 1)
        self.create_subscription(Odometry, '/localization/kinematic_state', self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 '/localization/pose_estimator/pose_with_covariance', self.on_fix,
                                 qos_profile_sensor_data)
        self.create_timer(float(gp('report_period')), self.report)
        self.create_timer(0.5, self.publish)

        self.trail = PathMsg()
        self.trail.header.frame_id = 'map'
        self.fixes = []                 # poses for display
        self.last = None                # (t, x, y) of the latest EKF pose
        self.history = PoseHistory()
        self.n_late = 0                 # fixes stamped outside the history: compared to the latest pose
        self.distance = 0.0
        self.moving_time = 0.0
        self.n_fix = 0
        self.n_fix_moving = 0           # the fix rate counts these, over the time spent moving
        self.moving = False
        self.disagree = []
        self.last_fix = None            # (t, distance) at the last fix
        self.gap_t = (0.0, 0.0)         # longest gap: seconds, metres
        self.off_path = []
        self.publish_path()

    def publish_path(self):
        if self.path is None:
            return
        msg = PathMsg()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in zip(self.path.x, self.path.y):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def on_odom(self, msg):
        t = stamp_sec(msg.header.stamp)
        p = msg.pose.pose.position
        self.moving = abs(msg.twist.twist.linear.x) > 0.05
        if self.last is not None:
            d = math.hypot(p.x - self.last[1], p.y - self.last[2])
            if d < 1.0:                      # ignore a re-seed jump
                self.distance += d
            if self.moving and 0.0 < t - self.last[0] < 1.0:
                self.moving_time += t - self.last[0]
        self.last = (t, p.x, p.y)
        self.history.add(t, p.x, p.y)
        poses = self.trail.poses
        if not poses or math.hypot(p.x - poses[-1].pose.position.x,
                                   p.y - poses[-1].pose.position.y) >= self.spacing:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            poses.append(ps)
            if self.path is not None:
                self.off_path.append(abs(self.path.project(p.x, p.y).cross_track))
        if self.last_fix is not None:
            gap = (t - self.last_fix[0], self.distance - self.last_fix[1])
            if gap[0] > self.gap_t[0]:
                self.gap_t = gap

    def on_fix(self, msg):
        t = stamp_sec(msg.header.stamp)
        p = msg.pose.pose.position
        self.n_fix += 1
        self.n_fix_moving += self.moving
        if self.last is not None:
            ref = self.history.at(t)
            if ref is None:
                self.n_late += 1
                ref = self.last[1:]
            self.disagree.append(math.hypot(p.x - ref[0], p.y - ref[1]))
        self.last_fix = (t, self.distance)
        pose = Pose()
        pose.position = p
        pose.orientation = msg.pose.pose.orientation
        self.fixes = (self.fixes + [pose])[-self.fixes_shown:]

    def publish(self):
        now = self.get_clock().now().to_msg()
        self.trail.header.stamp = now
        self.pub_trail.publish(self.trail)
        arr = PoseArray()
        arr.header.frame_id = 'map'
        arr.header.stamp = now
        arr.poses = list(self.fixes)
        self.pub_fixes.publish(arr)

    def summary(self) -> str:
        rate = self.n_fix_moving / self.moving_time if self.moving_time > 1.0 else float('nan')
        dis = np.array(self.disagree) * 100
        parts = [f'driven {self.distance:.1f} m', f'{self.n_fix} fixes ({rate:.1f}/s while moving)',
                 f'longest gap {self.gap_t[0]:.1f} s / {self.gap_t[1]:.1f} m']
        if len(dis):
            parts.append(f'fix vs odometry: median {np.median(dis):.1f} cm, '
                         f'p95 {np.percentile(dis, 95):.1f} cm, max {dis.max():.1f} cm'
                         + (f' ({self.n_late} not time-aligned)' if self.n_late else ''))
        if self.off_path:
            op = np.array(self.off_path) * 100
            parts.append(f'off the course path: median {np.median(op):.0f} cm, max {op.max():.0f} cm')
        return '; '.join(parts)

    def report(self):
        if self.last is None:
            self.get_logger().info('waiting for /localization/kinematic_state (EKF seeded?)',
                                   throttle_duration_sec=30.0)
            return
        self.get_logger().info(self.summary())


def main(args=None):
    rclpy.init(args=args)
    node = LocCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(f'loc_check: {node.summary()}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
