"""rc_sim_replay: play a graded trace on the topics Autoware's RViz reads.

What the student sees is exactly what was graded: the pose, speeds and PWM come
from the lockstep trace (100 Hz), replayed at `rate` x real time. Violations of
the grader's checks are marked on the map. No planner, no physics: cheap.
"""
from bisect import bisect_right

from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from autosdv_rc_sim.detmath import cos, sin
from autosdv_rc_sim.lockstep import Trace


SPHERE_SIZE = 1.5   # m
TEXT_SIZE = 1.0     # m


def index_at(times, elapsed: float) -> int:
    """The last row whose time is <= elapsed (clamped to the trace)."""
    return max(0, min(len(times) - 1, bisect_right(times, elapsed) - 1))


def mark_markers(marks) -> list:
    """A red sphere and a text label per violation, sized to read on a 115 m straight at 10 px/m.

    Labels at the same place are stacked so they do not overprint.
    """
    out, stacked = [], {}
    for i, m in enumerate(marks):
        level = stacked.get(m['s'], 0)
        stacked[m['s']] = level + 1
        for kind, size, z in ((Marker.SPHERE, SPHERE_SIZE, 0.0),
                              (Marker.TEXT_VIEW_FACING, TEXT_SIZE,
                               SPHERE_SIZE + TEXT_SIZE + 1.5 * TEXT_SIZE * level)):
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.ns, mk.id, mk.type = 'rc_sim_violations', 2 * i + (kind == Marker.TEXT_VIEW_FACING), kind
            mk.pose.position.x, mk.pose.position.z = float(m['s']), z
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = size
            mk.color.r, mk.color.a = 1.0, 1.0
            mk.text = m['text']
            out.append(mk)
    return out


class Replay(Node):
    def __init__(self, **kwargs):
        super().__init__('rc_sim_replay', **kwargs)
        self.declare_parameter('trace', '')
        self.declare_parameter('violations', '')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('loop', False)
        self.trace = Trace.read_csv(self.get_parameter('trace').value)
        self.times = [t - self.trace.rows[0][0] for t in self.trace.column('t')]
        self.rate = float(self.get_parameter('rate').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.cols = {name: self.trace.column(name) for name in
                     ('s', 'y', 'yaw', 'target', 'measured', 'speed', 'pwm', 'kappa_r', 'slip_r', 'delta')}
        self.pub_odom = self.create_publisher(Odometry, '/localization/kinematic_state', 10)
        self.pub_vel = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)
        self.pub_steer = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
        self.pub_values = self.create_publisher(Float32MultiArrayStamped, '/rc_sim/replay/values', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_marks = self.create_publisher(MarkerArray, '/rc_sim/replay/violations', latched)
        self.tf = tf2_ros.TransformBroadcaster(self)
        self.start = self.get_clock().now()
        self.publish_violations(self.get_parameter('violations').value)
        self.create_timer(0.01, self.tick)

    def publish_violations(self, path: str) -> None:
        if not path:
            return
        with open(path) as f:
            marks = yaml.safe_load(f) or []
        array = MarkerArray()
        array.markers.extend(mark_markers(marks))
        self.pub_marks.publish(array)

    def tick(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start).nanoseconds * 1e-9 * self.rate
        if self.loop and elapsed > self.times[-1]:
            self.start = now
            elapsed = 0.0
        i = index_at(self.times, elapsed)
        c = self.cols
        stamp = now.to_msg()
        qz, qw = sin(0.5 * c['yaw'][i]), cos(0.5 * c['yaw'][i])
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'map', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = c['s'][i], c['y'][i]
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf.sendTransform(tf)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'map', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = c['s'][i], c['y'][i]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = c['speed'][i]
        self.pub_odom.publish(odom)
        vel = VelocityReport()
        vel.header.stamp, vel.header.frame_id = stamp, 'base_link'
        vel.longitudinal_velocity = c['measured'][i]
        self.pub_vel.publish(vel)
        steer = SteeringReport()
        steer.stamp, steer.steering_tire_angle = stamp, c['delta'][i]
        self.pub_steer.publish(steer)
        values = Float32MultiArrayStamped()
        values.stamp = stamp
        values.data = [float(c[k][i]) for k in ('target', 'measured', 'speed', 'pwm', 'kappa_r', 'slip_r')]
        self.pub_values.publish(values)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Replay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
