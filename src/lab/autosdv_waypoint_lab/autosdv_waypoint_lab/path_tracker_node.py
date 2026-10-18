"""path_tracker: trajectory + localized pose -> control command, through the student law.

Replaces Autoware's trajectory_follower for the waypoint lab. Every cycle it
hands the student PathTracker the localized car and the trajectory window
(as a Path), passes the result through TrackerShell (steering limit, the
path's speed as a ceiling, the fault policy) and publishes an
autoware_control_msgs/Control on the topic vehicle_cmd_gate reads as its AUTO
input. The actuator's SpeedPID tracks longitudinal.velocity.

The law is chosen by import path, `tracker_class`, and receives every
`tracker.*` parameter as a plain dict with the prefix removed, plus the
vehicle geometry from vehicle_info.param.yaml.

It always publishes at `rate`, so the gate and the diagnostic graph see it
alive: with no fresh trajectory or pose, or with the law disabled, the
command is speed 0 with the last steering angle.

`lockstep` (default false; Lab 2 Part 1 on the simulated car sets it): take part in
the rc_sim plant's clock handshake (autosdv_vehicle_interface.hal). Each cycle is
acknowledged on /rc_sim/lockstep/ack after its command is published, so the plant
never lets a cycle be skipped on a loaded host, and the actuator can wait for the
command of its instant. While the plant's lockstep is active a cycle also waits for
the odometry of the latest odometry instant at or before the cycle's CLOCK time
(learned from the stamps), which the plant publishes just after that /clock. Not the
deadline: a 30 Hz deadline falls 39 ns before every third 100 ms mark, the plant's
/clock lands on the mark, and the odometry stamped at the mark was then a race.

`path_tracker: tracker_state` on /diagnostics stands in for
trajectory_follower's control_state in the lab graph (control-waypoint.yaml):
ERROR once the law is disabled, OK otherwise (a fault's message included).
"""

import math

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autosdv_lab_msgs.msg import TrackerDebug
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from autosdv_waypoint_lab.path import Path, wrap_angle
from autosdv_waypoint_lab.tracker_api import (
    TrackerShell, VehicleGeometry, VehicleState, load_tracker)

DEFAULT_TRACKER = 'autosdv_waypoint_lab.reference.path_tracker:PathTracker'


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PathTrackerNode(Node):
    def __init__(self):
        super().__init__(
            'path_tracker',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        defaults = {
            'tracker_class': DEFAULT_TRACKER,
            'trajectory_topic': '/planning/trajectory',
            'odom_topic': '/localization/kinematic_state',
            'control_topic': '/control/trajectory_follower/control_cmd',
            'operation_mode_topic': '/api/operation_mode/state',
            'rate': 30.0,
            'trajectory_timeout': 0.5,
            'odom_timeout': 0.3,
            'max_faults': 3,
            'lockstep': False,
            # vehicle_info.param.yaml, loaded by the launch file
            'wheel_base': 0.319,
            'max_steer_angle': 0.349,
        }
        for k, v in defaults.items():
            if not self.has_parameter(k):
                self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        self.vehicle = VehicleGeometry(float(gp('wheel_base')), float(gp('max_steer_angle')))
        params = {name: p.value for name, p in self.get_parameters_by_prefix('tracker').items()}
        spec = gp('tracker_class')
        self.shell = TrackerShell(load_tracker(spec, params, self.vehicle), self.vehicle,
                                  max_faults=int(gp('max_faults')))
        self.get_logger().info(f'tracker {spec} with {params}; {self.vehicle}')
        self.traj_timeout = float(gp('trajectory_timeout'))
        self.odom_timeout = float(gp('odom_timeout'))

        self.traj = None
        self.traj_rx = None
        self.path = None
        self.odom = None
        self.odom_rx = None
        self.autonomous = False
        self.t_prev = None
        self.last_steer = 0.0
        self.reported_disabled = False

        self.pub_cmd = self.create_publisher(Control, gp('control_topic'), 1)
        self.pub_dbg = self.create_publisher(TrackerDebug, '~/debug', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.diag_level = DiagnosticStatus.OK
        self.diag_message = 'tracking'
        self.create_timer(0.2, self.on_diag)
        self.create_subscription(Trajectory, gp('trajectory_topic'), self.on_traj, 1)
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 1)
        mode_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OperationModeState, gp('operation_mode_topic'), self.on_mode, mode_qos)
        self.timer = self.create_timer(1.0 / float(gp('rate')), self.on_timer)

        # Lab 2 Part 1 only (see the module docstring); the hal import stays off the car's path
        self.lockstep = None
        self.held_ns = None          # the held cycle's deadline (for its ack)
        self.held_at_ns = None       # and its clock time (what its odometry must reach)
        self.odom_stamp_ns = None
        self.odom_period_ns = None
        if bool(gp('lockstep')):
            from autosdv_vehicle_interface import hal
            self._hal = hal
            self.lockstep = hal.make_lockstep_participant(
                hal.HARDWARE_SIM, self, hal.PATH_TRACKER, on_status=self.on_lockstep_status)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_traj(self, msg):
        self.traj = msg
        self.traj_rx = self.now_sec()
        self.path = None   # rebuilt on use

    def on_odom(self, msg):
        self.odom = msg
        self.odom_rx = self.now_sec()
        if self.lockstep is not None:
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if self.odom_stamp_ns is not None and stamp > self.odom_stamp_ns:
                self.odom_period_ns = stamp - self.odom_stamp_ns
            self.odom_stamp_ns = stamp
            if self.held_ns is not None and not self.odom_awaiting(self.held_at_ns):
                held, self.held_ns = self.held_ns, None
                self.lockstep_cycle(held)

    def odom_awaiting(self, clock_ns: int) -> bool:
        """True while the odometry of the latest instant at or before ``clock_ns`` is missing."""
        if self.odom_stamp_ns is None or self.odom_period_ns is None:
            return False
        return self.odom_stamp_ns + self.odom_period_ns <= clock_ns

    def on_lockstep_status(self, active: bool) -> None:
        if not active and self.held_ns is not None:
            held, self.held_ns = self.held_ns, None
            self.lockstep_cycle(held)

    def lockstep_cycle(self, deadline_ns: int) -> None:
        self.cycle()
        self.lockstep.ack(deadline_ns, self.timer.timer_period_ns)

    def on_mode(self, msg):
        autonomous = msg.mode == OperationModeState.AUTONOMOUS
        if autonomous and not self.autonomous:
            self.shell.reset()
        self.autonomous = autonomous

    def current_path(self):
        if self.path is None and self.traj is not None and len(self.traj.points) >= 2:
            pts = self.traj.points
            xs = [p.pose.position.x for p in pts]
            ys = [p.pose.position.y for p in pts]
            vs = [float(p.longitudinal_velocity_mps) for p in pts]
            yaws = [yaw_from_quaternion(p.pose.orientation) for p in pts]
            self.path = Path(xs, ys, vs, yaw=yaws)
        return self.path

    def on_timer(self):
        if self.lockstep is None:
            self.cycle()
            return
        now_ns = self.get_clock().now().nanoseconds
        deadline = self._hal.timer_deadline_ns(self.timer, now_ns)
        if self.held_ns is not None:        # superseded (a lockstep timeout): run it late
            held, self.held_ns = self.held_ns, None
            self.lockstep_cycle(held)
        if self.lockstep.active and self.odom_awaiting(now_ns):
            self.held_ns, self.held_at_ns = deadline, now_ns
            return
        self.lockstep_cycle(deadline)

    def cycle(self):
        now = self.now_sec()
        dt = 0.0 if self.t_prev is None else now - self.t_prev
        self.t_prev = now
        stamp = self.get_clock().now().to_msg()
        dbg = TrackerDebug()
        dbg.header.stamp = stamp
        dbg.header.frame_id = 'map'
        dbg.autonomous = self.autonomous
        nan = float('nan')
        dbg.steering_raw = dbg.speed_raw = nan
        dbg.cross_track = dbg.heading_error = dbg.path_speed = nan

        fresh_odom = self.odom is not None and now - self.odom_rx <= self.odom_timeout
        fresh_traj = self.traj is not None and now - self.traj_rx <= self.traj_timeout
        path = self.current_path() if fresh_traj else None
        if not fresh_odom or path is None:
            why = 'no fresh pose' if not fresh_odom else 'no fresh trajectory'
            self.get_logger().warn(f'{why}: commanding stop', throttle_duration_sec=5.0)
            steer, speed = self.last_steer, 0.0
            dbg.fault = why
        else:
            pose = self.odom.pose.pose
            state = VehicleState(pose.position.x, pose.position.y,
                                 yaw_from_quaternion(pose.orientation),
                                 float(self.odom.twist.twist.linear.x))
            res = self.shell.step(state, path, dt)
            steer, speed = res.command.steering_angle, res.command.speed
            proj = path.project(state.x, state.y)
            dbg.x, dbg.y, dbg.yaw, dbg.speed = state.x, state.y, state.yaw, state.speed
            dbg.cross_track = proj.cross_track
            dbg.heading_error = wrap_angle(state.yaw - proj.yaw)
            dbg.path_speed = path.speed_at(proj.s)
            if res.raw is not None:
                dbg.steering_raw, dbg.speed_raw = res.raw.steering_angle, res.raw.speed
            dbg.limit = res.limit
            dbg.fault = res.fault
            dbg.disabled = res.disabled
            if res.fault:
                self.get_logger().error(
                    f'tracker fault ({self.shell.faults}/{self.shell.max_faults}): {res.fault}',
                    throttle_duration_sec=2.0)
            if res.disabled:
                self.diag_level = DiagnosticStatus.ERROR
                self.diag_message = 'tracker disabled after repeated faults'
            elif res.fault:
                # OK, not WARN: the graph treats WARN as unavailable, which
                # would make one fault an MRM. A fault already commands a stop
                # for that cycle; max_faults in a row is what disables.
                self.diag_level = DiagnosticStatus.OK
                self.diag_message = f'tracker fault: {res.fault}'
            else:
                self.diag_level = DiagnosticStatus.OK
                self.diag_message = 'tracking'
            if res.disabled and not self.reported_disabled:
                self.get_logger().error(
                    'tracker disabled after repeated faults: speed 0 until restart')
                self.reported_disabled = True
        self.last_steer = steer

        cmd = Control()
        cmd.stamp = stamp
        cmd.control_time = stamp
        cmd.lateral.stamp = stamp
        cmd.lateral.control_time = stamp
        cmd.lateral.steering_tire_angle = float(steer)
        cmd.longitudinal.stamp = stamp
        cmd.longitudinal.control_time = stamp
        cmd.longitudinal.velocity = float(speed)
        v_now = float(self.odom.twist.twist.linear.x) if self.odom is not None else 0.0
        cmd.longitudinal.acceleration = float(max(-1.5, min(1.0, (speed - v_now) / 0.5)))
        self.pub_cmd.publish(cmd)

        dbg.steering = float(steer)
        dbg.speed_command = float(speed)
        self.pub_dbg.publish(dbg)

    def on_diag(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [DiagnosticStatus(level=self.diag_level, name='path_tracker: tracker_state',
                                       hardware_id='path_tracker', message=self.diag_message)]
        self.pub_diag.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PathTrackerNode()
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
