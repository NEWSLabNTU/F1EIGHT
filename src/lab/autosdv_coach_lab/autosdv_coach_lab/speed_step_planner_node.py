"""speed_step_planner: a scripted target-speed sequence as /planning/trajectory.

The Lab 2 speed step test, for fitting the motor plant (lab2_fit_motor). It
takes board_pursuit_planner's place -- start the stack with
`launch_planner:=false` -- and publishes the same fixed-length straight
trajectory, built by the same pursuit shell functions, with the target speed
taken from a schedule instead of a CruiseController:

    steps: [0.0, 0.5, 1.0, 0.5, 1.5, 0.0]    m/s
    hold:  4.0                               s each (or one per step: holds)

Why a trajectory and not a control command. Publishing
/control/command/control_cmd directly would bypass vehicle_cmd_gate (and fight
it for the topic); an external command through the gate needs the REMOTE
operation mode and an external command selector the lab stack does not run;
and a scripted CruiseController inside board_pursuit_planner would still need
a valid coach state, i.e. a faked board, which would silently defeat the
standoff floor. A trajectory publisher changes nothing downstream: Autoware's
trajectory follower turns it into a command, the gate passes it only in
AUTONOMOUS mode, MRM still stops a stale trajectory, and the actuator's
command watchdog, emergency input and stop modes still have the last word.

The schedule starts when the operation mode becomes AUTONOMOUS (`just lab2
engage`) and ends stopped. Disengaging aborts it; engaging again restarts it
from the beginning. It also stops early, with a braking ramp, once the vehicle
has travelled `max_distance` metres since the start, so a schedule longer than
the track ends in a stop and not in a wall. The speed cap (v_max) and the rate
limits are the pursuit shell's.

/planning/coach_pursuit/debug carries one PursuitDebug per cycle:
v_controller is the scheduled target, v_command the clamped one, and limit
says `step i/n`, `idle`, `done` or `distance`.
"""

import math

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autosdv_lab_msgs.msg import PursuitDebug
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from autosdv_coach_lab import pursuit
from autosdv_coach_lab.calibration.speed_steps import (
    DEFAULT_HOLD, DEFAULT_SPEEDS, StepSchedule)


class SpeedStepPlanner(Node):
    def __init__(self):
        super().__init__('speed_step_planner')
        d = pursuit.PlannerParams()
        defaults = {
            'odom_topic': '/localization/kinematic_state',
            'trajectory_topic': '/planning/trajectory',
            'debug_topic': '/planning/coach_pursuit/debug',
            'operation_mode_topic': '/api/operation_mode/state',
            'rate': 10.0,
            'odom_timeout': 0.5,
            'steps': list(DEFAULT_SPEEDS),
            'hold': DEFAULT_HOLD,
            # Per-step hold times; [-1.0] (a ROS array may not be empty) means
            # "use hold for every step".
            'holds': [-1.0],
            'max_distance': 25.0,
            'v_max': d.v_max,
            'accel_max': d.accel_max,
            'decel_max': d.decel_max,
            'brake_decel': d.brake_decel,
            'path_length': d.path_length,
            'path_step': d.path_step,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        holds = [float(h) for h in gp('holds')]
        hold = holds if holds and holds[0] > 0.0 else float(gp('hold'))
        self.schedule = StepSchedule.build([float(v) for v in gp('steps')], hold)
        self.p = pursuit.PlannerParams(
            v_max=float(gp('v_max')), accel_max=float(gp('accel_max')),
            decel_max=float(gp('decel_max')), brake_decel=float(gp('brake_decel')),
            path_length=float(gp('path_length')), path_step=float(gp('path_step')))
        over = [v for v in self.schedule.speeds if v > self.p.v_max]
        if over:
            self.get_logger().warn(
                f'steps {over} exceed v_max {self.p.v_max:.2f} m/s and will be capped')
        self.max_distance = float(gp('max_distance'))
        self.odom_timeout = float(gp('odom_timeout'))

        self.odom = None
        self.odom_rx = None
        self.autonomous = False
        self.t_start = None      # schedule time origin (engage)
        self.state = 'idle'      # idle | running | done | distance
        self.step_index = -1
        self.odometer = 0.0
        self.last_xy = None
        self.v_prev = 0.0
        self.t_prev = None

        self.pub_traj = self.create_publisher(Trajectory, gp('trajectory_topic'), 1)
        self.pub_dbg = self.create_publisher(PursuitDebug, gp('debug_topic'), 10)
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 1)
        mode_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OperationModeState, gp('operation_mode_topic'), self.on_mode, mode_qos)
        self.create_timer(1.0 / float(gp('rate')), self.on_timer)
        self.get_logger().info(
            f'speed steps: {self.schedule.describe()}; stops after '
            f'{self.max_distance:.1f} m. Waiting for AUTONOMOUS (just lab2 engage).')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        self.odom = msg
        self.odom_rx = self.now_sec()
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_xy is not None and self.state == 'running':
            self.odometer += math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
        self.last_xy = xy

    def on_mode(self, msg):
        autonomous = msg.mode == OperationModeState.AUTONOMOUS
        if autonomous and not self.autonomous:
            self.t_start = self.now_sec()
            self.state = 'running'
            self.step_index = -1
            self.odometer = 0.0
            self.v_prev = 0.0
            self.get_logger().info('engaged: speed step schedule started')
        elif not autonomous and self.autonomous and self.state == 'running':
            self.state = 'idle'
            self.get_logger().warn('disengaged: speed step schedule aborted')
        self.autonomous = autonomous

    def scheduled_target(self, now):
        if self.state != 'running':
            return 0.0
        if self.odometer >= self.max_distance:
            self.state = 'distance'
            self.get_logger().warn(
                f'travelled {self.odometer:.1f} m >= max_distance '
                f'{self.max_distance:.1f} m: stopping')
            return 0.0
        v, i = self.schedule.target(now - self.t_start)
        if i != self.step_index:
            self.step_index = i
            if i >= len(self.schedule.speeds):
                self.state = 'done'
                self.get_logger().info(
                    f'speed step schedule done after {self.odometer:.1f} m; holding 0')
                return 0.0
            self.get_logger().info(f'step {i + 1}/{len(self.schedule.speeds)}: {v:.2f} m/s')
        return v

    def on_timer(self):
        now = self.now_sec()
        if self.odom is None or now - self.odom_rx > self.odom_timeout:
            self.get_logger().warn(
                'no fresh odometry; not publishing a trajectory',
                throttle_duration_sec=5.0)
            return
        dt = 0.0 if self.t_prev is None else now - self.t_prev
        self.t_prev = now

        v_target = self.scheduled_target(now)
        if not self.autonomous:
            self.v_prev = 0.0
        # No board: the envelope is given an unbounded range, so only v_max and
        # the rate limits bind.
        far = self.p.standoff_min + 1e6
        v_cmd, limit = pursuit.clamp_speed(v_target, self.v_prev, dt, far, True, self.p)
        self.v_prev = v_cmd
        ego_speed = self.odom.twist.twist.linear.x
        s_stop = pursuit.stop_distance(
            v_cmd, ego_speed, self.p.standoff_min + self.p.path_length, True, self.p)

        pose = self.odom.pose.pose
        q = pose.orientation
        heading = pursuit.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        pts = pursuit.build_trajectory(
            pose.position.x, pose.position.y, heading, v_cmd, s_stop, self.p,
            v_ego=ego_speed)

        stamp = self.get_clock().now().to_msg()
        frame = self.odom.header.frame_id or 'map'
        traj = Trajectory()
        traj.header.stamp = stamp
        traj.header.frame_id = frame
        qz, qw = math.sin(heading / 2.0), math.cos(heading / 2.0)
        for x, y, _, v, acc in pts:
            tp = TrajectoryPoint()
            tp.pose.position.x = x
            tp.pose.position.y = y
            tp.pose.position.z = pose.position.z
            tp.pose.orientation.z = qz
            tp.pose.orientation.w = qw
            tp.longitudinal_velocity_mps = float(v)
            tp.acceleration_mps2 = float(acc)
            traj.points.append(tp)
        self.pub_traj.publish(traj)

        if self.state == 'running':
            label = f'step {self.step_index + 1}/{len(self.schedule.speeds)}'
        else:
            label = self.state
        if limit:
            label += f' ({limit})'
        dbg = PursuitDebug()
        dbg.header.stamp = stamp
        dbg.header.frame_id = frame
        dbg.valid = True
        dbg.autonomous = self.autonomous
        dbg.ego_speed = float(ego_speed)
        dbg.v_controller = float(v_target)
        dbg.v_command = float(v_cmd)
        dbg.s_stop = float(s_stop)
        dbg.limit = label
        self.pub_dbg.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedStepPlanner()
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
