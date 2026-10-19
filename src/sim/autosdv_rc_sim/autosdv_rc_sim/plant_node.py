"""rc_sim_plant: the car's hardware for the interactive simulator (`just sim rc`).

  in   /rc_sim/pwm            Int32MultiArray [channel, count], from the actuator's SimPCA9685
       /initialpose3d         PoseWithCovarianceStamped: place the car (the planning
                              simulator's pose initializer publishes it)
  out  /clock                 every clock_period of simulated time
       /rc_sim/hall_edges     Int64MultiArray of edge times [ns], to velocity_report's SimGPIO;
                              under lockstep one per batch, its layout labelled
                              "until_ns=<ns>" (complete up to the /clock that follows)
       /sensing/camera/zedxm/imu/data   raw IMU in zedxm_imu_link, carrying the static
                              offsets imu_corrector removes
       /localization/kinematic_state    Odometry map -> base_link: the true pose
       /localization/acceleration       AccelWithCovarianceStamped
       TF map -> base_link
       /rc_sim/state          [t, x, y, yaw, vx, vy, yaw_rate, omega_f, omega_r, delta,
                              kappa_r, duty, brake]
       /rc_sim/lockstep/status   String JSON {"ok", "reason", "rtf", "enabled", ...}: latched,
                              1 Hz and on change; ok false for good once a participant timed out
  in   /rc_sim/lockstep/ack   String "<name> <sec> <nanosec> <period_ns> [key=value ...]"
                              (hal.format_ack), from velocity_report and the actuator
                              (sim only); in Lab 2 Part 1 also path_tracker and the
                              scenario's emergency timer ("emergency")

Physics runs at 1 kHz in its own thread, paced to real_time_factor x wall
time; when the CPU falls behind, simulated time slows instead of jumping.
This node owns /clock, so it runs with use_sim_time false and every other
node with true. PWM is held between writes, as the PCA9685 holds it.

Clock handshake (``lockstep``, default true): the plant never publishes a
/clock past a deadline of a participant (velocity_report, the actuator; in Lab 2
Part 1 also path_tracker and the scenario's emergency timer) that it has not
acknowledged, so under load the simulation slows instead of the speed loop
skipping ticks or computing on a stale report. The rules, the ack format and
the equal-instant order are in lockstep_sync.py; a participant silent for
``lockstep_timeout`` wall seconds invalidates the run (status ok false) and
is no longer waited for, so the run never hangs. With no participant (the
headless tests) it changes nothing.
"""
import json
import threading
import time

from autosdv_coach_lab.bench_plant import quat_conjugate, quat_multiply, quat_rotate
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    AccelWithCovarianceStamped, PoseWithCovarianceStamped, TransformStamped)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import (
    Float64MultiArray, Int32MultiArray, Int64MultiArray, MultiArrayDimension, String)
import tf2_ros

from autosdv_rc_sim.detmath import atan2, cos, sin
from autosdv_rc_sim.imu import ImuModel
from autosdv_rc_sim.lockstep_sync import LockstepSync
from autosdv_rc_sim.params import GRAVITY, load_vehicle
from autosdv_rc_sim.plant import Plant
from autosdv_rc_sim.track import default_track_dir, track_poses
# the one definition of the topic names and the ack format
from autosdv_vehicle_interface.hal import (
    EDGE_TOPIC, EDGES_UNTIL, LOCKSTEP_ACK_TOPIC, LOCKSTEP_STATUS_TOPIC, parse_ack, PWM_TOPIC)


#: the ESC and servo points a car's parameter files may set (actuator.yaml's names)
PWM_KEYS = ('init_pwm', 'min_pwm', 'max_pwm', 'brake_pwm', 'forward_start_pwm',
            'forward_hold_pwm', 'reverse_stop_pwm', 'reverse_start_pwm', 'reverse_arm_pwm',
            'init_steer', 'min_steer', 'max_steer', 'tire_angle_to_steer_ratio')


def pwm_overrides(files) -> dict:
    """The PWM_KEYS the actuator's parameter files set, later files winning, as the
    actuator loads them (actuator.yaml, the overrides, the car's individual_params
    file). pwm_param_files makes the plant one physical car's ESC and servo: the Lab 2
    vehicle gate runs a model against each car's points. A key set to 0 (an ESC
    threshold not measured) is not an override."""
    import yaml
    out = {}
    for f in files:
        with open(f) as fh:
            d = yaml.safe_load(fh) or {}
        params = (d.get('/**') or {}).get('ros__parameters') or {}
        for k in PWM_KEYS:
            if k in params and params[k] not in (None, 0):
                out[k] = params[k]
    # an unmeasured forward hold is the neutral, as the actuator's EscModel takes it
    if 'init_pwm' in out and 'forward_hold_pwm' not in out:
        out['forward_hold_pwm'] = out['init_pwm']
    return out


def stamp(t_ns: int) -> TimeMsg:
    msg = TimeMsg()
    msg.sec = t_ns // 1_000_000_000
    msg.nanosec = t_ns % 1_000_000_000
    return msg


def quat_yaw(yaw: float):
    return (0.0, 0.0, sin(0.5 * yaw), cos(0.5 * yaw))


class RcSimPlant(Node):
    def __init__(self, **kwargs):
        super().__init__('rc_sim_plant', **kwargs)
        if self.get_parameter('use_sim_time').value:
            raise RuntimeError('rc_sim_plant owns /clock: run it with use_sim_time:=false')
        for name, default in (('vehicle_file', ''), ('variant', 'nominal'), ('track_dir', ''),
                              ('real_time_factor', 1.0), ('clock_period', 0.005),
                              ('imu_rate', 100.0), ('odom_rate', 50.0),
                              ('imu_topic', '/sensing/camera/zedxm/imu/data'),
                              ('imu_frame', 'zedxm_imu_link'), ('base_frame', 'base_link'),
                              ('map_frame', 'map'), ('angular_velocity_offset_x', 0.0),
                              ('angular_velocity_offset_y', 0.0),
                              ('angular_velocity_offset_z', 0.0),
                              ('seed', 0), ('lockstep', True), ('lockstep_timeout', 2.0),
                              ('pwm_param_files', [''])):
            self.declare_parameter(name, default)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.p = load_vehicle(gp('vehicle_file') or None, gp('variant'),
                              **pwm_overrides([f for f in gp('pwm_param_files') if f]))
        start = track_poses(gp('track_dir') or default_track_dir())['start_pose']
        self.plant = Plant(self.p, start['x'], start['y'], start['yaw'])
        self.imu = ImuModel(self.p, int(gp('seed')))
        self.gyro_offset = tuple(float(gp(f'angular_velocity_offset_{a}')) for a in 'xyz')
        self.base_frame, self.map_frame = gp('base_frame'), gp('map_frame')
        self.imu_frame = gp('imu_frame')
        self.dt = self.p.physics_dt
        self.dt_ns = int(round(self.dt * 1e9))
        self.steps_clock = max(1, int(round(gp('clock_period') / self.dt)))
        self.steps_imu = max(1, int(round(1.0 / gp('imu_rate') / self.dt)))
        self.steps_odom = max(1, int(round(1.0 / gp('odom_rate') / self.dt)))
        self.real_time_factor = float(gp('real_time_factor'))
        self.sync = LockstepSync(gp('lockstep_timeout')) if gp('lockstep') else None
        self.clock_ns = 0                       # the last /clock published
        self.cond = threading.Condition()       # acks wake the physics thread

        self.lock = threading.Lock()
        self.pwm = [self.p.init_pwm, self.p.init_steer]
        self.pending_pose = None
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Int32MultiArray, PWM_TOPIC, self.on_pwm, reliable)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose3d', self.on_initial_pose, 1)
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_edges = self.create_publisher(Int64MultiArray, EDGE_TOPIC, reliable)
        self.pub_imu = self.create_publisher(Imu, gp('imu_topic'), 10)
        self.pub_odom = self.create_publisher(Odometry, '/localization/kinematic_state', 10)
        self.pub_accel = self.create_publisher(
            AccelWithCovarianceStamped, '/localization/acceleration', 10)
        self.pub_state = self.create_publisher(Float64MultiArray, '/rc_sim/state', 10)
        self.tf = tf2_ros.TransformBroadcaster(self)
        self.pub_status = self.create_publisher(String, LOCKSTEP_STATUS_TOPIC, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, LOCKSTEP_ACK_TOPIC, self.on_ack, reliable)
        self.status_wall = time.monotonic()
        self.status_clock_ns = 0
        self.rtf = 0.0
        self.publish_status(self.status_wall)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.q_base_imu = None

        self.outbox = []
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self.run, name='rc_sim_physics', daemon=True)
        self.thread.start()
        self.get_logger().info(
            f"rc_sim plant: variant {gp('variant')}, start {start}, /clock every "
            f'{self.steps_clock * self.dt * 1e3:.0f} ms, '
            f'real-time factor {gp("real_time_factor")}, '
            f"lockstep {'on, timeout %g s' % gp('lockstep_timeout') if self.sync else 'off'}")

    def stop(self) -> None:
        self._stop.set()
        with self.cond:
            self.cond.notify_all()
        self.thread.join(timeout=2.0)

    # ------------------------------------------------------------ input
    def on_pwm(self, msg) -> None:
        if len(msg.data) >= 2 and msg.data[0] in (0, 1):
            with self.lock:
                self.pwm[msg.data[0]] = int(msg.data[1])

    def on_ack(self, msg) -> None:
        if self.sync is None:
            return
        try:
            name, deadline_ns, period_ns, extra = parse_ack(msg.data)
        except ValueError as e:
            self.get_logger().warn(f'ignoring a malformed lockstep ack: {e}',
                                   throttle_duration_sec=5.0)
            return
        if 'pwm' in extra and 'steer' in extra:     # the actuator's ack carries what it wrote
            with self.lock:
                self.pwm = [int(extra['pwm']), int(extra['steer'])]
        with self.cond:
            self.sync.on_ack(name, deadline_ns, period_ns, extra)
            self.cond.notify_all()

    def on_initial_pose(self, msg) -> None:
        q = msg.pose.pose.orientation
        yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.pending_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    # ------------------------------------------------------------ physics
    def wait_for_lockstep(self):
        """Block until /clock may advance; return how far (None: no limit). Never hangs."""
        with self.cond:
            while not self._stop.is_set():
                wall = time.monotonic()
                limit = self.sync.limit(self.clock_ns)
                if limit is None or limit > self.clock_ns:
                    return limit
                was_ok = self.sync.ok
                for name in self.sync.check_timeouts(self.clock_ns, wall):
                    self.get_logger().error(
                        f'lockstep: {name} timed out at t={self.clock_ns * 1e-9:.3f} s; the run '
                        f'is invalid, continuing without waiting for it ({self.sync.reason})')
                if was_ok and not self.sync.ok:
                    self.publish_status(wall)
                elif wall - self.status_wall >= 1.0:
                    self.publish_status(wall)
                self.cond.wait(0.05)
        return None

    def publish_status(self, wall: float) -> None:
        if wall > self.status_wall:
            self.rtf = (self.clock_ns - self.status_clock_ns) * 1e-9 / (wall - self.status_wall)
        self.status_wall, self.status_clock_ns = wall, self.clock_ns
        status = {'ok': True, 'reason': '', 'rtf': round(self.rtf, 4),
                  'enabled': self.sync is not None, 't': self.clock_ns * 1e-9}
        if self.sync is not None:
            status.update(ok=self.sync.ok, reason=self.sync.reason, **self.sync.summary())
        self.pub_status.publish(String(data=json.dumps(status)))

    def run(self) -> None:
        edges, k = [], 0
        next_wall = time.monotonic()
        while not self._stop.is_set() and rclpy.ok():
            # The batch ends clock_period ahead, or at the earliest unacknowledged deadline.
            target = self.plant.t_ns + self.steps_clock * self.dt_ns
            if self.sync is not None:
                limit = self.wait_for_lockstep()
                if self._stop.is_set():
                    break
                if limit is not None:
                    # up to the physics step: a deadline off the 1 ms grid (path_tracker's 30 Hz)
                    # is passed by less than a step, never published ahead of the physics
                    target = min(target, -(-limit // self.dt_ns) * self.dt_ns)
            with self.lock:
                motor, steer = self.pwm
                pose, self.pending_pose = self.pending_pose, None
            if pose is not None:
                self.plant.reset_pose(*pose)
            # Step the batch, holding the stamped data (IMU, odometry, TF, state) back. Then
            # publish, in this order: the edges, the /clock at the batch's END time, the data.
            #  - Edges before the /clock: an edge carries its true time, so it is never
            #    "future", but velocity_report caps its estimate at 1.5 marker spacings per
            #    (clock - last edge): a /clock that leads the edges of its own batch by up to
            #    clock_period made the car read at most ~8 m/s (Task 18c).
            #  - The /clock before the data: no message is stamped later than the clock the
            #    other nodes have already seen.
            while self.plant.t_ns + self.dt_ns <= target:
                edges.extend(self.plant.step(self.dt, motor, steer).edges)
                k += 1
                if k % self.steps_imu == 0:
                    self.publish_imu()
                if k % self.steps_odom == 0:
                    self.publish_odometry()
            new_clock = self.plant.t_ns
            if self.sync is not None:
                # every batch, empty or not, marked complete up to the /clock that follows:
                # velocity_report holds a report until its edges are all in (hal.SimGPIO)
                msg = Int64MultiArray(data=edges)
                msg.layout.dim = [MultiArrayDimension(label=f'{EDGES_UNTIL}{new_clock}',
                                                      size=len(edges), stride=1)]
                self.pub_edges.publish(msg)
                edges = []
            elif edges:
                self.pub_edges.publish(Int64MultiArray(data=edges))
                edges = []
            advanced = new_clock - self.clock_ns
            self.clock_ns = new_clock
            clock = Clock()
            clock.clock = stamp(self.clock_ns)
            self.pub_clock.publish(clock)
            for publish, msg in self.outbox:
                publish(msg)
            self.outbox = []
            now = time.monotonic()
            if now - self.status_wall >= 1.0:
                with self.cond:
                    self.publish_status(now)
            next_wall += advanced * 1e-9 / self.real_time_factor
            delay = next_wall - now
            if delay > 0.0:
                time.sleep(delay)
            elif delay < -0.1:
                next_wall = time.monotonic()    # behind: let simulated time slow down

    # ------------------------------------------------------------ outputs
    def lookup_imu_mount(self) -> bool:
        if self.q_base_imu is not None:
            return True
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.imu_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return False
        r = t.transform.rotation
        self.q_base_imu = (r.x, r.y, r.z, r.w)
        return True

    def publish_imu(self) -> None:
        if not self.lookup_imu_mount():
            return
        s = self.plant.state
        gz, ax, ay = self.imu.sample(s)
        q_inv = quat_conjugate(self.q_base_imu)
        wx, wy, wz = quat_rotate(q_inv, (0.0, 0.0, gz))
        lx, ly, lz = quat_rotate(q_inv, (ax, ay, GRAVITY))
        qx, qy, qz, qw = quat_multiply(quat_yaw(s.yaw), self.q_base_imu)
        msg = Imu()
        msg.header.stamp = stamp(self.plant.t_ns)
        msg.header.frame_id = self.imu_frame
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = qx, qy, qz, qw
        msg.angular_velocity.x = wx + self.gyro_offset[0]
        msg.angular_velocity.y = wy + self.gyro_offset[1]
        msg.angular_velocity.z = wz + self.gyro_offset[2]
        acc = msg.linear_acceleration
        acc.x, acc.y, acc.z = lx, ly, lz
        g = max(self.p.gyro_noise_std, 1e-4)
        g *= g
        a = max(self.p.accel_noise_std, 1e-3)
        a *= a
        msg.angular_velocity_covariance = [g, 0.0, 0.0, 0.0, g, 0.0, 0.0, 0.0, g]
        msg.linear_acceleration_covariance = [a, 0.0, 0.0, 0.0, a, 0.0, 0.0, 0.0, a]
        msg.orientation_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
        self.outbox.append((self.pub_imu.publish, msg))

    def publish_odometry(self) -> None:
        s = self.plant.state
        x, y, yaw = self.plant.base_pose
        vx, vy, wz = self.plant.base_twist
        t = stamp(self.plant.t_ns)
        qx, qy, qz, qw = quat_yaw(yaw)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = t, self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        twist = odom.twist.twist
        twist.linear.x, twist.linear.y, twist.angular.z = vx, vy, wz
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = 1e-4
            odom.twist.covariance[i] = 1e-4
        self.outbox.append((self.pub_odom.publish, odom))
        accel = AccelWithCovarianceStamped()
        accel.header.stamp, accel.header.frame_id = t, self.base_frame
        accel.accel.accel.linear.x, accel.accel.accel.linear.y = s.ax, s.ay
        for i in (0, 7, 14, 21, 28, 35):
            accel.accel.covariance[i] = 1e-3
        self.outbox.append((self.pub_accel.publish, accel))
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id = t, self.map_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.outbox.append((self.tf.sendTransform, tf))
        esc = self.plant.esc_out
        self.outbox.append((self.pub_state.publish, Float64MultiArray(data=[
            self.plant.t_ns * 1e-9, x, y, yaw, s.vx, s.vy, s.yaw_rate, s.omega_f, s.omega_r,
            self.plant.delta, self.plant.kappa_rear, esc.duty, esc.brake])))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RcSimPlant()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.try_shutdown()
