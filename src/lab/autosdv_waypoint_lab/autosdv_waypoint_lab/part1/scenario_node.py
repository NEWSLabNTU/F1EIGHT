"""lab2_part1_scenario: engage the simulated car, inject the e-stop, release it.

    in   /rc_sim/state                       the plant's true state (x is s on the straight)
         /api/operation_mode/state           availability and the mode
         ~/input/gate_emergency              vehicle_cmd_gate's emergency verdict, moved off
                                             /control/command/emergency_cmd by the launch file
         wait_for_topic (default /lab2/part1/status)   the grader is up (std_msgs/String);
                                             no engage without it, abort after wait_for_timeout
         ~/input/control_cmd                 path_tracker's command, moved off
                                             /control/trajectory_follower/control_cmd
    out  ~/output/emergency                  /control/command/emergency_cmd, the actuator's
                                             ~/input/emergency: the gate's verdict OR the e-stop
         ~/output/control_cmd                /control/trajectory_follower/control_cmd, the
                                             gate's AUTO input: path_tracker's, except around
                                             the e-stop (below)
         /lab2/part1/event                   JSON {"event", "t", ...}; reliable, transient_local
    call /api/operation_mode/change_to_autonomous

Sequence: wait for the plant, the grader (wait_for_topic; aborted after wait_for_timeout),
autonomous availability and Autoware control enabled; engage (retried); publish "engaged".

Why also "control enabled" (OperationModeState.is_autoware_control_enabled): until it is
true, vehicle_cmd_gate replaces its previous command with the car's actual speed on every
cycle, so its output is the path's speed rate-limited around the car's speed, and when the
gate handles two commands close together (a loaded host) it is the car's speed. The flag
follows the vehicle interface's control mode, which turns autonomous a few WALL seconds
after start (control_mode_manager startup_delay), so engaging before it made the launch and
the first step depend on the host's load (docs: P1 report, round 2). When the car passes
estop_trigger_s (part1_profile.yaml) raise the emergency ("estop_on"); once the car has
been at rest (|v| < rest_speed) for rest_time of simulated time, clear it ("estop_off").
Neither takes effect when the state that triggers it is handled: each is scheduled for a
mark, the first multiple of estop_grid (0.1 s) at least estop_margin (0.05 s) after that
state's t, and applied by the emergency timer (emergency_rate, 20 Hz) at its first tick at
or after the mark; the events carry that tick's time. Handled at once, the e-stop landed
before or after an actuator tick by message timing, and the resume diverged between runs.
The e-stop therefore comes 0.05-0.15 s after the car passes estop_trigger_s (0.3-0.9 m at
6 m/s).

With `lockstep` (the launch's lockstep arg) the emergency timer takes part in the rc_sim
plant's clock handshake as participant "emergency" (autosdv_vehicle_interface.hal): each
tick's message is stamped with the tick's deadline and acknowledged after it is published,
and the actuator holds its own tick at that instant until the message has arrived, so the
e-stop acts on exactly the same actuator tick in every run, whatever the host's load. A
forwarded gate verdict is stamped with the last tick's deadline, so it never stands in
for the next tick's message.
The path's speed there is the resume target, approached on a ramp: from estop_on the
speed passed to the gate is 0, and from estop_off it rises from 0 at resume_accel
(part1_profile.yaml estop.resume_accel, 2.5 m/s^2), timed by the state's t at estop_off
and each command's own stamp, until it meets path_tracker's speed,
after which the command passes through unchanged again. A 0 -> 6 m/s step would spin the
rear wheels, which the hall sensor on the driven wheel cannot see. The ramp is here, not
in vehicle_cmd_gate: the gate's lon_acc_lim does rate-limit the commanded velocity, but
the gate never sees the e-stop, so its output would still be 6 m/s when the emergency
clears, and a 2.5 m/s^2 lon_acc_lim would also limit every other command change.

The start is aligned: after the engage the speed passed to the gate stays 0 until the first
multiple of start_grid (0.1 s, a multiple of every loop period: actuator 10 ms,
velocity_report 50 ms, path_tracker 33.3 ms) at least start_delay (0.3 s) later. The engage
itself lands at an arbitrary instant (a service call), so without this the car started at
a different phase of those loops in every run, and the brake-mode switching downstream
turned that into run-to-run spread in the graded numbers.

Anything that does not happen in time is "aborted" with a reason: the grader not up within
wait_for_timeout [wall s], or no engage within engage_timeout [wall s] (360: beyond the
grader's own 300 s engage timeout, so the grader decides first). After the engage,
with a grader (wait_for_topic set) this node never aborts: the grader ends the run (no
progress, MRM, its own timeout). Without one (grader:=false), the run not over within
run_timeout [sim s] (210: the grader's 150 + 60), or the car leaving autonomous mode before
it is at rest past the stop line, aborts. After an abort the e-stop stays raised and the
speed passed to the gate stays 0.

Why the emergency goes through this node: vehicle_cmd_gate publishes its emergency verdict
on every cycle, so a second publisher on the actuator's input would be overwritten 10 times
a second. The launch file remaps the gate's output to ~/input/gate_emergency, and this node
forwards every verdict at once, OR-ed with the e-stop, so the MRM path is unchanged.
"""
import json

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_control_msgs.msg import Control
from builtin_interfaces.msg import Time as TimeMsg
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
import yaml

EVENT_TOPIC = '/lab2/part1/event'


def next_mark_ns(t_ns: int, grid_ns: int, margin_ns: int) -> int:
    """The first multiple of grid_ns at least margin_ns after t_ns."""
    return -(-(t_ns + margin_ns) // grid_ns) * grid_ns


class Part1Scenario(Node):
    def __init__(self, **kwargs):
        super().__init__('lab2_part1_scenario', **kwargs)   # kwargs: parameter_overrides (tests)
        for name, default in (('profile_file', ''), ('wait_for_topic', '/lab2/part1/status'),
                              ('wait_for_timeout', 60.0), ('engage_timeout', 360.0),
                              ('run_timeout', 210.0), ('rest_speed', 0.05), ('rest_time', 1.0),
                              ('auto_engage', True), ('emergency_rate', 20.0),
                              ('resume_accel', 0.0),
                              ('start_grid', 0.1), ('start_delay', 0.3),
                              ('estop_grid', 0.1), ('estop_margin', 0.05), ('lockstep', False)):
            self.declare_parameter(name, default)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        if not gp('profile_file'):
            raise RuntimeError(
                'profile_file is required (config/courses/lab2_part1/part1_profile.yaml)')
        with open(gp('profile_file')) as f:
            prof = yaml.safe_load(f)
        self.trigger_s = float(prof['estop_trigger_s'])
        self.stop_line_s = float(prof['stop_line_s'])
        self.wait_for_timeout = float(gp('wait_for_timeout'))
        self.engage_timeout = float(gp('engage_timeout'))
        self.run_timeout = float(gp('run_timeout'))
        self.rest_speed = float(gp('rest_speed'))
        estop = prof.get('estop') or {}
        self.rest_time = float(estop.get('rest_time', gp('rest_time')))
        self.resume_accel = float(gp('resume_accel')) or float(estop.get('resume_accel', 2.5))
        self.ramp_v = None         # the resume ramp's speed while it limits the command
        self.ramp_t = None
        self.auto_engage = bool(gp('auto_engage'))
        self.start_grid_ns = int(round(float(gp('start_grid')) * 1e9))
        self.start_delay_ns = int(round(float(gp('start_delay')) * 1e9))
        self.release_ns = None     # commands stamped before it are passed as speed 0
        self.estop_grid_ns = int(round(float(gp('estop_grid')) * 1e9))
        self.estop_margin_ns = int(round(float(gp('estop_margin')) * 1e9))
        self.pending = None        # (action, mark_ns): the scheduled estop_on / estop_off
        self.em_stamp = None       # the last emergency tick's stamp (Time), under lockstep

        self.state = None          # [t, x, y, yaw, vx, ...]
        self.mode = None
        self.grader_seen = not gp('wait_for_topic')
        # With a grader, the grader alone ends the run (no progress, MRM, timeout): after the
        # engage this node never aborts, so a stalled car is graded, not pre-empted.
        self.grader_watched = bool(gp('wait_for_topic'))
        self.phase = 'waiting'     # waiting, engaging, running, estop, resumed, done, aborted
        self.engaged_t = None
        self.rest_since = None
        self.estop = False
        self.gate_emergency = False
        self.engage_future = None
        self.wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.wall_start = self.wall.now()

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        events = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_event = self.create_publisher(String, EVENT_TOPIC, events)
        self.pub_emergency = self.create_publisher(
            VehicleEmergencyStamped, '~/output/emergency', 1)
        self.create_subscription(Float64MultiArray, '/rc_sim/state', self.on_state, 10)
        self.create_subscription(
            OperationModeState, '/api/operation_mode/state', self.on_mode, latched)
        self.create_subscription(
            VehicleEmergencyStamped, '~/input/gate_emergency', self.on_gate, 1)
        self.pub_cmd = self.create_publisher(Control, '~/output/control_cmd', 1)
        self.create_subscription(Control, '~/input/control_cmd', self.on_cmd, 1)
        if gp('wait_for_topic'):
            self.create_subscription(String, gp('wait_for_topic'), self.on_grader, latched)
        self.engage_client = self.create_client(
            ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
        # Engage and its watchdog on WALL time: before the engage nothing guarantees /clock runs
        self.create_timer(0.5, self.on_wall_tick, clock=self.wall)
        self.em_timer = self.create_timer(
            1.0 / float(gp('emergency_rate')), self.on_emergency_timer)
        self.lockstep = None
        if bool(gp('lockstep')):
            from autosdv_vehicle_interface import hal
            self._hal = hal
            self.lockstep = hal.make_lockstep_participant(hal.HARDWARE_SIM, self, hal.EMERGENCY)
        self.get_logger().info(
            f'e-stop at s = {self.trigger_s:.2f} m, stop line {self.stop_line_s:.2f} m; '
            f'waiting for {gp("wait_for_topic") or "nothing"}, then autonomous availability')

    # ------------------------------------------------------------------ helpers
    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def wall_elapsed(self) -> float:
        return (self.wall.now() - self.wall_start).nanoseconds * 1e-9

    def event(self, name: str, t=None, **extra) -> None:
        body = {'event': name, 't': round(self.sim_now() if t is None else t, 4)}
        if self.state is not None:
            body.update(s=round(self.state[1], 3), v=round(self.state[4], 3))
        body.update(extra)
        self.pub_event.publish(String(data=json.dumps(body)))
        self.get_logger().info(f'event {json.dumps(body)}')

    def abort(self, reason: str) -> None:
        if self.phase in ('done', 'aborted'):
            return
        self.phase = 'aborted'
        # Brake and stay braked: the run is over, and an abort is no reason to drive on.
        self.estop = True
        self.publish_emergency()
        self.event('aborted', reason=reason)
        self.get_logger().error(f'aborted: {reason}')

    def publish_emergency(self, stamp=None) -> None:
        msg = VehicleEmergencyStamped()
        msg.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        msg.emergency = bool(self.gate_emergency or self.estop)
        self.pub_emergency.publish(msg)

    # ------------------------------------------------------------------ inputs
    def on_gate(self, msg) -> None:
        self.gate_emergency = bool(msg.emergency)
        if self.lockstep is not None:
            # at once (an MRM must not wait), but under the last tick's stamp: it must not
            # satisfy the actuator's wait for the next tick's message
            if self.em_stamp is not None:
                self.publish_emergency(self.em_stamp)
        else:
            self.publish_emergency(msg.stamp)

    def on_cmd(self, msg) -> None:
        """path_tracker -> the gate: 0 during the e-stop, the resume ramp after it."""
        stamp_ns = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        if self.phase in ('estop', 'aborted') or self.release_ns is None \
                or stamp_ns < self.release_ns:
            msg.longitudinal.velocity = 0.0
            msg.longitudinal.acceleration = min(0.0, msg.longitudinal.acceleration)
        elif self.ramp_v is not None:
            t = msg.stamp.sec + msg.stamp.nanosec * 1e-9    # the command's instant, not ours
            self.ramp_v += self.resume_accel * max(0.0, t - self.ramp_t)
            self.ramp_t = t
            if self.ramp_v >= msg.longitudinal.velocity:
                self.ramp_v = None          # caught up: pass through from here on
                self.get_logger().info('resume ramp done')
            else:
                msg.longitudinal.velocity = float(self.ramp_v)
                msg.longitudinal.acceleration = float(self.resume_accel)
        self.pub_cmd.publish(msg)

    def on_grader(self, msg) -> None:
        if not self.grader_seen:
            self.get_logger().info('grader is up')
        self.grader_seen = True

    def on_mode(self, msg) -> None:
        self.mode = msg
        if self.phase == 'engaging' and msg.mode == OperationModeState.AUTONOMOUS:
            self.phase = 'running'
            self.engaged_t = self.sim_now()
            now_ns = self.get_clock().now().nanoseconds + self.start_delay_ns
            grid = self.start_grid_ns
            self.release_ns = -(-now_ns // grid) * grid
            self.event('engaged', start=round(self.release_ns * 1e-9, 4))
        elif (self.phase in ('running', 'estop', 'resumed')
              and msg.mode != OperationModeState.AUTONOMOUS and not self.finished()):
            if self.grader_watched:
                self.get_logger().warn(
                    f'left autonomous mode (mode {msg.mode}); the grader decides')
            else:
                self.abort(f'left autonomous mode (mode {msg.mode})')

    def finished(self) -> bool:
        return (self.state is not None and self.state[1] > self.stop_line_s - 3.0
                and self.rest_since is not None)

    def on_state(self, msg) -> None:
        if len(msg.data) < 5:
            return
        self.state = list(msg.data)
        t, s, v = self.state[0], self.state[1], self.state[4]
        if abs(v) < self.rest_speed:
            self.rest_since = t if self.rest_since is None else self.rest_since
        else:
            self.rest_since = None
        if self.pending is not None:
            pass                    # an e-stop change is scheduled: wait for it
        elif self.phase == 'running' and s >= self.trigger_s:
            self.schedule('estop_on', t)
        elif self.phase == 'estop' and self.rest_since is not None \
                and t - self.rest_since >= self.rest_time:
            self.schedule('estop_off', t)
        elif self.phase == 'resumed' and self.finished() and t - self.rest_since >= 2.0:
            self.phase = 'done'
            self.get_logger().info(f'at rest past the stop line at s = {s:.2f} m: scenario done')
        if not self.grader_watched and self.phase in ('running', 'estop', 'resumed') \
                and self.engaged_t is not None \
                and self.sim_now() - self.engaged_t > self.run_timeout:
            self.abort(f'the run took over {self.run_timeout:.0f} s of simulated time')

    # ------------------------------------------------------------------ e-stop timing
    def schedule(self, action: str, t: float) -> None:
        """Raise (estop_on) or clear (estop_off) the e-stop at the next mark (see the header)."""
        mark_ns = next_mark_ns(int(round(t * 1e9)), self.estop_grid_ns, self.estop_margin_ns)
        self.pending = (action, mark_ns)

    def on_emergency_timer(self) -> None:
        """Every emergency tick: apply a due e-stop change, publish, acknowledge (lockstep)."""
        now_ns = self.get_clock().now().nanoseconds
        tick_ns = now_ns if self.lockstep is None else self._hal.timer_deadline_ns(
            self.em_timer, now_ns)
        if self.pending is not None and tick_ns >= self.pending[1]:
            action, self.pending = self.pending[0], None
            self.apply(action, tick_ns)
        if self.lockstep is None:
            self.publish_emergency()
            return
        sec, nanosec = divmod(tick_ns, 1_000_000_000)
        self.em_stamp = TimeMsg(sec=sec, nanosec=nanosec)
        self.publish_emergency(self.em_stamp)
        self.lockstep.ack(tick_ns, self.em_timer.timer_period_ns)

    def apply(self, action: str, tick_ns: int) -> None:
        mark = tick_ns * 1e-9
        if self.phase in ('done', 'aborted'):
            return
        if action == 'estop_on':
            self.phase = 'estop'
            self.estop = True
            self.rest_since = None
        else:
            self.phase = 'resumed'
            self.estop = False
            self.ramp_v, self.ramp_t = 0.0, mark
        self.event(action, t=mark)

    # ------------------------------------------------------------------ engage
    def on_wall_tick(self) -> None:
        if self.phase not in ('waiting', 'engaging'):
            return
        if self.wall_elapsed() > self.engage_timeout:
            why = ('no /rc_sim/state' if self.state is None else
                   'autonomous mode never became available'
                   if self.mode is None or not self.mode.is_autonomous_mode_available else
                   'Autoware control never became enabled (is_autoware_control_enabled)'
                   if not self.available() else
                   'the engage was refused')
            self.abort(f'not engaged within {self.engage_timeout:.0f} s: {why}')
            return
        if self.state is None:
            return
        if not self.grader_seen:
            # A grader that comes up after the engage would grade a partial run: never start
            # without it (grader:=false launches with wait_for_topic empty instead).
            if self.wall_elapsed() >= self.wait_for_timeout:
                self.abort(f'the grader is not up after {self.wait_for_timeout:.0f} s '
                           f'(no {self.get_parameter("wait_for_topic").value})')
            return
        if not self.auto_engage or not self.available():
            return
        if self.mode is not None and self.mode.mode == OperationModeState.AUTONOMOUS:
            self.phase = 'engaging'
            self.on_mode(self.mode)
            return
        if self.engage_future is not None and not self.engage_future.done():
            return
        if self.engage_future is not None:
            res = self.engage_future.result()
            if res is not None and not res.status.success:
                self.get_logger().warn(f'engage refused: {res.status.message}',
                                       throttle_duration_sec=5.0)
        if self.engage_client.service_is_ready():
            self.phase = 'engaging'
            self.engage_future = self.engage_client.call_async(ChangeOperationMode.Request())

    def available(self) -> bool:
        """Autonomous mode available and Autoware in control of the vehicle (see the header)."""
        return (self.mode is not None and bool(self.mode.is_autonomous_mode_available)
                and bool(self.mode.is_autoware_control_enabled))


def main(args=None):
    rclpy.init(args=args)
    node = Part1Scenario()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
