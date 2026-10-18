"""lab2_part1_grader: grade the live Lab 2 Part 1 run from the plant's true state.

Builds the 100 Hz trace on sim time (the plant's clock), evaluates the checks of
metrics.py as their windows close, and publishes:

    /lab2/part1/status      std_msgs/String JSON, 10 Hz and on change (transient_local)
                            {state, t, segment, checks: {name: {value, limit, pass}},
                             violations: [...], marks: {passed, total} | null,
                             invalid_reason, variant, stop_line_s, pwm_levels, trace_file}
                            pass is null while a check's window is still open
    /lab2/part1/trace       std_msgs/Float64MultiArray, the new trace rows (TRACE_FIELDS,
                            row-major), 10 Hz: what the dashboard plots
    /lab2/part1/violations  visualization_msgs/MarkerArray in map: where each check failed
    /lab2/part1/markers     the stop line and the e-stop trigger (latched)
    /lab2/part1/course_path nav_msgs/Path of path.csv (latched)
    /lab2/part1/car         the car glyph and live readout (rc_sim's replay glyph)

On finish (at rest for 2 s in the final brake segment or past it), an early valid end (the car
did not move or stopped for 10 s with a target; or run_timeout sim s after the engage: the
unreached checks fail, with a student-facing `message`) or invalidity (MRM, controller fault,
actuator not running, lockstep invalidated, scenario aborted, e-stop never raised, no engage within engage_timeout)
it writes
out_dir/trace.csv, then out_dir/marks.json (the runner waits for that file), publishes
the final status and stays alive.

Trace alignment: a row at t is held until the stamped streams have reached t (a control
command and an emergency message stamped after t) or ALIGN_LAG (50 ms) of simulated time has
passed, then `target` is the command with the latest stamp <= t (vehicle_cmd_gate and the
forwarder keep path_tracker's stamp, and the actuator's tick at t uses exactly that command)
and `emergency` is the e-stop state at t from the scenario's events (estop_on / estop_off
carry their exact time). The actuator's ':emergency' mode no longer counts: it is latched
whenever the message arrives (one row off at the clear), and the other emergencies it shows
(MRM, the gate's) invalidate the run anyway. `measured`, `pwm` and `mode`
remain the latest received (display only). Taking the last message received instead put the
e-stop and resume-window edges, which metrics.py reads from these two columns, one row early
or late depending on message timing.

Parameters: profile_file (part1_profile.yaml), course_dir (default: its directory;
path.csv and course.yaml give the path s is measured along), out_dir, variant,
run_timeout [sim s], the input topics, pwm levels for the dashboard.
"""
import bisect
from collections import deque
import json
import math
import os
from pathlib import Path
from typing import Optional

from autoware_adapi_v1_msgs.msg import MrmState
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, Int32MultiArray, MultiArrayDimension, String
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from autosdv_waypoint_lab.part1 import metrics as M
from autosdv_waypoint_lab.path import Path as CoursePolyline, load_csv

SAMPLE_DT = 0.01                    # s: the trace's 100 Hz grid
ALIGN_LAG = 0.05                    # s of sim time a row waits at most for stamped inputs
TRACE_HISTORY = 3000                # /lab2/part1/trace messages kept for a late dashboard (300 s)
TRACE_FIELDS = ('t', 's', 'y', 'target', 'measured', 'speed', 'pwm', 'slip_r', 'kappa_r',
                'emergency')
# /rc_sim/state: [t, x, y, yaw, vx, vy, yaw_rate, omega_f, omega_r, delta, kappa_r, duty, brake]
ST_T, ST_X, ST_Y, ST_YAW, ST_VX, ST_OMEGA_R, ST_KAPPA_R = 0, 1, 2, 3, 4, 8, 10
MRM_NAMES = {MrmState.MRM_OPERATING: 'MRM_OPERATING', MrmState.MRM_SUCCEEDED: 'MRM_SUCCEEDED',
             MrmState.MRM_FAILED: 'MRM_FAILED'}
SPHERE_SIZE, TEXT_SIZE = 1.5, 1.5   # m, as rc_sim's replay (readable on a 200 m straight)
LABEL_Y0, LABEL_DY, LABEL_ROWS = 3.0, 2.0, 6


class PathFrame:
    """Arc length and signed lateral offset along the course polyline, searched locally."""

    def __init__(self, xs, ys, window: int = 40):
        self.p = CoursePolyline(list(xs), list(ys), [0.0] * len(xs))
        self.window = window
        self.last = None
        self.s0 = 0.0

    def project(self, x: float, y: float):
        n = len(self.p.x) - 1
        if self.last is not None:
            lo, hi = max(0, self.last - self.window), min(n, self.last + self.window)
            best = self._search(x, y, lo, hi)
            if (best[1] == lo and lo > 0) or (best[1] == hi - 1 and hi < n):
                best = self._search(x, y, 0, n)   # moved beyond the window: search it all
        else:
            best = self._search(x, y, 0, n)
        _, i, u, fx, fy, dx, dy = best
        self.last = i
        norm = math.hypot(dx, dy) or 1.0
        lateral = (dx * (y - fy) - dy * (x - fx)) / norm
        # past the ends, extend along the end segments so s keeps growing (run-out)
        s = self.p.s[i] + u * (self.p.s[i + 1] - self.p.s[i])
        if (i == n - 1 and u >= 1.0) or (i == 0 and u <= 0.0):
            s += ((x - fx) * dx + (y - fy) * dy) / norm
        return s - self.s0, lateral

    def _search(self, x: float, y: float, lo: int, hi: int):
        best = None
        for i in range(lo, hi):
            x0, y0, x1, y1 = self.p.x[i], self.p.y[i], self.p.x[i + 1], self.p.y[i + 1]
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            u = 0.0 if seg2 == 0.0 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / seg2))
            fx, fy = x0 + u * dx, y0 + u * dy
            d = (x - fx) ** 2 + (y - fy) ** 2
            if best is None or d < best[0]:
                best = (d, i, u, fx, fy, dx, dy)
        return best

    def point_at(self, s: float):
        return self.p.point_at(s + self.s0)


def load_course(course_dir: Path):
    """(xs, ys, start_pose or None) from course.yaml + path.csv, or a straight along +x."""
    start, path_file = None, course_dir / 'path.csv'
    cy = course_dir / 'course.yaml'
    if cy.is_file():
        data = yaml.safe_load(cy.read_text()) or {}
        start = data.get('start_pose')
        path_file = course_dir / data.get('path', 'path.csv')
    xs, ys, _ = load_csv(str(path_file), 0.0)
    return xs, ys, start


class RowAligner:
    """Hold trace rows until the stamped inputs reach them, then fill target and emergency.

    Pure (no ROS): rows are dicts with 't' [s] and 'mode'; stamps are seconds.
    """

    def __init__(self, lag: float = ALIGN_LAG):
        self.lag = lag
        self.rows = deque()
        self.cmd_t, self.cmd_v = [], []         # control_cmd stamps (ascending) and speeds
        self.em_last = -math.inf                # the newest emergency message stamp
        self.events = []                        # (t, on): estop_on / estop_off, by time

    def on_cmd(self, stamp: float, velocity: float) -> None:
        if self.cmd_t and stamp < self.cmd_t[-1]:
            return                              # reordered: keep the stamps ascending
        self.cmd_t.append(stamp)
        self.cmd_v.append(velocity)

    def on_emergency(self, stamp: float) -> None:
        self.em_last = max(self.em_last, stamp)

    def on_estop(self, t: float, on: bool) -> None:
        bisect.insort(self.events, (t, on))

    def target_at(self, t: float) -> float:
        i = bisect.bisect_right(self.cmd_t, t + 1e-9) - 1
        return self.cmd_v[i] if i >= 0 else 0.0

    def estop_at(self, t: float) -> bool:
        i = bisect.bisect_right(self.events, (t + 1e-9, True)) - 1
        return self.events[i][1] if i >= 0 else False

    def add(self, row: dict) -> None:
        self.rows.append(row)

    def ready(self, state_t: float, flush: bool = False) -> list:
        """Rows whose inputs have reached them (all of them with flush), filled in."""
        out = []
        while self.rows:
            t = self.rows[0]['t']
            reached = (self.cmd_t and self.cmd_t[-1] > t + 1e-9 and self.em_last > t + 1e-9)
            if not (flush or reached or state_t >= t + self.lag - 1e-9):
                break
            row = self.rows.popleft()
            row['target'] = self.target_at(t)
            row['emergency'] = int(self.estop_at(t))
            out.append(row)
        # forget commands no pending row can need (keep the one at or before the oldest)
        keep_from = self.rows[0]['t'] if self.rows else (out[-1]['t'] if out else None)
        if keep_from is not None:
            i = bisect.bisect_right(self.cmd_t, keep_from + 1e-9) - 1
            if i > 0:
                del self.cmd_t[:i], self.cmd_v[:i]
        return out


class Part1Grader(Node):
    def __init__(self, **kwargs):
        super().__init__('lab2_part1_grader', **kwargs)
        for name, default in (
                ('profile_file', ''), ('course_dir', ''), ('out_dir', ''), ('variant', 'nominal'),
                ('run_timeout', 150.0), ('engage_timeout', 300.0), ('wheel_radius', 0.0),
                ('state_topic', '/rc_sim/state'), ('pwm_topic', '/rc_sim/pwm'),
                ('control_cmd_topic', '/control/command/control_cmd'),
                ('velocity_topic', '/vehicle/status/velocity_status'),
                ('mode_topic', '/autosdv/actuator_node/debug/mode'),
                ('mrm_topic', '/system/fail_safe/mrm_state'),
                ('event_topic', '/lab2/part1/event'),
                ('lockstep_topic', '/rc_sim/lockstep/status'),
                ('emergency_topic', '/control/command/emergency_cmd'),
                ('init_pwm', 370), ('min_pwm', 360), ('max_pwm', 470), ('brake_pwm', 340)):
            self.declare_parameter(name, default)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        if not gp('profile_file'):
            raise RuntimeError('lab2_part1_grader: profile_file is required (part1_profile.yaml)')
        self.profile = M.load_profile(gp('profile_file'))
        course_dir = Path(gp('course_dir') or Path(gp('profile_file')).expanduser().parent)
        xs, ys, start = load_course(course_dir.expanduser())
        self.frame = PathFrame(xs, ys)
        if start is not None:
            self.frame.s0 = 0.0
            self.frame.s0, _ = self.frame.project(float(start['x']), float(start['y']))
            self.frame.last = None
        self.s0_from_pose = start is None
        self.variant = gp('variant')
        self.out_dir = Path(gp('out_dir')).expanduser() if gp('out_dir') else None
        self.run_timeout = float(gp('run_timeout'))        # sim s from the engage
        self.engage_timeout = float(gp('engage_timeout'))  # sim s from the first sample
        self.wheel_radius = float(gp('wheel_radius')) or self._wheel_radius(self.variant)
        self.pwm_levels = {k: int(gp(k)) for k in ('init_pwm', 'min_pwm', 'max_pwm', 'brake_pwm')}

        self.trace = M.Trace()
        self.aligner = RowAligner()
        self.state_t = 0.0                  # the newest /rc_sim/state time
        self.next_g: Optional[int] = None   # next grid sample, in SAMPLE_DT units
        self.prev_state = None
        self.target = 0.0
        self.measured = 0.0
        self.pwm = self.pwm_levels['init_pwm']
        self.steer_pwm = 0
        self.mode = 'waiting'
        self.mode_seen = False              # the actuator has published its mode
        self.estop = False
        self.engaged = False
        self.t_engaged: Optional[float] = None
        self.message: Optional[str] = None
        self.min_dt: Optional[float] = None
        self.gaps = 0
        self.largest_gap = 0.0
        self.aborted: Optional[str] = None
        self.mrm: Optional[str] = None
        self.fault: Optional[str] = None
        self.lockstep: Optional[str] = None
        self.startup_stall: Optional[str] = None
        self.notes = []
        self.state = 'waiting'
        self.checks = M.evaluate(self.trace, self.profile, finished=False)
        self.final = None
        self.published_rows = 0
        self.last_status = None
        self._eval_len = -1

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        events_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Float64MultiArray, gp('state_topic'), self.on_state, reliable)
        self.create_subscription(Int32MultiArray, gp('pwm_topic'), self.on_pwm, reliable)
        self.create_subscription(Control, gp('control_cmd_topic'), self.on_cmd, 10)
        self.create_subscription(VelocityReport, gp('velocity_topic'), self.on_velocity, 10)
        self.create_subscription(String, gp('mode_topic'), self.on_mode, latched)
        self.create_subscription(MrmState, gp('mrm_topic'), self.on_mrm, 10)
        self.create_subscription(String, gp('event_topic'), self.on_event, events_qos)
        self.create_subscription(String, gp('lockstep_topic'), self.on_lockstep, 10)
        self.create_subscription(VehicleEmergencyStamped, gp('emergency_topic'),
                                 self.on_emergency, reliable)
        self.pub_status = self.create_publisher(String, '/lab2/part1/status', latched)
        history = QoSProfile(depth=TRACE_HISTORY, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_trace = self.create_publisher(Float64MultiArray, '/lab2/part1/trace', history)
        self.pub_violations = self.create_publisher(MarkerArray, '/lab2/part1/violations', latched)
        self.pub_markers = self.create_publisher(MarkerArray, '/lab2/part1/markers', latched)
        self.pub_path = self.create_publisher(PathMsg, '/lab2/part1/course_path', latched)
        self.pub_car = self.create_publisher(MarkerArray, '/lab2/part1/car', 10)
        self.publish_course()
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.tick, clock=steady)
        self.get_logger().info(
            f'lab2_part1_grader: variant {self.variant}, stop line at s = '
            f'{self.profile.stop_line_s:.2f} m, '
            f'e-stop at s = {self.profile.estop_trigger_s:.2f} m, '
            f"limits {'calibrated' if M.LIMITS_CALIBRATED else 'UNCALIBRATED (placeholders)'}, "
            f'out_dir {self.out_dir}')

    @staticmethod
    def _wheel_radius(variant: str) -> float:
        try:
            from autosdv_rc_sim.params import load_vehicle
            return float(load_vehicle(variant=variant).wheel_radius)
        except Exception:  # noqa: B902 -- the plant's nominal wheel when rc_sim is not importable
            return 0.0525

    # ------------------------------------------------------------ inputs
    def on_cmd(self, msg: Control) -> None:
        self.target = float(msg.longitudinal.velocity)
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.aligner.on_cmd(stamp if stamp > 0.0 else self.state_t, self.target)
        self._flush()

    def on_emergency(self, msg: VehicleEmergencyStamped) -> None:
        self.aligner.on_emergency(msg.stamp.sec + msg.stamp.nanosec * 1e-9)
        self._flush()

    def on_velocity(self, msg: VelocityReport) -> None:
        self.measured = float(msg.longitudinal_velocity)

    def on_pwm(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 2:
            if msg.data[0] == 0:
                self.pwm = int(msg.data[1])
            elif msg.data[0] == 1:
                self.steer_pwm = int(msg.data[1])

    def on_mode(self, msg: String) -> None:
        self.mode = msg.data
        self.mode_seen = True
        reason = msg.data.partition(':')[2]
        if self.final is None and reason == 'controller_fault':
            self.fault = ('the speed controller raised or returned an invalid value '
                          "(see the actuator's log)")
        elif (self.final is None and self.engaged
              and reason in ('command_timeout', 'velocity_timeout')):
            note = f'actuator override {reason} at t = {self._t():.2f} s'
            if note not in self.notes:
                self.notes.append(note)

    def on_mrm(self, msg: MrmState) -> None:
        if self.final is None and self.engaged and msg.state in MRM_NAMES:
            self.mrm = f'{MRM_NAMES[msg.state]} (behavior {msg.behavior})'

    def on_event(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except ValueError:
            self.get_logger().warning(f'unreadable event {msg.data!r}')
            return
        kind = ev.get('event')
        if kind == 'engaged':
            self.engaged = True
            if self.t_engaged is None:
                t = ev.get('t')
                self.t_engaged = float(t) if isinstance(t, (int, float)) else self._t()
        elif kind in ('estop_on', 'estop_off'):
            self.estop = kind == 'estop_on'
            t = ev.get('t')
            self.aligner.on_estop(float(t) if isinstance(t, (int, float)) else self.state_t,
                                  self.estop)
        elif kind == 'aborted' and self.final is None:
            self.aborted = str(ev.get('reason') or 'no reason given')

    def on_lockstep(self, msg: String) -> None:
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if self.final is None and st.get('ok') is False:
            reason = str(st.get('reason') or 'a participant timed out')
            if self.engaged:
                self.lockstep = reason
            elif self.startup_stall is None:
                self.startup_stall = reason   # before the engage: not the student's run

    def on_state(self, msg: Float64MultiArray) -> None:
        d = msg.data
        if self.final is not None or len(d) <= ST_KAPPA_R:
            return
        t = float(d[ST_T])
        if self.prev_state is not None:
            dt = t - float(self.prev_state[ST_T])
            if dt > 0.0:
                if self.min_dt is not None and dt > 2.0 * self.min_dt + 1e-6:
                    self.gaps += 1
                    self.largest_gap = max(self.largest_gap, dt)
                self.min_dt = dt if self.min_dt is None else min(self.min_dt, dt)
        if self.s0_from_pose and self.prev_state is None and not self.trace['t']:
            self.frame.s0 = 0.0
            self.frame.s0, _ = self.frame.project(float(d[ST_X]), float(d[ST_Y]))
            self.frame.last = None
        if self.next_g is None:
            self.next_g = int(math.ceil(t / SAMPLE_DT - 1e-6))
        while self.next_g * SAMPLE_DT <= t + 1e-6:
            g = self.next_g * SAMPLE_DT
            src = d if (self.prev_state is None or t - g < 0.5 * SAMPLE_DT) else self.prev_state
            self.aligner.add(self._row(round(g, 3), src))
            self.next_g += 1
        self.prev_state = list(d)
        self.state_t = t
        self._flush()

    def _flush(self, all_rows: bool = False) -> None:
        if self.final is not None:
            return
        for row in self.aligner.ready(self.state_t, flush=all_rows):
            self.trace.append(row)

    def _row(self, t: float, d) -> dict:
        s, y = self.frame.project(float(d[ST_X]), float(d[ST_Y]))
        vx = float(d[ST_VX])
        emergency = self.estop or self.mode.endswith(':emergency')
        # target and emergency are filled in by the RowAligner when the row is released
        return {'t': t, 's': s, 'y': y, 'target': self.target, 'measured': self.measured,
                'speed': vx, 'pwm': self.pwm, 'mode': self.mode,
                'slip_r': float(d[ST_OMEGA_R]) * self.wheel_radius - vx,
                'kappa_r': float(d[ST_KAPPA_R]), 'emergency': int(emergency),
                'segment': self.profile.segment_at(s)}

    def _t(self) -> float:
        return self.trace['t'][-1] if len(self.trace) else 0.0

    # ------------------------------------------------------------ evaluation
    def tick(self) -> None:
        if self.final is None:
            self.step()
        self.publish_trace()
        self.publish_status()
        self.publish_car()

    def step(self) -> None:
        n = len(self.trace)
        if n and (self.engaged or max(self.trace['speed'][-50:]) > M.LAUNCH_MOVE):
            self.state = 'running'
        not_engaged = None
        waited = self.trace['t'][-1] - self.trace['t'][0] if n else 0.0
        if not self.engaged and waited > self.engage_timeout:
            not_engaged = f'no engage within {self.engage_timeout:g} s of sim time'
        reason = M.invalid_reason(aborted=self.aborted, mrm=self.mrm, fault=self.fault,
                                  lockstep=self.lockstep, startup_stall=self.startup_stall,
                                  estop_missing=M.estop_missing(self.trace, self.profile),
                                  not_engaged=not_engaged,
                                  actuator_missing=M.actuator_missing(
                                      self.mode_seen, self._t(), self.t_engaged))
        if reason:
            self.finalize(reason)
            return
        if self.engaged and n:
            if M.is_finished(self.trace, self.profile):
                self.finalize(None)
                return
            msg = M.stall_message(self.trace, self.t_engaged)
            if msg is None and self.trace['t'][-1] - self.t_engaged > self.run_timeout:
                msg = (f'Your run did not finish within {self.run_timeout:g} s of sim time '
                       'after the engage. The checks it did not reach fail.')
            if msg is not None:
                self.finalize(None, msg)
                return
        if n != self._eval_len and (n - self._eval_len >= 50 or n < self._eval_len):
            before = {k for k, c in self.checks.items() if c['pass'] is not None}
            self.checks = M.evaluate(self.trace, self.profile, finished=False)
            self._eval_len = n
            for name, c in self.checks.items():
                if c['pass'] is not None and name not in before:
                    self.get_logger().info(('PASS ' if c['pass'] else 'FAIL ')
                                           + M.describe(name, c))
            self.publish_violations()

    def finalize(self, reason: Optional[str], message: Optional[str] = None) -> None:
        """End the run: finished (both None), ended early but valid (message), or invalid."""
        self._flush(all_rows=True)
        valid = reason is None
        complete = valid and message is None
        self.message = message or (None if valid else f'Invalid run: {reason}.')
        self.checks = M.evaluate(self.trace, self.profile, finished=complete,
                                 unclosed='fail' if valid else 'pending')
        if self.gaps:
            self.notes.append(f'{self.gaps} gap(s) in /rc_sim/state longer than two periods '
                              f'(largest {self.largest_gap:.3f} s); the trace holds the last '
                              'sample across a gap')
        passed, total = M.summary(self.checks, valid)
        self.state = 'finished' if valid else 'invalid'
        self.final = {
            'variant': self.variant, 'valid': valid, 'invalid_reason': reason,
            'message': self.message,
            'passed': passed, 'total': total,
            'checks': self.checks, 'violations': M.violations(self.checks),
            'limits_calibrated': M.LIMITS_CALIBRATED,
            'stop_line_s': self.profile.stop_line_s,
            'estop_trigger_s': self.profile.estop_trigger_s,
            't_end': self._t(), 'rows': len(self.trace), 'notes': self.notes,
            'trace': 'trace.csv',
        }
        if self.out_dir is not None:
            try:
                self.out_dir.mkdir(parents=True, exist_ok=True)
                self.trace.write_csv(self.out_dir / 'trace.csv')
                tmp = self.out_dir / 'marks.json.part'
                tmp.write_text(json.dumps(self.final, indent=2) + '\n')
                os.replace(tmp, self.out_dir / 'marks.json')
            except OSError as e:
                self.get_logger().error(f'cannot write the results to {self.out_dir}: {e}')
        else:
            self.get_logger().warning('out_dir is empty: trace.csv and marks.json not written')
        self.publish_violations()
        verdict = f'{passed}/{total}' if valid else f'INVALID ({reason}): 0/{total}'
        lines = [f'lab2 part1 [{self.variant}] {verdict}']
        if self.message:
            lines.append('  ' + self.message)
        word = {True: 'PASS ', False: 'FAIL ', None: '---- '}
        lines += ['  ' + word[c['pass']] + M.describe(n, c) for n, c in self.checks.items()]
        self.get_logger().info('\n'.join(lines))

    # ------------------------------------------------------------ outputs
    def status(self) -> dict:
        n = len(self.trace)
        out = {
            'state': self.state, 't': self._t(),
            'segment': self.trace['segment'][-1] if n else 'start',
            'checks': {k: {'value': c['value'], 'limit': c['limit'], 'pass': c['pass'],
                           'unit': c['unit']} for k, c in self.checks.items()},
            'violations': M.violations(self.checks),
            'marks': None, 'invalid_reason': None, 'message': self.message,
            'variant': self.variant, 'stop_line_s': self.profile.stop_line_s,
            'estop_trigger_s': self.profile.estop_trigger_s, 'pwm_levels': self.pwm_levels,
            'limits_calibrated': M.LIMITS_CALIBRATED, 'trace_file': None,
        }
        if self.final is not None:
            out['marks'] = {'passed': self.final['passed'], 'total': self.final['total']}
            out['invalid_reason'] = self.final['invalid_reason']
            if self.out_dir is not None:
                out['trace_file'] = str((self.out_dir / 'trace.csv').resolve())
        return out

    def publish_status(self) -> None:
        text = json.dumps(self.status())
        self.pub_status.publish(String(data=text))
        self.last_status = text

    def publish_trace(self) -> None:
        n = len(self.trace)
        if n <= self.published_rows:
            return
        msg = Float64MultiArray()
        rows = n - self.published_rows
        width = len(TRACE_FIELDS)
        msg.layout.dim = [MultiArrayDimension(label='rows', size=rows, stride=rows * width),
                          MultiArrayDimension(label=','.join(TRACE_FIELDS), size=width,
                                              stride=width)]
        data = []
        for i in range(self.published_rows, n):
            data.extend(float(self.trace[c][i]) for c in TRACE_FIELDS)
        msg.data = data
        self.pub_trace.publish(msg)
        self.published_rows = n

    def publish_car(self) -> None:
        if not len(self.trace):
            return
        try:
            from autosdv_rc_sim.replay_node import car_markers
        except ImportError:
            return
        row = {'target': self.target, 'speed': self.trace['speed'][-1], 'pwm': self.pwm,
               'steer_pwm': self.steer_pwm, 'slip_r': self.trace['slip_r'][-1]}
        array = MarkerArray()
        array.markers.extend(car_markers(row))
        for m in array.markers:
            m.ns = 'lab2_part1_car'
        self.pub_car.publish(array)

    def _marker(self, ns: str, i: int, kind: int, s: float, lateral: float, z: float,
                size, rgb, text: str = '') -> Marker:
        x, y, yaw = self.frame.point_at(s)
        mk = Marker()
        mk.header.frame_id = 'map'
        mk.ns, mk.id, mk.type = ns, i, kind
        mk.pose.position.x = x - math.sin(yaw) * lateral
        mk.pose.position.y = y + math.cos(yaw) * lateral
        mk.pose.position.z = z
        mk.pose.orientation.z, mk.pose.orientation.w = math.sin(0.5 * yaw), math.cos(0.5 * yaw)
        mk.scale.x, mk.scale.y, mk.scale.z = size
        mk.color.r, mk.color.g, mk.color.b = rgb
        mk.color.a = 1.0
        mk.text = text
        return mk

    def publish_violations(self) -> None:
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for i, v in enumerate(M.violations(self.checks)):
            s = v['s'] if v['s'] is not None else self._fallback_s(v['name'])
            row = LABEL_Y0 + LABEL_DY * (i % LABEL_ROWS)
            array.markers.append(self._marker('lab2_part1_violations', 2 * i, Marker.SPHERE, s,
                                              0.0, 0.0, (SPHERE_SIZE,) * 3, (1.0, 0.0, 0.0)))
            array.markers.append(self._marker('lab2_part1_violations', 2 * i + 1,
                                              Marker.TEXT_VIEW_FACING, s, row,
                                              SPHERE_SIZE + TEXT_SIZE, (0.0, 0.0, TEXT_SIZE),
                                              (1.0, 0.3, 0.3), v['text']))
        self.pub_violations.publish(array)

    def _fallback_s(self, name: str) -> float:
        if name.startswith('estop_'):
            return self.profile.estop_trigger_s
        if name.startswith('stop'):
            return self.profile.stop_line_s
        return self.trace['s'][-1] if len(self.trace) else 0.0

    def publish_course(self) -> None:
        p = self.profile
        array = MarkerArray()
        for i, (s, rgb, text) in enumerate(((p.stop_line_s, (1.0, 1.0, 1.0), 'stop line'),
                                            (p.estop_trigger_s, (1.0, 0.8, 0.0), 'e-stop'))):
            array.markers.append(self._marker('lab2_part1_course', 2 * i, Marker.CUBE, s, 0.0,
                                              0.02, (0.15, 1.2, 0.04), rgb))
            array.markers.append(self._marker('lab2_part1_course', 2 * i + 1,
                                              Marker.TEXT_VIEW_FACING, s, -1.5, 1.0,
                                              (0.0, 0.0, 1.0), rgb, text))
        self.pub_markers.publish(array)
        path = PathMsg()
        path.header.frame_id = 'map'
        for x, y in zip(self.frame.p.x, self.frame.p.y):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Part1Grader()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
