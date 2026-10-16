#!/usr/bin/env python3
"""`just lab2 run`: one graded run from one terminal.

    just lab2 run                          the car: settings from ~/lab2/lab2.yaml
    just lab2 run config:=FILE car:=B      another file, a command-line override
    just lab2 run --bench                  the same flow on the vehicle bench (no car)
    just lab2 run --bench --camera mock    ... with rendered boards into the real localizer
    just lab2 run --bench --yes            engage without asking (tests; --bench only)

In order, with short status lines on the terminal and everything else in log
files under BAG_DIR/run_logs/<bag name>/ (stack.log, record.log, preflight.log,
run.log, play_log/):

  1. start the stack (`just lab2 vehicle`, or `just lab2 bench`) in its own
     session; route and team_dir/submission from the settings
  2. wait until it is up, then run the preflight; any FAIL stops here
  3. start recording (`lab2_record`: BAG_DIR/lab2_uturn_<next>, car and team
     from the settings)
  4. ask before engaging (the TA must hold the transmitter); refuse to engage
     while the localization is not usable
  5. watch the run, one status line a second, until the car has stopped at the
     end of the route, or an MRM, a disengage, a LOCALIZATION LOSS (below), the
     timeout (3 x the route's driving time) or Ctrl-C ends it; anything but a
     finish disengages at once
  6. record 3 s more, disengage if still engaged, stop the recording, stop
     the stack
  7. audit the bag (`lab2_audit`: verdict, metrics, violations, audit.png)

Localization loss stops the car (disengage, so Autoware's stop mode brakes it)
as soon as any of these holds while engaged, without waiting for the MRM the
stack would raise itself:

  - localization_monitor reports ERROR: no accepted marker fix for more than
    its error_age (5 s), or the car more than its error_distance (0.8 m) from
    the path, or no course progress
  - localization_monitor silent for more than MONITOR_SILENT s
  - no /localization/kinematic_state for more than POSE_SILENT s
  - the EKF's position standard deviation sqrt(var_x + var_y) above
    --max-pose-std (default MAX_POSE_STD m; the bench runs at 0.06-0.10 m)

Ctrl-C (or a closed terminal) at any point: disengage if engaged, stop the
recording, stop the stack. Every process this starts is in a session of its
own, and is stopped by process group: SIGINT, then SIGTERM, then SIGKILL.

Exit status: 0 for a finished run whose audit is VALID, 1 otherwise, 2 for a
usage or settings error. With --integration (`just lab2 integration-test`):
0 only if the run finished, check-bags (bench mode) accepted the bag, the grade
is valid, audit.png exists and the localization against the bench's true pose
is within audit.LOC_LIMITS.
"""

import json
import math
import os
import select
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import lab2_config as lc  # noqa: E402

REPO = lc.REPO
COURSES_DIR = REPO / 'src/lab/autosdv_waypoint_lab/config/courses'

PROMPT = ('Ready. Engage only when the TA holds the transmitter. '
          'Press Enter to engage, or q then Enter to abort.')
MAX_POSE_STD = 0.5          # m, sqrt(var_x + var_y) of the EKF pose
POSE_SILENT = 0.5           # s without kinematic_state
MONITOR_SILENT = 2.0        # s without a localization_monitor status
AT_REST = 0.02              # m/s
REST_HOLD = 1.0             # s at rest after the finish
TIMEOUT_FACTOR = 3.0        # x the route's driving time
MIN_TIMEOUT = 60.0          # s
SETTLE_AFTER_END = 3.0      # s recorded after the end; with REST_HOLD past route_grade.SETTLE[1] (3 s)
READY_STEADY = 3.0          # s of continuous availability before the preflight
MONITOR_STATUSES = ('localization_monitor: marker_fix', 'localization_monitor: path_deviation')
ERROR_LEVEL = 2

INTERRUPTED: List[str] = []    # set by the signal handler, read by Run.check_interrupt

OWN_FLAGS = {'--bench', '--yes', '--integration'}
OWN_VALUE_FLAGS = {'--camera', '--timeout', '--ready-timeout', '--max-pose-std'}


# ------------------------------------------------------------ arguments (pure)

@dataclass
class Options:
    bench: bool = False
    yes: bool = False
    integration: bool = False
    camera: Optional[str] = None
    timeout: Optional[float] = None
    ready_timeout: Optional[float] = None
    max_pose_std: float = MAX_POSE_STD
    config_args: Tuple[str, ...] = ()
    launch_args: Tuple[str, ...] = ()


class UsageError(Exception):
    pass


def parse_args(argv: List[str]) -> Options:
    """Split our flags, the settings (config:=, KEY:=value) and the launch arguments.
    `team:=` here is the team NUMBER (as in record); the directory is team_dir:=."""
    o = Options()
    cfg, launch = [], []
    it = iter(argv)
    for a in it:
        flag, eq, val = a.partition('=')
        if a in OWN_FLAGS:
            setattr(o, a[2:], True)
        elif a in OWN_VALUE_FLAGS or (eq and flag in OWN_VALUE_FLAGS and ':' not in flag):
            if not eq:
                try:
                    val = next(it)
                except StopIteration:
                    raise UsageError(f'{a} needs a value') from None
            key = (flag if eq else a)[2:].replace('-', '_')
            if key == 'camera':
                if val not in ('virtual', 'mock'):
                    raise UsageError('--camera is virtual or mock')
                o.camera = val
            else:
                try:
                    setattr(o, key, float(val))
                except ValueError:
                    raise UsageError(f'{a}: not a number') from None
        elif a == '--config':
            try:
                cfg += ['--config', next(it)]
            except StopIteration:
                raise UsageError('--config needs a value') from None
        elif a.split(':=', 1)[0] in lc.KEYS + ('config',) and ':=' in a:
            cfg.append(a)
        elif a in ('-h', '--help'):
            raise UsageError('help')
        else:
            launch.append(a)
    if o.yes and not o.bench:
        raise UsageError('--yes engages without asking, which is for the bench only: '
                         'on the car the TA must hold the transmitter first')
    if o.camera and not o.bench:
        raise UsageError('--camera picks the bench\'s camera model; the car has a camera')
    o.config_args, o.launch_args = tuple(cfg), tuple(launch)
    return o


def vehicle_config_args(cfg) -> List[str]:
    """The settings for the stack: team:= (the number) is the recorder's, not the stack's."""
    return [a for a in cfg if not a.startswith('team:=')]


def route_dir(route: str) -> Path:
    d = Path(route)
    return d if d.is_dir() else COURSES_DIR / route


def route_seconds(course_dir) -> float:
    """The route's driving time at v_ref, s."""
    import yaml
    d = yaml.safe_load((Path(course_dir) / 'course.yaml').read_text()) or {}
    v = float(d.get('v_ref', 0.8))
    xs, ys = [], []
    for line in (Path(course_dir) / str(d.get('path', 'path.csv'))).read_text().splitlines():
        line = line.split('#')[0].strip()
        if line:
            a = line.split(',')
            xs.append(float(a[0]))
            ys.append(float(a[1]))
    length = sum(math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]) for i in range(len(xs) - 1))
    laps = max(1, int(d.get('laps', 1)))
    return laps * length / max(v, 0.05)


def run_timeout(course_dir, given: Optional[float] = None) -> float:
    if given:
        return given
    return max(MIN_TIMEOUT, TIMEOUT_FACTOR * route_seconds(course_dir))


# ------------------------------------------------------------ the run's state (pure)

@dataclass
class Snapshot:
    """What the monitor knows at one instant; times in seconds."""
    now: float
    engaged: bool                    # mode is AUTONOMOUS
    mrm: Optional[str] = None        # None while NORMAL, else a description
    finished: bool = False           # the path publisher's progress says so
    speed: float = 0.0
    rest_since: Optional[float] = None
    pose_rx: Optional[float] = None  # last kinematic_state
    pose_std: float = 0.0
    monitor: Optional[Dict[str, Tuple[int, str, float]]] = None   # name -> (level, message, rx)
    stack_alive: bool = True
    recorder_alive: bool = True


@dataclass
class End:
    kind: str        # finished | mrm | disengaged | localization | timeout | stack | recorder
    text: str

    @property
    def abort(self) -> bool:
        return self.kind != 'finished'


def localization_problem(s: Snapshot, max_pose_std: float = MAX_POSE_STD) -> Optional[str]:
    """Why the pose is not usable, or None."""
    if s.pose_rx is None:
        return 'no localized pose (/localization/kinematic_state) yet'
    if s.now - s.pose_rx > POSE_SILENT:
        return f'no localized pose for {s.now - s.pose_rx:.1f} s'
    mon = s.monitor or {}
    for name in MONITOR_STATUSES:
        if name not in mon:
            return f'no {name} status yet'
        level, msg, rx = mon[name]
        if s.now - rx > MONITOR_SILENT:
            return f'localization_monitor silent for {s.now - rx:.1f} s'
        if level >= ERROR_LEVEL:
            return f'{name}: {msg}'
    if s.pose_std > max_pose_std:
        return f'EKF position uncertainty {s.pose_std:.2f} m (limit {max_pose_std:.2f} m)'
    return None


def end_reason(s: Snapshot, t_engage: float, timeout: float,
               max_pose_std: float = MAX_POSE_STD) -> Optional[End]:
    """Has the run ended, and how? Safety first: a dead stack, an MRM and a lost pose
    win over a finish seen in the same instant."""
    if not s.stack_alive:
        return End('stack', 'the stack exited')
    if s.mrm:
        return End('mrm', f'MRM: {s.mrm}')
    if s.finished and s.rest_since is not None and s.now - s.rest_since >= REST_HOLD:
        return End('finished', 'stopped at the end of the route')
    if not s.engaged:
        if s.finished:
            return End('finished', 'left autonomous mode after the finish')
        return End('disengaged', 'left autonomous mode before the finish '
                                 '(transmitter, disengage, or the stack)')
    loc = localization_problem(s, max_pose_std)
    if loc:
        return End('localization', f'localization lost: {loc}')
    if not s.recorder_alive:
        return End('recorder', 'the recording stopped')
    if s.now - t_engage > timeout:
        return End('timeout', f'not finished after {timeout:.0f} s')
    return None


def engage_refusal(s: Snapshot, available: Optional[bool], max_pose_std: float = MAX_POSE_STD) -> Optional[str]:
    """Why engaging now would be wrong, or None."""
    if not s.stack_alive:
        return 'the stack exited'
    if not s.recorder_alive:
        return 'the recording stopped'
    if s.mrm:
        return f'MRM active ({s.mrm})'
    loc = localization_problem(s, max_pose_std)
    if loc:
        return f'localization not usable: {loc}'
    if available is False:
        return 'autonomous mode not available (see `just lab2 preflight`)'
    return None


def status_line(s: Snapshot, t_engage: float, progress: Optional[float], length: Optional[float],
                mode: str) -> str:
    along = (f'{progress:5.1f}/{length:.1f} m' if progress is not None and length
             else '   -  m')
    return (f'  t {s.now - t_engage:5.1f} s  v {s.speed:4.2f} m/s  s {along}  {mode:<10}  '
            f'MRM {"NORMAL" if not s.mrm else s.mrm.split(",")[0]}')


# ------------------------------------------------------------ processes (pure-ish)

def session_pgids(sid: int) -> List[int]:
    """The process groups with a live process in session `sid` (from /proc)."""
    out = set()
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            stat = (p / 'stat').read_text()
        except OSError:
            continue
        fields = stat[stat.rindex(')') + 2:].split()
        state, pgid, session = fields[0], int(fields[2]), int(fields[3])
        if session == sid and state != 'Z':
            out.add(pgid)
    return sorted(out)


class Child:
    """A command in a session of its own, its output in a log file."""

    def __init__(self, name: str, cmd: List[str], log: Path, env=None, cwd=REPO):
        self.name, self.cmd, self.log, self.env, self.cwd = name, list(cmd), Path(log), env, cwd
        self.proc: Optional[subprocess.Popen] = None

    def start(self):
        self.log.parent.mkdir(parents=True, exist_ok=True)
        f = open(self.log, 'w')
        f.write(' '.join(self.cmd) + '\n')
        f.flush()
        self.proc = subprocess.Popen(self.cmd, cwd=self.cwd, env=self.env, stdout=f,
                                     stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                     start_new_session=True)
        f.close()
        return self

    @property
    def pid(self) -> Optional[int]:
        return self.proc.pid if self.proc else None

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def remaining(self) -> List[int]:
        """Process groups of this child's session still alive (play_launch gives its
        nodes groups of their own, in the same session)."""
        if self.proc is None:
            return []
        self.proc.poll()
        return session_pgids(self.proc.pid)

    def stop(self, steps=((signal.SIGINT, 20.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)),
             say=print) -> bool:
        """Signal every process group of the session, escalating; True once none is left."""
        if self.proc is None:
            return True
        prev_wait = 0.0
        for sig, wait in steps:
            groups = self.remaining()
            if not groups:
                return True
            if sig != steps[0][0]:
                say(f'  {self.name}: still running after {prev_wait:.0f} s, sending {sig.name}')
            # the leader's group first: play_launch shuts its nodes down on SIGINT itself
            for g in ([self.proc.pid] if sig == steps[0][0] and self.proc.pid in groups else groups):
                try:
                    os.killpg(g, sig)
                except (ProcessLookupError, PermissionError):
                    pass
            end = time.monotonic() + wait
            while time.monotonic() < end and self.remaining():
                time.sleep(0.2)
            prev_wait = wait
        return not self.remaining()


# ------------------------------------------------------------ the ROS side

def make_monitor():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from autoware_adapi_v1_msgs.msg import MrmState, OperationModeState
    from autoware_adapi_v1_msgs.srv import ChangeOperationMode
    from autoware_vehicle_msgs.msg import VelocityReport
    from autosdv_lab_msgs.msg import WaypointProgress
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tier4_system_msgs.msg import OperationModeAvailability

    # our own SIGINT handling: Ctrl-C must still be able to disengage through rclpy
    # args=[]: not sys.argv, whose key:=value words rcl would read as remap rules
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)

    class Monitor(Node):
        def __init__(self):
            super().__init__('lab2_run')
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.AUTONOMOUS = OperationModeState.AUTONOMOUS
            self.MODES = {getattr(OperationModeState, k): k for k in
                          ('UNKNOWN', 'STOP', 'AUTONOMOUS', 'LOCAL', 'REMOTE')}
            self.MRM_NORMAL = MrmState.NORMAL
            self.MRM_STATES = {MrmState.MRM_OPERATING: 'operating', MrmState.MRM_SUCCEEDED: 'succeeded',
                               MrmState.MRM_FAILED: 'failed', MrmState.UNKNOWN: 'unknown'}
            self.MRM_BEHAVIORS = {MrmState.EMERGENCY_STOP: 'EMERGENCY_STOP',
                                  MrmState.COMFORTABLE_STOP: 'COMFORTABLE_STOP',
                                  MrmState.PULL_OVER: 'PULL_OVER'}
            self.mode = None
            self.mrm = None
            self.available = None
            self.speed = 0.0
            self.rest_since = None
            self.progress = None
            self.pose_rx = None
            self.pose_std = 0.0
            self.monitor = {}
            self.create_subscription(OperationModeState, '/api/operation_mode/state', self.on_mode, latched)
            self.create_subscription(MrmState, '/system/fail_safe/mrm_state', self.on_mrm, 10)
            self.create_subscription(OperationModeAvailability, '/system/operation_mode/availability',
                                     self.on_avail, 10)
            self.create_subscription(VelocityReport, '/vehicle/status/velocity_status', self.on_vel, 10)
            self.create_subscription(WaypointProgress, '/planning/waypoint_path_publisher/progress',
                                     self.on_progress, 10)
            self.create_subscription(Odometry, '/localization/kinematic_state', self.on_pose, 10)
            self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 50)
            self.cli_auto = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
            self.cli_stop = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_stop')
            self.Req = ChangeOperationMode.Request

        def on_mode(self, m):
            self.mode = int(m.mode)

        def on_mrm(self, m):
            if m.state == self.MRM_NORMAL:
                self.mrm = None
            elif self.mrm is None:
                self.mrm = (f'{self.MRM_STATES.get(m.state, m.state)} '
                            f'{self.MRM_BEHAVIORS.get(m.behavior, "behavior " + str(m.behavior))}')

        def on_avail(self, m):
            self.available = bool(m.autonomous)

        def on_vel(self, m):
            self.speed = float(m.longitudinal_velocity)
            if abs(self.speed) < AT_REST:
                self.rest_since = self.rest_since or time.monotonic()
            else:
                self.rest_since = None

        def on_progress(self, m):
            self.progress = m

        def on_pose(self, m):
            self.pose_rx = time.monotonic()
            c = m.pose.covariance
            self.pose_std = math.sqrt(max(0.0, c[0]) + max(0.0, c[7]))

        def on_diag(self, m):
            now = time.monotonic()
            for st in m.status:
                if st.name in MONITOR_STATUSES:
                    lvl = st.level[0] if isinstance(st.level, bytes) else int(st.level)
                    self.monitor[st.name] = (lvl, st.message, now)

        def snapshot(self, stack_alive=True, recorder_alive=True) -> Snapshot:
            p = self.progress
            return Snapshot(time.monotonic(), self.mode == self.AUTONOMOUS, self.mrm,
                            bool(p and p.finished), self.speed, self.rest_since, self.pose_rx,
                            self.pose_std, dict(self.monitor), stack_alive, recorder_alive)

        def mode_name(self):
            return self.MODES.get(self.mode, '?') if self.mode is not None else '-'

        def call(self, client, timeout=5.0):
            if not client.wait_for_service(timeout_sec=timeout):
                return False, 'service not available'
            fut = client.call_async(self.Req())
            end = time.monotonic() + timeout
            while not fut.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not fut.done():
                return False, 'no answer'
            st = fut.result().status
            return bool(st.success), st.message

    return rclpy, Monitor()


# ------------------------------------------------------------ the run

class Run:
    def __init__(self, opts: Options):
        self.o = opts
        self.stack: Optional[Child] = None
        self.recorder: Optional[Child] = None
        self.rclpy = None
        self.mon = None
        self.engaged_by_us = False
        self.end: Optional[End] = None
        self.logf = None
        self.tty = sys.stdout.isatty()
        self.bag: Optional[Path] = None

    # ---- output
    def say(self, msg=''):
        try:
            if self.tty:
                sys.stdout.write('\r\033[K')
            print(msg, flush=True)
        except OSError:
            pass   # the terminal went away; the log still has it
        if self.logf:
            self.logf.write(msg + '\n')
            self.logf.flush()

    def status(self, msg, every=5.0):
        """The live line: rewritten in place on a terminal; elsewhere (a log, a pipe) at
        most one line per `every` s."""
        try:
            if self.tty:
                sys.stdout.write('\r\033[K' + msg)
                sys.stdout.flush()
            elif msg and time.monotonic() - getattr(self, '_last_status', 0.0) >= every:
                self._last_status = time.monotonic()
                print(msg, flush=True)
        except OSError:
            pass

    def check_interrupt(self):
        """Raise KeyboardInterrupt here, in our own code, if a signal arrived. The signal
        handler only sets a flag: an exception raised inside rclpy's executor (where a
        Ctrl-C usually lands) comes out as a RuntimeError and breaks the executor, and
        with it the disengage call that has to follow."""
        if INTERRUPTED:
            raise KeyboardInterrupt(INTERRUPTED[0])

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.check_interrupt()
            self.rclpy.spin_once(self.mon, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))
        self.check_interrupt()

    def snap(self) -> Snapshot:
        return self.mon.snapshot(self.stack.alive(), self.recorder.alive() if self.recorder else True)

    # ---- steps
    def plan(self):
        o = self.o
        stack_args, summary = lc.plan('vehicle', vehicle_config_args(o.config_args) + list(o.launch_args))
        record_args, _ = lc.plan('record', list(o.config_args))
        audit_args, _ = lc.plan('audit', list(o.config_args) + ['BAG'])
        out_dir = Path(record_args[record_args.index('--out-dir') + 1])
        name = record_args[record_args.index('--name') + 1]
        self.bag = out_dir / f'lab2_{name}'
        # plan('vehicle') always names the course: route:=, course:= or course_dir:=
        course = next((a.split(':=', 1)[1] for a in stack_args if a.startswith('course_dir:=')), None) \
            or next((a.split(':=', 1)[1] for a in stack_args if a.startswith(('route:=', 'course:='))), None)
        self.course_dir = route_dir(course)
        self.timeout = run_timeout(self.course_dir, o.timeout)
        self.logdir = out_dir / 'run_logs' / self.bag.name
        if self.bag.exists():
            raise lc.ConfigError(f'{self.bag} exists already')
        cfg_file = [a for a in o.config_args if a.startswith('config:=')]
        if '--config' in o.config_args:
            cfg_file = ['--config', o.config_args[o.config_args.index('--config') + 1]]
        if o.bench:
            camera = o.camera or ('mock' if o.integration else 'virtual')
            stack = ['just', 'lab2', 'bench', '--log-dir', str(self.logdir / 'play_log')] + stack_args
            if not any(a.startswith('camera:=') for a in stack_args):
                stack.append(f'camera:={camera}')
        else:
            stack = ['just', 'lab2', 'vehicle', '--log-dir', str(self.logdir / 'play_log')] \
                + cfg_file + stack_args
        self.stack_cmd = stack
        self.record_cmd = ['ros2', 'run', 'autosdv_waypoint_lab', 'lab2_record'] + record_args \
            + (['--bench'] if o.bench else [])
        self.audit_cmd = ['ros2', 'run', 'autosdv_waypoint_lab', 'lab2_audit'] \
            + [str(self.bag) if a == 'BAG' else a for a in audit_args]
        return summary

    def start_stack(self, env):
        self.logdir.mkdir(parents=True, exist_ok=True)
        self.logf = open(self.logdir / 'run.log', 'a')
        self.say(f'[1/7] stack: just lab2 {self.stack_cmd[2]}'
                 + (f' {self.stack_cmd[-1]}' if self.o.bench else '')
                 + f', log {self.logdir / "stack.log"}')
        self.stack = Child('stack', self.stack_cmd, self.logdir / 'stack.log', env).start()

    def wait_ready(self) -> bool:
        limit = self.o.ready_timeout or (150.0 if self.o.bench else 180.0)
        self.say(f'[2/7] waiting for the stack (autonomous available {READY_STEADY:.0f} s, up to {limit:.0f} s)')
        t0 = time.monotonic()
        since = None
        while time.monotonic() - t0 < limit:
            self.spin(0.2)
            if not self.stack.alive():
                self.say(f'      the stack exited; see {self.logdir / "stack.log"}')
                return False
            if self.mon.available and self.own_node_alive('waypoint_path_publisher'):
                since = since or time.monotonic()
                if time.monotonic() - since >= READY_STEADY:
                    self.status('')
                    self.say(f'      up after {time.monotonic() - t0:.0f} s')
                    return True
            else:
                since = None
            s = self.snap()
            why = localization_problem(s, self.o.max_pose_std) or 'autonomous not available yet'
            self.status(f'      {time.monotonic() - t0:4.0f} s: {why}')
        self.status('')
        self.say(f'      not ready after {limit:.0f} s; the preflight shows why')
        return True   # the preflight explains it and fails

    def own_node_alive(self, node) -> bool:
        """Is `node` running in OUR stack's session? A stack left over on the same
        ROS_DOMAIN_ID would otherwise look ready."""
        for f in (self.logdir / 'play_log').glob(f'*/node/{node}/pid'):
            try:
                pid = int(f.read_text().strip())
                if os.getsid(pid) == self.stack.pid:
                    return True
            except (ValueError, OSError):
                continue
        return False

    def preflight(self, env) -> bool:
        log = self.logdir / 'preflight.log'
        cmd = [sys.executable, str(HERE / 'waypoint_preflight.py')]
        with open(log, 'w') as f:
            proc = subprocess.Popen(cmd, cwd=REPO, env=env, stdout=f, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)
            try:
                end = time.monotonic() + 120.0
                while proc.poll() is None and time.monotonic() < end:
                    self.spin(0.1)       # Ctrl-C stays live during the preflight
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
        text = log.read_text(errors='replace')
        lines = text.splitlines()
        r = proc
        bad = [ln for ln in lines if ln.startswith(('FAIL', 'WARN')) or ln.startswith('        ')]
        summary = next((ln for ln in lines if ln.startswith('---')), '--- no summary')
        for ln in bad:
            self.say('      ' + ln)
        self.say(f'      preflight {summary[4:]} (all lines: {log})')
        return r.returncode == 0

    def start_recording(self, env) -> bool:
        self.say(f'[3/7] recording {self.bag}')
        self.recorder = Child('recording', self.record_cmd, self.logdir / 'record.log', env).start()
        end = time.monotonic() + 30.0
        while time.monotonic() < end:
            self.spin(0.2)
            if not self.recorder.alive():
                self.say(f'      the recorder exited: {self.tail(self.recorder.log)}')
                return False
            if (self.bag / 'metadata.yaml').exists() or any(self.bag.glob('*.db3')) \
                    or any(self.bag.glob('*.mcap')):
                self.spin(2.0)      # subscriptions matched, a few messages in
                return True
        self.say(f'      the recording did not start in 30 s; see {self.recorder.log}')
        return False

    @staticmethod
    def tail(path, n=2):
        try:
            return ' | '.join(Path(path).read_text(errors='replace').strip().splitlines()[-n:])
        except OSError:
            return '(no log)'

    def ask_engage(self) -> bool:
        """True once engaged; False if the user aborts."""
        self.say('[4/7] ' + ('engaging (--yes, bench)' if self.o.yes else PROMPT))
        while True:
            if not self.o.yes:
                line = self.wait_line()
                if line is None:
                    return False
                if line.strip().lower().startswith('q'):
                    self.say('      aborted before engaging')
                    return False
            self.spin(0.5)
            why = engage_refusal(self.snap(), self.mon.available, self.o.max_pose_std)
            if why is None:
                ok, msg = self.mon.call(self.mon.cli_auto)
                if ok:
                    t0 = time.monotonic()
                    while time.monotonic() - t0 < 10.0 and self.mon.mode != self.mon.AUTONOMOUS:
                        self.spin(0.1)
                    if self.mon.mode == self.mon.AUTONOMOUS:
                        self.engaged_by_us = True
                        self.say('      engaged')
                        return True
                    why = 'the mode never became AUTONOMOUS'
                else:
                    why = f'engage refused: {msg or "no reason given"}'
            self.say(f'      not engaged: {why}')
            if self.o.yes:
                if getattr(self, '_tries', 0) >= 5:
                    return False
                self._tries = getattr(self, '_tries', 0) + 1
                self.spin(3.0)
            elif not self.stack.alive() or (self.recorder and not self.recorder.alive()):
                return False
            else:
                self.say('      ' + PROMPT)

    def wait_line(self) -> Optional[str]:
        """A line from stdin, spinning ROS meanwhile; None if the stack or the recording dies."""
        while True:
            if not self.stack.alive() or not self.recorder.alive():
                self.say('      the ' + ('stack' if not self.stack.alive() else 'recording') + ' exited')
                return None
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if r:
                line = sys.stdin.readline()
                if line == '':
                    self.say('      no terminal input (stdin closed)')
                    return None
                return line
            self.rclpy.spin_once(self.mon, timeout_sec=0.0)
            self.check_interrupt()

    def disengage(self, why):
        if self.mon is None:
            return
        if self.mon.mode == self.mon.AUTONOMOUS or self.engaged_by_us:
            ok, msg = self.mon.call(self.mon.cli_stop, timeout=3.0)
            self.engaged_by_us = False
            self.say(f'      disengaged ({why})' if ok else f'      disengage FAILED ({msg}): use the transmitter')

    def watch(self) -> End:
        self.say(f'[5/7] driving; ends at the stop, on an MRM, a disengage, a localization loss, '
                 f'or after {self.timeout:.0f} s')
        t0 = time.monotonic()
        last = 0.0
        length = None
        while True:
            self.spin(0.05)
            s = self.snap()
            end = end_reason(s, t0, self.timeout, self.o.max_pose_std)
            if end:
                self.status('')
                return end
            if time.monotonic() - last >= 1.0:
                last = time.monotonic()
                p = self.mon.progress
                length = getattr(p, 'lap_length', None) if p else length
                self.status(status_line(s, t0, p.progress if p else None, length, self.mon.mode_name()),
                            every=1.0)

    def stop_recording(self):
        if self.recorder is None:
            return
        self.say('[6/7] stopping the recording')
        ok = self.recorder.stop(steps=((signal.SIGINT, 30.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)),
                                say=self.say)
        valid = (self.bag / 'metadata.yaml').is_file()
        self.say(f'      bag {"closed" if valid else "NOT CLOSED (no metadata.yaml)"}: {self.bag}')
        self.recorder = None if ok else self.recorder

    def stop_stack(self):
        if self.stack is None:
            return
        self.say('      stopping the stack (SIGINT, then SIGTERM, SIGKILL)')
        ok = self.stack.stop(say=self.say)
        left = self.stack.remaining()
        self.say('      stack stopped, nothing left running' if ok and not left
                 else f'      process groups still alive: {left}')

    def audit(self, env) -> Optional[dict]:
        if not (self.bag / 'metadata.yaml').is_file():
            return None
        self.say('[7/7] audit')
        r = subprocess.run(self.audit_cmd, cwd=REPO, env=env, capture_output=True, text=True,
                           start_new_session=True, timeout=300)
        (self.logdir / 'audit.log').write_text(r.stdout + r.stderr)
        for ln in r.stdout.splitlines():
            self.say(ln)
        if r.returncode not in (0, 1):
            self.say(f'      the audit failed: {self.tail(self.logdir / "audit.log", 4)}')
        try:
            return json.loads((self.bag / 'audit.json').read_text())
        except (OSError, ValueError):
            return None


def integration_verdict(end: Optional[End], audit: Optional[dict]) -> Tuple[bool, List[str]]:
    """(PASS, [check lines]) for `just lab2 integration-test`."""
    lines = []

    def check(ok, text):
        lines.append(f'{"PASS" if ok else "FAIL"}  {text}')
        return ok

    ok = check(end is not None and end.kind == 'finished',
               f'run finished ({end.text if end else "did not run"})')
    a = audit or {}
    ok &= check(bool(a.get('check_ok')),
                'check-bags (bench mode): ' + ('ACCEPT' if a.get('check_ok') else
                                               'REJECT ' + '; '.join(a.get('check_reasons') or ['no audit'])))
    ok &= check(bool(a.get('grade_valid')),
                'grade: ' + (f'valid, {a.get("total", 0):.2f} / {a.get("max_total", 0):g}'
                             if a.get('grade_valid') else
                             'INVALID ' + '; '.join(a.get('gate_failures') or ['no audit'])))
    png = a.get('png')
    ok &= check(bool(png) and Path(png).is_file(), f'audit.png written ({png})')
    loc = a.get('localization') or {}
    if not loc:
        ok &= check(False, 'localization vs true pose: not measured (no true pose in the bag)')
    else:
        ok &= check(bool(loc.get('ok')), 'localization vs true pose: ' + loc.get('summary', '')
                    + ('' if loc.get('ok') else ' -- ' + '; '.join(loc.get('failures', []))))
    return bool(ok), lines


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        o = parse_args(argv)
    except UsageError as e:
        if str(e) == 'help':
            print(__doc__)
            return 0
        print(f'lab2 run: {e}', file=sys.stderr)
        return 2
    env = dict(os.environ)
    if o.bench:
        env.setdefault('ROS_DOMAIN_ID', '73')
        if env['ROS_DOMAIN_ID'] == '0':
            print('lab2 run: refusing the bench on ROS_DOMAIN_ID 0 (shared host)', file=sys.stderr)
            return 2
        env.setdefault('LAB2_WEB_ADDR', '127.0.0.1:8091')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID',) if k in env})
    run = Run(o)
    try:
        summary = run.plan()
    except lc.ConfigError as e:
        print(f'lab2 run: {e}', file=sys.stderr)
        return 2
    print(f'\033[2m{summary}\033[0m' if run.tty else summary, flush=True)

    def interrupt(signum, _frame):
        INTERRUPTED.append(signal.Signals(signum).name)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)

    audit = None
    finished_ok = False
    try:
        run.rclpy, run.mon = make_monitor()
        run.start_stack(env)
        if not run.wait_ready():
            run.end = End('stack', 'the stack did not come up')
            return 1
        if not run.preflight(env):
            run.say('      preflight FAILED: fix the FAIL lines above, then run again')
            run.end = End('stack', 'preflight failed')
            return 1
        if not run.start_recording(env):
            run.end = End('recorder', 'the recording did not start')
            return 1
        if not run.ask_engage():
            run.end = End('aborted', 'not engaged')
            return 1
        run.end = run.watch()
        tag = 'END ' if not run.end.abort else 'STOP'
        run.say(f'{tag}  {run.end.text}')
        if run.end.abort:
            run.disengage(run.end.kind)
        run.spin(SETTLE_AFTER_END)
        finished_ok = run.end.kind == 'finished'
    except KeyboardInterrupt as e:
        run.status('')
        run.say(f'interrupted ({e or "Ctrl-C"}): disengaging, stopping the recording and the stack')
        run.end = End('interrupted', 'interrupted')
    finally:
        # never interrupted half-way: a second Ctrl-C only says so
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, lambda *_: run.say('      cleaning up, please wait'))
        try:
            run.disengage(run.end.kind if run.end else 'end of run')
        except Exception as e:  # noqa: BLE001  (the stack may be gone already)
            run.say(f'      disengage: {e}')
        run.stop_recording()
        run.stop_stack()
        if run.mon is not None:
            run.mon.destroy_node()
            run.rclpy.try_shutdown()
    if run.end and run.end.kind != 'interrupted' and run.bag and (run.bag / 'metadata.yaml').is_file():
        audit = run.audit(env)
    elif run.bag and (run.bag / 'metadata.yaml').is_file():
        run.say(f'audit it with: just lab2 audit {run.bag}')
    if o.integration:
        ok, lines = integration_verdict(run.end, audit)
        run.say('')
        run.say('integration test:')
        for ln in lines:
            run.say('  ' + ln)
        run.say('INTEGRATION TEST ' + ('PASSED' if ok else 'FAILED'))
        return 0 if ok else 1
    return 0 if finished_ok and audit and audit.get('valid') else 1


if __name__ == '__main__':
    sys.exit(main())
