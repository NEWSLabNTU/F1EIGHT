#!/usr/bin/env python3
"""Lab 2 vehicle-bench checks: field protocol section 3.1, in software.

Each check starts a fresh vehicle bench (coach_pursuit_bench.launch.xml: the
real vehicle stack with a plant model for the car), drives it through the same
recipes a TA uses (`just coach engage`, `just coach record`), and prints one
PASS or FAIL line. Exit status is the number of FAILs (0 = all passed).

    just coach bench-checks                       everything (about 5 min)
    just coach bench-checks --only follow,mrm     a subset
    just coach bench-checks --list

Checks (protocol 3.1 numbering in brackets):

  follow      engage on the first try; follow the default scenario; record it
              with `just coach record` and grade it with ta/grade.py --runs
  mrm     [3] kill board_pursuit_planner while following: MRM emergency stop,
              the actuator brakes (override_brake:emergency), the car stops
  gate    [1] unload vehicle_cmd_gate while following: the actuator's command
              watchdog fires (override_brake:command_timeout) within
              control_cmd_timeout plus margin, then neutral; the car stops
  exit    [2] actuator SIGINT / SIGTERM leave the motor at init_pwm and the
              steering at init_steer: the actuator's process-level test
              (test_actuator_node.py::test_process_exit_leaves_outputs_neutral),
              because a dry run does not publish its exit write
  fault   [4] a SpeedPID that raises in update(): neutral, retry after the
              holdoff, disabled after 3 faults, the node stays up
  speedpid    the course's reference SpeedPID drives the car; the handout's
              stub (returns neutral) does not

Rules it keeps on a shared host: it runs on ROS_DOMAIN_ID (default 73, refuses
0), starts each bench with setsid and stops it by process group, signals only
processes inside that group, and leaves nothing running.
"""

import argparse
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
DEFAULT_COURSE = Path('/home/aeon/repos/ntu-sdv-course/2026/labs/Lab2')
CHECKS = ['follow', 'mrm', 'gate', 'exit', 'fault', 'speedpid']

results = []


def report(name, ok, detail):
    tag = 'PASS' if ok else 'FAIL'
    results.append((name, ok))
    print(f'{tag}  {name}: {detail}', flush=True)


def warn(name, detail):
    """A finding that is not a failure of the check."""
    print(f'WARN  {name}: {detail}', flush=True)


def info(msg):
    print(f'      {msg}', flush=True)


# ============================================================ bench process

class Bench:
    """One `just coach bench` in its own process group."""

    def __init__(self, name, args, workdir, env):
        self.name = name
        self.args = list(args)
        self.logdir = workdir / name
        self.env = env
        self.proc = None

    def start(self):
        shutil.rmtree(self.logdir, ignore_errors=True)
        self.logdir.mkdir(parents=True)
        cmd = ['just', 'coach', 'bench', '--log-dir', str(self.logdir / 'play_log')] + self.args
        info(f'bench: {" ".join(cmd)}')
        self.out = open(self.logdir / 'launch.log', 'w')
        self.proc = subprocess.Popen(cmd, cwd=REPO, env=self.env, stdout=self.out,
                                     stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                     start_new_session=True)
        self.pgid = self.proc.pid  # start_new_session: the child leads its group

    def members(self):
        """PIDs in this bench's session (only these are ever signalled).

        play_launch puts its nodes in process groups of their own, but they
        stay in the session `just` started.
        """
        pids = []
        for d in Path('/proc').iterdir():
            if not d.name.isdigit():
                continue
            try:
                if os.getsid(int(d.name)) == self.pgid:
                    pids.append(int(d.name))
            except (ProcessLookupError, PermissionError):
                pass
        return pids

    def find(self, pattern):
        """PIDs in the group whose command line matches `pattern` (a regex)."""
        out = []
        for pid in self.members():
            try:
                cmd = (Path(f'/proc/{pid}/cmdline').read_bytes()
                       .replace(b'\0', b' ').decode(errors='replace'))
            except OSError:
                continue
            if re.search(pattern, cmd):
                out.append(pid)
        return out

    def alive(self, pattern):
        return bool(self.find(pattern))

    def signal(self, pattern, sig):
        pids = self.find(pattern)
        for pid in pids:
            os.kill(pid, sig)
        return pids

    def stop(self):
        if self.proc is None:
            return
        for sig, wait in ((signal.SIGINT, 15.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)):
            # play_launch first (it stops its nodes); stragglers individually
            targets = [self.pgid] if sig == signal.SIGINT else self.members()
            for pid in targets:
                try:
                    os.killpg(pid, sig) if pid == self.pgid else os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline and self.members():
                time.sleep(0.2)
            if not self.members():
                break
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            pass
        self.out.close()
        self.proc = None
        left = self.members()
        if left:
            info(f'WARNING: processes left in session {self.pgid}: {left}')


def actuator_log(bench):
    """The actuator's stderr+stdout from the bench's play_log."""
    text = ''
    for f in sorted(bench.logdir.glob('play_log/*/node/actuator_node/*')):
        if f.name in ('err', 'out'):
            text += f.read_text(errors='replace')
    return text


def run_recipe(args, env, timeout=30):
    p = subprocess.run(['just', 'coach'] + args, cwd=REPO, env=env, capture_output=True,
                       text=True, timeout=timeout)
    return p.returncode, p.stdout + p.stderr


# ============================================================ ROS side

def make_monitor():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from autoware_adapi_v1_msgs.msg import MrmState, OperationModeState
    from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from composition_interfaces.srv import ListNodes, UnloadNode

    class Monitor(Node):
        def __init__(self):
            super().__init__('lab2_bench_checks')
            self.lock = threading.Lock()
            self.reset()
            latched = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/autosdv/actuator_node/debug/mode',
                                     self.on_mode, latched)
            self.create_subscription(OperationModeState, '/api/operation_mode/state',
                                     self.on_opmode, latched)
            self.create_subscription(MrmState, '/system/fail_safe/mrm_state', self.on_mrm, 10)
            self.create_subscription(Float32MultiArrayStamped,
                                     '/autosdv/actuator_node/debug/pwm_values', self.on_pwm, 10)
            self.create_subscription(Odometry, '/vehicle_bench_plant/odometry', self.on_odom, 10)
            self.create_subscription(PoseStamped, '/virtual_coach/truth', self.on_truth, 10)
            self.create_subscription(Odometry, '/localization/kinematic_state', self.on_dr, 10)
            self.ListNodes, self.UnloadNode = ListNodes, UnloadNode
            self.MrmState, self.OperationModeState = MrmState, OperationModeState

        def reset(self):
            with getattr(self, 'lock', threading.Lock()):
                self.modes = []      # (t, mode string)
                self.opmode = None
                self.mrm = []        # (t, state, behavior)
                self.pwm = []        # (t, motor, steer)
                self.odom = None
                self.speed_max = 0.0
                self.truth = None
                self.min_gap = math.inf
                self.dr = None

        def on_mode(self, m):
            with self.lock:
                self.modes.append((time.monotonic(), m.data))

        def on_opmode(self, m):
            self.opmode = m

        def on_mrm(self, m):
            with self.lock:
                if not self.mrm or self.mrm[-1][1:] != (m.state, m.behavior):
                    self.mrm.append((time.monotonic(), m.state, m.behavior))

        def on_pwm(self, m):
            if len(m.data) >= 2:
                with self.lock:
                    self.pwm.append((time.monotonic(), m.data[0], m.data[1]))
                    if len(self.pwm) > 20000:
                        del self.pwm[:10000]

        def on_odom(self, m):
            self.odom = m
            self.speed_max = max(self.speed_max, m.twist.twist.linear.x)
            if self.truth is not None:
                p, b = m.pose.pose.position, self.truth.pose.position
                self.min_gap = min(self.min_gap, math.hypot(b.x - p.x, b.y - p.y))

        def on_truth(self, m):
            self.truth = m

        def on_dr(self, m):
            self.dr = m

        @property
        def speed(self):
            return self.odom.twist.twist.linear.x if self.odom else 0.0

        @property
        def x(self):
            return self.odom.pose.pose.position.x if self.odom else 0.0

        def mode_now(self):
            with self.lock:
                return self.modes[-1][1] if self.modes else None

        def modes_since(self, t0):
            with self.lock:
                return [(t, m) for t, m in self.modes if t >= t0]

        def unload(self, container, node_name, timeout=10.0):
            lst = self.create_client(self.ListNodes, f'{container}/_container/list_nodes')
            unl = self.create_client(self.UnloadNode, f'{container}/_container/unload_node')
            if not lst.wait_for_service(timeout_sec=timeout):
                return False, 'list_nodes unavailable'
            fut = lst.call_async(self.ListNodes.Request())
            if not wait_future(fut, timeout):
                return False, 'list_nodes timed out'
            res = fut.result()
            ids = [i for n, i in zip(res.full_node_names, res.unique_ids) if n == node_name]
            if not ids:
                return False, f'{node_name} not in {list(res.full_node_names)}'
            req = self.UnloadNode.Request()
            req.unique_id = ids[0]
            fut = unl.call_async(req)
            if not wait_future(fut, timeout):
                return False, 'unload_node timed out'
            r = fut.result()
            return bool(r.success), r.error_message

    rclpy.init()
    node = Monitor()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    return node, ex


def wait_future(fut, timeout):
    deadline = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    return fut.done()


def wait_for(cond, timeout, period=0.1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(period)
    return cond()


# ============================================================ shared steps

def bring_up(mon, bench, timeout=90.0):
    """Start the bench and wait until autonomous is available."""
    mon.reset()
    bench.start()
    ok = wait_for(lambda: mon.opmode is not None and mon.opmode.is_autonomous_mode_available
                  and mon.odom is not None, timeout, 0.5)
    if not ok:
        info(f'autonomous never became available; logs in {bench.logdir}')
    return ok


def engage(mon, env):
    """One engage attempt through the recipe; True if AUTONOMOUS follows."""
    rc, out = run_recipe(['engage'], env)
    success = 'success=True' in out
    ok = success and wait_for(
        lambda: mon.opmode is not None
        and mon.opmode.mode == mon.OperationModeState.AUTONOMOUS, 5.0)
    line = next((ln for ln in out.splitlines() if 'success=' in ln), f'rc {rc}, no response')
    m = re.search(r'success=\w+, code=\d+, message=\'[^\']*\'', line)
    return ok, m.group(0) if m else line.strip()


def drive_until_moving(mon, speed=0.6, timeout=40.0):
    return wait_for(lambda: mon.speed > speed, timeout)


# ============================================================ checks

def check_follow(mon, args, env, workdir):
    bench = Bench('follow', ['scenario:=default'], workdir, env)
    try:
        if not bring_up(mon, bench):
            report('follow.engage', False, 'autonomous not available')
            return
        name = f'benchcheck_default_{time.strftime("%Y%m%d_%H%M%S")}'
        bag = REPO / 'rosbags' / f'lab2_{name}'
        rec = subprocess.Popen(['just', 'coach', 'record', name], cwd=REPO, env=env,
                               stdout=open(bench.logdir / 'record.log', 'w'),
                               stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(3.0)
        ok, line = engage(mon, env)
        report('follow.engage', ok, f'first attempt: {line}')
        if not ok:
            os.killpg(rec.pid, signal.SIGINT)
            rec.wait(timeout=10)
            return
        # default scenario: 10 s static, 25 s walk at 0.8 m/s, then still
        time.sleep(args.follow_time)
        os.killpg(rec.pid, signal.SIGINT)
        try:
            rec.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(rec.pid, signal.SIGKILL)
        dist = mon.x
        report('follow.drive', dist > 15.0 and mon.min_gap > 1.0,
               f'drove {dist:.1f} m (true pose), peak {mon.speed_max:.2f} m/s, '
               f'min true gap {mon.min_gap:.2f} m, still AUTONOMOUS: '
               f'{mon.opmode.mode == mon.OperationModeState.AUTONOMOUS}')
        grade = args.course / 'ta' / 'grade.py'
        if not grade.exists():
            report('follow.grade', False, f'{grade} not found (--course)')
            return
        p = subprocess.run(['python3', str(grade), '--runs', str(bag), '--scenario', 'default'],
                           cwd=REPO, env=env, capture_output=True, text=True, timeout=120)
        lines = [ln for ln in p.stdout.splitlines() if ln.strip() and '[INFO]' not in ln]
        for ln in lines:
            info(f'grade: {ln.strip()}')
        text = '\n'.join(lines)
        m = re.search(r'whole run min ([\d.]+) m', text)
        rmse = re.search(r'RMSE ([\d.]+) m', text)
        rows = re.search(r'(\d+) rows', text)
        # Sane: the bag graded as a run of the scenario, with a following phase
        # that tracked; the 1.0 m floor is a finding, reported below, not a
        # property of the bench.
        ok = (p.returncode == 0 and 'error' not in text and m is not None and rows is not None
              and int(rows.group(1)) > 300 and rmse is not None and float(rmse.group(1)) < 0.5)
        report('follow.grade', ok,
               f'ta/grade.py --runs {bag.name}: {rows.group(1) if rows else "?"} rows, follow RMSE '
               f'{rmse.group(1) if rmse else "?"} m, min headway {m.group(1) if m else "?"} m')
        if mon.dr is not None and mon.odom is not None:
            p_dr, p_true = mon.dr.pose.pose.position, mon.odom.pose.pose.position
            info(f'dead reckoning vs true pose at the end: dx {p_dr.x - p_true.x:+.2f} m, '
                 f'dy {p_dr.y - p_true.y:+.2f} m (grade.py measures headway against dead reckoning)')
        if m is not None and float(m.group(1)) < 1.0:
            warn('follow.grade', f'min headway {m.group(1)} m is below the 1.0 m floor '
                 f'(true minimum gap {mon.min_gap:.2f} m)')
    finally:
        bench.stop()


def check_mrm(mon, args, env, workdir):
    bench = Bench('mrm', [], workdir, env)
    try:
        if not bring_up(mon, bench) or not engage(mon, env)[0]:
            report('mrm', False, 'could not engage')
            return
        if not drive_until_moving(mon):
            report('mrm', False, f'never reached 0.6 m/s (peak {mon.speed_max:.2f})')
            return
        v0 = mon.speed
        t0 = time.monotonic()
        killed = bench.signal(r'board_pursuit_planner', signal.SIGINT)
        info(f'SIGINT board_pursuit_planner (pids {killed}) at {v0:.2f} m/s')
        S = mon.MrmState
        got_mrm = wait_for(lambda: any(t >= t0 and s in (S.MRM_OPERATING, S.MRM_SUCCEEDED)
                                       for t, s, _ in mon.mrm), 5.0)
        t_mrm = next((t for t, s, _ in mon.mrm if t >= t0
                      and s in (S.MRM_OPERATING, S.MRM_SUCCEEDED)), None)
        behavior = next((b for t, s, b in mon.mrm if t >= t0
                         and s in (S.MRM_OPERATING, S.MRM_SUCCEEDED)), None)
        braked = wait_for(lambda: any(m.startswith('override_brake:emergency')
                                      for _, m in mon.modes_since(t0)), 5.0)
        stopped = wait_for(lambda: mon.speed < 0.01, 8.0)
        t_stop = time.monotonic()
        modes = [m for _, m in mon.modes_since(t0)]
        report('mrm', got_mrm and braked and stopped,
               f'MRM {f"after {t_mrm - t0:.2f} s, behavior {behavior} (2 = EMERGENCY_STOP)" if got_mrm else "never"}; '
               f'actuator modes {modes}; stopped {stopped} ({t_stop - t0:.1f} s after the kill)')
    finally:
        bench.stop()


def check_gate(mon, args, env, workdir):
    bench = Bench('gate', [], workdir, env)
    try:
        if not bring_up(mon, bench) or not engage(mon, env)[0]:
            report('gate', False, 'could not engage')
            return
        if not drive_until_moving(mon):
            report('gate', False, f'never reached 0.6 m/s (peak {mon.speed_max:.2f})')
            return
        v0 = mon.speed
        t0 = time.monotonic()
        ok, msg = mon.unload('/control/control_container', '/control/vehicle_cmd_gate')
        info(f'unload /control/vehicle_cmd_gate at {v0:.2f} m/s: {ok} {msg}')
        if not ok:
            report('gate', False, f'could not unload the gate: {msg}')
            return
        fired = wait_for(lambda: any(':' in m for _, m in mon.modes_since(t0)), 3.0)
        first = next(((t, m) for t, m in mon.modes_since(t0) if ':' in m), None)
        neutral = wait_for(lambda: (mon.mode_now() or '').startswith('override_neutral'), 6.0)
        stopped = wait_for(lambda: mon.speed < 0.01, 6.0)
        modes = [m for _, m in mon.modes_since(t0)]
        latency = first[0] - t0 if first else None
        # control_cmd_timeout 0.3 s, plus the unload call and a 100 Hz tick
        ok = (fired and first[1] == 'override_brake:command_timeout' and latency < 0.6
              and neutral and stopped)
        last_pwm = mon.pwm[-1][1:] if mon.pwm else None
        report('gate', ok,
               f'first override {first[1] if first else None} '
               f'{f"{latency:.2f} s" if latency is not None else ""} after the unload '
               f'(timeout 0.3 s); modes {modes}; stopped {stopped}; last PWM {last_pwm}')
    finally:
        bench.stop()


def check_exit(mon, args, env, workdir):
    test = (REPO / 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/test/'
            'test_actuator_node.py')
    p = subprocess.run(['python3', '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
                        f'{test}::test_process_exit_leaves_outputs_neutral'],
                       cwd=test.parent.parent, env=env, capture_output=True, text=True,
                       timeout=180)
    tail = [ln for ln in p.stdout.splitlines() if ln.strip()][-1:] or ['(no output)']
    report('exit', p.returncode == 0,
           f'actuator process SIGINT/SIGTERM leave motor 370, steering 489 (fake PCA9685): '
           f'{tail[0]}')


def check_fault(mon, args, env, workdir):
    cls = 'autosdv_coach_lab.faulty_speed_controller:RaisingSpeedPID'
    bench = Bench('fault', [f'speed_controller_class:={cls}'], workdir, env)
    try:
        if not bring_up(mon, bench):
            report('fault', False, 'autonomous not available')
            return
        t0 = time.monotonic()
        ok, line = engage(mon, env)
        if not ok:
            report('fault', False, f'could not engage: {line}')
            return
        time.sleep(8.0)
        modes = mon.modes_since(t0)
        # A retry raises again inside the same 100 Hz tick, so the published
        # mode never leaves controller_fault; the actuator's log has each one.
        log = actuator_log(bench)
        stamps = {}
        for m in re.finditer(r'\[(\d+\.\d+)\] \[autosdv\.actuator_node\]: Speed controller '
                             r'update\(\) failed \((?:fault (\d+)|(\d+) faults)\)', log):
            stamps[int(m.group(2) or m.group(3))] = float(m.group(1))
        latched_log = 'disabled until the actuator restarts' in log
        gaps = [stamps[i + 1] - stamps[i] for i in sorted(stamps) if i + 1 in stamps]
        still = (mon.mode_now() or '').endswith(':controller_fault')
        alive = bench.alive(r'lib/autosdv_vehicle_interface/actuator')
        recent = [p for p in mon.pwm if p[0] > time.monotonic() - 1.0]
        publishing = len(recent) > 50
        neutral = bool(recent) and all(int(p[1]) == 370 for p in recent)
        gaps_ok = len(gaps) == 2 and all(0.9 < g < 1.3 for g in gaps)
        ok = (sorted(stamps) == [1, 2, 3] and gaps_ok and latched_log and still and alive
              and publishing and neutral)
        report('fault', ok,
               f'{len(stamps)} faults, retries after '
               f'{", ".join(f"{g:.2f} s" for g in gaps) or "-"} (holdoff 1.0 s); '
               f'disabled after fault 3: {latched_log}; mode now {mon.mode_now()}; '
               f'actuator alive {alive}, PWM at {len(recent)} Hz, motor '
               f'{int(recent[-1][1]) if recent else None}')
        info(f'modes: {[m for _, m in modes]}')
    finally:
        bench.stop()


def check_speedpid(mon, args, env, workdir):
    ref = args.course / 'ta' / 'reference' / 'speed_pid.py'
    ref_yaml = args.course / 'handout' / 'lab2_speed' / 'config' / 'speed_pid_car.yaml'
    ref_cal = args.course / 'ta' / 'cars' / 'car_A' / 'calibration_ref.yaml'
    stub = args.course / 'handout' / 'lab2_speed' / 'lab2_speed' / 'speed_pid.py'
    if not (ref.exists() and stub.exists()):
        report('speedpid', False, f'{ref} or {stub} not found (--course)')
        return
    plug = workdir / 'plugins' / 'lab2_bench_plugins'
    shutil.rmtree(plug.parent, ignore_errors=True)
    plug.mkdir(parents=True)
    (plug / '__init__.py').write_text('')
    shutil.copy(ref, plug / 'ref_speed_pid.py')
    shutil.copy(stub, plug / 'stub_speed_pid.py')
    import yaml
    params = yaml.safe_load(ref_yaml.read_text()) if ref_yaml.exists() else {}
    if ref_cal.exists():        # the car file's feed-forward comes from the calibration
        cal = yaml.safe_load(ref_cal.read_text())
        params.setdefault('ff_deadband', float(cal['deadband']))
        params.setdefault('ff_gain', float(cal['gain']))
    flow = '{' + ', '.join(f'{k}: {v}' for k, v in params.items()) + '}'
    penv = dict(env)
    penv['PYTHONPATH'] = f'{plug.parent}:{env.get("PYTHONPATH", "")}'
    out = {}
    for label, cls, extra in (
            ('reference', 'lab2_bench_plugins.ref_speed_pid:SpeedPID',
             [f'speed_controller_params:={flow}']),
            ('stub', 'lab2_bench_plugins.stub_speed_pid:SpeedPID', [])):
        bench = Bench(f'speedpid_{label}', [f'speed_controller_class:={cls}'] + extra,
                      workdir, penv)
        try:
            if not bring_up(mon, bench) or not engage(mon, penv)[0]:
                out[label] = None
                continue
            time.sleep(20.0)
            recent = [p[1] for p in mon.pwm if p[0] > time.monotonic() - 2.0]
            out[label] = (mon.x, mon.speed_max, mon.mode_now(),
                          max(recent) if recent else None)
            info(f'{label}: {out[label][0]:.2f} m, peak {out[label][1]:.2f} m/s, '
                 f'mode {out[label][2]}, max motor PWM (last 2 s) {out[label][3]}')
        finally:
            bench.stop()
    r, s = out.get('reference'), out.get('stub')
    ok = r is not None and s is not None and r[0] > 5.0 and s[0] < 0.2
    report('speedpid', ok,
           f'reference {r[0]:.1f} m / peak {r[1]:.2f} m/s; stub {s[0]:.2f} m / peak '
           f'{s[1]:.2f} m/s in 20 s after engage' if r and s else f'engage failed: {out}')


# ============================================================ main

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--only', help=f'comma-separated subset of {",".join(CHECKS)}')
    ap.add_argument('--list', action='store_true', help='list the checks and exit')
    ap.add_argument('--domain', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '73') or 73),
                    help='ROS_DOMAIN_ID for the benches (default: $ROS_DOMAIN_ID or 73; 0 refused)')
    ap.add_argument('--web-addr', default=os.environ.get('LAB2_WEB_ADDR', '127.0.0.1:8091'),
                    help='play_launch web UI address (default: $LAB2_WEB_ADDR or 127.0.0.1:8091)')
    ap.add_argument('--course', type=Path, default=Path(os.environ.get('LAB2_COURSE_DIR', DEFAULT_COURSE)),
                    help='the course Lab2 directory (ta/grade.py, the SpeedPIDs)')
    ap.add_argument('--follow-time', type=float, default=45.0,
                    help='seconds to record after engage in the follow check')
    ap.add_argument('--workdir', type=Path, default=REPO / 'tmp' / 'lab2bench' / 'checks')
    args = ap.parse_args()
    if args.list:
        print(__doc__.split('Checks (protocol')[1].split('Rules')[0])
        return 0
    if args.domain == 0:
        print('refusing ROS_DOMAIN_ID 0 on a shared host; pass --domain', file=sys.stderr)
        return 2
    only = CHECKS if not args.only else [c.strip() for c in args.only.split(',')]
    bad = [c for c in only if c not in CHECKS]
    if bad:
        print(f'unknown checks {bad}; choose from {CHECKS}', file=sys.stderr)
        return 2

    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    env = dict(os.environ)
    env['LAB2_WEB_ADDR'] = args.web_addr
    args.workdir.mkdir(parents=True, exist_ok=True)
    print(f'Lab 2 bench checks: domain {args.domain}, logs in {args.workdir}', flush=True)

    mon, ex = make_monitor()
    fns = {'follow': check_follow, 'mrm': check_mrm, 'gate': check_gate,
           'exit': check_exit, 'fault': check_fault, 'speedpid': check_speedpid}
    try:
        for c in only:
            print(f'--- {c}', flush=True)
            try:
                fns[c](mon, args, env, args.workdir)
            except Exception as e:  # report and go on to the next check
                report(c, False, f'{type(e).__name__}: {e}')
    finally:
        ex.shutdown()
        mon.destroy_node()
    failed = [n for n, ok in results if not ok]
    print(f'--- {len(results) - len(failed)} passed, {len(failed)} failed'
          + (f': {", ".join(failed)}' if failed else ''), flush=True)
    return len(failed)


if __name__ == '__main__':
    sys.exit(main())
