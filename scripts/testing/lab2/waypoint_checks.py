#!/usr/bin/env python3
"""Lab 2 (waypoint) vehicle-bench checks: the safety behaviour, in software.

Each check starts a fresh vehicle bench (`just lab2 bench`: the real vehicle
stack with a plant model for the car and a model of the camera + AR-tag
localizer), drives it, and prints one PASS or FAIL line. Exit status is the
number of FAILs.

    just lab2 bench-checks                          everything (about 6 min)
    just lab2 bench-checks --only follow,marker_loss
    just lab2 bench-checks --list

Checks:

  follow        engage on the first try, drive the course to the end, stop at
                the start; recorded with `just lab2 record` and graded with
                lab2_grade_run, every metric within the plan's pass limit
  marker_loss   kill virtual_marker_localizer while driving (the camera or
                the localizer dies): MRM within error_age (5 s) + 1.5 s of the
                last fix, the car stopped within 2 s of the MRM
  path_loss     kill waypoint_path_publisher while driving: MRM (the
                trajectory rate check), the car stops
  tracker_fault a PathTracker that raises: disabled after 3 faults, autonomous
                never available, an engage request refused
  no_markers    no marker ever detected (dropout 1.0): autonomous never
                available, an engage request refused

The actuator-level checks (command timeout, exit to neutral, a faulty
SpeedPID) are the same code in both labs: `just coach bench-checks --only
gate,exit,fault`.

Rules it keeps on a shared host: ROS_DOMAIN_ID (default 73, refuses 0), each
bench started with setsid and stopped by process group, only processes inside
that group signalled, nothing left running.
"""

import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
COURSE = REPO / 'src/lab/autosdv_waypoint_lab/config/courses/hall_demo'
CHECKS = ['follow', 'marker_loss', 'path_loss', 'tracker_fault', 'no_markers']
results = []


def report(name, ok, detail):
    results.append((name, ok))
    print(f'{"PASS" if ok else "FAIL"}  {name}: {detail}', flush=True)


def info(msg):
    print(f'      {msg}', flush=True)


class Bench:
    def __init__(self, name, args, workdir, env):
        self.name, self.args, self.env = name, list(args), env
        self.logdir = workdir / name
        self.proc = None

    def start(self):
        shutil.rmtree(self.logdir, ignore_errors=True)
        self.logdir.mkdir(parents=True)
        cmd = ['just', 'lab2', 'bench', '--log-dir', str(self.logdir / 'play_log')] + self.args
        info(f'bench: {" ".join(cmd)}')
        self.proc = subprocess.Popen(cmd, cwd=REPO, env=self.env, start_new_session=True,
                                     stdout=open(self.logdir / 'launch.log', 'w'),
                                     stderr=subprocess.STDOUT)

    def node_pid(self, node):
        for f in self.logdir.glob(f'play_log/*/node/{node}/pid'):
            try:
                return int(f.read_text().strip())
            except ValueError:
                pass
        return None

    def kill_node(self, node):
        pid = self.node_pid(node)
        if pid is None or os.getsid(pid) != self.proc.pid:
            info(f'{node}: not found in this bench')
            return False
        os.kill(pid, signal.SIGKILL)
        return True

    def stop(self):
        if self.proc is None:
            return
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            self.proc.wait(timeout=20)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def make_monitor():
    import rclpy
    from autoware_adapi_v1_msgs.msg import MrmState, OperationModeState
    from tier4_system_msgs.msg import OperationModeAvailability
    from autosdv_lab_msgs.msg import WaypointProgress
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

    rclpy.init()

    class Monitor(Node):
        def __init__(self):
            super().__init__('lab2_waypoint_checks')
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OperationModeAvailability,
                                     '/system/operation_mode/availability', self.on_avail, 10)
            self.create_subscription(OperationModeState, '/api/operation_mode/state',
                                     self.on_mode, latched)
            self.create_subscription(MrmState, '/system/fail_safe/mrm_state', self.on_mrm, 10)
            self.create_subscription(Odometry, '/vehicle_bench_plant/odometry', self.on_odom, 10)
            self.create_subscription(PoseWithCovarianceStamped,
                                     '/localization/pose_estimator/pose_with_covariance',
                                     self.on_fix, qos_profile_sensor_data)
            self.create_subscription(WaypointProgress, '/planning/waypoint_path_publisher/progress',
                                     self.on_progress, 10)
            self.MRM_NORMAL = MrmState.NORMAL
            self.AUTONOMOUS = OperationModeState.AUTONOMOUS
            self.reset()

        def reset(self):
            self.available = None
            self.mode = None
            self.mrm_since = None
            self.mrm_state = None
            self.speed = 0.0
            self.x = self.y = None
            self.last_fix = None
            self.progress = None

        def on_avail(self, m):
            self.available = bool(m.autonomous)

        def on_mode(self, m):
            self.mode = int(m.mode)

        def on_mrm(self, m):
            if m.state != self.MRM_NORMAL and self.mrm_since is None:
                self.mrm_since = time.monotonic()
            self.mrm_state = int(m.state)

        def on_odom(self, m):
            self.speed = m.twist.twist.linear.x
            self.x, self.y = m.pose.pose.position.x, m.pose.pose.position.y

        def on_fix(self, m):
            self.last_fix = time.monotonic()

        def on_progress(self, m):
            self.progress = m

    return rclpy, Monitor()


def spin_until(rclpy, mon, cond, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rclpy.spin_once(mon, timeout_sec=0.05)
        if cond():
            return True
    return False


def call(env, *args, timeout=30):
    return subprocess.run(['just', 'lab2', *args], cwd=REPO, env=env, timeout=timeout,
                          capture_output=True, text=True)


def bring_up(rclpy, mon, bench, timeout=90, settle=3.0):
    """Start the bench; return once autonomous has been available for `settle` s.

    Settling matters straight after another bench: availability can flicker
    while the new stack's monitors fill their windows, and an engage in that
    gap is refused.
    """
    mon.reset()
    bench.start()
    since = [None]

    def steady():
        if not mon.available:
            since[0] = None
            return False
        since[0] = since[0] or time.monotonic()
        return time.monotonic() - since[0] >= settle

    if not spin_until(rclpy, mon, steady, timeout):
        info(f'autonomous never became steadily available; logs in {bench.logdir}')
        return False
    return True


def engage(rclpy, mon, env):
    call(env, 'engage')
    return spin_until(rclpy, mon, lambda: mon.mode == mon.AUTONOMOUS, 10)


def check_follow(rclpy, mon, env, workdir):
    bench = Bench('follow', ['seed:=11'], workdir, env)
    rec = None
    try:
        if not bring_up(rclpy, mon, bench):
            return report('follow', False, 'never available')
        name = f'check_{int(time.time())}'
        rec = subprocess.Popen(['just', 'lab2', 'record', name], cwd=REPO, env=env,
                               start_new_session=True, stdout=open(bench.logdir / 'record.log', 'w'),
                               stderr=subprocess.STDOUT)
        time.sleep(3)
        if not engage(rclpy, mon, env):
            return report('follow', False, 'engage refused on the first try')
        done = spin_until(rclpy, mon, lambda: mon.progress is not None and mon.progress.finished
                          and abs(mon.speed) < 0.01, 220)
        time.sleep(2)
        os.killpg(rec.pid, signal.SIGINT)
        rec.wait(timeout=15)
        rec = None
        if not done:
            return report('follow', False, 'the run did not finish within 220 s')
        dist = math.hypot(mon.x, mon.y)
        bag = REPO / 'rosbags' / f'lab2wp_{name}'
        g = call(env, 'grade', str(bag), '--course', str(COURSE), timeout=120)
        out = g.stdout
        (bench.logdir / 'grade.txt').write_text(out + g.stderr)
        fails = [ln.strip() for ln in out.splitlines() if 'FAIL' in ln]
        ok = not fails and dist < 0.3
        detail = f'stopped {dist:.2f} m from the start; ' + (
            'all metrics within limits' if not fails else '; '.join(fails))
        info('\n      '.join(ln for ln in out.splitlines() if ln.startswith('  ')))
        shutil.rmtree(bag, ignore_errors=True)
        report('follow', ok, detail)
    finally:
        if rec is not None:
            os.killpg(rec.pid, signal.SIGINT)
        bench.stop()


def check_loss(rclpy, mon, env, workdir, name, node, limit):
    bench = Bench(name, ['seed:=12'], workdir, env)
    try:
        if not bring_up(rclpy, mon, bench) or not engage(rclpy, mon, env):
            return report(name, False, 'could not engage')
        spin_until(rclpy, mon, lambda: False, 15)
        if mon.speed < 0.3:
            return report(name, False, f'not driving before the kill ({mon.speed:.2f} m/s)')
        mon.mrm_since = None
        t_kill = time.monotonic()
        if not bench.kill_node(node):
            return report(name, False, f'{node} not found')
        last_fix = mon.last_fix if node == 'virtual_marker_localizer' else t_kill
        if not spin_until(rclpy, mon, lambda: mon.mrm_since is not None, limit + 5):
            return report(name, False, f'no MRM within {limit + 5} s of killing {node}')
        t_mrm = mon.mrm_since - last_fix
        stopped = spin_until(rclpy, mon, lambda: abs(mon.speed) < 0.02, 3)
        t_stop = time.monotonic() - mon.mrm_since
        ok = t_mrm <= limit + 1.5 and stopped and t_stop <= 2.0
        report(name, ok, f'MRM {t_mrm:.1f} s after the last '
               f'{"fix" if node == "virtual_marker_localizer" else "trajectory"}; '
               f'stopped {t_stop:.1f} s after the MRM')
    finally:
        bench.stop()


def check_refused(rclpy, mon, env, workdir, name, args, why):
    bench = Bench(name, args, workdir, env)
    try:
        mon.reset()
        bench.start()
        spin_until(rclpy, mon, lambda: False, 40)
        available = mon.available
        call(env, 'engage')
        spin_until(rclpy, mon, lambda: False, 3)
        refused = mon.mode != mon.AUTONOMOUS
        report(name, available is False and refused,
               f'autonomous available: {available}; engage {"refused" if refused else "ACCEPTED"} ({why})')
    finally:
        bench.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--only', default='')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--workdir', default=str(REPO / 'tmp/lab2_waypoint_checks'))
    args = ap.parse_args()
    if args.list:
        print('\n'.join(CHECKS))
        return 0
    only = [c for c in args.only.split(',') if c] or CHECKS
    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', '73')
    if env['ROS_DOMAIN_ID'] == '0':
        print('refusing to run on ROS_DOMAIN_ID 0 (shared host)')
        return 1
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    env.setdefault('LAB2_WEB_ADDR', '127.0.0.1:8091')
    workdir = Path(args.workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    rclpy, mon = make_monitor()
    try:
        for c in only:
            print(f'--- {c}', flush=True)
            if c == 'follow':
                check_follow(rclpy, mon, env, workdir)
            elif c == 'marker_loss':
                check_loss(rclpy, mon, env, workdir, c, 'virtual_marker_localizer', 5.0)
            elif c == 'path_loss':
                check_loss(rclpy, mon, env, workdir, c, 'waypoint_path_publisher', 2.0)
            elif c == 'tracker_fault':
                check_refused(rclpy, mon, env, workdir, c,
                              ['tracker_class:=autosdv_waypoint_lab.faulty_path_tracker:RaisingPathTracker'],
                              'the law is disabled')
            elif c == 'no_markers':
                check_refused(rclpy, mon, env, workdir, c, ['dropout:=1.0'], 'no marker fix')
            else:
                print(f'unknown check {c}')
    finally:
        mon.destroy_node()
        rclpy.shutdown()
    fails = sum(1 for _, ok in results if not ok)
    print(f'\n{len(results) - fails}/{len(results)} passed')
    return fails


if __name__ == '__main__':
    sys.exit(main())
