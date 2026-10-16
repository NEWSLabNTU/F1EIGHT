#!/usr/bin/env python3
"""Lab 2 vehicle-bench gate: does the team's SpeedPID drive the graded route (lab2_uturn)?

Each route (by default the one graded route) starts a fresh vehicle bench (`just lab2 bench`: the car's actuator
with the SpeedPID plug-in, the EKF, the reference tracker, the MRM path, and a
plant model for the car), engages, waits for the run to finish and prints one
PASS or FAIL line. Exit status is the number of FAILs. A team has to pass
before it gets car time.

    just lab2 gate speed_controller_class:=lab2_waypoint.speed_pid:SpeedPID \\
                   "speed_controller_params:={kp: 15.0, ki: 5.0}"
    just lab2 gate --only hall_demo              # another course instead
    just lab2 gate --list

Arguments after the options go to the bench's launch file unchanged, which is
how a team's SpeedPID, its parameters and its lookahead file are passed. With
none, the launch file's defaults apply (on the student branch: the stub, which
fails).

A route PASSES when all of these hold:

  finished   the path publisher reports the run finished with every lap done,
             and the car is at rest afterwards (timeout: --timeout s)
  no MRM     the MRM state stayed NORMAL for the whole run
  clean      the actuator never reported a SpeedPID fault (an exception or a
             non-finite PWM puts the motor in neutral and shows as
             `...:controller_fault` on ~/debug/mode)
  moving     progress is not stalled: 0.5 m within 20 s of the engage
  speed      the median speed while moving is 0.6 to 1.15 times the route's
             v_ref, and the peak is under 1.5 times v_ref

This is a smoke gate on the full stack, not the grade: the grade is the speed
bench and the graded bags. Rules it keeps on a shared host: ROS_DOMAIN_ID
(default 73, refuses 0), each bench started with setsid and stopped by process
group, nothing left running.
"""

import argparse
import json
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import waypoint_checks as wc  # noqa: E402  (Bench, spin_until, call, bring_up, engage)

REPO = wc.REPO
COURSES_DIR = REPO / 'src/lab/autosdv_waypoint_lab/config/courses'
ROUTES = ['lab2_uturn']

SPEED_MEDIAN = (0.6, 1.15)   # x v_ref
SPEED_PEAK = 1.5             # x v_ref
STALL_S, STALL_M = 20.0, 0.5

# Verdict helpers are pure so the unit test can drive them without ROS.


def judge(v_ref, speeds, finished, laps_done, laps, mrm, faults, stalled, at_rest, timed_out):
    """Return (ok, [reasons it failed]) from what one run recorded."""
    why = []
    if faults:
        why.append(f'SpeedPID fault ({faults[0]})')
    if mrm:
        why.append('MRM raised')
    if stalled:
        why.append(f'car did not move ({STALL_M} m in {STALL_S:.0f} s)')
    elif timed_out and not finished:
        why.append('did not finish in time')
    elif finished and (laps_done < laps):
        why.append(f'{laps_done}/{laps} laps')
    elif finished and not at_rest:
        why.append('still moving after the run')
    moving = [s for s in speeds if s > 0.1]
    if moving and not stalled:
        med, peak = statistics.median(moving), max(moving)
        if not SPEED_MEDIAN[0] * v_ref <= med <= SPEED_MEDIAN[1] * v_ref:
            why.append(f'median speed {med:.2f} m/s outside '
                       f'{SPEED_MEDIAN[0] * v_ref:.2f}..{SPEED_MEDIAN[1] * v_ref:.2f}')
        if peak > SPEED_PEAK * v_ref:
            why.append(f'peak speed {peak:.2f} m/s over {SPEED_PEAK * v_ref:.2f}')
    return (not why), why


def course_info(name):
    import yaml
    d = Path(name)
    if not d.is_dir():
        d = COURSES_DIR / name
    cfg = yaml.safe_load((d / 'course.yaml').read_text())
    return d, float(cfg.get('v_ref', 0.8)), int(cfg.get('laps', 1))


def load_error(logdir):
    """The actuator's `Cannot load speed controller ...` line, or None."""
    for f in Path(logdir).glob('play_log/*/node/actuator_node/err'):
        for line in f.read_text(errors='replace').splitlines():
            if 'Cannot load speed controller' in line:
                return line.split(']: ', 1)[-1].strip()
    return None


def make_monitor():
    import rclpy
    from autoware_adapi_v1_msgs.msg import MrmState, OperationModeState
    from tier4_system_msgs.msg import OperationModeAvailability
    from autosdv_lab_msgs.msg import WaypointProgress
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()

    class Monitor(Node):
        def __init__(self):
            super().__init__('lab2_bench_gate')
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OperationModeAvailability,
                                     '/system/operation_mode/availability', self.on_avail, 10)
            self.create_subscription(OperationModeState, '/api/operation_mode/state',
                                     self.on_mode, latched)
            self.create_subscription(MrmState, '/system/fail_safe/mrm_state', self.on_mrm, 10)
            self.create_subscription(Odometry, '/vehicle_bench_plant/odometry', self.on_odom, 10)
            self.create_subscription(WaypointProgress, '/planning/waypoint_path_publisher/progress',
                                     self.on_progress, 10)
            self.create_subscription(String, '/autosdv/actuator_node/debug/mode',
                                     self.on_actuator_mode, latched)
            self.MRM_NORMAL = MrmState.NORMAL
            self.AUTONOMOUS = OperationModeState.AUTONOMOUS
            self.reset()

        def reset(self):
            self.available = None
            self.mode = None
            self.mrm_since = None
            self.speed = 0.0
            self.speeds = []
            self.recording = False
            self.progress = None
            self.faults = []

        def on_avail(self, m):
            self.available = bool(m.autonomous)

        def on_mode(self, m):
            self.mode = int(m.mode)

        def on_mrm(self, m):
            if m.state != self.MRM_NORMAL and self.mrm_since is None:
                self.mrm_since = time.monotonic()

        def on_odom(self, m):
            self.speed = m.twist.twist.linear.x
            if self.recording:
                self.speeds.append(self.speed)

        def on_progress(self, m):
            self.progress = m

        def on_actuator_mode(self, m):
            if 'controller_fault' in m.data:
                self.faults.append(m.data)

    return rclpy, Monitor()


def run_route(rclpy, mon, env, workdir, route, extra, timeout):
    course_dir, v_ref, laps = course_info(route)
    name = Path(route).name
    bench = wc.Bench(f'gate_{name}', [f'course_dir:={course_dir}', 'seed:=21'] + extra,
                     workdir, env)
    result = {'route': name, 'v_ref': v_ref, 'laps': laps}
    try:
        if not wc.bring_up(rclpy, mon, bench):
            result.update(ok=False, why=['stack never became available (the SpeedPID may not '
                                         f'load; see {bench.logdir}/launch.log)'])
            return result
        problem = load_error(bench.logdir)
        if problem:
            result.update(ok=False, why=[f'the actuator refused your SpeedPID: {problem}'])
            return result
        mon.speeds, mon.recording = [], True
        if not wc.engage(rclpy, mon, env):
            result.update(ok=False, why=['engage refused'])
            return result
        t0 = time.monotonic()

        def over():
            p = mon.progress
            if mon.faults or mon.mrm_since is not None:
                return True
            if p is not None and p.finished and abs(mon.speed) < 0.02:
                return True
            return False

        stalled = False

        def stall():
            nonlocal stalled
            p = mon.progress
            if time.monotonic() - t0 > STALL_S and (p is None or p.progress < STALL_M):
                stalled = True
            return stalled

        wc.spin_until(rclpy, mon, lambda: over() or stall(), timeout)
        wc.spin_until(rclpy, mon, lambda: False, 1.5)   # catch a late fault or MRM
        mon.recording = False
        p = mon.progress
        finished = bool(p and p.finished)
        ok, why = judge(v_ref, mon.speeds, finished, int(p.laps_done) if p else 0, laps,
                        mon.mrm_since is not None, mon.faults, stalled,
                        abs(mon.speed) < 0.05, time.monotonic() - t0 >= timeout)
        moving = [s for s in mon.speeds if s > 0.1]
        result.update(ok=ok, why=why, seconds=round(time.monotonic() - t0, 1),
                      laps_done=int(p.laps_done) if p else 0,
                      median=round(statistics.median(moving), 3) if moving else 0.0,
                      peak=round(max(moving), 3) if moving else 0.0)
        return result
    finally:
        bench.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 usage='%(prog)s [--only ROUTES] [--timeout S] [launch args ...]')
    ap.add_argument('--only', default='', help='comma list; a name under config/courses or a dir')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--timeout', type=float, default=150.0, help='per route [s]')
    ap.add_argument('--workdir', default=str(REPO / 'tmp/lab2_bench_gate'))
    args, extra = ap.parse_known_args()
    if args.list:
        print('\n'.join(ROUTES))
        return 0
    routes = [r for r in args.only.split(',') if r] or ROUTES
    for r in routes:
        if not (Path(r).is_dir() or (COURSES_DIR / r).is_dir()):
            print(f'unknown route {r}: no such course directory')
            return 1
    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', '73')
    if env['ROS_DOMAIN_ID'] == '0':
        print('refusing to run on ROS_DOMAIN_ID 0 (shared host)')
        return 1
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    env.setdefault('LAB2_WEB_ADDR', '127.0.0.1:8091')
    workdir = Path(args.workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    print('launch args: ' + (' '.join(extra) or '(defaults)'), flush=True)
    rclpy, mon = make_monitor()
    results = []
    try:
        for r in routes:
            print(f'--- {r}', flush=True)
            res = run_route(rclpy, mon, env, workdir, r, extra, args.timeout)
            results.append(res)
            if res['ok']:
                detail = (f"{res['laps_done']}/{res['laps']} laps in {res['seconds']} s, median "
                          f"{res['median']} m/s, peak {res['peak']} m/s, no MRM, SpeedPID clean")
            else:
                detail = '; '.join(res['why'])
            print(f"{'PASS' if res['ok'] else 'FAIL'}  {res['route']}: {detail}", flush=True)
    finally:
        mon.destroy_node()
        rclpy.shutdown()
    (workdir / 'result.json').write_text(json.dumps(results, indent=2))
    fails = sum(1 for r in results if not r['ok'])
    print(f'\n{len(results) - fails}/{len(results)} routes passed'
          + ('  -> GATE PASSED' if not fails else '  -> GATE FAILED'))
    return fails


if __name__ == '__main__':
    sys.exit(main())
