#!/usr/bin/env python3
"""Lab 2 vehicle-bench gate: does the team's SpeedPID drive the graded route (lab2_uturn)?

Each route (by default the one graded route) starts a fresh vehicle bench (`just lab2 bench`: the car's actuator
with the SpeedPID plug-in, the EKF, the reference tracker, the MRM path, and a
plant model for the car), engages, waits for the run to finish and prints one
PASS or FAIL line. Exit status is the number of FAILs. A team has to pass
before it gets car time.

    just lab2 gate submission:=~/lab2/B12345678_lab_2_1   # lab2_speed + car file + calibration
    just lab2 gate speed_controller_class:=lab2_speed.speed_pid:SpeedPID \\
                   "speed_controller_params:={kp: 20.0, ki: 25.0}"
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
  localized  /localization/kinematic_state arrives (a stack whose localization
             never started, or stopped, is reported as that, not as a stall)
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
LOC_STALE_S = 2.0            # kinematic_state older than this at the end: localization stopped

# Verdict helpers are pure so the unit test can drive them without ROS.


def localization_problem(kin_first, kin_last, launched, now):
    """Why localization was not running, or None.

    kin_first, kin_last: monotonic times of the first and the latest
    /localization/kinematic_state (None: never); launched: when the bench was
    started; now: the end of the run (or of the bring-up).
    """
    if kin_first is None:
        return (f'localization never started: no /localization/kinematic_state in '
                f'{now - launched:.0f} s after launch (the EKF was never seeded or activated; '
                'see start_pose_init and ekf_localizer in the play_log)')
    if now - kin_last > LOC_STALE_S:
        return (f'localization stopped: the last /localization/kinematic_state was '
                f'{now - kin_last:.1f} s before the end of the run')
    return None


def judge(v_ref, speeds, finished, laps_done, laps, mrm, faults, stalled, at_rest, timed_out,
          localization=None):
    """Return (ok, [reasons it failed]) from what one run recorded.

    localization: localization_problem()'s reason, or None. It goes first, and a
    car that did not move or finish because of it is not reported again.
    """
    why = []
    if localization:
        why.append(localization)
        stalled, timed_out = False, False
    if faults:
        why.append(f'SpeedPID fault ({faults[0]})')
    if mrm:
        # `mrm` is True, or the monitor's one-line description of what raised it.
        why.append('MRM raised' + (f' ({mrm})' if isinstance(mrm, str) else ''))
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
    from autoware_adapi_v1_msgs.msg import DiagGraphStatus, DiagGraphStruct
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
            self.create_subscription(Odometry, '/localization/kinematic_state', self.on_kin, 10)
            self.create_subscription(WaypointProgress, '/planning/waypoint_path_publisher/progress',
                                     self.on_progress, 10)
            self.create_subscription(String, '/autosdv/actuator_node/debug/mode',
                                     self.on_actuator_mode, latched)
            self.create_subscription(DiagGraphStruct, '/diagnostics_graph/struct',
                                     self.on_struct, latched)
            self.create_subscription(DiagGraphStatus, '/diagnostics_graph/status',
                                     self.on_status, 10)
            self.MRM_NORMAL = MrmState.NORMAL
            self.MRM_STATES = {MrmState.MRM_OPERATING: 'operating', MrmState.MRM_SUCCEEDED: 'succeeded',
                               MrmState.MRM_FAILED: 'failed'}
            self.MRM_BEHAVIORS = {MrmState.EMERGENCY_STOP: 'EMERGENCY_STOP',
                                  MrmState.COMFORTABLE_STOP: 'COMFORTABLE_STOP',
                                  MrmState.PULL_OVER: 'PULL_OVER'}
            self.AUTONOMOUS = OperationModeState.AUTONOMOUS
            self.reset()

        def reset(self):
            self.available = None
            self.mode = None
            self.mrm_since = None
            self.mrm_info = None
            self.engaged_at = None
            self.avail_at = None
            self.avail_false_at = None
            self.diag_names = []
            self.diag_bad = {}           # leaf name -> (time last non-OK, message)
            self.speed = 0.0
            self.speeds = []
            self.recording = False
            self.progress = None
            self.faults = []
            self.kin_first = None
            self.kin_last = None

        def on_avail(self, m):
            self.available = bool(m.autonomous)
            self.avail_at = time.monotonic()
            if not self.available:
                self.avail_false_at = self.avail_at

        def on_struct(self, m):
            # the unit path each leaf hangs under, e.g. /adapi/mrm_request/delegate
            self.diag_names = [m.nodes[d.parent].path if d.parent < len(m.nodes) else d.name
                               for d in m.diags]

        def on_status(self, m):
            now = time.monotonic()
            for i, d in enumerate(m.diags):
                lvl = d.level[0] if isinstance(d.level, bytes) else int(d.level)   # a ROS `byte`
                if lvl != 0:
                    name = self.diag_names[i] if i < len(self.diag_names) else f'diag #{i}'
                    level = {1: 'WARN', 2: 'ERROR', 3: 'STALE'}.get(lvl, 'not OK')
                    self.diag_bad[name] = (now, f'{level}: {d.message}' if d.message else level)

        def describe_mrm(self, m, now):
            """What the fail-safe did and when, from the first MRM message."""
            what = (f"{self.MRM_STATES.get(m.state, m.state)} "
                    f"{self.MRM_BEHAVIORS.get(m.behavior, 'behavior ' + str(m.behavior))}")
            when = (f'{now - self.engaged_at:.1f} s after engage' if self.engaged_at is not None
                    else 'before engage')
            return f'{what}, {when}'

        def mrm_reason(self):
            """The MRM line plus the diagnostics that were not OK around it.

            The status message that carries the cause can arrive just after the
            MRM message, so this is read once the run is over, not in the callback.
            """
            if self.mrm_since is None:
                return None
            bad = [f'{n} {msg}' for n, (t, msg) in self.diag_bad.items()
                   if abs(t - self.mrm_since) < 2.0]
            return self.mrm_info + ('; not OK around it: ' + ', '.join(bad[:4]) if bad
                                    else '; no diagnostic was not OK (the availability message '
                                         'went false)')

        def on_mode(self, m):
            self.mode = int(m.mode)

        def on_mrm(self, m):
            if m.state != self.MRM_NORMAL and self.mrm_since is None:
                self.mrm_since = time.monotonic()
                self.mrm_info = self.describe_mrm(m, self.mrm_since)

        def on_odom(self, m):
            self.speed = m.twist.twist.linear.x
            if self.recording:
                self.speeds.append(self.speed)

        def on_kin(self, m):
            self.kin_last = time.monotonic()
            self.kin_first = self.kin_first or self.kin_last

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
        launched = time.monotonic()
        if not wc.bring_up(rclpy, mon, bench):
            loc = localization_problem(mon.kin_first, mon.kin_last, launched, time.monotonic())
            result.update(ok=False, why=[loc] if loc else [
                'stack never became available (the SpeedPID may not '
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
        mon.engaged_at = t0

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
        loc = None if finished else localization_problem(mon.kin_first, mon.kin_last, launched,
                                                         time.monotonic())
        ok, why = judge(v_ref, mon.speeds, finished, int(p.laps_done) if p else 0, laps,
                        mon.mrm_reason() or False, mon.faults, stalled,
                        abs(mon.speed) < 0.05, time.monotonic() - t0 >= timeout, loc)
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
