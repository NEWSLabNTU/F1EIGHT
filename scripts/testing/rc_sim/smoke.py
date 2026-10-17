#!/usr/bin/env python3
"""Smoke test of the interactive RC simulator: Autoware drives the straight and stops.

    ROS_DOMAIN_ID=73 flock tmp/bench.lock python3 scripts/testing/rc_sim/smoke.py
    ... --resources tmp/rc_sim_budget.csv --taskset 0-3        the laptop budget (Task 20)
    ... speed_controller_params:='{...}'        replaces the default coefficients (SPEED_CONTROLLER_PARAMS)

Starts `play_launch launch autosdv_rc_sim rc_sim.launch.xml rviz:=false` in its own
session (output in tmp/rc_sim_smoke_stack.log), waits until sim_pose_init has
initialized the localization, sets the route to the track's goal (the stop line),
waits until autonomous mode is available, engages, and watches /rc_sim/state.
PASS: the car exceeded 9 m/s (the lab profile peaks at 11), then came to rest
(|v| < 0.05 m/s for 2 s of simulated time) within STOP_TOLERANCE (1 m) of the stop line,
in the lane (|y| < 0.5 m),
with no MRM and no loss of autonomous availability after the engage (Task 18b). Everything
is stopped by process group: SIGINT, SIGTERM, SIGKILL. Refuses ROS_DOMAIN_ID 0.
"""
import argparse
import collections
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

REPO = Path(__file__).resolve().parents[3]
DIAG_LEVELS = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}

def reference_params():
    """The grader's reference coefficients, read from the package so they cannot drift."""
    path = REPO / 'src/sim/autosdv_rc_sim/config/reference_speed_controller.yaml'
    params = yaml.safe_load(path.read_text())
    return '{' + ', '.join(f'{k}: {str(v).lower() if isinstance(v, bool) else v}'
                           for k, v in sorted(params.items())) + '}'


SPEED_CONTROLLER_PARAMS = reference_params()
# How far from the stop line the interactive run may come to rest. The same controller stops
# within 0.5 m in `just sim grade`; measured interactive: 0.02 m short (Task 18c), so 1.0 m
# leaves room for simulated-clock races without hiding a regression (the hall-speed bug of
# Task 18c gave 28 m past).
STOP_TOLERANCE = 1.0


def stop_group(proc, label):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 15.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            print(f'{label}: still running after {sig.name}')


def drive(track, timeout, on_ready, stack):
    import rclpy
    from autoware_adapi_v1_msgs.msg import LocalizationInitializationState, MrmState, OperationModeState
    from autoware_adapi_v1_msgs.srv import ChangeOperationMode, SetRoutePoints
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Float64MultiArray
    from tier4_system_msgs.msg import DiagGraphStatus, DiagGraphStruct, OperationModeAvailability

    rclpy.init()
    node = Node('rc_sim_smoke')
    seen = {'loc': None, 'op': None, 'state': None}
    # Task 18b: from the engage on, any MRM or loss of autonomous availability is a failure
    flaps = {'armed': False, 'mrm': 0, 'unavailable': 0, 'last_mrm': None,
             'culprits': collections.Counter()}
    graph = {'names': None, 'status': None}   # the diagnostic graph, to name what went bad
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(LocalizationInitializationState, '/api/localization/initialization_state',
                             lambda m: seen.__setitem__('loc', m.state), latched)

    def on_avail(m):
        # the diagnostic graph's verdict (not the API's, which also folds in the engage
        # conditions): it is what flapped to STALE once a second before Task 18b
        if flaps['armed'] and not m.autonomous:
            x = (seen['state'] or [0.0, 0.0])[1]
            if x < track['stop_line_s'] - 3.0:   # the route legitimately ends at the goal
                flaps['unavailable'] += 1
                flaps.setdefault('first_x', x)
                flaps['culprits'].update(bad_diags())

    def bad_diags():
        # leaf diagnostics not OK in the latest graph status, as 'name (LEVEL)'
        names, status = graph['names'], graph['status']
        if names is None or status is None or len(names) != len(status.diags):
            return ['(diagnostic graph not seen)']
        return [f'{n} ({DIAG_LEVELS.get(d.level, d.level)})'
                for n, d in zip(names, status.diags) if d.level != 0]

    def on_mrm(m):
        if flaps['armed'] and m.state != MrmState.NORMAL:
            flaps['mrm'] += 1
            flaps['last_mrm'] = (m.state, m.behavior)

    node.create_subscription(OperationModeState, '/api/operation_mode/state',
                             lambda m: seen.__setitem__('op', m), latched)
    node.create_subscription(OperationModeAvailability, '/system/operation_mode/availability', on_avail, 10)
    node.create_subscription(MrmState, '/api/fail_safe/mrm_state', on_mrm, latched)
    node.create_subscription(DiagGraphStruct, '/diagnostics_graph/struct',
                             lambda m: graph.__setitem__('names', [d.name for d in m.diags]), latched)
    node.create_subscription(DiagGraphStatus, '/diagnostics_graph/status',
                             lambda m: graph.__setitem__('status', m), 10)
    node.create_subscription(Float64MultiArray, '/rc_sim/state',
                             lambda m: seen.__setitem__('state', list(m.data)), 10)
    deadline = time.monotonic() + timeout

    def spin_until(predicate, what):
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if predicate():
                return True
            if stack.poll() is not None:
                print(f'FAIL  the launch exited ({stack.returncode}) while waiting for {what}; '
                      'see tmp/rc_sim_smoke_stack.log')
                return False
        print(f'FAIL  timed out waiting for {what}')
        return False

    def call(client, request, what):
        while time.monotonic() < deadline:
            if stack.poll() is not None:
                print(f'FAIL  the launch exited ({stack.returncode}) during {what}')
                return False
            if client.wait_for_service(timeout_sec=2.0):
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
                result = future.result()
                if result is not None and result.status.success:
                    return True
            time.sleep(2.0)
        print(f'FAIL  {what} never succeeded')
        return False

    try:
        if not spin_until(lambda: seen['loc'] == LocalizationInitializationState.INITIALIZED,
                          'the localization to be initialized (sim_pose_init)'):
            return 1
        on_ready()
        route = SetRoutePoints.Request()
        route.header.frame_id = 'map'
        goal = track['goal_pose']
        route.goal.position.x, route.goal.position.y = float(goal['x']), float(goal['y'])
        route.goal.orientation.z = math.sin(goal['yaw'] / 2.0)
        route.goal.orientation.w = math.cos(goal['yaw'] / 2.0)
        if not call(node.create_client(SetRoutePoints, '/api/routing/set_route_points'), route, 'set route'):
            return 1
        if not spin_until(lambda: seen['op'] is not None and seen['op'].is_autonomous_mode_available,
                          'autonomous mode to be available'):
            return 1
        if not call(node.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous'),
                    ChangeOperationMode.Request(), 'engage'):
            return 1
        flaps['armed'] = True
        vmax, still_since, last = 0.0, None, None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if stack.poll() is not None:
                print(f'FAIL  the launch exited ({stack.returncode}) while driving')
                return 1
            if seen['state'] is None:
                continue
            last = seen['state']
            t, vx = last[0], last[4]
            vmax = max(vmax, vx)
            if vmax >= 9.0 and abs(vx) < 0.05:
                still_since = t if still_since is None else still_since
                if t - still_since >= 2.0:
                    break
            else:
                still_since = None
        if last is None:
            print('FAIL  no /rc_sim/state')
            return 1
        x, y = last[1], last[2]
        ok = (vmax >= 9.0 and still_since is not None
              and abs(x - track['stop_line_s']) <= STOP_TOLERANCE and abs(y) < 0.5)
        steady = flaps['mrm'] == 0 and flaps['unavailable'] == 0
        print(f"{'PASS' if ok and steady else 'FAIL'}  vmax {vmax:.2f} m/s, at rest at x {x:.2f} m "
              f"(stop line {track['stop_line_s']:.2f}), y {y:+.2f} m")
        if not steady:
            print(f"FAIL  after engage: {flaps['mrm']} MRM state changes (last {flaps['last_mrm']}), "
                  f"autonomous mode unavailable {flaps['unavailable']} times "
                  f"(first at x {flaps.get('first_x', 0.0):.1f} m)")
            for name, n in flaps['culprits'].most_common(5):
                print(f"      not OK at {n} of those: {name}")
        ok = ok and steady
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--track', default=str(REPO / 'src/sim/autosdv_rc_sim/tracks/straight_2_6_11'))
    ap.add_argument('--timeout', type=float, default=300.0, help='wall seconds for the whole run')
    ap.add_argument('--resources', default='', help='sample host resources into this CSV')
    ap.add_argument('--resource-seconds', type=float, default=120.0)
    ap.add_argument('--taskset', default='', help='pin the stack to these CPUs, e.g. 0-3')
    ap.add_argument('launch_args', nargs='*')
    args = ap.parse_args(argv)
    if os.environ.setdefault('ROS_DOMAIN_ID', '73') == '0':
        print('refusing ROS_DOMAIN_ID 0 (shared host)')
        return 2
    track_dir = Path(args.track).resolve()
    track = yaml.safe_load((track_dir / 'track.yaml').read_text())
    launch_args = list(args.launch_args)
    if not any(a.startswith('speed_controller_params:=') for a in launch_args):
        launch_args.append(f'speed_controller_params:={SPEED_CONTROLLER_PARAMS}')
    cmd = ['play_launch', 'launch', '--web-addr', '127.0.0.1:8093', 'autosdv_rc_sim',
           'rc_sim.launch.xml', 'rviz:=false', f'track_dir:={track_dir}', *launch_args]
    if args.taskset:
        cmd = ['taskset', '-c', args.taskset, *cmd]
    (REPO / 'tmp').mkdir(exist_ok=True)
    log = open(REPO / 'tmp/rc_sim_smoke_stack.log', 'w')
    stack = subprocess.Popen(cmd, cwd=REPO, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    sampler = None

    def start_sampler():
        nonlocal sampler
        if args.resources:
            sampler = subprocess.Popen(
                [sys.executable, str(REPO / 'scripts/profiling/host_resource_sampler.py'),
                 '-o', args.resources, '--pgid', str(os.getpgid(stack.pid)),
                 '--seconds', str(args.resource_seconds), '--label', 'rc_sim'],
                cwd=REPO, start_new_session=True)

    try:
        verdict = drive(track, args.timeout, start_sampler, stack)
        if sampler is not None:
            sampler.wait()
        return verdict
    finally:
        stop_group(sampler, 'resource sampler')
        stop_group(stack, 'rc_sim stack')
        log.close()


if __name__ == '__main__':
    sys.exit(main())
