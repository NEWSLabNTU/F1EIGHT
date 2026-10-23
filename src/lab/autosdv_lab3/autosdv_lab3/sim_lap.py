"""One lap of the team's loop in the planning simulator, headless.

    lab3_sim_lap MAP_DIR [--timeout S] [--keep-log] [--web-addr 127.0.0.1:8095]

Runs AutoSDV's RC simulator (`just sim rc`: Autoware in planning-simulator mode, the
car's own vehicle interface on simulated hardware, the rc_sim plant, the AutoSDV
vehicle model) on the team's map, rviz off. The simulator takes its map as
track_dir and its start pose from track_dir/track.yaml, so the run directory holds
the team's three map files (symlinked) beside a generated track.yaml.

Then: initial pose at the route start (/api/localization/initialize, which also
moves the plant through /initialpose3d), the loop route (`route.loop_route`: a goal
just behind the start, waypoints round the loop, driven as two or more legs:
route.py says why), engage, and wait for the route state ARRIVED within a timeout
scaled to the loop length, sampling the simulated pose all the way. A failure names which: no route, never engaged, stuck, off lane,
timeout, or the launch died; and keeps the play_launch log.

The stack runs in its own session and is stopped by process group only (SIGINT,
SIGTERM, SIGKILL), so nothing outlives the lap. Set ROS_DOMAIN_ID when other ROS
work shares the host.
"""

import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml

from autosdv_lab3 import mapdir
from autosdv_lab3.route import AdApi, MapError, load_loop, loop_route

WEB_ADDR = '127.0.0.1:8095'
STARTUP_TIMEOUT = 300.0    # wall s for Autoware to come up and accept the initial pose
LAP_SPEED = 0.3            # m/s: the slowest average speed that still counts as driving
LAP_MARGIN = 60.0          # s added to length / LAP_SPEED
STUCK_AFTER = 30.0         # simulated s without 0.3 m of progress, while engaged
OFF_LANE_TOL = 0.05        # m outside the lanelet polygons before it counts


@dataclass
class LapResult:
    ok: bool
    reason: str
    value: str = ''
    wall_s: float = 0.0
    lap_s: Optional[float] = None    # simulated s from engage to arrival
    run_dir: str = ''
    samples: int = 0
    extra: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {'ok': self.ok, 'reason': self.reason, 'value': self.value,
                'wall_s': round(self.wall_s, 1),
                'lap_s': None if self.lap_s is None else round(self.lap_s, 1),
                'run_dir': self.run_dir, 'samples': self.samples, **self.extra}


def _repo_tmp() -> Path:
    from autosdv_lab3.lanelet_check import repo_root
    root = repo_root()
    return (root / 'tmp') if root else Path.cwd() / 'tmp'


def prepare_run_dir(map_dir: str, route, run_dir: Path) -> Path:
    """run_dir/map: the team's files, and the track.yaml the plant reads its start from."""
    track = run_dir / 'map'
    track.mkdir(parents=True, exist_ok=True)
    for name in (mapdir.LANELET, mapdir.PCD, mapdir.PROJECTOR):
        os.symlink(os.path.abspath(os.path.join(map_dir, name)), track / name)
    pose = lambda p: {'x': round(p[0], 4), 'y': round(p[1], 4), 'yaw': round(p[2], 5)}  # noqa: E731
    (track / 'track.yaml').write_text(
        '# lab3_sim_lap: the loop route start and goal for the rc_sim plant\n'
        + yaml.safe_dump({'start_pose': pose(route.start), 'goal_pose': pose(route.goal),
                          'length': round(route.length, 3)}, sort_keys=False))
    return track


def stop_group(proc: subprocess.Popen) -> None:
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 20.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            pass


def run_lap(map_dir: str, timeout: Optional[float] = None, keep_log: bool = False,
            web_addr: str = WEB_ADDR, log=print, start=None) -> LapResult:
    t_wall = time.monotonic()
    try:
        loop = load_loop(map_dir)
        route = loop_route(loop, start=start)
    except MapError as e:
        return LapResult(False, 'no route', str(e))
    lap_timeout = timeout if timeout is not None else LAP_MARGIN + loop.length / LAP_SPEED
    run_dir = _repo_tmp() / 'lab3_sim_lap' / time.strftime('%Y%m%d_%H%M%S')
    track = prepare_run_dir(map_dir, route, run_dir)
    log_path = run_dir / 'stack.log'
    cmd = ['play_launch', 'launch', '--web-addr', web_addr, '--log-dir', str(run_dir / 'play_log'),
           'autosdv_rc_sim', 'rc_sim.launch.xml', 'rviz:=false', f'track_dir:={track}',
           'planning_profile:=rc']   # Lab 3 drives with the RC planning profile
    log(f'sim lap: {loop.length:.1f} m loop, {len(route.legs)} route(s), '
        f'{len(route.waypoints)} waypoints, lap timeout {lap_timeout:.0f} s; log {log_path}')
    cwd = _repo_tmp().parent if (_repo_tmp().parent / 'versions.yaml').is_file() else run_dir
    with open(log_path, 'w') as out:
        stack = subprocess.Popen(cmd, cwd=cwd, stdout=out, stderr=subprocess.STDOUT,
                                 start_new_session=True)   # its own process group
    try:
        res = _drive(loop, route, stack, lap_timeout, log)
    except Exception as e:   # noqa: BLE001 - reported as the lap's failure, the stack still stops
        res = LapResult(False, 'error', f'{type(e).__name__}: {e}')
    finally:
        stop_group(stack)
    res.wall_s = time.monotonic() - t_wall
    res.run_dir = str(run_dir)
    if res.ok and not keep_log:
        shutil.rmtree(run_dir, ignore_errors=True)
        res.run_dir = ''
    return res


def _drive(loop, route, stack, lap_timeout, log) -> LapResult:
    import rclpy
    t0 = time.monotonic()
    step = lambda what: log(f'  {time.monotonic() - t0:6.1f} s  {what}')  # noqa: E731
    from rclpy.node import Node
    rclpy.init()
    node = Node('lab3_sim_lap')
    api = AdApi(node)
    from autoware_planning_msgs.msg import Trajectory
    traj = {}
    node.create_subscription(Trajectory, '/planning/scenario_planning/trajectory',
                             lambda m: traj.__setitem__('msg', m), 1)

    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tier4_system_msgs.msg import DiagGraphStatus, DiagGraphStruct
    graph = {}
    node.create_subscription(DiagGraphStruct, '/diagnostics_graph/struct',
                             lambda m: graph.__setitem__('names', [d.name for d in m.diags]),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(DiagGraphStatus, '/diagnostics_graph/status',
                             lambda m: graph.__setitem__('status', m), 1)

    def why_stopped(goal):
        """What holds the car: diagnostics at ERROR (an MRM), or where the trajectory stops."""
        names, status = graph.get('names'), graph.get('status')
        if names and status is not None and len(names) == len(status.diags):
            lv = lambda d: d.level[0] if isinstance(d.level, (bytes, bytearray)) else int(d.level)  # noqa: E731
            errors = [n for n, d in zip(names, status.diags) if lv(d) == 2]
            if errors:
                hint = (' (the planning validator rejects any trajectory curving tighter than '
                        'radius 1 m: widen the curves)' if any('trajectory_validation' in n for n in errors) else '')
                return '; emergency stop on ' + ', '.join(errors) + hint
        m = traj.get('msg')
        if m is not None and m.points and min(q.longitudinal_velocity_mps for q in m.points) < -0.01:
            return ('; the planned trajectory reverses (Autoware\'s start planner backing out of a '
                    'start pose it judged misaligned), which the car does not follow')
        for q in (m.points if m is not None else []):
            p = q.pose.position
            if abs(q.longitudinal_velocity_mps) < 0.01 and math.hypot(p.x - goal[0], p.y - goal[1]) > 1.0:
                return (f'; the planned trajectory stops at ({p.x:.2f}, {p.y:.2f}), short of the goal '
                        '(a curve the planner cannot fit inside the lane? widen it)')
        return '; no trajectory' if m is None or not m.points else ''

    died = lambda: stack.poll() is not None  # noqa: E731
    try:
        # sim_pose_init initializes at the origin once the stack is up; then we place the car
        if not api.wait_for(lambda: api.loc_state == api.LocState.INITIALIZED, STARTUP_TIMEOUT, died):
            return LapResult(False, 'launch died' if died() else 'timeout',
                             'the simulator never initialized its localization')
        step('localization initialized; placing the car at the route start')
        ok, msg = api.initialize(route.start)
        if not ok:
            return LapResult(False, 'no route', f'initial pose refused: {msg}')
        if not api.wait_for(lambda: (p := api.pose()) is not None
                            and math.hypot(p[0] - route.start[0], p[1] - route.start[1]) < 0.1,
                            30.0, died):
            return LapResult(False, 'no route', 'the simulated car never reached the start pose')

        samples, off_lane = 0, None
        t_engage = None
        progress = {'s': None, 'travel': 0.0, 'last_move': None}

        def sample():
            nonlocal samples, off_lane
            p = api.pose()
            if p is None:
                return
            samples += 1
            if off_lane is None and not loop.contains(p[0], p[1], OFF_LANE_TOL):
                off_lane = p
            s = loop.project(p[0], p[1])
            t = api.odom.header.stamp.sec + 1e-9 * api.odom.header.stamp.nanosec
            if progress['s'] is None:
                progress.update(s=s, last_move=t)
            else:
                ds = (s - progress['s']) % loop.length
                if 0.3 <= ds < 0.5 * loop.length:     # forward progress (wrap-safe)
                    progress['travel'] += ds
                    progress.update(s=s, last_move=t)
            return t

        deadline = None
        for i, leg in enumerate(route.legs):
            ok, msg = api.set_route(leg)
            step(f'route {i + 1}/{len(route.legs)}: {"set" if ok else "refused: " + msg}')
            if not ok:
                return LapResult(False, 'no route', f'route {i + 1} refused: {msg}', samples=samples)
            if not api.wait_for(lambda: api.op is not None and api.op.is_autonomous_mode_available,
                                60.0, died):
                return LapResult(False, 'never engaged', 'autonomous mode never became available',
                                 samples=samples)
            ok, msg = api.engage()
            step(f'engage: {msg if not ok else "autonomous"}')
            if not ok:
                return LapResult(False, 'never engaged', f'change_to_autonomous refused: {msg}',
                                 samples=samples)
            if t_engage is None:
                deadline = time.monotonic() + lap_timeout
                t_engage = sample()
            while not api.arrived():
                api.spin(0.05)
                if api.keep_engaged():
                    step('re-engaged (the mode had dropped to STOP)')
                t = sample()
                if died():
                    return LapResult(False, 'launch died', f'play_launch exited ({stack.returncode})',
                                     samples=samples)
                if off_lane is not None:
                    return LapResult(False, 'off lane',
                                     f'left the lanelets at ({off_lane[0]:.2f}, {off_lane[1]:.2f})',
                                     samples=samples)
                if t is not None and progress['last_move'] is not None and t - progress['last_move'] > STUCK_AFTER:
                    p = api.pose()
                    return LapResult(False, 'stuck', f'no progress for {STUCK_AFTER:.0f} s at '
                                     f'({p[0]:.2f}, {p[1]:.2f}), {progress["travel"]:.1f} m driven'
                                     + why_stopped(leg.goal), samples=samples)
                if time.monotonic() > deadline:
                    return LapResult(False, 'timeout', f'not arrived in {lap_timeout:.0f} s; '
                                     f'{progress["travel"]:.1f} of {loop.length:.1f} m driven',
                                     samples=samples)
        t_arrive = sample()
        step(f'arrived; {progress["travel"]:.1f} m driven')
        lap_s = (t_arrive - t_engage) if (t_arrive is not None and t_engage is not None) else None
        travel = progress['travel']
        if travel < 0.8 * loop.length:
            return LapResult(False, 'no route', f'arrived after only {travel:.1f} of {loop.length:.1f} m: '
                             'the route did not go round the loop', samples=samples, lap_s=lap_s)
        return LapResult(True, 'arrived', f'{travel:.1f} m in {lap_s:.0f} s sim' if lap_s else f'{travel:.1f} m',
                         samples=samples, lap_s=lap_s, extra={'travel_m': round(travel, 2)})
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Drive one lap of a Lab 3 loop in the planning simulator.')
    ap.add_argument('map_dir')
    ap.add_argument('--timeout', type=float, default=None,
                    help=f'wall s from engage to arrival (default {LAP_MARGIN:g} + length / {LAP_SPEED:g} m/s)')
    ap.add_argument('--keep-log', action='store_true', help='keep the run directory after a pass')
    ap.add_argument('--web-addr', default=WEB_ADDR, help="play_launch's web UI address")
    args = ap.parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        print('note: ROS_DOMAIN_ID is 0; set another when other ROS work shares the host', file=sys.stderr)
    res = run_lap(args.map_dir, args.timeout, args.keep_log, args.web_addr)
    print(f"{'PASS' if res.ok else 'FAIL'}  {res.reason}: {res.value} "
          f"(wall {res.wall_s:.0f} s{', log ' + res.run_dir if res.run_dir else ''})")
    return 0 if res.ok else 1


if __name__ == '__main__':
    sys.exit(main())
