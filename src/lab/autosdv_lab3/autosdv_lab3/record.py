"""`just lab3 record`: the mapping bag (week 1) or the run bag (week 2).

    lab3_record --car car_b --team 3 [--kind map|run] [--map DIR] [--name NAME]
                [--out-dir ~/lab3/bags]

Run it in a second terminal while the stack runs in the first (`just lab3 stack` in
week 1, `just lab3 run` in week 2). Ctrl-C stops the recording; a mapping bag is then
checked at once (`lab3_check_bag`), so a bag GLIM cannot use is found in the hall, not
at home.

Besides the topics, the bag carries a latched /lab3/record_meta (JSON: kind, car,
team, LiDAR, AutoSDV commit, and for a run the map folder's hash), also written beside
it as lab3_meta.json.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

from autosdv_lab3 import cars, mapdir

META_TOPIC = '/lab3/record_meta'
SIDECAR = 'lab3_meta.json'

# week 2: what grading reads besides the sensors — the localization, its NDT internals,
# the initializer, the route and the operation mode
RUN_EXTRA = [
    '/localization/kinematic_state',
    '/localization/pose_estimator/pose_with_covariance',
    '/localization/pose_estimator/transform_probability',
    '/localization/pose_estimator/nearest_voxel_transformation_likelihood',
    '/localization/pose_estimator/iteration_num',
    '/api/localization/initialization_state',
    '/api/routing/state',
    '/api/operation_mode/state',
    '/control/command/control_cmd',
    cars.BOARD_POSE_RUN,
    '/planning/mission_planning/route',   # each route's goal, to tell scripted from hand-set
    '/initialpose',                       # a pose set in RViz
    '/lab3/run_meta',                     # launch time and map hash (lab3_run_meta)
    '/lab3/route_events',                 # the legs lab3_route set
]


def topics_for(kind: str, sensor: str):
    base = [cars.LIDAR_TOPIC[sensor], cars.IMU_RAW, cars.IMU, '/tf_static', '/tf',
            cars.VELOCITY, cars.TWIST]
    if kind == 'map':
        return base + [cars.BOARD_POSE_MAPPING]
    return base + RUN_EXTRA


def build_meta(kind, car, team, sensor, map_dir=None):
    meta = {'lab': 'lab3', 'kind': kind, 'car': car, 'team': str(team), 'sensor': sensor,
            'lidar_topic': cars.LIDAR_TOPIC[sensor], 'autosdv': mapdir.git_rev(),
            'time': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    if map_dir:
        meta['map_dir'] = os.path.abspath(map_dir)
        meta['map_hash'] = mapdir.folder_hash(map_dir)
    return meta


def meta_publisher(meta_json: str):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('lab3_record')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(String, META_TOPIC, latched)
    node.create_timer(1.0, lambda: pub.publish(String(data=meta_json)))
    pub.publish(String(data=meta_json))
    return node


def main(argv=None):
    ap = argparse.ArgumentParser(prog='lab3_record', description='Record a Lab 3 bag.')
    ap.add_argument('--car', required=True, choices=sorted(cars.CAR_SENSOR))
    ap.add_argument('--team', required=True, help='team number')
    ap.add_argument('--kind', choices=('map', 'run'), default='map',
                    help='map: the week-1 mapping drive (default); run: the week-2 laps')
    ap.add_argument('--map', dest='map_dir', help='run: the map folder the stack loaded')
    ap.add_argument('--name', default='', help='bag name (default: kind, team and time)')
    ap.add_argument('--out-dir', default=os.path.expanduser('~/lab3/bags'))
    ap.add_argument('--no-check', action='store_true', help='skip the check after recording')
    args = ap.parse_args(argv)

    if args.kind == 'run' and not args.map_dir:
        print('lab3_record: a run bag needs --map DIR, the folder the stack loaded',
              file=sys.stderr)
        return 2
    sensor = cars.sensor_of(args.car)
    meta = build_meta(args.kind, args.car, args.team, sensor, args.map_dir)
    name = args.name or f'team{args.team}_{args.kind}_{time.strftime("%Y%m%d_%H%M%S")}'
    out = os.path.join(args.out_dir, name)
    if os.path.exists(out):
        print(f'lab3_record: {out} exists; pick another --name', file=sys.stderr)
        return 2
    os.makedirs(args.out_dir, exist_ok=True)

    node = meta_publisher(json.dumps(meta, sort_keys=True))
    import rclpy
    cmd = ['ros2', 'bag', 'record', '-o', out, *topics_for(args.kind, sensor), META_TOPIC]
    print(f'{args.kind} bag, car {args.car} ({sensor}), team {args.team} -> {out}')
    if args.kind == 'map':
        # GLIM adds a keyframe per 0.15 m driven, so a parked car contributes one scan
        # to the map: the board is filled in by driving past it, not by facing it
        print('drive: 10 s still facing the board, the room at <= 0.8 m/s closing the loop '
              'twice and passing the board slowly each time, back facing the board, 10 s '
              'still; then Ctrl-C')
    else:
        # the board pose that initializes the stack is not latched: a recorder started
        # after INITIALIZED cannot show the board did it
        print('run: start this before `just lab3 run`, so the bag holds the board '
              'initialization; then `just lab3 route`; Ctrl-C after the last lap')
    proc = subprocess.Popen(cmd, start_new_session=True)
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    try:
        while proc.poll() is None and not stop['now']:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        node.destroy_node()
        rclpy.try_shutdown()
    if not os.path.isdir(out):
        print('lab3_record: nothing was recorded', file=sys.stderr)
        return 1
    with open(os.path.join(out, SIDECAR), 'w') as f:
        json.dump(meta, f, indent=2, sort_keys=True)
    if args.kind == 'map' and not args.no_check:
        from autosdv_lab3 import check_bag
        print()
        return check_bag.main([out])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
