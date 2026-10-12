#!/usr/bin/env python3
"""Lab 2 preflight: is the running stack ready to engage?

Run it after `just coach vehicle` (or `just coach bench` / `just coach sim`) is
up, before the first engage:

    just coach preflight                 auto-detects vehicle, bench or sim
    just coach preflight --no-board      board rate is a WARN (board not placed yet)
    just coach preflight --mode vehicle --window 5

One line per check, PASS / WARN / FAIL, and exit status 1 if any FAIL:

  nodes        the nodes the lab stack needs are alive (the slim set, per mode)
  rates        LiDAR cloud, IMU, velocity_status, /perception/coach/board and
               /localization/kinematic_state against thresholds (LiDAR, IMU
               and velocity only where the mode has them)
  tf           base_link -> the LiDAR frame (vehicle), map -> base_link
  availability /system/operation_mode/availability has exactly one publisher,
               the graph converter (a second, always-"available" publisher can
               hide a failed check from engage and the MRM handler)
  mrm          /system/fail_safe/mrm_state is NORMAL
  mode         operation mode STOP, not engaged, not in transition
  graph        the diagnostic graph: autonomous available, and if not, the
               units that block it

The mode is detected from the node list: simple_planning_simulator -> sim,
vehicle_bench_plant -> bench, otherwise vehicle. Sensor checks apply to the
vehicle only; the bench's IMU and wheel speed come from its plant model and are
checked like the car's.
"""

import argparse
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

PASS, WARN, FAIL = 'PASS', 'WARN', 'FAIL'
lines = []


def out(level, name, detail):
    lines.append(level)
    print(f'{level}  {name}: {detail}', flush=True)


# ------------------------------------------------------------------ the set

COMMON_NODES = [
    '/control/vehicle_cmd_gate', '/control/trajectory_follower/controller_node_exe',
    '/control/autoware_operation_mode_transition_manager', '/control/external_cmd_selector',
    '/system/aggregator', '/system/converter', '/system/mrm_handler',
    '/system/mrm_emergency_stop_operator', '/system/mrm_comfortable_stop_operator',
    '/system/duplicated_node_checker', '/adapi/node/operation_mode', '/adapi/node/mrm_request',
    '/coach_tracker', '/robot_state_publisher',
] + [f'/system/topic_state_monitor_{n}' for n in (
    'coach_board', 'coach_state', 'control_command_control_cmd', 'localization_kinematic_state',
    'scenario_planning_trajectory', 'system_emergency_control_cmd',
    'trajectory_follower_control_cmd', 'transform_map_to_base_link',
    'vehicle_status_steering_status', 'vehicle_status_velocity_status')]
CAR_NODES = [
    '/autosdv/actuator_node', '/autosdv/steering_status_node',
    '/autosdv/control_mode_manager_node', '/autosdv/gear_manager_node',
    '/dead_reckon_odometry', '/twist2accel', '/sensing/imu/imu_corrector',
]
MODE_NODES = {
    'vehicle': CAR_NODES + ['/autosdv/velocity_report_node', '/board_tracking'],
    'bench': CAR_NODES + ['/vehicle_bench_plant', '/virtual_coach'],
    'sim': ['re:simple_planning_simulator', '/virtual_coach'],
}

# topic: (name, fail below [Hz], warn below [Hz], modes)
RATES = [
    ('lidar', None, 7.0, 9.0, ('vehicle',)),
    ('/sensing/imu/imu_data', 'IMU', 20.0, 50.0, ('vehicle', 'bench')),
    ('/vehicle/status/velocity_status', 'velocity_status', 10.0, 18.0, ('vehicle', 'bench')),
    ('/perception/coach/board', 'coach board', 8.0, 9.0, ('vehicle', 'bench', 'sim')),
    ('/localization/kinematic_state', 'kinematic_state', 10.0, 18.0, ('vehicle', 'bench', 'sim')),
]
LIDAR_TOPICS = ['/sensing/lidar/iv_points', '/sensing/lidar/velodyne_points',
                '/sensing/lidar/pointcloud_raw_ex', '/sensing/lidar/top/pointcloud_raw_ex']


class Preflight(Node):
    def __init__(self):
        super().__init__('lab2_preflight')
        self.counts = {}
        self.first = {}
        self.mrm = None
        self.opmode = None
        self.graph_struct = None
        self.graph_status = None

    def count(self, topic, msg_type):
        self.counts[topic] = 0

        def cb(raw, topic=topic):
            self.counts[topic] += 1
            if topic not in self.first:
                self.first[topic] = raw
        self.create_subscription(msg_type, topic, cb, qos_profile_sensor_data, raw=True)


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def node_names(node):
    return sorted({(ns.rstrip('/') + '/' + n) for n, ns in node.get_node_names_and_namespaces()})


def detect_mode(names):
    if any('simple_planning_simulator' in n for n in names):
        return 'sim'
    if '/vehicle_bench_plant' in names:
        return 'bench'
    return 'vehicle'


def check_nodes(names, mode):
    want = COMMON_NODES + MODE_NODES[mode]
    missing = []
    for w in want:
        if w.startswith('re:'):
            if not any(re.search(w[3:], n) for n in names):
                missing.append(w[3:])
        elif w not in names:
            missing.append(w)
    if missing:
        out(FAIL, 'nodes', f'{len(want) - len(missing)}/{len(want)} alive; missing {missing}')
    else:
        out(PASS, 'nodes', f'all {len(want)} required nodes alive')
    planners = [n for n in ('/board_pursuit_planner', '/speed_step_planner') if n in names]
    if not planners:
        out(WARN, 'nodes', 'no board_pursuit_planner or speed_step_planner: nothing publishes '
            '/planning/trajectory, so engage will be refused (launch_planner:=false?)')
    elif len(planners) > 1:
        out(FAIL, 'nodes', f'both {planners} run: two publishers of /planning/trajectory')
    else:
        out(PASS, 'nodes', f'trajectory from {planners[0]}')
    if mode == 'vehicle':
        lidar = [n for n in names if n.startswith('/sensing/lidar/')]
        out(PASS if lidar else FAIL, 'nodes', f'LiDAR driver: {lidar or "none under /sensing/lidar"}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', choices=['auto', 'vehicle', 'bench', 'sim'], default='auto')
    ap.add_argument('--window', type=float, default=3.0, help='rate sampling window [s]')
    ap.add_argument('--no-board', action='store_true',
                    help='a missing or slow board is a WARN, for before the board is placed')
    args = ap.parse_args()

    rclpy.init()
    node = Preflight()
    from autoware_adapi_v1_msgs.msg import MrmState, OperationModeState
    from tier4_system_msgs.msg import DiagGraphStatus, DiagGraphStruct
    import tf2_ros

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MrmState, '/system/fail_safe/mrm_state',
                             lambda m: setattr(node, 'mrm', m), 10)
    node.create_subscription(OperationModeState, '/api/operation_mode/state',
                             lambda m: setattr(node, 'opmode', m), latched)
    node.create_subscription(DiagGraphStruct, '/diagnostics_graph/struct',
                             lambda m: setattr(node, 'graph_struct', m), latched)
    node.create_subscription(DiagGraphStatus, '/diagnostics_graph/status',
                             lambda m: setattr(node, 'graph_status', m), 10)
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    spin_for(node, 2.0)   # discovery
    names = node_names(node)
    mode = detect_mode(names) if args.mode == 'auto' else args.mode
    print(f'Lab 2 preflight, mode {mode}'
          f'{" (detected)" if args.mode == "auto" else ""}, {len(names)} nodes', flush=True)
    if len(names) <= 1:
        out(FAIL, 'nodes', 'no ROS nodes visible: is the stack up, on this ROS_DOMAIN_ID '
            'and RMW (source scripts/env.sh)?')
        return 1

    check_nodes(names, mode)

    # ---- rates
    types = dict(node.get_topic_names_and_types())
    from rosidl_runtime_py.utilities import get_message
    plan = []
    for topic, label, fail_hz, warn_hz, modes in RATES:
        if mode not in modes:
            continue
        if topic == 'lidar':
            topic = next((t for t in LIDAR_TOPICS if t in types), None)
            label = f'LiDAR {topic}' if topic else 'LiDAR'
            if topic is None:
                out(FAIL, 'rates', f'no LiDAR cloud topic among {LIDAR_TOPICS}')
                continue
        if topic not in types:
            level = WARN if (args.no_board and 'board' in topic) else FAIL
            out(level, 'rates', f'{label}: {topic} has no publisher')
            continue
        node.count(topic, get_message(types[topic][0]))
        plan.append((topic, label, fail_hz, warn_hz))
    t0 = time.monotonic()
    spin_for(node, args.window)
    elapsed = time.monotonic() - t0
    lidar_frame = None
    for topic, label, fail_hz, warn_hz in plan:
        hz = node.counts[topic] / elapsed
        level = FAIL if hz < fail_hz else WARN if hz < warn_hz else PASS
        if level != PASS and args.no_board and 'board' in topic:
            level = WARN
        hint = ''
        if 'board' in topic and level != PASS:
            hint = ' (protocol 4.1: board at 4 m, at least 8 Hz)'
        out(level, 'rates', f'{label} {hz:.1f} Hz (fail < {fail_hz:g}, warn < {warn_hz:g}){hint}')
        if label.startswith('LiDAR') and topic in node.first:
            from rclpy.serialization import deserialize_message
            msg = deserialize_message(node.first[topic], get_message(types[topic][0]))
            lidar_frame = msg.header.frame_id

    # ---- TF
    pairs = [('map', 'base_link')]
    if mode == 'vehicle':
        if lidar_frame:
            pairs.insert(0, ('base_link', lidar_frame))
        else:
            out(FAIL, 'tf', 'no LiDAR message received, so no LiDAR frame to check')
    for parent, child in pairs:
        ok = tf_buffer.can_transform(parent, child, rclpy.time.Time())
        out(PASS if ok else FAIL, 'tf', f'{parent} -> {child} {"available" if ok else "MISSING"}')

    # ---- availability publishers
    pubs = node.get_publishers_info_by_topic('/system/operation_mode/availability')
    who = sorted(f'{p.node_namespace.rstrip("/")}/{p.node_name}' for p in pubs)
    if len(who) == 1 and who[0].endswith('converter'):
        out(PASS, 'availability', f'one publisher: {who[0]}')
    else:
        out(FAIL, 'availability', f'{len(who)} publishers {who}; want exactly the converter')

    # ---- MRM, operation mode (wait a little more if not heard yet)
    end = time.monotonic() + 2.0
    while (node.mrm is None or node.opmode is None or node.graph_status is None) \
            and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    names_mrm = {getattr(MrmState, k): k for k in
                 ('UNKNOWN', 'NORMAL', 'MRM_OPERATING', 'MRM_SUCCEEDED', 'MRM_FAILED')}
    if node.mrm is None:
        out(FAIL, 'mrm', 'no /system/fail_safe/mrm_state')
    else:
        s = names_mrm.get(node.mrm.state, node.mrm.state)
        out(PASS if node.mrm.state == MrmState.NORMAL else FAIL, 'mrm', f'state {s}')
    names_mode = {getattr(OperationModeState, k): k for k in
                  ('UNKNOWN', 'STOP', 'AUTONOMOUS', 'LOCAL', 'REMOTE')}
    om = node.opmode
    if om is None:
        out(FAIL, 'mode', 'no /api/operation_mode/state')
    else:
        m = names_mode.get(om.mode, om.mode)
        ok = om.mode == OperationModeState.STOP and not om.is_in_transition
        out(PASS if ok else FAIL, 'mode',
            f'{m}{" (in transition)" if om.is_in_transition else ""}; autonomous available: '
            f'{om.is_autonomous_mode_available}; Autoware control enabled: '
            f'{om.is_autoware_control_enabled}')

    # ---- diagnostic graph
    check_graph(node)

    nfail, nwarn = lines.count(FAIL), lines.count(WARN)
    print(f'--- {lines.count(PASS)} passed, {nwarn} warnings, {nfail} failed', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return 1 if nfail else 0


LEVELS = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}


def _byte(v):
    """A msg `byte` field arrives as bytes of length 1 in rclpy."""
    return v[0] if isinstance(v, (bytes, bytearray)) else int(v)


def check_graph(node):
    st, ss = node.graph_struct, node.graph_status
    if st is None or ss is None:
        out(FAIL, 'graph', 'no /diagnostics_graph/struct or status (aggregator not running?)')
        return
    if st.id != ss.id or len(ss.nodes) != len(st.nodes) or len(ss.diags) != len(st.diags):
        out(WARN, 'graph', 'struct and status disagree (graph restarted?); rerun')
        return
    n = len(st.nodes)
    names = [x.path for x in st.nodes] + [x.name for x in st.diags]
    level = [_byte(x.level) for x in ss.nodes] + [_byte(x.level) for x in ss.diags]
    msgs = [''] * n + [x.message for x in ss.diags]
    children = {}
    for link in st.links:
        children.setdefault(link.parent, []).append(link.child)
    for i, d in enumerate(st.diags):
        if n + i not in children.get(d.parent, []):
            children.setdefault(d.parent, []).append(n + i)
    root = next((i for i, x in enumerate(st.nodes) if x.path == '/autoware/modes/autonomous'), None)
    if root is None:
        out(FAIL, 'graph', 'no /autoware/modes/autonomous unit in the graph')
        return
    if level[root] == 0:
        out(PASS, 'graph', '/autoware/modes/autonomous OK: nothing blocks autonomous')
        return
    # the deepest non-OK units under autonomous: those whose children are all OK
    blockers, seen, stack = [], set(), [root]
    while stack:
        u = stack.pop()
        if u in seen:
            continue
        seen.add(u)
        bad = [c for c in children.get(u, []) if level[c] != 0]
        if bad:
            stack.extend(bad)
        elif level[u] != 0:
            blockers.append(u)
    out(FAIL, 'graph', f'/autoware/modes/autonomous is {LEVELS.get(level[root], level[root])}; '
        f'blocked by {len(blockers)} unit(s):')
    for u in sorted(blockers, key=lambda i: names[i]):
        extra = f' -- {msgs[u]}' if msgs[u] else ''
        print(f'        {LEVELS.get(level[u], level[u]):5s} {names[u]}{extra}', flush=True)


if __name__ == '__main__':
    sys.exit(main())
