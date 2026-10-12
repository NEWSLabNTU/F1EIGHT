#!/usr/bin/env python3
"""Lab 2 (waypoint) preflight: is the running stack ready to engage?

Run it after `just lab2 vehicle` (or `just lab2 bench`) is up, with the car in
the start box, before the first engage:

    just lab2 preflight                  auto-detects vehicle or bench
    just lab2 preflight --mode vehicle --window 5

One line per check, PASS / WARN / FAIL, and exit status 1 if any FAIL:

  nodes        the nodes the waypoint stack needs are alive, per mode
  rates        camera image and IMU (vehicle), velocity_status, IMU,
               kinematic_state, the trajectory and path_tracker's command
  fixes        the AR-tag localizer has accepted marker fixes recently (the
               car sees a marker from the start box)
  start        the localized pose is near the course's start box (from
               waypoint_path_publisher's progress: distance to the path, and
               progress near zero)
  tf           map -> base_link; base_link -> the camera's optical frame (vehicle)
  availability /system/operation_mode/availability has exactly one publisher,
               the graph converter
  mrm          /system/fail_safe/mrm_state is NORMAL
  mode         operation mode STOP, not engaged, not in transition
  graph        the diagnostic graph: autonomous available, or the units that
               block it

The mode is detected from the node list: vehicle_bench_plant -> bench,
otherwise vehicle. The availability, MRM, mode and graph checks are the coach
preflight's (preflight.py), shared.
"""

import argparse
import os
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import preflight as pf  # noqa: E402
from preflight import FAIL, PASS, WARN, out  # noqa: E402

COMMON_NODES = [
    '/control/vehicle_cmd_gate', '/control/path_tracker',
    '/control/autoware_operation_mode_transition_manager', '/control/external_cmd_selector',
    '/system/aggregator', '/system/converter', '/system/mrm_handler',
    '/system/mrm_emergency_stop_operator', '/system/mrm_comfortable_stop_operator',
    '/system/duplicated_node_checker', '/adapi/node/operation_mode', '/adapi/node/mrm_request',
    '/robot_state_publisher', '/planning/waypoint_path_publisher',
    '/localization/localization_monitor', '/localization/pose_twist_fusion_filter/ekf_localizer',
    '/localization/pose_twist_fusion_filter/start_pose_init', '/localization/twist2accel',
    '/localization/twist_estimator/gyro_odometer',
    '/localization/twist_estimator/vehicle_velocity_converter',
    '/map/lanelet2_map_loader', '/map/map_projection_loader', '/sensing/imu/imu_corrector',
    '/autosdv/actuator_node', '/autosdv/steering_status_node',
    '/autosdv/control_mode_manager_node', '/autosdv/gear_manager_node',
] + [f'/system/topic_state_monitor_{n}' for n in (
    'control_command_control_cmd', 'localization_kinematic_state',
    'scenario_planning_trajectory', 'system_emergency_control_cmd',
    'trajectory_follower_control_cmd', 'transform_map_to_base_link',
    'vehicle_status_steering_status', 'vehicle_status_velocity_status')]
MODE_NODES = {
    'vehicle': ['/autosdv/velocity_report_node', '/sensing/camera/zedxm/zedxm',
                '/localization/pose_estimator/ar_tag_based_localizer'],
    'bench': ['/vehicle_bench_plant', '/localization/pose_estimator/virtual_marker_localizer'],
}
# nodes that must NOT run: they would be a second publisher of a lab topic
FORBIDDEN = ['/control/trajectory_follower/controller_node_exe', '/dead_reckon_odometry',
             '/board_pursuit_planner', '/speed_step_planner']

# topic: (label, fail below [Hz], warn below [Hz], modes)
IMAGE = '/sensing/camera/zedxm/rgb/color/rect/image'
RATES = [
    (IMAGE, 'camera image', 8.0, 12.0, ('vehicle',)),
    ('/sensing/imu/imu_data', 'IMU', 20.0, 50.0, ('vehicle', 'bench')),
    ('/vehicle/status/velocity_status', 'velocity_status', 10.0, 18.0, ('vehicle', 'bench')),
    ('/localization/kinematic_state', 'kinematic_state', 20.0, 40.0, ('vehicle', 'bench')),
    ('/planning/trajectory', 'trajectory', 5.0, 8.0, ('vehicle', 'bench')),
    ('/control/trajectory_follower/control_cmd', 'path_tracker command', 15.0, 25.0,
     ('vehicle', 'bench')),
]
FIX = '/localization/pose_estimator/pose_with_covariance'
PROGRESS = '/planning/waypoint_path_publisher/progress'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', choices=['auto', 'vehicle', 'bench'], default='auto')
    ap.add_argument('--window', type=float, default=3.0, help='rate sampling window [s]')
    args = ap.parse_args()

    rclpy.init()
    node = pf.Preflight()
    from autoware_adapi_v1_msgs.msg import MrmState, OperationModeState
    from autosdv_lab_msgs.msg import WaypointProgress
    from geometry_msgs.msg import PoseWithCovarianceStamped
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
    fixes = []
    progress = []
    node.create_subscription(PoseWithCovarianceStamped, FIX,
                             lambda m: fixes.append(time.monotonic()), pf.qos_profile_sensor_data)
    node.create_subscription(WaypointProgress, PROGRESS, lambda m: progress.append(m), 10)
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    pf.spin_for(node, 2.0)
    names = pf.node_names(node)
    mode = ('bench' if '/vehicle_bench_plant' in names else 'vehicle') \
        if args.mode == 'auto' else args.mode
    print(f'Lab 2 (waypoint) preflight, mode {mode}'
          f'{" (detected)" if args.mode == "auto" else ""}, {len(names)} nodes', flush=True)
    if len(names) <= 1:
        out(FAIL, 'nodes', 'no ROS nodes visible: is the stack up, on this ROS_DOMAIN_ID '
            'and RMW (source scripts/env.sh)?')
        return 1

    want = COMMON_NODES + MODE_NODES[mode]
    missing = [w for w in want if w not in names]
    out(FAIL if missing else PASS, 'nodes',
        f'{len(want) - len(missing)}/{len(want)} required nodes alive'
        + (f'; missing {missing}' if missing else ''))
    extra = [f for f in FORBIDDEN if f in names]
    if extra:
        out(FAIL, 'nodes', f'{extra} running: a second publisher of a lab topic '
            '(a coach-lab launch still up?)')

    types = dict(node.get_topic_names_and_types())
    from rosidl_runtime_py.utilities import get_message
    plan = []
    for topic, label, fail_hz, warn_hz, modes in RATES:
        if mode not in modes:
            continue
        if topic not in types:
            out(FAIL, 'rates', f'{label}: {topic} has no publisher')
            continue
        node.count(topic, get_message(types[topic][0]))
        plan.append((topic, label, fail_hz, warn_hz))
    t0 = time.monotonic()
    pf.spin_for(node, max(args.window, 3.0))
    elapsed = time.monotonic() - t0
    cam_frame = None
    for topic, label, fail_hz, warn_hz in plan:
        hz = node.counts[topic] / elapsed
        level = FAIL if hz < fail_hz else WARN if hz < warn_hz else PASS
        out(level, 'rates', f'{label} {hz:.1f} Hz (fail < {fail_hz:g}, warn < {warn_hz:g})')
        if topic == IMAGE and topic in node.first:
            from rclpy.serialization import deserialize_message
            cam_frame = deserialize_message(node.first[topic],
                                            get_message(types[topic][0])).header.frame_id

    # ---- marker fixes and the start box
    recent = [t for t in fixes if time.monotonic() - t < elapsed + 0.5]
    if not recent:
        out(FAIL, 'fixes', f'no marker fix in {elapsed:.0f} s: no marker in view from the start '
            'box, the EKF not seeded (start_pose_init), or the camera/localizer down')
    else:
        rate = len(recent) / elapsed
        out(PASS if rate >= 2.0 else WARN, 'fixes', f'{rate:.1f} accepted marker fixes/s')
    if not progress:
        out(FAIL, 'start', f'no {PROGRESS}')
    else:
        p = progress[-1]
        near = p.distance_to_path < 0.3 and (p.progress < 0.5 or p.progress > p.lap_length - 0.5)
        out(PASS if near and not p.finished else FAIL, 'start',
            f'{p.distance_to_path:.2f} m from the path, {p.progress:.2f} m along it'
            f'{", run already finished (just lab2 restart-run)" if p.finished else ""}'
            f'{"" if near else " -- is the car in the start box? (just lab2 reinit)"}')

    # ---- TF
    pairs = [('map', 'base_link')]
    if mode == 'vehicle':
        if cam_frame:
            pairs.append(('base_link', cam_frame))
        else:
            out(FAIL, 'tf', 'no camera image received, so no camera frame to check')
    for parent, child in pairs:
        ok = tf_buffer.can_transform(parent, child, rclpy.time.Time())
        out(PASS if ok else FAIL, 'tf', f'{parent} -> {child} {"available" if ok else "MISSING"}')

    # ---- the shared checks
    pubs = node.get_publishers_info_by_topic('/system/operation_mode/availability')
    who = sorted(f'{p.node_namespace.rstrip("/")}/{p.node_name}' for p in pubs)
    if len(who) == 1 and who[0].endswith('converter'):
        out(PASS, 'availability', f'one publisher: {who[0]}')
    else:
        out(FAIL, 'availability', f'{len(who)} publishers {who}; want exactly the converter')
    end = time.monotonic() + 2.0
    while (node.mrm is None or node.opmode is None or node.graph_status is None) \
            and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.mrm is None:
        out(FAIL, 'mrm', 'no /system/fail_safe/mrm_state')
    else:
        names_mrm = {getattr(MrmState, k): k for k in
                     ('UNKNOWN', 'NORMAL', 'MRM_OPERATING', 'MRM_SUCCEEDED', 'MRM_FAILED')}
        out(PASS if node.mrm.state == MrmState.NORMAL else FAIL, 'mrm',
            f'state {names_mrm.get(node.mrm.state, node.mrm.state)}')
    om = node.opmode
    if om is None:
        out(FAIL, 'mode', 'no /api/operation_mode/state')
    else:
        ok = om.mode == OperationModeState.STOP and not om.is_in_transition
        names_mode = {getattr(OperationModeState, k): k for k in
                      ('UNKNOWN', 'STOP', 'AUTONOMOUS', 'LOCAL', 'REMOTE')}
        out(PASS if ok else FAIL, 'mode',
            f'{names_mode.get(om.mode, om.mode)}{" (in transition)" if om.is_in_transition else ""}; '
            f'autonomous available: {om.is_autonomous_mode_available}')
    pf.check_graph(node)

    nfail, nwarn = pf.lines.count(FAIL), pf.lines.count(WARN)
    print(f'--- {pf.lines.count(PASS)} passed, {nwarn} warnings, {nfail} failed', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return 1 if nfail else 0


if __name__ == '__main__':
    sys.exit(main())
