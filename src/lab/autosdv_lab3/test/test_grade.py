"""The week-2 grader: each item on hand-built runs, then one synthetic bag end to end."""

import json
import math

import numpy as np
import pytest

from autosdv_lab3 import grade, mapdir
from autosdv_lab3.grade import Run

from fixtures import make_map


def _odom(segments, dt=0.05):
    """Rows t x y yaw v from (seconds, x, y, yaw) holds, joined by 1 s straight moves."""
    rows, t = [], 0.0
    prev = None
    for secs, x, y, yaw in segments:
        if prev is not None:
            for k in range(1, 21):
                f = k / 20
                rows.append((t, prev[0] + f * (x - prev[0]), prev[1] + f * (y - prev[1]), yaw, 0.5))
                t += dt
        for _ in range(int(secs / dt)):
            rows.append((t, x, y, yaw, 0.0))
            t += dt
        prev = (x, y)
    return np.array(rows, float)


# ------------------------------------------------------------------ pieces

def test_ramp():
    assert grade.ramp(0.05, 0.10, 0.40) == 1.0
    assert grade.ramp(0.25, 0.10, 0.40) == pytest.approx(0.5)
    assert grade.ramp(0.50, 0.10, 0.40) == 0.0


def test_stops_finds_each_hold():
    odom = _odom([(4.0, 0, 0, 0), (2.0, 1, 0, 0), (3.5, 2, 1, 1.0)])
    s = grade.stops(odom)
    assert [(round(x, 2), round(y, 2)) for _, _, x, y, _ in s] == [(0, 0), (2, 1)]
    assert s[1][4] == pytest.approx(1.0)


def test_autonomous_share():
    op = [(0.0, 1), (10.0, 2), (20.0, 1), (21.0, 2)]
    # from the first autonomous moment (10 s) to 30 s: 1 s of 20 not autonomous
    assert grade.autonomous_share(op, 5.0, 30.0) == pytest.approx(19 / 20)
    assert grade.autonomous_share([(0.0, 1)], 5.0, 30.0) is None


def test_chain_composes_static_tree():
    a = np.eye(4)
    a[:3, 3] = [0.1, 0, 0.2]
    b = np.eye(4)
    b[:3, 3] = [0.0, 0.05, 0.1]
    m = grade.chain([('base_link', 'kit', a), ('kit', 'robin_w', b), ('kit', 'imu', np.eye(4))],
                    'base_link', 'robin_w')
    assert m[:3, 3].tolist() == pytest.approx([0.1, 0.05, 0.3])
    assert grade.chain([('base_link', 'kit', a)], 'base_link', 'robin_w') is None


# ------------------------------------------------------------------ items

MARKS = [{'id': 'A', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0},
         {'id': 'B', 'x': 4.0, 'y': 0.0, 'yaw_deg': 90.0},
         {'id': 'C', 'x': 4.0, 'y': 4.0, 'yaw_deg': 180.0}]


def test_checkpoints_lines():
    r = Run(odom=_odom([(4.0, 0.05, 0.0, 0.0),                       # A: 5 cm, full
                        (4.0, 4.25, 0.0, math.radians(90)),          # B: 25 cm, half position
                        (4.0, 9.0, 9.0, 0.0)]))                      # C: never visited
    out = grade.grade_checkpoints(r, MARKS)
    a, b, c = out['marks']
    assert a['score'] == pytest.approx(1.0)
    assert b['score'] == pytest.approx(0.75, abs=1e-3)
    assert c['score'] == 0.0 and 'no stop' in c['note']
    assert out['score'] == pytest.approx(15 * 1.75 / 3, abs=0.01)


def test_checkpoints_take_the_first_stop():
    r = Run(odom=_odom([(4.0, 0.30, 0.0, 0.0), (4.0, 0.0, 0.0, 0.0)]))
    out = grade.grade_checkpoints(r, MARKS[:1])
    assert out['marks'][0]['position_m'] == pytest.approx(0.30, abs=0.01)


def _lap_run(hand_route_in_leg=None, no_arrival_leg=None, manual_leg=None, wait=0.0):
    """Four 20 s legs; the car sits at each leg's goal when it arrives."""
    goals = [(4.0, 0.0), (0.0, 0.0), (4.0, 0.0), (0.0, 0.0)]
    events, route_state, routes, op, odom = [], [], [], [(0.0, 1)], []
    for k, g in enumerate(goals):
        t = 10.0 + 20.0 * k
        events.append((t, {'event': 'leg_set', 'run': 'r1', 'lap': k // 2 + 1, 'leg': k % 2 + 1,
                           'legs': 2, 'goal': [g[0], g[1], 0.0]}))
        routes.append((t - 0.5, g[0], g[1]))
        route_state.append((t - 0.4, 2))
        op.append((t + 1.0 + wait, 2))
        if manual_leg == k + 1:
            op.append((t + 3.0, 1))
            op.append((t + 15.0, 2))
        if hand_route_in_leg == k + 1:
            routes.append((t + 5.0, 2.0, 2.0))
        if no_arrival_leg != k + 1:
            route_state.append((t + 18.0, 3))
            op.append((t + 18.5, 1))
        for i in range(400):
            tt = t + i * 0.05
            f = float(np.clip((tt - t - wait) / (18.0 - wait), 0.0, 1.0))
            start = goals[k - 1] if k else (0.0, 0.0)
            odom.append((tt, start[0] + f * (g[0] - start[0]), start[1] + f * (g[1] - start[1]),
                         0.0, 0.3 if tt >= t + wait else 0.0))
    return Run(events=events, route_state=sorted(route_state), routes=routes, op=op,
               odom=np.array(odom), end=100.0)


def test_laps_all_four_legs():
    out, graded = grade.grade_laps(_lap_run(), [])
    assert out['score'] == 15.0 and len(graded) == 4


@pytest.mark.parametrize('kw,leg,why', [
    ({'hand_route_in_leg': 2}, 2, 'a route set by hand'),
    ({'no_arrival_leg': 3}, 3, 'did not arrive'),
    ({'manual_leg': 1}, 1, 'not under autonomous control'),
])
def test_laps_a_failed_leg_costs_one_leg(kw, leg, why):
    out, _ = grade.grade_laps(_lap_run(**kw), [])
    assert out['score'] == pytest.approx(15.0 - 3.75)
    bad = [x for x in out['legs'] if not x['ok']]
    assert [x['graded_leg'] for x in bad] == [leg] and why in bad[0]['problems']


def test_laps_the_wait_before_engage_is_not_counted():
    # measured in the RC simulator: STOP for 5 s after each leg is set, car standing
    out, _ = grade.grade_laps(_lap_run(wait=5.0), [])
    assert out['score'] == 15.0, out


def test_laps_takeover_from_the_sheet():
    out, _ = grade.grade_laps(_lap_run(), [4])
    assert out['score'] == pytest.approx(11.25)
    assert 'takeover (TA sheet)' in out['legs'][3]['problems']


def test_laps_only_the_first_four_legs_count():
    r = _lap_run(no_arrival_leg=1)
    r.events.append((95.0, {'event': 'leg_set', 'run': 'r2', 'lap': 1, 'leg': 1, 'legs': 2,
                            'goal': [4.0, 0.0, 0.0]}))
    out, graded = grade.grade_laps(r, [])
    assert len(graded) == 4 and out['score'] == pytest.approx(11.25)


def _init_run(init_after=12.0, rviz=False, board_off=0.0):
    ndt = np.array([(init_after + 0.5, 1.0 + board_off, 0.0, 0.0)])
    return Run(run_meta={'launch_time': 1000.0},
               init_states=[(init_after, 1000.0 + init_after, grade.INITIALIZED)],
               board=[(init_after - 1.0, 1.0, 0.0, 0.0)], ndt=ndt,
               initialposes=[1.0] if rviz else [])


def test_init_passes_and_fails():
    assert grade.grade_init(_init_run())['score'] == 5.0
    assert grade.grade_init(_init_run(init_after=40.0))['score'] == 0.0
    assert grade.grade_init(_init_run(rviz=True))['score'] == 0.0
    assert grade.grade_init(_init_run(board_off=1.0))['score'] == 0.0
    r = _init_run()
    r.run_meta = {}
    assert grade.grade_init(r)['score'] is None
    # recorder started after the stack initialized: the latched state arrives at once,
    # stamped before the bag began, and no board pose precedes it
    late = _init_run()
    late.start, late.init_states, late.board = 20.0, [(20.0, 1012.0, grade.INITIALIZED)], []
    late.ndt = np.array([(20.5, 1.0, 0.0, 0.0)])
    out = grade.grade_init(late)
    assert out['score'] is None and 'recorder' in out['reason']


def test_localized_checks():
    r = _lap_run()
    _, graded = grade.grade_laps(r, [])
    t = np.arange(10.0, 90.0, 0.1)
    xs = np.interp(t, r.odom[:, 0], r.odom[:, 1])
    r.ndt = np.c_[t, xs, np.zeros_like(t), np.zeros_like(t)]
    r.nvtl = np.c_[t, np.full_like(t, 2.0)]
    r.init_states = [(1.0, 0.0, grade.INITIALIZED)]
    out = grade.grade_localized(r, graded, 1.3, 0.05)
    assert out['score'] == 10.0, out
    r.ndt[300, 2] = 0.8                                     # one 0.8 m sideways jump
    r.nvtl[:100, 1] = 1.0                                   # 12.5 % under the threshold
    out = grade.grade_localized(r, graded, 1.3, 0.2)
    assert out['score'] == 0.0 + 2.5                        # only no-reinitialization holds
    assert out['checks']['no_pose_jump']['jumps'] >= 1


def test_map_hashes(tmp_path):
    team = make_map(tmp_path / 'team')
    ta = make_map(tmp_path / 'ta', 'narrow')
    h = mapdir.folder_hash
    assert grade.grade_map(Run(run_meta={'map_hash': h(team)}), team, [ta])['score'] == 0.0
    out = grade.grade_map(Run(run_meta={'map_hash': h(ta)}), team, [ta])
    assert out['score'] == -10.0 and out['map'] == 'fallback'
    assert grade.grade_map(Run(run_meta={'map_hash': 'x'}), team, [ta])['score'] is None


# ------------------------------------------------------------------ a bag, end to end

rosbag2_py = pytest.importorskip('rosbag2_py')


def _write_bag(path, map_points, offset=0.0):
    """A run bag of four legs on the synthetic room, with a LiDAR that sees the map
    from where the localization says the car is, plus `offset` metres of error."""
    from autoware_adapi_v1_msgs.msg import (LocalizationInitializationState,
                                            OperationModeState, RouteState)
    from autoware_planning_msgs.msg import LaneletRoute, LaneletSegment
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Float32, Header, String
    from tf2_msgs.msg import TFMessage

    lidar = '/sensing/lidar/iv_points'
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for name, typ in [(grade.ODOM, 'nav_msgs/msg/Odometry'),
                      (grade.NDT_POSE, 'geometry_msgs/msg/PoseWithCovarianceStamped'),
                      (grade.NVTL, 'std_msgs/msg/Float32'),
                      (grade.INIT_STATE, 'autoware_adapi_v1_msgs/msg/LocalizationInitializationState'),
                      (grade.ROUTE_STATE, 'autoware_adapi_v1_msgs/msg/RouteState'),
                      (grade.OP_MODE, 'autoware_adapi_v1_msgs/msg/OperationModeState'),
                      (grade.ROUTE, 'autoware_planning_msgs/msg/LaneletRoute'),
                      (grade.RUN_META, 'std_msgs/msg/String'), (grade.EVENTS, 'std_msgs/msg/String'),
                      ('/tf_static', 'tf2_msgs/msg/TFMessage'),
                      (grade.cars.BOARD_POSE_RUN, 'geometry_msgs/msg/PoseWithCovarianceStamped'),
                      (lidar, 'sensor_msgs/msg/PointCloud2')]:
        w.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr'))
    ns = lambda t: int((1000.0 + t) * 1e9)  # noqa: E731

    tf = TFMessage()
    tr = TransformStamped()
    tr.header.frame_id, tr.child_frame_id = 'base_link', 'robin_w'
    tr.transform.translation.x, tr.transform.translation.z = 0.15, 0.25
    tr.transform.rotation.w = 1.0
    tf.transforms.append(tr)
    w.write('/tf_static', serialize_message(tf), ns(0.0))
    w.write(grade.RUN_META, serialize_message(String(data=json.dumps(
        {'launch_time': 1000.0, 'map_hash': 'h'}))), ns(0.0))
    st = LocalizationInitializationState(state=grade.INITIALIZED)
    st.stamp.sec = 1008
    board = PoseWithCovarianceStamped()
    board.pose.pose.position.x, board.pose.pose.position.y = 2.0, -1.0
    board.pose.pose.orientation.w = 1.0
    w.write(grade.cars.BOARD_POSE_RUN, serialize_message(board), ns(7.0))
    w.write(grade.INIT_STATE, serialize_message(st), ns(8.0))
    w.write(grade.OP_MODE, serialize_message(OperationModeState(mode=1)), ns(0.0))

    # a rectangle, 2 m in from the room's walls: legs alternate between two corners
    corners = [(2.0, -1.0), (6.0, -1.0), (6.0, 3.0), (2.0, 3.0)]
    goals = [(6.0, 3.0), (2.0, -1.0), (6.0, 3.0), (2.0, -1.0)]
    t = 10.0
    for k, g in enumerate(goals):
        route = LaneletRoute()
        route.goal_pose.position.x, route.goal_pose.position.y = g
        route.segments = [LaneletSegment()]
        w.write(grade.ROUTE, serialize_message(route), ns(t - 0.5))
        w.write(grade.ROUTE_STATE, serialize_message(RouteState(state=2)), ns(t - 0.4))
        w.write(grade.EVENTS, serialize_message(String(data=json.dumps(
            {'event': 'leg_set', 'run': 'r', 'lap': k // 2 + 1, 'leg': k % 2 + 1, 'legs': 2,
             'goal': [g[0], g[1], 0.0]}))), ns(t))
        w.write(grade.OP_MODE, serialize_message(OperationModeState(mode=2)), ns(t + 0.5))
        path = corners[:3] if k % 2 == 0 else corners[2:] + corners[:1]
        legs_t = np.linspace(0.0, 1.0, 81)
        seg = np.array(path, float)
        lens = np.r_[0, np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))]
        for i, f in enumerate(legs_t):
            tt = t + 0.5 + i * 0.2
            x = float(np.interp(f * lens[-1], lens, seg[:, 0]))
            y = float(np.interp(f * lens[-1], lens, seg[:, 1]))
            o = Odometry()
            o.pose.pose.position.x, o.pose.pose.position.y = x, y
            o.pose.pose.orientation.w = 1.0
            o.twist.twist.linear.x = 0.5
            w.write(grade.ODOM, serialize_message(o), ns(tt))
            p = PoseWithCovarianceStamped()
            p.pose.pose = o.pose.pose
            w.write(grade.NDT_POSE, serialize_message(p), ns(tt))
            w.write(grade.NVTL, serialize_message(Float32(data=2.0)), ns(tt))
            if i % 5 == 0:
                # the scan the LiDAR would see from (x + offset, y), in its own frame
                rel = map_points - np.array([x + offset + 0.15, y, 0.25])
                rel = rel[np.linalg.norm(rel[:, :2], axis=1) < 6.0]
                cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='robin_w'), rel.tolist())
                w.write(lidar, serialize_message(cloud), ns(tt))
        t_arr = t + 0.5 + 80 * 0.2
        w.write(grade.ROUTE_STATE, serialize_message(RouteState(state=3)), ns(t_arr))
        w.write(grade.OP_MODE, serialize_message(OperationModeState(mode=1)), ns(t_arr + 0.3))
        t = t_arr + 2.0
    del w


@pytest.mark.parametrize('offset,resid_ok', [(0.0, True), (0.4, False)])
def test_grade_bag_end_to_end(tmp_path, offset, resid_ok):
    from reflective_pose_core.pointcloud_io import read_cloud
    team = make_map(tmp_path / 'team')
    pts = np.asarray(read_cloud(f'{team}/{mapdir.PCD}').points, float)
    bag = tmp_path / 'run'
    _write_bag(bag, pts, offset)
    out = tmp_path / 'report.yaml'
    import argparse
    args = argparse.Namespace(map=team, run=str(bag), checkpoints=None, marks=None, sheet=None,
                              fallback_map=None, nvtl_threshold=1.3, no_residual=False,
                              output=str(out))
    rep = grade.grade(args)
    items = {i['item']: i for i in rep['items']}
    assert items['board initialization']['score'] == 5.0
    assert items['autonomous laps']['score'] == 15.0
    loc = items['staying localized']
    assert loc['checks']['residual']['ok'] is resid_ok, loc
    assert loc['score'] == (10.0 if resid_ok else 7.5)
    # the bag's map hash ('h') is neither map: the TA decides, so no total
    assert items['fallback map']['score'] is None and rep['total'] is None
