"""Replay: picks the right row in time; publishes the trace's pose; marks violations.

Only the tests that need ROS skip without it (importorskip inside them); the violation
marks are plain Python and run everywhere.
"""
import os
import time

import pytest
import yaml

from autosdv_rc_sim import grade
from autosdv_rc_sim.lockstep import COLUMNS, Trace
from autosdv_rc_sim.track import default_track_dir, load_track


def test_index_at():
    pytest.importorskip('rclpy')
    from autosdv_rc_sim.replay_node import index_at
    times = [0.0, 0.01, 0.02, 0.03]
    assert [index_at(times, e) for e in (-1.0, 0.0, 0.015, 0.02, 9.0)] == [0, 0, 1, 2, 3]


def test_violations_are_placed_where_they_happened(tmp_path):
    profile = load_track(default_track_dir())
    report = {'variants': {'nominal': {'checks': {
        'hold_rmse_6': {'pass': False, 'value': 0.4, 'limit': 0.3},
        'stop_error': {'pass': False, 'value': 6.0, 'limit': 4.1},
        'launch_slip': {'pass': True, 'value': 0.1, 'limit': 0.5}}}}}
    n = grade.write_violations(report, profile, 'nominal', tmp_path / 'v.yaml')
    marks = yaml.safe_load((tmp_path / 'v.yaml').read_text())
    assert n == 2
    hold6 = [seg for seg in profile.segments if seg.kind == 'hold'][1]
    assert marks[0]['s'] == pytest.approx(hold6.s0) and 'hold_rmse_6' in marks[0]['text']
    assert marks[1]['s'] == pytest.approx(profile.stop_line_s)


def test_publishes_the_trace_pose(tmp_path):
    os.environ.setdefault('ROS_DOMAIN_ID', '73')
    rclpy = pytest.importorskip('rclpy')
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from autosdv_rc_sim.replay_node import Replay

    rows = []
    for i in range(300):
        r = dict(t=0.01 * i, s=0.02 * i, y=0.01, yaw=0.0, target=2.0, measured=1.9, speed=2.0,
                 pwm=390, steer_pwm=489, mode='active', kappa_r=0.0, slip_r=0.0, ax=0.0,
                 delta=0.0, emergency=False, phase='run')
        rows.append(tuple(r[c] for c in COLUMNS))
    Trace('profile', 'nominal', rows).write_csv(tmp_path / 't.csv')
    rclpy.init()
    try:
        node = Replay(parameter_overrides=[Parameter('trace', value=str(tmp_path / 't.csv'))])
        probe = rclpy.create_node('replay_probe')
        got = []
        probe.create_subscription(Odometry, '/localization/kinematic_state',
                                  lambda m: got.append(m.pose.pose.position.x), 10)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(probe)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and (not got or got[-1] <= 2.0):
            ex.spin_once(timeout_sec=0.01)
        assert got and got == sorted(got) and got[-1] > 2.0
        node.destroy_node()
        probe.destroy_node()
    finally:
        rclpy.try_shutdown()
