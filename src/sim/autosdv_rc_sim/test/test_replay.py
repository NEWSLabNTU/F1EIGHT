"""Replay: picks the right row in time; publishes the trace's pose; marks violations.

Only the tests that need ROS skip without it (importorskip inside them); the violation
marks are plain Python and run everywhere.
"""
import os
from pathlib import Path
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


def test_markers_are_red_in_map_at_s(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from visualization_msgs.msg import Marker
    from autosdv_rc_sim.replay_node import Replay
    from rclpy.parameter import Parameter
    rows = [tuple(dict(t=0.01 * i, s=0.0, y=0.0, yaw=0.0, target=0.0, measured=0.0, speed=0.0,
                       pwm=370, steer_pwm=489, mode='active', kappa_r=0.0, slip_r=0.0, ax=0.0,
                       delta=0.0, emergency=False, phase='run')[c] for c in COLUMNS)
            for i in range(3)]
    Trace('profile', 'nominal', rows).write_csv(tmp_path / 't.csv')
    (tmp_path / 'v.yaml').write_text(yaml.safe_dump([{'s': 12.5, 'text': 'hold_rmse_6: 0.4 (limit 0.3)'}]))
    os.environ.setdefault('ROS_DOMAIN_ID', '73')
    rclpy.init()
    try:
        node = Replay(parameter_overrides=[Parameter('trace', value=str(tmp_path / 't.csv')),
                                           Parameter('violations', value=str(tmp_path / 'v.yaml'))])
        sent = []
        node.pub_marks.publish = sent.append
        node.publish_violations(str(tmp_path / 'v.yaml'))
        marks = sent[0].markers
        assert [m.id for m in marks] == [0, 1]
        assert [m.type for m in marks] == [Marker.SPHERE, Marker.TEXT_VIEW_FACING]
        assert all(m.header.frame_id == 'map' and m.pose.position.x == 12.5 for m in marks)
        assert all(m.color.r == 1.0 and m.color.g == 0.0 and m.color.a == 1.0 for m in marks)
        assert 'hold_rmse_6' in marks[1].text
        # visible at the view's default zoom (10 px/m): a sphere of about 1.5 m, text 1.5 m
        assert marks[0].scale.x >= 1.5 and marks[1].scale.z >= 1.5
        assert marks[1].pose.position.z > marks[0].pose.position.z + marks[0].scale.z / 2
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_labels_do_not_overprint(tmp_path):
    pytest.importorskip('rclpy')
    from autosdv_rc_sim.replay_node import mark_markers
    marks = [{'s': 5.0, 'text': 'a'}, {'s': 5.0, 'text': 'b'}, {'s': 9.0, 'text': 'c'}]
    labels = [m for m in mark_markers(marks) if m.type == 9]
    ys = [m.pose.position.y for m in labels]
    assert len(set(ys)) == 3 and min(ys) > 2.0     # beside the lane, one row each


def test_car_glyph_and_readout():
    pytest.importorskip('rclpy')
    from visualization_msgs.msg import Marker
    from autosdv_rc_sim.replay_node import car_markers
    arrow, text = car_markers(dict(speed=5.234, target=6.0, pwm=412, steer_pwm=489, slip_r=0.031))
    assert arrow.type == Marker.ARROW and arrow.header.frame_id == 'base_link'
    assert arrow.scale.x >= 2.0 and arrow.color.b == 1.0          # far larger than the 0.4 m model
    assert text.type == Marker.TEXT_VIEW_FACING and text.header.frame_id == 'base_link'
    assert abs(text.pose.position.y) >= 2.0                        # beside the lane
    for part in ('5.23 m/s', 'cmd 6.00', 'PWM 412', 'steer PWM 489', 'slip 0.031'):
        assert part in text.text


def test_rviz_declutters_and_shows_the_lane():
    cfg = (Path(__file__).parents[1] / 'config' / 'rc_sim_replay.rviz').read_text()
    assert 'PolarGridDisplay' not in cfg          # the stock 10 m rings: removed, not just disabled
    assert '/rc_sim/replay/car' in cfg and 'road_lanelets: true' in cfg


def test_rviz_follows_the_car():
    cfg = yaml.safe_load((Path(__file__).parents[1] / 'config' / 'rc_sim_replay.rviz').read_text())
    assert cfg['Visualization Manager']['Views']['Current']['Target Frame'] == 'base_link'


def _submission(tmp_path):
    d = tmp_path / 'sub'
    d.mkdir()
    (d / 'speed_controller_params.yaml').write_text('kp_speed: 7.0\n')
    return d


def test_view_variant_defaults_to_first_graded(tmp_path, monkeypatch):
    seen = {}
    monkeypatch.setattr(grade, 'view', lambda csv, track, v: seen.update(csv=str(csv)) or 0)
    rc = grade.main([str(_submission(tmp_path)), '--variant', 'low_mu', '--view',
                     '--out', str(tmp_path / 'o')])
    assert rc == 0 and seen['csv'].endswith('low_mu_profile.csv')


def test_view_variant_not_graded_is_a_clear_error(tmp_path, capsys):
    rc = grade.main([str(_submission(tmp_path)), '--variant', 'low_mu', '--view',
                     '--view-variant', 'nominal', '--out', str(tmp_path / 'o')])
    assert rc == 2 and 'was not graded' in capsys.readouterr().err


def test_view_without_play_launch(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv('PATH', str(tmp_path))
    assert grade.view('t.csv', '.', 'v.yaml') == 2
    assert 'play_launch not found' in capsys.readouterr().err


@pytest.mark.parametrize('sig', ['SIGTERM', 'SIGHUP'])
def test_view_stops_its_stack_when_grade_is_signalled(tmp_path, sig):
    import signal
    import subprocess
    import sys
    fake = tmp_path / 'play_launch'
    child = tmp_path / 'child.pid'
    # a launcher with a child of its own; without the handler grade dies and leaves both running
    fake.write_text(f"#!/bin/sh\nsleep 300 &\necho $! > {child}\nwait\n")
    fake.chmod(0o755)
    env = dict(os.environ, PATH=f"{tmp_path}{os.pathsep}{os.environ['PATH']}",
               PYTHONPATH=os.pathsep.join(sys.path))   # the checkout conftest put first
    code = 'import sys; from autosdv_rc_sim import grade; sys.exit(grade.view("t.csv", ".", "v.yaml"))'
    proc = subprocess.Popen([sys.executable, '-c', code], env=env)
    try:
        deadline = time.monotonic() + 10.0
        while not (child.exists() and child.read_text().strip()):
            assert proc.poll() is None and time.monotonic() < deadline, 'fake play_launch never started'
            time.sleep(0.05)
        pid = int(child.read_text())
        proc.send_signal(getattr(signal, sig))
        assert proc.wait(timeout=60.0) == 0
        deadline = time.monotonic() + 5.0
        while True:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                break
            assert time.monotonic() < deadline, f'{sig} left the replay stack running'
            time.sleep(0.05)
    finally:
        proc.kill()
        if child.exists() and child.read_text().strip():
            try:
                os.killpg(os.getpgid(int(child.read_text())), signal.SIGKILL)  # a failed run's stack
            except ProcessLookupError:
                pass
