"""`just lab2 run`'s helpers, without ROS: python3 -m pytest scripts/testing/lab2/test_lab2_run.py"""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lab2_run as lr  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
TMP = REPO / 'tmp' / 'test_lab2_run'


# ---- arguments

def test_parse_splits_settings_flags_and_launch_args():
    o = lr.parse_args(['--bench', 'config:=x.yaml', 'car:=B', 'team:=3', 'team_dir:=/t',
                       'seed:=21', '--camera', 'mock', '--timeout=90'])
    assert o.bench and not o.yes
    assert o.camera == 'mock' and o.timeout == 90.0
    assert o.config_args == ('config:=x.yaml', 'car:=B', 'team:=3', 'team_dir:=/t')
    assert o.launch_args == ('seed:=21',)
    # team:= is the number, for the recorder; the stack gets team_dir only
    assert lr.vehicle_config_args(o.config_args) == ['config:=x.yaml', 'car:=B', 'team_dir:=/t']


def test_yes_only_on_the_bench():
    with pytest.raises(lr.UsageError, match='bench only'):
        lr.parse_args(['--yes'])
    assert lr.parse_args(['--bench', '--yes']).yes


def test_camera_only_on_the_bench_and_checked():
    with pytest.raises(lr.UsageError):
        lr.parse_args(['--camera', 'mock'])
    with pytest.raises(lr.UsageError):
        lr.parse_args(['--bench', '--camera', 'webcam'])


def test_timeout_is_three_route_times():
    d = REPO / 'src/lab/autosdv_waypoint_lab/config/courses/lab2_uturn'
    t = lr.route_seconds(d)
    assert 28 < t < 34                      # 24.7 m at 0.8 m/s
    assert lr.run_timeout(d) == pytest.approx(3 * t)
    assert lr.run_timeout(d, 42.0) == 42.0


# ---- the run's state

OK_MON = {n: (0, 'fine', 100.0) for n in lr.MONITOR_STATUSES}


def snap(**kw):
    base = dict(now=100.0, engaged=True, pose_rx=99.9, pose_std=0.08, monitor=dict(OK_MON))
    base.update(kw)
    return lr.Snapshot(**base)


def test_driving_is_not_an_end():
    assert lr.end_reason(snap(speed=0.8), t_engage=90.0, timeout=93.0) is None


def test_finished_needs_rest_for_a_second():
    assert lr.end_reason(snap(finished=True, rest_since=99.5), 90.0, 93.0) is None
    e = lr.end_reason(snap(finished=True, rest_since=98.9), 90.0, 93.0)
    assert e.kind == 'finished' and not e.abort


def test_mrm_wins_over_a_finish():
    e = lr.end_reason(snap(finished=True, rest_since=90.0, mrm='operating EMERGENCY_STOP'), 90.0, 93.0)
    assert e.kind == 'mrm' and e.abort and 'EMERGENCY_STOP' in e.text


def test_disengage_before_and_after_the_finish():
    assert lr.end_reason(snap(engaged=False), 90.0, 93.0).kind == 'disengaged'
    assert lr.end_reason(snap(engaged=False, finished=True), 90.0, 93.0).kind == 'finished'


@pytest.mark.parametrize('kw,word', [
    (dict(pose_rx=99.0), 'no localized pose for'),
    (dict(pose_rx=None), 'no localized pose'),
    (dict(pose_std=0.9), 'uncertainty 0.90 m'),
    (dict(monitor={**OK_MON, 'localization_monitor: marker_fix':
                   (2, 'no marker fix for 5.2 s (limit 5.0 s)', 100.0)}), 'no marker fix for 5.2 s'),
    (dict(monitor={**OK_MON, 'localization_monitor: path_deviation': (2, '0.91 m from the path', 100.0)}),
     '0.91 m from the path'),
    (dict(monitor={n: (0, 'fine', 97.0) for n in lr.MONITOR_STATUSES}), 'silent'),
    (dict(monitor={}), 'no localization_monitor'),
])
def test_localization_loss_stops_the_run(kw, word):
    e = lr.end_reason(snap(**kw), 90.0, 93.0)
    assert e.kind == 'localization' and e.abort
    assert word in e.text


def test_timeout_and_dead_children():
    assert lr.end_reason(snap(), 0.0, 93.0).kind == 'timeout'
    assert lr.end_reason(snap(stack_alive=False, mrm='x'), 90.0, 93.0).kind == 'stack'
    assert lr.end_reason(snap(recorder_alive=False), 90.0, 93.0).kind == 'recorder'


def test_engage_refused_without_a_usable_pose():
    assert lr.engage_refusal(snap(engaged=False), True) is None
    why = lr.engage_refusal(snap(engaged=False, monitor={**OK_MON, 'localization_monitor: marker_fix':
                                                         (2, 'no marker fix yet', 100.0)}), True)
    assert 'no marker fix yet' in why
    assert 'not available' in lr.engage_refusal(snap(engaged=False), False)
    assert 'MRM' in lr.engage_refusal(snap(engaged=False, mrm='operating'), True)


def test_status_line_is_one_short_line():
    line = lr.status_line(snap(speed=0.79), 88.0, 9.84, 24.71, 'AUTONOMOUS')
    assert '\n' not in line and len(line) < 80
    assert '12.0 s' in line and '0.79 m/s' in line and '9.8/24.7 m' in line and 'NORMAL' in line


def test_integration_verdict(tmp_path):
    png = tmp_path / 'audit.png'
    png.write_bytes(b'x')
    good = {'check_ok': True, 'grade_valid': True, 'total': 45.0, 'max_total': 45, 'png': str(png),
            'localization': {'ok': True, 'summary': 's', 'failures': []}}
    ok, lines = lr.integration_verdict(lr.End('finished', 'stopped'), good)
    assert ok and all(ln.startswith('PASS') for ln in lines)
    ok, lines = lr.integration_verdict(lr.End('mrm', 'MRM'), dict(good, check_ok=False,
                                                                   check_reasons=['bench bag']))
    assert not ok and sum(ln.startswith('FAIL') for ln in lines) == 2
    ok, _ = lr.integration_verdict(lr.End('finished', 'x'), dict(good, localization=None))
    assert not ok


# ---- process groups

def test_child_stop_escalates_and_leaves_nothing():
    TMP.mkdir(parents=True, exist_ok=True)
    # a session that ignores SIGINT (as a hung node would); set -m gives each job a
    # process group of its own in the same session, as play_launch does for its nodes
    cmd = ['bash', '-c', 'set -m; trap "" INT; sleep 60 & sleep 60 & wait']
    c = lr.Child('t', cmd, TMP / 'child.log').start()
    time.sleep(0.5)
    assert c.alive() and len(c.remaining()) == 3      # bash, and each sleep
    said = []
    t0 = time.monotonic()
    ok = c.stop(steps=((signal.SIGINT, 1.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 2.0)), say=said.append)
    assert ok and c.remaining() == []
    assert any('SIGTERM' in s for s in said)
    assert time.monotonic() - t0 < 6


def test_child_stop_sigint_is_enough_for_a_polite_group():
    TMP.mkdir(parents=True, exist_ok=True)
    c = lr.Child('t', ['sleep', '60'], TMP / 'child2.log').start()
    said = []
    assert c.stop(say=said.append) and said == []
