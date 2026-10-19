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


def test_timeout_is_three_route_times(tmp_path):
    import math
    import yaml
    d = REPO / 'src/lab/autosdv_waypoint_lab/config/courses/lab2_uturn'
    spec = yaml.safe_load((d / 'course.yaml').read_text())
    length = 2 * spec['uturn']['leg'] + math.pi * spec['uturn']['radius']
    t = lr.route_seconds(d)
    assert t == pytest.approx(length / spec['v_ref'], rel=0.01)
    assert lr.run_timeout(d) == pytest.approx(max(lr.MIN_TIMEOUT, lr.TIMEOUT_FACTOR * t))
    assert lr.run_timeout(d, 42.0) == 42.0
    # a route long enough that three driving times exceed the floor
    (tmp_path / 'course.yaml').write_text('path: path.csv\nv_ref: 0.8\nlaps: 1\n')
    (tmp_path / 'path.csv').write_text('0.0,0.0\n30.0,0.0\n')
    assert lr.route_seconds(tmp_path) == pytest.approx(37.5)
    assert lr.run_timeout(tmp_path) == pytest.approx(3 * 37.5)


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
    line = lr.status_line(snap(speed=0.79), 88.0, 2.64, 7.45, 'AUTONOMOUS')
    assert '\n' not in line and len(line) < 80
    assert '12.0 s' in line and '0.79 m/s' in line and '2.6/7.5 m' in line and 'NORMAL' in line


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


# ---- the simple command: `just lab2 run team:=3` / `just lab2 run DIR`

def team_dir(home, name='team3', controller='B12345678_lab_2_1'):
    d = home / name
    pkg = d / controller / 'lab2_speed'
    (pkg / 'lab2_speed').mkdir(parents=True)
    (pkg / 'package.xml').write_text('<package/>')
    (pkg / 'lab2_speed' / 'speed_pid.py').write_text('class SpeedPID: pass\n')
    (pkg / 'config').mkdir()
    (pkg / 'config' / 'speed_pid_car.yaml').write_text('kp: 1.0\n')
    return d


def test_parse_takes_team_number_dir_and_force_flags():
    o = lr.parse_args(['team:=3'])
    assert o.team == '3' and o.team_pos is None and o.config_args == ()
    o = lr.parse_args(['/x/team3', '--car'])
    assert o.team_pos == '/x/team3' and o.car
    o = lr.parse_args(['team:=/x/t', 'team_number:=7'])
    assert o.team == '/x/t' and o.team_number == '7'
    with pytest.raises(lr.UsageError, match='one team'):
        lr.parse_args(['team:=3', '/x/team3'])
    with pytest.raises(lr.UsageError, match='car:=A'):
        lr.parse_args(['A'])                        # a car letter is car:=, and the TA's


def test_team_number_resolves_under_lab2_home(tmp_path):
    d = team_dir(tmp_path)
    args, note = lr.resolve_team(lr.parse_args(['team:=3']), {}, tmp_path)
    assert args == [f'team_dir:={d}', 'team:=3']


def test_team_directory_gives_its_number(tmp_path):
    d = team_dir(tmp_path, 'team12')
    args, _ = lr.resolve_team(lr.parse_args([str(d)]), {}, tmp_path)
    assert args == [f'team_dir:={d}', 'team:=12']
    args, _ = lr.resolve_team(lr.parse_args([f'team:={d}']), {}, tmp_path)
    assert args == [f'team_dir:={d}', 'team:=12']


def test_other_directory_needs_a_team_number(tmp_path):
    d = team_dir(tmp_path, 'red_team')
    with pytest.raises(lr.UsageError, match='team_number:='):
        lr.resolve_team(lr.parse_args([str(d)]), {}, tmp_path)
    args, _ = lr.resolve_team(lr.parse_args([str(d), 'team_number:=5']), {}, tmp_path)
    assert args == [f'team_dir:={d}', 'team:=5']
    args, note = lr.resolve_team(lr.parse_args([str(d)]), {'team': '4'}, tmp_path)
    assert args == [f'team_dir:={d}', 'team:=4'] and 'settings' in note


def test_missing_or_broken_team_directory_is_a_usage_error(tmp_path):
    with pytest.raises(lr.UsageError, match='team3'):
        lr.resolve_team(lr.parse_args(['team:=3']), {}, tmp_path)
    (tmp_path / 'team4').mkdir()                    # no controller inside
    with pytest.raises(lr.UsageError, match='lab_2_1'):
        lr.resolve_team(lr.parse_args(['team:=4']), {}, tmp_path)


def test_no_team_falls_back_to_the_settings_or_asks(tmp_path):
    assert lr.resolve_team(lr.parse_args([]), {'team_dir': '/t', 'team': '3'}, tmp_path) == ([], '')
    with pytest.raises(lr.UsageError, match='just lab2 run team:='):
        lr.resolve_team(lr.parse_args([]), {}, tmp_path)


def test_team_with_an_explicit_controller_is_the_number(tmp_path):
    """integration-test passes team:=0 beside speed_controller_class:= / submission:=."""
    for extra in ('speed_controller_class:=a.b:C', 'submission:=/s', 'team_dir:=/t'):
        o = lr.parse_args(['--bench', 'team:=0', extra])
        assert lr.resolve_team(o, {}, tmp_path) == ([], '')
        assert 'team:=0' in o.config_args


def test_gate_verdict_passes_a_clean_finish():
    ok, line = lr.gate_verdict(lr.End('finished', 'stopped'), 'lab2_uturn', 0.8, 1,
                               [0.0, 0.8, 0.8, 0.79, 0.0], laps_done=1, faults=[],
                               stalled=False, at_rest=True)
    assert ok and line.startswith('PASS  lab2_uturn')


@pytest.mark.parametrize('end, kw, word', [
    (lr.End('mrm', 'MRM: operating EMERGENCY_STOP'), {}, 'MRM'),
    (lr.End('timeout', 'not finished after 60 s'), {}, 'did not finish'),
    (lr.End('stalled', 'car did not move'), {'stalled': True}, 'did not move'),
    (lr.End('localization', 'localization lost: x'), {}, 'localization lost'),
    (lr.End('stack', 'the stack exited'), {}, 'the stack exited'),
    (lr.End('finished', 'stopped'), {'faults': ['neutral:controller_fault']}, 'SpeedPID fault'),
    (lr.End('finished', 'stopped'), {'speeds': [2.5] * 20}, 'speed'),
])
def test_gate_verdict_fails_with_the_reason(end, kw, word):
    a = dict(speeds=[0.0, 0.8, 0.8, 0.0], laps_done=1, faults=[], stalled=False, at_rest=True)
    a.update(kw)
    ok, line = lr.gate_verdict(end, 'lab2_uturn', 0.8, 1, **a)
    assert not ok and line.startswith('FAIL') and word in line


def test_bench_ends_a_run_that_does_not_move():
    def snap(now, progress):
        return lr.Snapshot(now=now, engaged=True, pose_rx=now - 0.1, pose_std=0.08,
                           monitor={n: (0, 'fine', now) for n in lr.MONITOR_STATUSES},
                           progress=progress)
    s = snap(now=121.0, progress=0.2)
    assert lr.end_reason(s, 100.0, 300.0, stall=True).kind == 'stalled'
    assert lr.end_reason(s, 100.0, 300.0) is None                 # the car: no such rule
    assert lr.end_reason(snap(now=121.0, progress=0.6), 100.0, 300.0, stall=True) is None
    assert lr.end_reason(snap(now=110.0, progress=0.0), 100.0, 300.0, stall=True) is None


def test_car_stack_carries_the_car_profile(tmp_path, monkeypatch):
    import lab2_config as lc
    monkeypatch.setattr(lc, 'SURVEY_LATEST', tmp_path / 'no_survey')
    f = tmp_path / 'lab2.yaml'
    f.write_text(f'car: A\nteam: 3\nteam_dir: /t/team3\nbag_dir: {tmp_path / "bags"}\n')
    run = lr.Run(lr.parse_args([f'config:={f}', 'car:=B']))
    run.plan()
    cmd = run.stack_cmd
    assert cmd[:3] == ['just', 'lab2', 'vehicle'] and 'car_profile:=car_b' in cmd
    # `just lab2 vehicle` plans again from the same file: car:=B goes with it, no conflict
    again, _ = lc.plan('vehicle', cmd[5:])
    assert [a for a in again if a.startswith('car_profile:=')] == ['car_profile:=car_b']
    # no car anywhere: refused before anything starts
    f.write_text(f'team: 3\nteam_dir: /t/team3\nbag_dir: {tmp_path / "bags"}\n')
    with pytest.raises(lc.ConfigError, match='no car set'):
        lr.Run(lr.parse_args([f'config:={f}'])).plan()
