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
    o = lr.parse_args(['--bench', 'config:=x.yaml', 'car:=B', 'team:=3',
                       'vehicle_model:=team3_vehicle', 'seed:=21', '--camera', 'mock', '--timeout=90'])
    assert o.bench and not o.yes
    assert o.camera == 'mock' and o.timeout == 90.0 and o.team == '3'
    assert o.config_args == ('config:=x.yaml', 'car:=B', 'vehicle_model:=team3_vehicle')
    assert o.launch_args == ('seed:=21',)
    # team:= (the number) is the recorder's; the stack gets the model
    assert lr.vehicle_config_args(o.config_args + ('team:=3',)) == \
        ['config:=x.yaml', 'car:=B', 'vehicle_model:=team3_vehicle']


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

class Settings(dict):
    """lab2_config.Config's get/source over a dict: every value from 'the file'."""
    def source(self, k):
        return 'file' if k in self else 'not set'


def built(*models):
    return lambda m: m in models


def test_parse_takes_a_team_number_and_force_flags():
    o = lr.parse_args(['team:=3'])
    assert o.team == '3' and o.config_args == ()
    o = lr.parse_args(['team:=3', '--car', 'team_number:=7'])
    assert o.car and o.team_number == '7'
    with pytest.raises(lr.UsageError, match='team NUMBER'):
        lr.parse_args(['team:=/x/team3'])
    with pytest.raises(lr.UsageError, match='no longer a directory'):
        lr.parse_args(['/x/team3'])
    with pytest.raises(lr.UsageError, match='car:=A'):
        lr.parse_args(['A'])                        # a car letter is car:=, and the TA's
    for gone in ('team_dir:=/t', 'submission:=/s', 'speed_controller_class:=a.b:C'):
        with pytest.raises(lr.UsageError, match='is gone'):
            lr.parse_args([gone])


def test_team_number_runs_its_model():
    args, note, model = lr.resolve_model(lr.parse_args(['team:=3']), Settings(),
                                         built('team3_vehicle'))
    assert args == ['vehicle_model:=team3_vehicle', 'team:=3'] and model == 'team3_vehicle'


def test_a_model_gives_its_team_number_or_asks():
    o = lr.parse_args(['vehicle_model:=team12_vehicle'])
    s = Settings(vehicle_model='team12_vehicle')
    assert lr.resolve_model(o, s, built('team12_vehicle'))[0] == \
        ['vehicle_model:=team12_vehicle', 'team:=12']
    s = Settings(vehicle_model='red_vehicle')
    with pytest.raises(lr.UsageError, match='team_number:='):
        lr.resolve_model(lr.parse_args([]), s, built('red_vehicle'))
    assert lr.resolve_model(lr.parse_args(['team_number:=5']), s, built('red_vehicle'))[0] == \
        ['vehicle_model:=red_vehicle', 'team:=5']
    args, note, _ = lr.resolve_model(lr.parse_args([]), Settings(vehicle_model='red_vehicle', team='4'),
                                     built('red_vehicle'))
    assert args == ['vehicle_model:=red_vehicle', 'team:=4'] and 'settings' in note


def test_no_team_runs_the_reference_as_team_0():
    args, note, model = lr.resolve_model(lr.parse_args([]), Settings(), built('autosdv_vehicle'))
    assert args == ['vehicle_model:=autosdv_vehicle', 'team:=0'] and 'reference' in note
    # the settings' team names the model
    assert lr.resolve_model(lr.parse_args([]), Settings(team='3'), built('team3_vehicle'))[0] == \
        ['vehicle_model:=team3_vehicle', 'team:=3']


def test_an_unbuilt_model_says_how_to_build_it():
    with pytest.raises(lr.UsageError, match='src/lab2_vehicles.*packages-up-to team3_vehicle_launch'):
        lr.resolve_model(lr.parse_args(['team:=3']), Settings(), built())


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
    (lr.End('finished', 'stopped'), {'faults': ['neutral:controller_fault']}, 'speed controller fault'),
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


def test_car_stack_carries_the_vehicle_id(tmp_path, monkeypatch):
    import lab2_config as lc
    monkeypatch.setattr(lc, 'SURVEY_LATEST', tmp_path / 'no_survey')
    f = tmp_path / 'lab2.yaml'
    f.write_text(f'car: A\nteam: 3\nbag_dir: {tmp_path / "bags"}\n')
    run = lr.Run(lr.parse_args([f'config:={f}', 'car:=B']))
    run.plan()
    cmd = run.stack_cmd
    assert cmd[:3] == ['just', 'lab2', 'vehicle'] and 'vehicle_id:=car_b' in cmd
    # `just lab2 vehicle` plans again from the same file: car:=B goes with it, no conflict
    again, _ = lc.plan('vehicle', cmd[5:])
    assert [a for a in again if a.startswith('vehicle_id:=')] == ['vehicle_id:=car_b']
    # no car anywhere: refused before anything starts
    f.write_text(f'team: 3\nbag_dir: {tmp_path / "bags"}\n')
    with pytest.raises(lc.ConfigError, match='no car set'):
        lr.Run(lr.parse_args([f'config:={f}'])).plan()


def test_the_live_view_opens_on_the_runs_display(tmp_path, monkeypatch):
    import lab2_config as lc
    monkeypatch.setattr(lc, 'SURVEY_LATEST', tmp_path / 'no_survey')
    f = tmp_path / 'lab2.yaml'
    f.write_text(f'car: A\nteam: 3\nbag_dir: {tmp_path / "bags"}\n')
    run = lr.Run(lr.parse_args([f'config:={f}']))
    run.viewer = ':1'
    run.plan()
    assert run.stack_cmd[-2:] == ['live_view:=true', 'viewer_display:=:1']
    # `just lab2 vehicle` passes them on to the launch
    again, _ = lc.plan('vehicle', run.stack_cmd[5:])
    assert 'live_view:=true' in again and 'viewer_display:=:1' in again
    # headless: none; live_view:=false: kept closed
    run = lr.Run(lr.parse_args([f'config:={f}']))
    run.plan()
    assert not any(a.startswith('live_view:=') for a in run.stack_cmd)
    run = lr.Run(lr.parse_args([f'config:={f}', 'live_view:=false']))
    run.viewer = ':1'
    run.plan()
    assert [a for a in run.stack_cmd if a.startswith('live_view:=')] == ['live_view:=false']


def test_the_loaded_controller_is_read_from_the_actuator_log(tmp_path):
    d = tmp_path / 'play_log' / '2026' / 'node' / 'actuator_node'
    d.mkdir(parents=True)
    (d / 'err').write_text('[INFO] [1.0] [autosdv.actuator_node]: vehicle_id: sim\n'
                           '[INFO] [1.1] [autosdv.actuator_node]: Speed controller: '
                           'team3_vehicle_interface.speed_pid:SpeedPID\n')
    assert lr.loaded_controller(tmp_path) == 'team3_vehicle_interface.speed_pid:SpeedPID'
    assert lr.loaded_controller_problem('team3_vehicle', tmp_path) is None
    assert 'team4_vehicle' in lr.loaded_controller_problem('team4_vehicle', tmp_path)
    assert lr.loaded_controller_problem('autosdv_vehicle', tmp_path) is None   # the reference
    assert 'no speed controller' in lr.loaded_controller_problem('team3_vehicle', tmp_path / 'none')


def test_a_template_runs_its_own_speedpid_and_an_edited_one_is_refused(tmp_path, monkeypatch):
    import lab2_ws
    import new_team as nt
    files, source = nt.read_reference()
    root = nt.generate(3, tmp_path, files, source, nt.read_lab_files(worktree=True),
                       nt.vehicle_ids())
    monkeypatch.setattr(lab2_ws, 'model_source_root', lambda m: root)
    assert lab2_ws.controller_problems('team3_vehicle') == []
    assert lab2_ws.controller_problems('autosdv_vehicle') == []
    pid = root / 'team3_vehicle_interface/team3_vehicle_interface/speed_pid.py'
    pid.write_text('from team3_vehicle_interface.speed_controller import AckermannSpeedPID as SpeedPID\n')
    assert lab2_ws.controller_problems('team3_vehicle')


# ---- the bench's car

def _car_file(root, model, car, body):
    f = root / f'{model}_launch' / 'config' / f'car_{car}' / 'speed_controller.param.yaml'
    f.parent.mkdir(parents=True, exist_ok=True)
    f.write_text('/**:\n  ros__parameters:\n    speed_controller_params:\n' + body)


def test_bench_drives_the_calibrated_car(tmp_path):
    import lab2_ws
    model = 'team7_vehicle'
    _car_file(tmp_path, model, 'a', '      kp: 20.0\n      ff_deadband: 5.3\n      ff_gain: 0.0306\n')
    _car_file(tmp_path, model, 'b', '      kp: 20.0\n')
    _car_file(tmp_path, model, 'c', '      kp: 20.0\n      ff_gain: 0.0\n')
    cars = lab2_ws.calibrated_cars(tmp_path, model)
    assert cars == ['A']
    car, why = lr.bench_car(cars)
    assert car == 'A' and 'config/car_a/' in why


def test_bench_falls_back_to_the_nominal_car(tmp_path):
    import lab2_ws
    assert lab2_ws.calibrated_cars(None, 'team7_vehicle') == []
    assert lr.bench_car(None) == ('SIM', '')                  # the reference model
    car, why = lr.bench_car([])
    assert car == 'SIM' and 'ff_gain' in why                  # no calibration yet: said so
    car, why = lr.bench_car(['A', 'B'])
    assert car == 'SIM' and 'car:=X' in why
