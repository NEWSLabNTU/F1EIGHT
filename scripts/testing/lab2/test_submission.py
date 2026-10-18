"""submission.py: python3 -m pytest scripts/testing/lab2/test_submission.py"""
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import submission  # noqa: E402

HERE = Path(__file__).resolve().parent


# the handout's config/speed_pid_car.yaml: every key but the feed-forward's
CAR_YAML = ('kp: 20.0\nki: 25.0\nkd: 0.0\nintegral_limit: 10.0\nerror_limit: 0.25\n'
            'integral_band: 0.25\nenable_conditional_integration: true\n'
            'derivative_filter_alpha: 0.15\nmax_pwm_offset: 100\nmax_brake_offset: 10\n'
            'kp_brake: 20.0\nbrake_enter: 0.2\nbrake_exit: 0.05\nbrake_deadband: 5\n')
CAR_ARGS = ('speed_controller_params:={kp: 20.0, ki: 25.0, kd: 0.0, integral_limit: 10.0, '
            'error_limit: 0.25, integral_band: 0.25, enable_conditional_integration: true, '
            'derivative_filter_alpha: 0.15, max_pwm_offset: 100, max_brake_offset: 10, '
            'kp_brake: 20.0, brake_enter: 0.2, brake_exit: 0.05, brake_deadband: 5, '
            'ff_deadband: 5.0, ff_gain: 0.03}')


def make(tmp_path, pid=CAR_YAML, cal='car: A\ndeadband: 5.0\ngain: 0.03\n', nested=True):
    root = tmp_path / 'B1_lab_2_1' if nested else tmp_path
    pkg = root / 'lab2_speed' if nested else root
    (pkg / 'lab2_speed').mkdir(parents=True)
    (pkg / 'config').mkdir()
    (pkg / 'lab2_speed' / 'speed_pid.py').write_text('class SpeedPID: pass\n')
    (pkg / 'config' / 'speed_pid_car.yaml').write_text(pid)
    (pkg / 'config' / 'calibration.yaml').write_text(cal)
    return root, pkg


def test_merges_calibration_under_the_gains(tmp_path):
    root, pkg = make(tmp_path)
    got_pkg, args, warnings = submission.resolve(root)
    assert got_pkg == pkg.resolve() and warnings == []
    assert args[0] == 'speed_controller_class:=lab2_speed.speed_pid:SpeedPID'
    assert args[1] == CAR_ARGS


def test_package_dir_itself_and_the_car_file_wins(tmp_path):
    _, pkg = make(tmp_path, pid=CAR_YAML + 'ff_gain: 0.04\n')
    _, args, _ = submission.resolve(pkg)
    assert 'ff_gain: 0.04' in args[1] and 'ff_gain: 0.03' not in args[1]


def test_uncalibrated_warns_and_gives_no_feed_forward(tmp_path):
    root, _ = make(tmp_path, cal='car: A\ndeadband: 4.0\ngain: 0.0\n')
    _, args, warnings = submission.resolve(root)
    assert any('not done' in w for w in warnings)
    assert 'ff_deadband: 0.0' in args[1] and 'ff_gain: 0.0' in args[1]


def test_old_lab2_waypoint_layout_is_refused_with_the_new_name(tmp_path):
    (tmp_path / 'lab2_waypoint' / 'lab2_waypoint').mkdir(parents=True)
    (tmp_path / 'lab2_waypoint' / 'lab2_waypoint' / 'speed_pid.py').write_text('')
    with pytest.raises(submission.SubmissionError, match='lab2_speed'):
        submission.resolve(tmp_path)


def test_car_file_is_checked_like_part1(tmp_path):
    root, pkg = make(tmp_path)
    (pkg / 'config' / 'speed_pid_car.yaml').write_text('kp: -1.0\n')
    with pytest.raises(submission.SubmissionError, match='kp'):
        submission.resolve(root)


def test_car_file_without_ff_takes_the_calibration(tmp_path):
    root, _ = make(tmp_path)          # calibration deadband 5.0, gain 0.03
    _, args, warnings = submission.resolve(root)
    assert 'ff_deadband: 5.0' in args[1] and 'ff_gain: 0.03' in args[1]
    assert not any('ff_' in w for w in warnings)


def test_car_file_unset_key_still_warns(tmp_path):
    root, _ = make(tmp_path, pid='kp: 20.0\n')
    warnings = submission.resolve(root)[2]
    assert any('ki is not set' in w for w in warnings)
    assert any('speed_pid_car.yaml' in w for w in warnings)


def test_missing_files_are_errors(tmp_path):
    with pytest.raises(submission.SubmissionError):
        submission.resolve(tmp_path)
    root, pkg = make(tmp_path)
    (pkg / 'config' / 'calibration.yaml').unlink()
    with pytest.raises(submission.SubmissionError):
        submission.resolve(root)
    (pkg / 'config' / 'calibration.yaml').write_text('car: A\n')
    (pkg / 'config' / 'speed_pid_car.yaml').unlink()
    with pytest.raises(submission.SubmissionError, match='speed_pid_car.yaml'):
        submission.resolve(root)


def test_shell_helper_replaces_the_argument(tmp_path):
    root, pkg = make(tmp_path)
    script = (f'cd {HERE.parents[2]}; source scripts/testing/lab2/submission.sh; '
              f'lab2_apply_submission course:=x submission:={root} y:=1; '
              'printf "%s\\n" "${LAB2_ARGS[@]}"; echo "PP=$PYTHONPATH"')
    out = subprocess.run(['bash', '-c', script], capture_output=True, text=True, check=True).stdout
    lines = out.splitlines()
    assert lines[-5:-1] == ['course:=x', 'y:=1', 'speed_controller_class:=lab2_speed.speed_pid:SpeedPID',
                            CAR_ARGS]
    assert lines[-1].startswith(f'PP={pkg.resolve()}')


# ---- Part 2: a team directory, its controller and its lookahead

def team(tmp_path, tracker=None):
    make(tmp_path)                       # tmp_path/B1_lab_2_1/lab2_speed/...
    if tracker is not None:
        (tmp_path / 'path_tracker.param.yaml').write_text(tracker)
    return tmp_path


def test_team_flat_lookahead_is_merged_into_the_package_file(tmp_path):
    t = team(tmp_path, 'k_lookahead: 0.4\nlookahead_max: 1.5\n')
    _, args, warnings = submission.resolve_team(t, out_dir=tmp_path / 'out')
    tf = [a for a in args if a.startswith('tracker_param_file:=')]
    assert len(tf) == 1
    doc = submission.yaml.safe_load(Path(tf[0].split(':=', 1)[1]).read_text())
    tr = doc['/**']['ros__parameters']['tracker']
    assert tr['k_lookahead'] == 0.4 and tr['lookahead_max'] == 1.5
    # the rest from the package file, whichever keys it holds (develop's has more than Lab2's)
    base = submission.yaml.safe_load(submission.TRACKER_DEFAULT.read_text())
    for k, v in base['/**']['ros__parameters']['tracker'].items():
        if k not in ('k_lookahead', 'lookahead_max'):
            assert tr[k] == v
    assert any('lookahead' in w for w in warnings)


def test_team_whole_package_file_with_three_edits_is_accepted(tmp_path):
    text = submission.TRACKER_DEFAULT.read_text().replace('k_lookahead: 0.6', 'k_lookahead: 0.9')
    _, args, _ = submission.resolve_team(team(tmp_path, text), out_dir=tmp_path / 'out')
    assert any(a.startswith('tracker_param_file:=') for a in args)


def test_team_other_tracker_settings_are_refused(tmp_path):
    t = team(tmp_path, 'tracker:\n  k_lookahead: 0.5\n  a_lat_max: 2.0\n')
    with pytest.raises(submission.SubmissionError, match='a_lat_max'):
        submission.resolve_team(t, out_dir=tmp_path / 'out')


def test_team_min_above_max_is_refused(tmp_path):
    t = team(tmp_path, 'lookahead_min: 2.5\nlookahead_max: 2.0\n')
    with pytest.raises(submission.SubmissionError, match='above'):
        submission.resolve_team(t, out_dir=tmp_path / 'out')


def test_team_without_tracker_file_warns_and_uses_the_default(tmp_path):
    _, args, warnings = submission.resolve_team(team(tmp_path), out_dir=tmp_path / 'out')
    assert not any(a.startswith('tracker_param_file:=') for a in args)
    assert any('default lookahead' in w for w in warnings)


def test_team_needs_exactly_one_controller(tmp_path):
    with pytest.raises(submission.SubmissionError, match='exactly one'):
        submission.resolve_team(tmp_path, out_dir=tmp_path / 'out')


# ---- Part 1 on the simulated car: lab2_speed/ (just lab2 sim)

PART1_YAML = (HERE.parents[2] / 'src/lab/autosdv_waypoint_lab/config/speed_pid_part1.yaml').read_text()


def make_part1(tmp_path, pid=PART1_YAML, nested=True):
    root = tmp_path / 'B1_lab_2_1' if nested else tmp_path / 'lab2_speed'
    pkg = root / 'lab2_speed' if nested else root
    (pkg / 'lab2_speed').mkdir(parents=True)
    (pkg / 'config').mkdir()
    (pkg / 'lab2_speed' / 'speed_pid.py').write_text('class SpeedPID: pass\n')
    (pkg / 'config' / 'speed_pid.yaml').write_text(pid)
    return root, pkg


def test_part1_defaults_resolve_to_every_key(tmp_path):
    root, pkg = make_part1(tmp_path)
    got_pkg, args, warnings = submission.resolve_part1(root)
    assert got_pkg == pkg.resolve() and warnings == []
    assert args[0] == 'speed_controller_class:=lab2_speed.speed_pid:SpeedPID'
    assert args[1].startswith('speed_controller_params:={')
    mapping = submission.yaml.safe_load(args[1].split(':=', 1)[1])
    assert set(mapping) == set(submission.part1_keys())
    assert mapping['enable_conditional_integration'] is True
    assert isinstance(mapping['max_pwm_offset'], int)


def test_part1_package_dir_itself(tmp_path):
    _, pkg = make_part1(tmp_path, nested=False)
    assert submission.resolve_part1(pkg)[0] == pkg.resolve()


def test_part1_missing_key_warns_and_is_left_out(tmp_path):
    root, _ = make_part1(tmp_path, pid='kp: 12\nki: 3.5\n')
    _, args, warnings = submission.resolve_part1(root)
    assert args[1] == 'speed_controller_params:={kp: 12.0, ki: 3.5}'
    assert any('kd is not set' in w for w in warnings)


@pytest.mark.parametrize('pid,match', [
    ('kp: 1\nkp_speed: 2\n', 'unknown key'),
    ('kp: -1\n', 'outside'),
    ('kp: .nan\n', 'finite'),
    ('kp: fast\n', 'number'),
    ('kp: true\n', 'number'),
    ('enable_conditional_integration: 1\n', 'true or false'),
    ('enable_conditional_integration: "yes"\n', 'true or false'),
    ('derivative_filter_alpha: 0\n', 'outside'),
    ('derivative_filter_alpha: 1.5\n', 'outside'),
    ('integral_limit: 0\n', 'outside'),
    ('max_pwm_offset: 101\n', 'outside'),
    ('max_pwm_offset: 0\n', 'outside'),
    ('max_pwm_offset: 50.5\n', 'whole number'),
    ('max_brake_offset: 11\n', 'outside'),
    ('max_brake_offset: -1\n', 'outside'),
    ('ff_gain: 0\n', 'outside'),
    ('brake_enter: 0\n', 'outside'),
    ('brake_exit: -0.1\n', 'outside'),
    ('brake_deadband: 11\n', 'outside'),
    ('brake_deadband: 2.5\n', 'whole number'),
    ('brake_enter: 0.2\nbrake_exit: 0.2\n', 'hysteresis'),
    ('brake_enter: 0.2\nbrake_exit: 0.3\n', 'below brake_enter'),
    ('kp: 1\nkp: 2\n', 'twice'),
    ('[1, 2]\n', 'mapping'),
    ('kp: [1\n', 'not valid YAML'),
])
def test_part1_bad_values_are_refused_with_the_reason(tmp_path, pid, match):
    root, _ = make_part1(tmp_path, pid=pid)
    with pytest.raises(submission.SubmissionError, match=match):
        submission.resolve_part1(root)


def test_part1_integral_float_offsets_and_zero_brake_are_accepted(tmp_path):
    root, _ = make_part1(tmp_path, pid='max_pwm_offset: 80.0\nmax_brake_offset: 0\n')
    mapping = submission.yaml.safe_load(submission.resolve_part1(root)[1][1].split(':=', 1)[1])
    assert mapping == {'max_pwm_offset': 80, 'max_brake_offset': 0}


def test_part1_offset_limits_follow_the_actuator_file():
    keys = submission.part1_keys({'init_pwm': 380, 'min_pwm': 365, 'max_pwm': 480})
    assert keys['max_pwm_offset'][2] == 100 and keys['max_brake_offset'][2] == 15


def test_part1_missing_package_or_yaml_is_an_error(tmp_path):
    with pytest.raises(submission.SubmissionError, match='lab2_speed'):
        submission.resolve_part1(tmp_path)
    root, pkg = make_part1(tmp_path)
    (pkg / 'config' / 'speed_pid.yaml').unlink()
    with pytest.raises(submission.SubmissionError, match='missing'):
        submission.resolve_part1(root)


def test_part1_cli_and_shell_helper(tmp_path, capsys):
    root, pkg = make_part1(tmp_path, pid='kp: 12\n')
    assert submission.main([str(root), '--part1', '--launch-args']) == 0
    out = capsys.readouterr().out.splitlines()
    assert out == ['speed_controller_class:=lab2_speed.speed_pid:SpeedPID',
                   'speed_controller_params:={kp: 12.0}']
    assert submission.main([str(root), '--launch-args']) == 2   # no config/speed_pid_car.yaml: not a Part 2 submission
    script = (f'cd {HERE.parents[2]}; source scripts/testing/lab2/submission.sh; '
              f'lab2_apply_submission --part1 variant:=nominal submission:={root}; '
              'printf "%s\\n" "${LAB2_ARGS[@]}"; echo "PP=$PYTHONPATH"')
    lines = subprocess.run(['bash', '-c', script], capture_output=True, text=True,
                           check=True).stdout.splitlines()
    assert lines[-4:-1] == ['variant:=nominal', 'speed_controller_class:=lab2_speed.speed_pid:SpeedPID',
                            'speed_controller_params:={kp: 12.0}']
    assert lines[-1].startswith(f'PP={pkg.resolve()}')
