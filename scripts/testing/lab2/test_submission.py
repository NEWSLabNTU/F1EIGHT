"""submission.py: python3 -m pytest scripts/testing/lab2/test_submission.py"""
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import submission  # noqa: E402

HERE = Path(__file__).resolve().parent


def make(tmp_path, pid='kp: 15.0\n', cal='car: A\ndeadband: 5.5\ngain: 0.031\n', nested=True):
    root = tmp_path / 'B1_lab_2_1' if nested else tmp_path
    pkg = root / 'lab2_waypoint' if nested else root
    (pkg / 'lab2_waypoint').mkdir(parents=True)
    (pkg / 'config').mkdir()
    (pkg / 'lab2_waypoint' / 'speed_pid.py').write_text('class SpeedPID: pass\n')
    (pkg / 'config' / 'speed_pid.yaml').write_text(pid)
    (pkg / 'config' / 'calibration.yaml').write_text(cal)
    return root, pkg


def test_merges_calibration_under_the_gains(tmp_path):
    root, pkg = make(tmp_path)
    got_pkg, args, warnings = submission.resolve(root)
    assert got_pkg == pkg.resolve() and warnings == []
    assert args[0] == 'speed_controller_class:=lab2_waypoint.speed_pid:SpeedPID'
    assert args[1] == 'speed_controller_params:={kp: 15.0, ff_deadband: 5.5, ff_gain: 0.031}'


def test_package_dir_itself_and_speed_pid_yaml_wins(tmp_path):
    _, pkg = make(tmp_path, pid='kp: 1\nff_gain: 0.04\n')
    _, args, _ = submission.resolve(pkg)
    assert 'ff_gain: 0.04' in args[1] and 'ff_gain: 0.031' not in args[1]


def test_uncalibrated_warns(tmp_path):
    root, _ = make(tmp_path, cal='car: A\ndeadband: 0.0\ngain: 0.0\n')
    assert any('not done' in w for w in submission.resolve(root)[2])


def test_missing_files_are_errors(tmp_path):
    with pytest.raises(submission.SubmissionError):
        submission.resolve(tmp_path)
    root, pkg = make(tmp_path)
    (pkg / 'config' / 'calibration.yaml').unlink()
    with pytest.raises(submission.SubmissionError):
        submission.resolve(root)


def test_shell_helper_replaces_the_argument(tmp_path):
    root, pkg = make(tmp_path)
    script = (f'cd {HERE.parents[2]}; source scripts/testing/lab2/submission.sh; '
              f'lab2_apply_submission course:=x submission:={root} y:=1; '
              'printf "%s\\n" "${LAB2_ARGS[@]}"; echo "PP=$PYTHONPATH"')
    out = subprocess.run(['bash', '-c', script], capture_output=True, text=True, check=True).stdout
    lines = out.splitlines()
    assert lines[-5:-1] == ['course:=x', 'y:=1', 'speed_controller_class:=lab2_waypoint.speed_pid:SpeedPID',
                            'speed_controller_params:={kp: 15.0, ff_deadband: 5.5, ff_gain: 0.031}']
    assert lines[-1].startswith(f'PP={pkg.resolve()}')


# ---- Part 2: a team directory, its controller and its lookahead

def team(tmp_path, tracker=None):
    make(tmp_path)                       # tmp_path/B1_lab_2_1/lab2_waypoint/...
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
    assert tr['lookahead_min'] == 0.6 and 'a_lat_max' in tr   # the rest from the package
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
