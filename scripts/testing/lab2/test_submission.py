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
