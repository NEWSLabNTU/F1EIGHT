"""submission.py: python3 -m pytest scripts/testing/lab2/test_submission.py"""
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import submission  # noqa: E402

HERE = Path(__file__).resolve().parent


# ---- Part 1 on the simulated car: the simulation model, lab2_sim_vehicle (just lab2 sim)

PART1_YAML = (HERE.parents[2] / 'src/lab/autosdv_waypoint_lab/config/speed_pid_part1.yaml').read_text()


def nest(flat_yaml):
    """A flat SpeedPID yaml as a model's config/sim/speed_controller.param.yaml."""
    body = ''.join(f'      {ln}\n' for ln in flat_yaml.splitlines())
    return '/**:\n  ros__parameters:\n    speed_controller_params:\n' + body


def make_model(tmp_path, tuning=None, pid='class SpeedPID:\n    pass\n',
               model=submission.SIM_MODEL):
    """A minimal lab2_sim_vehicle_launch/ source tree, as new_team.py --sim lays it out."""
    root = tmp_path / 'src' / f'{model}_launch'
    for suffix in ('_description', '_interface', '_launch'):
        pkg = root / f'{model}{suffix}'
        pkg.mkdir(parents=True)
        (pkg / 'package.xml').write_text(f'<package><name>{model}{suffix}</name></package>\n')
    mod = root / f'{model}_interface' / f'{model}_interface'
    mod.mkdir()
    (mod / '__init__.py').write_text('')
    (mod / 'speed_pid.py').write_text(pid)
    cfg = root / f'{model}_launch' / 'config' / 'sim'
    cfg.mkdir(parents=True)
    (cfg / 'speed_controller.param.yaml').write_text(nest(PART1_YAML) if tuning is None else tuning)
    return model, root


def test_part1_model_resolves_with_every_key(tmp_path):
    model, root = make_model(tmp_path)
    got, got_root, mapping, warnings = submission.resolve_part1(root)
    assert (got, got_root) == (model, root.resolve()) and warnings == []
    assert set(mapping) == set(submission.part1_keys())
    assert mapping['ff_deadband'] == 5.27 and mapping['ff_gain'] == 0.1433
    assert mapping['enable_conditional_integration'] is True
    assert isinstance(mapping['max_pwm_offset'], int)


def test_part1_model_found_one_level_down(tmp_path):
    model, root = make_model(tmp_path)
    assert submission.resolve_part1(root.parent)[:2] == (model, root.resolve())
    with pytest.raises(submission.SubmissionError, match='no lab2_sim_vehicle_launch'):
        submission.resolve_part1(tmp_path)
    make_model(tmp_path / 'copy')
    (tmp_path / 'copy' / 'src' / 'lab2_sim_vehicle_launch').rename(tmp_path / 'src' / 'second')
    with pytest.raises(submission.SubmissionError, match='several places'):
        submission.resolve_part1(root.parent)
    other, oroot = make_model(tmp_path / 'o', model='other_sim')
    assert submission.resolve_part1(oroot, model='other_sim')[:2] == (other, oroot.resolve())


def test_part1_missing_key_warns_and_is_left_out(tmp_path):
    _, root = make_model(tmp_path, tuning=nest('kp: 12\nki: 3.5\n'))
    _, _, mapping, warnings = submission.resolve_part1(root)
    assert mapping == {'kp': 12.0, 'ki': 3.5}
    assert any('kd is not set' in w for w in warnings)


def test_part1_other_ros_parameters_warn(tmp_path):
    tuning = nest('kp: 12\n') + '    max_pwm: 470\n'
    _, root = make_model(tmp_path, tuning=tuning)
    _, _, mapping, warnings = submission.resolve_part1(root)
    assert mapping == {'kp': 12.0} and any('max_pwm' in w and 'not checked' in w for w in warnings)


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
])
def test_part1_bad_values_are_refused_with_the_reason(tmp_path, pid, match):
    _, root = make_model(tmp_path, tuning=nest(pid))
    with pytest.raises(submission.SubmissionError, match=match):
        submission.resolve_part1(root)


@pytest.mark.parametrize('tuning,match', [
    ('kp: 12\n', 'nested under speed_controller_params'),             # flat, the old layout
    ('/**:\n  ros__parameters:\n    kp: 12\n', 'kp sit directly under'),
    ('/**:\n  kp: 12\n', 'nested under'),
    ('a:\n  ros__parameters: {speed_controller_params: {}}\nb: {}\n', 'nested under'),
    ('/**:\n  ros__parameters:\n    speed_controller_params: [1]\n', 'mapping'),
    ('/**: [1\n', 'not valid YAML'),
])
def test_part1_tuning_shape_is_checked(tmp_path, tuning, match):
    _, root = make_model(tmp_path, tuning=tuning)
    with pytest.raises(submission.SubmissionError, match=match):
        submission.resolve_part1(root)


def test_part1_float_offsets_and_zero_brake_are_accepted(tmp_path):
    _, root = make_model(tmp_path, tuning=nest('max_pwm_offset: 80.0\nmax_brake_offset: 0\n'))
    assert submission.resolve_part1(root)[2] == {'max_pwm_offset': 80, 'max_brake_offset': 0}


def test_part1_offset_limits_follow_the_actuator_file():
    keys = submission.part1_keys({'init_pwm': 380, 'min_pwm': 365, 'max_pwm': 480})
    assert keys['max_pwm_offset'][2] == 100 and keys['max_brake_offset'][2] == 15


def test_part1_missing_pieces_are_errors(tmp_path):
    model, root = make_model(tmp_path)
    (root / f'{model}_launch' / 'config' / 'sim' / 'speed_controller.param.yaml').unlink()
    with pytest.raises(submission.SubmissionError, match='missing'):
        submission.resolve_part1(root)
    model, root = make_model(tmp_path / 'b')
    (root / f'{model}_description' / 'package.xml').unlink()
    with pytest.raises(submission.SubmissionError, match=f'{model}_description/package.xml'):
        submission.resolve_part1(root)
    model, root = make_model(tmp_path / 'c')
    (root / f'{model}_interface' / f'{model}_interface' / 'speed_pid.py').unlink()
    with pytest.raises(submission.SubmissionError, match='speed_pid.py: missing'):
        submission.resolve_part1(root)


@pytest.mark.parametrize('pid,match', [
    ('class PID:\n    pass\n', 'defines no class SpeedPID'),
    ('SpeedPID = object\n', 'defines no class SpeedPID'),
    ('class SpeedPID(\n', 'does not parse'),
])
def test_part1_speed_pid_must_define_the_class(tmp_path, pid, match):
    _, root = make_model(tmp_path, pid=pid)
    with pytest.raises(submission.SubmissionError, match=match):
        submission.resolve_part1(root)


@pytest.mark.parametrize('pid', [
    'from autosdv_waypoint_lab.reference.speed_pid_part1 import SpeedPID\n',
    'import autosdv_waypoint_lab.reference.speed_pid_part1 as r\nclass SpeedPID(r.SpeedPID):\n    pass\n',
    'from autosdv_waypoint_lab import reference\nclass SpeedPID:\n    pass\n',
    'import importlib\nSpeedPID = 1\nclass SpeedPID:\n'
    '    m = importlib.import_module("autosdv_waypoint_lab.reference.speed_pid_part1")\n',
])
def test_part1_the_reference_is_refused(tmp_path, pid):
    _, root = make_model(tmp_path, pid=pid)
    with pytest.raises(submission.SubmissionError, match='TA reference'):
        submission.resolve_part1(root)


def test_part1_the_reference_named_in_a_launch_file_is_refused(tmp_path):
    model, root = make_model(tmp_path)
    launch = root / f'{model}_launch' / 'launch'
    launch.mkdir()
    (launch / 'vehicle_interface.launch.xml').write_text(
        '<launch>\n<!-- autosdv_waypoint_lab.reference in a comment is fine -->\n'
        '<arg name="speed_controller_class" '
        'default="autosdv_waypoint_lab.reference.speed_pid_part1:SpeedPID"/>\n</launch>\n')
    with pytest.raises(submission.SubmissionError, match=r'vehicle_interface.launch.xml:3'):
        submission.resolve_part1(root)


def test_part1_the_reference_in_a_docstring_or_comment_is_not_a_use(tmp_path):
    pid = ('"""Run me as autosdv_waypoint_lab.reference.speed_pid_part1:SpeedPID would."""\n'
           '# autosdv_waypoint_lab.reference is the TA\'s\n'
           'class SpeedPID:\n    """not autosdv_waypoint_lab.reference"""\n')
    _, root = make_model(tmp_path, pid=pid)
    assert submission.resolve_part1(root)[0] == 'lab2_sim_vehicle'


def test_part1_cli(tmp_path, capsys):
    model, root = make_model(tmp_path, tuning=nest('kp: 12\n'))
    assert submission.main(['--part1', str(root)]) == 0
    out = capsys.readouterr()
    assert out.out.splitlines() == [f'model {model}', f'source {root.resolve()}']
    assert 'warning: speed_controller.param.yaml: ki is not set' in out.err
    assert submission.main(['--part1', str(root.parent), '--model']) == 0
    assert capsys.readouterr().out == f'{model}\n'
    assert submission.main(['--part1', str(tmp_path)]) == 2
    assert 'submission error' in capsys.readouterr().err
    with pytest.raises(SystemExit):
        submission.main(['--part1', str(root), '--launch-args'])


def test_part1_new_team_template_passes(tmp_path):
    """The package `just lab2 new-sim` writes passes untouched (the stub, the seed), as
    the simulation model."""
    import new_team
    import test_new_team
    files = dict(test_new_team.FILES)          # the reference model, reduced to its shape
    files['autosdv_vehicle_interface/package.xml'] = b'<name>autosdv_vehicle_interface</name>'
    new_team.generate(1, tmp_path, files, 'test', new_team.read_lab_files(worktree=True),
                      ids=('sim',), kind='sim')
    model, root, mapping, warnings = submission.resolve_part1(tmp_path)
    assert model == 'lab2_sim_vehicle' and warnings == []
    assert mapping == submission.check_part1_yaml(
        HERE.parents[2] / 'src/lab/autosdv_waypoint_lab/config/speed_pid_part1.yaml')[0]


def test_offset_limits_follow_the_vehicle_id(tmp_path):
    cp = submission._instances()
    a, b = cp.load('sim'), cp.load('car_b')
    assert submission.actuator_pwm() == submission.actuator_pwm('sim') == \
        {k: a[k] for k in ('init_pwm', 'min_pwm', 'max_pwm')}
    pwm_b = submission.actuator_pwm('car_b')
    assert pwm_b['init_pwm'] == b['init_pwm'] != a['init_pwm']
    keys = submission.part1_keys(pwm_b)
    assert keys['max_pwm_offset'][2] == b['max_pwm'] - b['init_pwm']
    assert keys['max_brake_offset'][2] == b['init_pwm'] - b['min_pwm']
    with pytest.raises(submission.SubmissionError, match='unknown vehicle_id'):
        submission.actuator_pwm('car_z')
    # an offset the simulated car has room for, car B has not
    over = b['max_pwm'] - b['init_pwm'] + 1
    assert over <= a['max_pwm'] - a['init_pwm']
    _, root = make_model(tmp_path, tuning=nest(f'max_pwm_offset: {over}\n'))
    submission.resolve_part1(root)
    with pytest.raises(submission.SubmissionError, match='max_pwm_offset'):
        submission.resolve_part1(root, pwm=pwm_b)
    cmd = [sys.executable, str(HERE / 'submission.py'), '--part1', str(root)]
    assert subprocess.run(cmd, capture_output=True).returncode == 0
    r = subprocess.run(cmd + ['--vehicle-id', 'car_b'], capture_output=True, text=True)
    assert r.returncode == 2 and 'max_pwm_offset' in r.stderr


def test_a_student_submission_dir_holding_the_model_and_a_report(tmp_path):
    sub_dir = tmp_path / 'B12345678_lab_2_1'
    model, root = make_model(sub_dir)                    # <dir>/src/lab2_sim_vehicle_launch
    (sub_dir / 'report.pdf').write_bytes(b'%PDF-1.4\n')
    flat = sub_dir / 'lab2_sim_vehicle_launch'
    root.rename(flat)                                    # <dir>/lab2_sim_vehicle_launch, as handed in
    (sub_dir / 'src').rmdir()
    assert submission.resolve_part1(sub_dir)[:2] == (model, flat.resolve())
    assert submission.main([str(sub_dir), '--model']) == 0   # --part1 implied
