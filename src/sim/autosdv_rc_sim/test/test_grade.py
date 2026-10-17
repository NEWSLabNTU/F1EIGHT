"""Submission loading, metrics on synthetic traces, and grading of known coefficient sets."""
import json

import pytest
import yaml

from autosdv_rc_sim import grade
from autosdv_rc_sim.lockstep import COLUMNS, load_actuator_params, Lockstep, ProfileScenario, Trace
from autosdv_rc_sim.params import load_vehicle
from autosdv_rc_sim.track import default_track_dir, load_track

PROFILE = load_track(default_track_dir())
OPEN = {'max_pwm_offset': 100, 'integral_limit': 100.0}
DEFAULTS = {'mode': 'active', 'phase': 'run', 'pwm': 370, 'steer_pwm': 489, 'emergency': False}


def synthetic(rows):
    return Trace('profile', 'nominal',
                 [tuple(r.get(c, DEFAULTS.get(c, 0.0)) for c in COLUMNS) for r in rows])


def write(tmp_path, data, sub=''):
    d = tmp_path / sub if sub else tmp_path
    d.mkdir(parents=True, exist_ok=True)
    (d / grade.SUBMISSION_FILE).write_text(yaml.safe_dump(data))
    return tmp_path


def test_loads_the_allowed_keys(tmp_path):
    params = grade.load_submission(write(tmp_path, {'kp_speed': 15, 'max_pwm_offset': 100}))
    assert params == {'kp_speed': 15.0, 'max_pwm_offset': 100}


def test_finds_config_subdirectory(tmp_path):
    assert grade.load_submission(write(tmp_path, {'ki_speed': 4.0}, 'config')) == {'ki_speed': 4.0}


@pytest.mark.parametrize('data, match', [
    ({'min_pwm': 350}, 'not tunable'),
    ({'max_pwm_offset': 150}, 'max_pwm_offset'),
    ({'max_pwm_offset': 80.5}, 'integer'),
    ({'kp_speed': '12'}, 'number'),
    ({'enable_conditional_integration': 1}, 'true or false'),
    ({'derivative_filter_alpha': 0.0}, 'derivative_filter_alpha'),
])
def test_refusals(tmp_path, data, match):
    with pytest.raises(grade.SubmissionError, match=match):
        grade.load_submission(write(tmp_path, data))


def test_missing_file(tmp_path):
    with pytest.raises(grade.SubmissionError, match=grade.SUBMISSION_FILE):
        grade.load_submission(tmp_path)


def test_flow_mapping():
    assert grade.flow_mapping({'kp_speed': 15.0, 'max_pwm_offset': 100}) == '{kp_speed: 15.0, max_pwm_offset: 100}'


def test_launch_slip_metric_finite_at_standstill():
    """Review Focus 3: a spinning wheel on a stopped car is a slip velocity, not an infinite ratio."""
    trace = synthetic([{'t': 0.01 * i, 's': 0.0, 'speed': 0.0, 'slip_r': 0.3, 'kappa_r': 2.0}
                       for i in range(100)])
    m = grade.launch_metrics(trace, PROFILE)
    assert m['slip'] == pytest.approx(0.3)


def test_a_perfect_follower_scores_full_hold_marks():
    rows, s, t = [], 0.0, 0.0
    while s < PROFILE.stop_line_s:
        v = PROFILE.v_target(s)
        rows.append({'t': t, 's': s, 'target': v, 'speed': v, 'measured': v})
        s += v * 0.01
        t += 0.01
    holds = grade.hold_metrics(synthetic(rows), PROFILE)
    for key in ('2', '6', '11'):
        assert holds[key]['rmse'] == pytest.approx(0.0, abs=1e-9)
        assert holds[key]['overshoot_pct'] == 0.0
        assert holds[key]['reach_s'] == 0.0


def test_pwm_output_alpha_is_inert():
    """actuator.py reads pwm_output_alpha and never applies it (Spec Findings): two runs that
    differ only in that key leave identical traces. A key that does act (kp_speed) must not."""
    def run(**extra):
        ls = Lockstep(load_vehicle(), load_actuator_params(), dict(OPEN, **extra), PROFILE)
        return ls.run(ProfileScenario(PROFILE))
    a, b, other = run(pwm_output_alpha=0.25), run(pwm_output_alpha=1.0), run(kp_speed=20.0)
    assert a.rows == b.rows and a.digest() == b.digest()
    assert other.digest() != a.digest()


def test_production_defaults_lose_the_top_speed(tmp_path):
    report = grade.evaluate({}, variants=['nominal'], out_dir=tmp_path)
    c = report['variants']['nominal']['checks']
    assert not c['hold_rmse_11']['pass'] and not c['reach_11']['pass']
    assert report['total'] == 16
    marks = json.loads((tmp_path / 'marks.json').read_text())
    assert marks['marks'] == report['marks']
    assert (tmp_path / 'traces' / 'nominal_profile.csv').is_file()
    assert (tmp_path / 'traces' / 'nominal_estop.csv').is_file()


def test_print_params(tmp_path, capsys):
    assert grade.main([str(write(tmp_path, {'kp_speed': 15.0, 'max_pwm_offset': 100})), '--print-params']) == 0
    assert capsys.readouterr().out.strip() == '{kp_speed: 15.0, max_pwm_offset: 100}'


def test_bad_submission_exits_2(tmp_path, capsys):
    assert grade.main([str(write(tmp_path, {'min_pwm': 1}))]) == 2


def test_non_string_key_is_refused(tmp_path):
    with pytest.raises(grade.SubmissionError, match='keys must be names'):
        grade.load_submission(write(tmp_path, {1: 2, 'kp_speed': 3.0}))


def test_huge_integer_is_refused(tmp_path):
    (tmp_path / grade.SUBMISSION_FILE).write_text('kp_speed: 1' + '0' * 400 + '\n')
    with pytest.raises(grade.SubmissionError, match='too large'):
        grade.load_submission(tmp_path)
    assert grade.main([str(tmp_path)]) == 2


def test_non_utf8_file_exits_2(tmp_path, capsys):
    (tmp_path / grade.SUBMISSION_FILE).write_bytes(b'kp_speed: \xff\xfe\n')
    with pytest.raises(grade.SubmissionError, match='UTF-8'):
        grade.load_submission(tmp_path)
    assert grade.main([str(tmp_path)]) == 2
    assert 'error:' in capsys.readouterr().err


def test_non_string_key_exits_2(tmp_path):
    assert grade.main([str(write(tmp_path, {1: 2}))]) == 2
