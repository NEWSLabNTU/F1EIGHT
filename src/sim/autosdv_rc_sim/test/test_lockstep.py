"""The lockstep harness: the car's classes, driven on a simulated clock."""
from autosdv_rc_sim.lockstep import (
    EstopScenario, load_actuator_params, Lockstep, ProfileScenario)
from autosdv_rc_sim.params import load_vehicle
from autosdv_rc_sim.track import default_track_dir, load_track, stop_overshoot

PROFILE = load_track(default_track_dir())
OPEN = {'max_pwm_offset': 100, 'integral_limit': 100.0}
# A set that can follow the whole profile: the reference Task 16 finds by search
TUNED = dict(OPEN, kp_speed=7.0, ki_speed=43.0, kd_speed=1.5)


def run(overrides=None, scenario=None, variant='nominal'):
    vehicle = load_vehicle(variant=variant)
    ls = Lockstep(vehicle, load_actuator_params(), overrides or {}, PROFILE, variant)
    return ls.run(scenario or ProfileScenario(PROFILE))


def test_lockstep_uses_rc_sim_ratio():
    assert load_actuator_params()['tire_angle_to_steer_ratio'] == -143.0
    assert load_vehicle().tire_angle_to_steer_ratio == -143.0
    assert load_actuator_params()['speed_controller_dt_source'] == 'loop'


def test_production_coefficients_cannot_reach_11():
    trace = run()
    assert max(trace.column('speed')) < 8.0          # 370 + 55 tops out near 6.4 m/s
    assert max(trace.column('pwm')) <= 425


def test_opened_limit_reaches_11_stays_in_lane_and_stops():
    trace = run(OPEN)
    assert max(trace.column('speed')) > 10.5
    assert max(abs(y) for y in trace.column('y')) < 0.2
    assert abs(trace.column('speed')[-1]) < 0.01
    assert 'active' in trace.column('mode') and 'brake' in trace.column('mode')


def test_a_tuned_controller_stops_at_the_line_with_no_allowance():
    """d 2.5 is within the brake (stop_overshoot 0), so the stop error is the controller's alone."""
    assert stop_overshoot(PROFILE, load_vehicle()) == 0.0
    trace = run(TUNED)
    assert abs(trace.column('speed')[-1]) < 0.01
    assert abs(trace.column('s')[-1] - PROFILE.stop_line_s) <= 0.5


def test_emergency_stop_overrides_then_resumes():
    trace = run(OPEN, EstopScenario(PROFILE))
    phases = trace.column('phase')
    assert phases.index('run') < phases.index('estop') < phases.index('resume')
    modes = [m for m, ph in zip(trace.column('mode'), phases) if ph == 'estop']
    assert set(modes) <= {'override_brake', 'override_neutral'}
    assert min(trace.column('speed')) >= -0.05
    resumed = [v for v, ph in zip(trace.column('speed'), phases) if ph == 'resume']
    assert max(resumed) > 5.0


def test_two_runs_are_bit_identical():
    assert run(OPEN).digest() == run(OPEN).digest()


def test_csv_round_trip(tmp_path):
    trace = run(OPEN)
    trace.write_csv(tmp_path / 't.csv')
    again = type(trace).read_csv(tmp_path / 't.csv')
    assert again.rows == trace.rows and again.csv_text() == trace.csv_text()


def test_timing_must_agree_with_the_car():
    import dataclasses
    import pytest
    from autosdv_rc_sim.lockstep import check_timing, load_velocity_report_params
    v, a, vr = load_vehicle(), load_actuator_params(), load_velocity_report_params()
    check_timing(v, a, vr)
    with pytest.raises(ValueError, match='physics_dt'):
        check_timing(dataclasses.replace(v, physics_dt=0.002), a, vr)
    with pytest.raises(ValueError, match='actuator rate'):
        check_timing(v, dict(a, rate=50.0), vr)
    with pytest.raises(ValueError, match='velocity_report rate'):
        check_timing(v, a, dict(vr, rate=10.0))
    with pytest.raises(ValueError, match='lockstep timing'):
        Lockstep(dataclasses.replace(v, physics_dt=0.002), a, {}, PROFILE)
