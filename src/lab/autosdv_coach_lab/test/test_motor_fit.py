"""lab2_fit_motor recovers MotorPlantParams from recordings made with them.

The recordings are the speed step test run through the actuator's own loop
(LongitudinalShell + the default speed PID at 100 Hz on a 20 Hz hall-sensor
report), with motor_plant.MotorPlant as the vehicle, so the fit is checked
against exactly the signal path it will see on the car.

Stated tolerances, with realistic noise (2 cm/s on each report, a 2 cm/s/sqrt(s)
speed disturbance, markers 5 % unevenly spaced, 2 ms report jitter):

    deadband        exact
    gain            +-3 %
    time_constant   +-10 %
    brake_decel     the truth lies in the reported +5 %-RMSE range, or the
                    fit flags it weak -- a stop lasts a handful of 20 Hz
                    reports, so the hall sensor barely sees the brake
    coast           not excited by the step test: reported as not identified
                    and held at the prior

Noise-free, deadband, gain and time_constant come back to within 1 %; the
brake is held to the same rule as with noise.
"""
from dataclasses import asdict

import numpy as np
import pytest
import yaml

from autosdv_vehicle_interface.motor_plant import MotorPlantParams

from autosdv_coach_lab.calibration import motor_fit as mf

STEPS = [0.0, 0.5, 1.0, 0.5, 1.5, 0.0]
NOISE = dict(report_noise=0.02, process_noise=0.02, marker_jitter=0.05, report_jitter=0.002)
TRUTH_A = MotorPlantParams(deadband=7, gain=0.025, time_constant=0.6,
                           coast_time_constant=2.0, brake_decel=2.2)
TRUTH_B = MotorPlantParams(deadband=4, gain=0.035, time_constant=0.3, brake_decel=3.5)


@pytest.fixture(scope='module')
def clean():
    rec, truth = mf.synthesize(TRUTH_A, STEPS, 4.0)
    return rec, truth, mf.fit(rec, deadband_max=10)


def test_noise_free_recovery(clean):
    rec, _, res = clean
    p = res.params
    assert p.deadband == TRUTH_A.deadband
    for k in ('gain', 'time_constant'):
        assert getattr(p, k) == pytest.approx(getattr(TRUTH_A, k), rel=0.01), k
    lo, hi = res.intervals['brake_decel']
    assert lo <= TRUTH_A.brake_decel <= hi or 'brake_decel' in res.weak
    assert res.rmse_fit < 0.01
    assert not res.identified['coast_time_constant']
    assert p.coast_time_constant == MotorPlantParams().coast_time_constant


@pytest.mark.parametrize('truth,seed', [(TRUTH_A, 1), (TRUTH_B, 3)])
def test_noisy_recovery(truth, seed):
    rec, _ = mf.synthesize(truth, STEPS, 4.0, mf.SynthNoise(seed=seed, **NOISE))
    res = mf.fit(rec, deadband_max=10)
    p = res.params
    assert p.deadband == truth.deadband
    assert p.gain == pytest.approx(truth.gain, rel=0.03)
    assert p.time_constant == pytest.approx(truth.time_constant, rel=0.10)
    lo, hi = res.intervals['brake_decel']
    assert lo <= truth.brake_decel <= hi or 'brake_decel' in res.weak
    assert not res.identified['coast_time_constant']
    # Far better than the provisional parameters, and near the noise floor.
    assert res.rmse_moving < 0.5 * res.rmse_prior
    assert res.rmse_moving < 0.08


def test_yaml_loads_into_motor_plant_params(clean):
    _, _, res = clean
    text = mf.to_yaml(res, 'synthetic')
    loaded = MotorPlantParams(**yaml.safe_load(text))
    assert loaded.deadband == res.params.deadband
    assert loaded.gain == pytest.approx(res.params.gain, rel=1e-3)
    assert 'prior, not fitted' in text      # coast
    flow = yaml.safe_load(mf.flow_mapping(res))
    assert set(flow) == set(mf.FIT_FIELDS)
    MotorPlantParams(**flow)
    assert 'residual RMSE' in mf.report(res)


def test_simulate_reproduces_its_own_recording(clean):
    """The forward model, fed the recorded PWM, reproduces the report exactly."""
    rec, _, _ = clean
    r = mf.residuals(TRUTH_A, rec)
    assert np.sqrt(np.mean(r ** 2)) < 1e-3


def test_csv_round_trip(tmp_path, clean):
    rec = clean[0]
    path = str(tmp_path / 'rec.csv')
    mf.write_csv(path, rec)
    back = mf.read_csv(path)
    assert np.allclose(back.pwm, rec.pwm) and np.allclose(back.v, rec.v, atol=1e-6)
    assert np.allclose(back.v_t, rec.v_t, atol=1e-6)


def test_bag_round_trip(tmp_path, clean):
    pytest.importorskip('rosbag2_py')
    pytest.importorskip('autoware_internal_debug_msgs.msg')
    rec = clean[0]
    path = str(tmp_path / 'bag')
    mf.write_synthetic_bag(path, rec)
    back = mf.recording_from_bag(path)
    assert len(back.pwm) == len(rec.pwm) and len(back.v) == len(rec.v)
    assert np.allclose(back.pwm, rec.pwm)
    assert np.allclose(back.v, rec.v, atol=1e-6)
    assert np.allclose(np.diff(back.v_t), np.diff(rec.v_t), atol=1e-6)


def test_window_and_validation():
    rec, _ = mf.synthesize(TRUTH_A, [0.0, 0.5], 2.0)
    w = rec.window(1.0, 3.0)
    assert w.pwm_t[0] >= rec.pwm_t[0] + 1.0 and w.pwm_t[-1] <= rec.pwm_t[0] + 3.0
    with pytest.raises(ValueError):
        mf.Recording([0.0], [370], [0.0, 0.1], [0.0, 0.0])
    assert asdict(TRUTH_A)['deadband'] == 7
