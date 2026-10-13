"""lab2_export_speed_csv: the CSV the students fit from carries the recording's PWM and speed."""
import numpy as np

from autosdv_coach_lab.calibration import motor_fit as mf
from autosdv_coach_lab.calibration import speed_csv


def _rec():
    pwm_t = np.arange(0.0, 10.0, 0.01)
    pwm = np.where(pwm_t < 5.0, 370.0, 395.0)
    v_t = np.arange(0.0, 10.0, 0.05)
    v = np.where(v_t < 5.5, 0.0, 0.8)
    return mf.Recording(pwm_t, pwm, v_t, v, 'test')


def test_rows_are_speed_reports_with_the_pwm_in_force():
    rows = list(speed_csv.rows(_rec()))
    assert len(rows) == 200
    t, pwm, v = rows[0]
    assert (t, pwm, v) == (0.0, 370, 0.0)
    t, pwm, v = rows[-1]
    assert pwm == 395 and v == 0.8 and 9.9 < t < 10.0


def test_csv_round_trip(tmp_path):
    out = tmp_path / 'car.csv'
    n = speed_csv.write(str(out), _rec())
    lines = out.read_text().splitlines()
    assert lines[0] == 't,pwm,measured_speed'
    assert len(lines) == n + 1 == 201
    t, pwm, v = lines[-1].split(',')
    assert int(pwm) == 395 and float(v) == 0.8
