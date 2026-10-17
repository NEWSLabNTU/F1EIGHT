"""The calibrated grader: the reference passes everywhere, poor coefficients do not."""
import os
from pathlib import Path

from autosdv_rc_sim import grade

REFERENCE = Path(__file__).resolve().parents[1] / 'config' / 'reference_speed_controller.yaml'
JOBS = min(4, os.cpu_count() or 1)


def test_reference_passes_every_check_on_every_variant_with_margin():
    report = grade.evaluate(grade.load_submission(REFERENCE), jobs=JOBS)
    assert report['passed'] == report['total'] == 8 * 16 and report['score'] == 1.0
    for variant, r in report['variants'].items():
        assert len(r['checks']) == 16
        for name, c in r['checks'].items():
            assert c['value'] is not None, (variant, name)
            assert c['value'] / c['limit'] <= 0.9, (variant, name, c)    # 10 % inside the limit, stop error included


def test_wild_coefficients_lose_marks():
    wild = {'kp_speed': 60.0, 'ki_speed': 80.0, 'integral_limit': 100.0, 'max_pwm_offset': 100}
    report = grade.evaluate(wild, variants=['nominal'], jobs=JOBS)
    assert report['total'] - report['passed'] >= 3


def test_physical_limits_were_not_moved():
    assert grade.LIMITS['launch_slip'] == 0.5 and grade.LIMITS['brake_kappa'] == -0.2
    assert grade.LIMITS['reverse_speed'] == -0.05 and grade.LIMITS['resume_slip'] == 0.5
    assert grade.LIMITS['stop_error'] == 0.5          # plain: no allowance, not calibrated
