import math

import pytest

from autosdv_waypoint_lab import steer_cal as sc


def _circles(ratio, centre=0.0, offsets=(-50, -25, -10, 10, 25, 50), noise=0.0):
    """Radii a car with this ratio would trace; positive offset turns left iff ratio > 0."""
    out = []
    for o in offsets:
        delta = (o - centre) / ratio
        out.append((o, math.copysign(sc.WHEEL_BASE / math.tan(abs(delta)), delta) * (1 + noise)))
    return out


def test_fit_recovers_ratio_and_lock():
    r = sc.fit(_circles(-143.0))
    assert r.ratio == pytest.approx(-143.0, rel=1e-6)
    assert r.max_angle == pytest.approx(50 / 143.0, rel=1e-6)
    assert r.centre == pytest.approx(0.0, abs=1e-6)
    assert r.angle_needed == pytest.approx(math.atan(sc.WHEEL_BASE / sc.R_FIRST_CIRCLE))
    assert r.margin_ok == (r.angle_needed <= sc.MARGIN * r.max_angle)


def test_the_gate_is_the_graded_uturn_radius():
    import yaml
    from autosdv_waypoint_lab import route_grade as rg
    with open(rg.template_dir('uturn') + '/course.yaml') as f:
        assert sc.R_FIRST_CIRCLE == float(yaml.safe_load(f)['uturn']['radius'])


def test_fit_sees_a_centre_error():
    assert sc.fit(_circles(-143.0, centre=4.0)).centre == pytest.approx(4.0, abs=1e-6)


def test_the_shipped_ratio_cannot_hold_the_tight_circle():
    # the servo window would be reached at 0.05 rad: the tightest circle needs far more
    r = sc.fit(_circles(-1000.0))
    assert r.max_angle == pytest.approx(0.05)
    assert not r.margin_ok
    assert any('FAIL' in line for line in r.lines())


def test_a_marginal_car_fails():
    # the ratio whose lock the tightest circle uses exactly MARGIN of
    needed = math.atan(sc.WHEEL_BASE / sc.R_FIRST_CIRCLE)
    limit = sc.SERVO_HALF_RANGE * sc.MARGIN / needed
    assert not sc.fit(_circles(-1.05 * limit)).margin_ok     # 84 % of lock
    assert sc.fit(_circles(-0.99 * limit)).margin_ok          # 79 %


def test_measurement_noise_is_tolerated():
    assert sc.fit(_circles(-143.0, noise=0.03)).ratio == pytest.approx(-143.0, rel=0.1)


def test_positive_ratio_is_flagged():
    assert any('LEFT' in line for line in sc.fit(_circles(+143.0)).lines())


def test_too_few_or_degenerate_circles_are_refused():
    with pytest.raises(ValueError):
        sc.fit([(10, 3.0), (25, 1.2)])
    with pytest.raises(ValueError):
        sc.fit([(10, 3.0), (10, 3.0), (10, 3.0)])


@pytest.mark.parametrize('side,measured,axle', [('axle', 1.5, 1.5),
                                                ('inner', 1.4, 1.4 + 0.1315),
                                                ('outer', 1.6, 1.6 - 0.1315)])
def test_tyre_track_radius_converts_to_axle_radius(side, measured, axle):
    assert sc.radius_at_axle(measured, side) == pytest.approx(axle)


@pytest.mark.parametrize('ratio', [-1000.0, -143.0, 143.0])
@pytest.mark.parametrize('offset', [-50, -25, -10, 10, 25, 50])
def test_hold_makes_the_actuator_write_that_offset(ratio, offset):
    # the actuator: init_steer + int(average_angle * ratio), with int() truncating
    delta = sc.command_for_offset(offset, ratio)
    if abs(delta) > sc.MAX_COMMAND_ANGLE:
        pytest.skip('beyond the actuator clamp')
    assert int(sc.average_wheel_angle(delta) * ratio) == offset


def test_hold_zero_is_straight():
    assert sc.command_for_offset(0) == 0.0


def test_cli_fit_exit_codes(capsys):
    ok = ['-50:+1.55', '-25:+3.4', '-10:+8.9', '10:-8.9', '25:-3.4', '50:-1.55']
    assert sc.main(['fit', *ok]) in (0, 1)
    assert 'tire_angle_to_steer_ratio' in capsys.readouterr().out
    assert sc.main(['fit', '10:3', '20:2']) == 2
