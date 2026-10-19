import math

import pytest

from autosdv_waypoint_lab import steer_cal as sc

CAR_A = sc.Servo(489, 439, 539, -1000.0)
CAR_B = sc.Servo(400, 355, 445, -132.0)


def _circles(ratio, centre=0.0, offsets=(-45, -25, -10, 10, 25, 45), noise=0.0):
    """Radii a car with this ratio would trace; positive offset turns left iff ratio > 0."""
    out = []
    for o in offsets:
        delta = (o - centre) / ratio
        out.append((o, math.copysign(sc.WHEEL_BASE / math.tan(abs(delta)), delta) * (1 + noise)))
    return out


def test_fit_recovers_ratio_and_lock():
    r = sc.fit(_circles(-143.0), CAR_B)
    assert r.ratio == pytest.approx(-143.0, rel=1e-6)
    assert r.max_angle == pytest.approx(45 / 143.0, rel=1e-6)    # car B: +-45 counts
    assert r.centre == pytest.approx(0.0, abs=1e-6)
    assert r.angle_needed == pytest.approx(math.atan(sc.WHEEL_BASE / sc.R_FIRST_CIRCLE))
    assert r.min_radius == pytest.approx(sc.WHEEL_BASE / math.tan(r.max_angle))
    assert r.margin_ok == (r.min_radius <= sc.MAX_MIN_RADIUS)


def test_the_gate_is_the_graded_uturn_radius():
    import yaml
    from autosdv_waypoint_lab import route_grade as rg
    with open(rg.template_dir('uturn') + '/course.yaml') as f:
        assert sc.R_FIRST_CIRCLE == float(yaml.safe_load(f)['uturn']['radius'])
    assert sc.MAX_MIN_RADIUS == pytest.approx(sc.R_FIRST_CIRCLE - 0.2)


def test_the_stand_in_ratio_passes():
    # -143 gives 0.350 rad of lock: a 0.87 m minimum radius, inside 0.9 m at R 1.1 m
    assert sc.fit(_circles(-143.0), CAR_A).margin_ok          # car A: +-50 counts


def test_fit_sees_a_centre_error():
    assert sc.fit(_circles(-143.0, centre=4.0), CAR_B).centre == pytest.approx(4.0, abs=1e-6)


def test_the_shipped_ratio_cannot_hold_the_tight_circle():
    # the servo window would be reached at 0.05 rad: the tightest circle needs far more
    r = sc.fit(_circles(-1000.0, offsets=(-50, -25, 25, 50)), CAR_A)
    assert r.max_angle == pytest.approx(0.05)
    assert not r.margin_ok
    assert any('FAIL' in line for line in r.lines())


def test_a_marginal_car_fails():
    # the ratio whose lock turns the car on exactly MAX_MIN_RADIUS
    lock = math.atan(sc.WHEEL_BASE / sc.MAX_MIN_RADIUS)
    limit = CAR_B.half_range / lock
    assert not sc.fit(_circles(-1.05 * limit), CAR_B).margin_ok     # less lock: radius over the gate
    assert sc.fit(_circles(-0.99 * limit), CAR_B).margin_ok          # a little more lock: inside


def test_measurement_noise_is_tolerated():
    assert sc.fit(_circles(-143.0, noise=0.03), CAR_B).ratio == pytest.approx(-143.0, rel=0.1)


def test_positive_ratio_is_flagged():
    assert any('LEFT' in line for line in sc.fit(_circles(+143.0), CAR_B).lines())


def test_too_few_or_degenerate_circles_are_refused():
    with pytest.raises(ValueError):
        sc.fit([(10, 3.0), (25, 1.2)], CAR_B)
    with pytest.raises(ValueError):
        sc.fit([(10, 3.0), (10, 3.0), (10, 3.0)], CAR_B)


def test_fit_refuses_offsets_the_servo_clamped():
    with pytest.raises(ValueError, match='outside the servo window'):
        sc.fit(_circles(-132.0, offsets=(-50, -25, 25, 50)), CAR_B)     # car B is +-45


@pytest.mark.parametrize('side,measured,axle', [('axle', 1.5, 1.5),
                                                ('inner', 1.4, 1.4 + 0.1315),
                                                ('outer', 1.6, 1.6 - 0.1315)])
def test_tyre_track_radius_converts_to_axle_radius(side, measured, axle):
    assert sc.radius_at_axle(measured, side) == pytest.approx(axle)


def _actuator_pwm(servo, delta):
    """actuator_core: clamp to max_steer_angle, init_steer + int(average * ratio), clamp PWM."""
    delta = max(-servo.max_steer_angle, min(servo.max_steer_angle, delta))
    pwm = servo.init_steer + int(sc.average_wheel_angle(delta, servo.wheel_base,
                                                        servo.wheel_tread) * servo.ratio)
    return max(servo.min_steer, min(servo.max_steer, pwm))


@pytest.mark.parametrize('servo', [CAR_A, CAR_B, sc.Servo(400, 355, 445, 143.0)])
@pytest.mark.parametrize('offset', [-50, -45, -25, -10, 10, 25, 45, 50])
def test_hold_makes_the_actuator_write_that_offset_or_refuses(servo, offset):
    delta, why = sc.hold_check(servo, offset)
    if delta is None:
        assert why
        lo, hi = servo.window
        assert not lo <= offset <= hi or 'clamp' in why
        return
    assert _actuator_pwm(servo, delta) == servo.init_steer + offset


def test_hold_refuses_offsets_outside_car_b_window():
    delta, why = sc.hold_check(CAR_B, 50)
    assert delta is None and 'window -45..+45' in why and '355..445' in why
    assert sc.hold_check(CAR_B, -45)[0] is not None


def test_hold_on_car_b_is_not_car_a():
    # the old hard-coded -1000 commanded ~7.6x too small an angle on car B
    d_b, _ = sc.hold_check(CAR_B, -25)
    d_a, _ = sc.hold_check(CAR_A, -25)
    assert abs(d_b) / abs(d_a) == pytest.approx(1000 / 132, rel=0.1)


def test_hold_zero_is_straight():
    assert sc.command_for_offset(0, -132.0) == 0.0


def test_servo_from_node_params():
    p = {'init_steer': 400, 'min_steer': 355, 'max_steer': 445,
         'tire_angle_to_steer_ratio': -132.0, 'max_steer_angle': 0.349,
         'wheel_base': 0.319, 'wheel_tread': 0.263}
    s = sc.Servo.from_params(p, sc.ACTUATOR_NODE)
    assert (s.init_steer, s.window, s.half_range, s.ratio) == (400, (-45, 45), 45, -132.0)
    with pytest.raises(ValueError, match='/autosdv/actuator_node: missing init_steer'):
        sc.Servo.from_params({**p, 'init_steer': None}, sc.ACTUATOR_NODE)


def test_servo_half_range_is_the_nearer_limit():
    assert sc.Servo(400, 350, 445, -132.0).half_range == 45


def test_cli_fit_exit_codes(capsys):
    ok = ['-45:+0.9', '-25:+1.7', '-10:+4.2', '10:-4.2', '25:-1.7', '45:-0.9']
    assert sc.main(['fit', '--vehicle-id', 'car_b', *ok]) in (0, 1)
    out = capsys.readouterr().out
    assert 'tire_angle_to_steer_ratio' in out and 'individual_params/config/car_b/autosdv_vehicle/actuator.param.yaml' in out
    assert 'init_steer 400' in out
    assert sc.main(['fit', '--vehicle-id', 'car_b', '10:3', '20:2']) == 2
    assert sc.main(['fit', '--vehicle-id', 'car_b', '-50:+0.9', '-25:+1.7', '25:-1.7']) == 2


def test_cli_fit_needs_a_car():
    with pytest.raises(SystemExit):
        sc.main(['fit', '-25:+1.7', '-10:+4.2', '10:-4.2'])
