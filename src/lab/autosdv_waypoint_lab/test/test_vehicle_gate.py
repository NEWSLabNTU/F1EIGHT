from autosdv_waypoint_lab import vehicle_gate as vg

CAR_C = vg.CarPoints(init_pwm=385, forward_hold_pwm=390, reverse_arm_pwm=340,
                     control_cmd_timeout=0.3)


def run(model, car=CAR_C, dt=0.02):
    """Samples from a toy model: model(phase, t_in_phase, speed) -> pwm. The car moves
    at 0.6 m/s while the PWM drives, and stops 0.5 s after it no longer does."""
    out, t, speed, last_drive = [], 0.0, 0.0, -1e9
    for p, (a, b) in zip(vg.PHASES, vg.phase_times().values()):
        while t < b - 1e-9:
            pwm = model(p, t - a, speed)
            if pwm > car.no_drive:
                speed, last_drive = 0.6, t
            elif t - last_drive > 0.5:
                speed = 0.0
            out.append(vg.Sample(round(t, 4), p.name, pwm, speed))
            t += dt
    return out


def good(p, t, speed):
    """Drives when asked; brakes 0.4 s then neutral on 0, on a lost command, an MRM or
    an emergency."""
    if p.speed and not p.mrm and not p.emergency:
        return 400
    if p.speed is None and t < 0.3:          # the watchdog has not fired yet
        return 400
    return 375 if t < 0.4 and p.name != 'rest' else 385


def verdict(samples, car=CAR_C):
    return {c['check']: c['ok'] for c in vg.judge(samples, car)}


def test_a_good_model_passes_everything():
    v = verdict(run(good))
    assert all(v.values()), v


def test_a_wrong_neutral_fails_neutral_and_every_stop():
    v = verdict(run(lambda p, t, s: 370 if good(p, t, s) == 385 else good(p, t, s)))
    assert not v['neutral'] and not v['stop'] and not v['watchdog'] and not v['mrm']
    assert v['drives'] and v['no reverse']


def test_no_watchdog_fails_watchdog_only():
    v = verdict(run(lambda p, t, s: 400 if p.name == 'watchdog' else good(p, t, s)))
    assert not v['watchdog']
    assert v['stop'] and v['mrm'] and v['emergency'] and v['neutral']


def test_ignoring_the_mrm_fails_mrm():
    v = verdict(run(lambda p, t, s: 400 if p.mrm else good(p, t, s)))
    assert not v['mrm'] and v['emergency']


def test_a_brake_that_arms_reverse_fails_no_reverse():
    v = verdict(run(lambda p, t, s: 340 if good(p, t, s) == 375 else good(p, t, s)))
    assert not v['no reverse']


def test_a_model_that_never_drives_is_not_called_safe():
    v = verdict(run(lambda p, t, s: 385))
    assert not v['drives']
    assert not v['watchdog'] and not v['mrm'] and not v['emergency']   # not tested
    assert v['neutral'] and v['stop']


def test_car_points_read_the_files_in_order(tmp_path):
    a = tmp_path / 'actuator.yaml'
    a.write_text('/**:\n  ros__parameters:\n    init_pwm: 370\n    control_cmd_timeout: 0.3\n'
                 '    forward_hold_pwm: 0\n    reverse_arm_pwm: 0\n')
    c = tmp_path / 'car.yaml'
    c.write_text('/**:\n  ros__parameters:\n    init_pwm: 380\n    reverse_arm_pwm: 345\n')
    p = vg.car_points([str(a), '', str(c)])
    assert (p.init_pwm, p.reverse_arm_pwm, p.no_drive) == (380, 345, 380)
