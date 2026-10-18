"""Coast-down fit recovers the model's road load; the skidpad relation."""
from dataclasses import replace

import pytest

from autosdv_rc_sim.drivetrain import road_load
from autosdv_rc_sim.fit import fit_coastdown, main, skidpad_mu
from autosdv_rc_sim.params import load_vehicle
from autosdv_rc_sim.plant import Plant


def _coast(p):
    plant = Plant(p)
    w = 9.0 / p.wheel_radius
    plant.state = replace(plant.state, vx=9.0, omega_f=w, omega_r=w)
    t, v = [], []
    for k in range(8000):
        plant.step(p.physics_dt, 370, 489)
        if k % 10 == 0 and plant.state.vx > 1.0:
            t.append(k * p.physics_dt)
            v.append(plant.state.vx)
    return t, v


def test_coastdown_fit_recovers_the_road_load():
    p = load_vehicle()
    t, v = _coast(p)
    a0, a1, a2 = fit_coastdown(t, v, p.effective_mass)
    for speed in (2.0, 6.0, 9.0):
        fitted = a0 + a1 * speed + a2 * speed * speed
        assert fitted == pytest.approx(road_load(p, speed), rel=0.05)
    assert a2 == pytest.approx(p.aero_drag, rel=0.10)
    assert a1 == pytest.approx(p.drivetrain_viscous / (p.wheel_radius ** 2), abs=0.05)


def test_bad_input_raises():
    with pytest.raises(ValueError, match='usable'):
        fit_coastdown([0, 1, 2], [3, 2, 1], 1.0)
    with pytest.raises(ValueError, match='increasing'):
        fit_coastdown([0, 1, 1, 2, 3, 4, 5], [6, 5, 4, 3, 2, 1, 1], 1.0)
    with pytest.raises(ValueError, match='cannot separate'):
        fit_coastdown(list(range(10)), [3.0] * 10, 1.0)
    noisy = [3.0 + 0.01 * ((k * 7) % 5 - 2) for k in range(40)]
    with pytest.raises(ValueError, match='span 0.04 m/s'):
        fit_coastdown([0.05 * k for k in range(40)], noisy, 1.0)
    for bad in (float('nan'), float('inf')):
        with pytest.raises(ValueError, match='finite'):
            fit_coastdown(list(range(10)), [9, 8, 7, bad, 5, 4, 3, 2, 1.5, 1.2], 1.0)
    with pytest.raises(ValueError, match='finite'):
        fit_coastdown([0, 1, float('nan'), 3, 4, 5, 6], [7, 6, 5, 4, 3, 2, 1], 1.0)


def test_v_min_drops_standstill():
    p = load_vehicle()
    t, v = _coast(p)
    t2 = t + [t[-1] + 0.1 * (k + 1) for k in range(20)]
    v2 = v + [0.0] * 20
    a = fit_coastdown(t2, v2, p.effective_mass)
    b = fit_coastdown(t, v, p.effective_mass)
    assert a == pytest.approx(b, rel=1e-3)


def test_main_good_and_bad_csv(tmp_path, capsys):
    p = load_vehicle()
    t, v = _coast(p)
    good = tmp_path / 'good.csv'
    good.write_text('t,v\n' + ''.join(f'{a},{b}\n' for a, b in zip(t, v)))
    assert main(['coastdown', str(good)]) == 0
    assert 'lumped estimates' in capsys.readouterr().out
    bad = tmp_path / 'bad.csv'
    bad.write_text('time,speed\n1,2\n')
    assert main(['coastdown', str(bad)]) == 2
    assert 'columns t and v' in capsys.readouterr().err
    bad.write_text('t,v\n1,x\n')
    assert main(['coastdown', str(bad)]) == 2
    assert main(['coastdown', str(tmp_path / 'missing.csv')]) == 2


def test_skidpad_mu():
    assert skidpad_mu(2.0, 4.0) == pytest.approx(16.0 / (9.81 * 2.0))
