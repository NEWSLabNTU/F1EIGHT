"""Coast-down fit recovers the model's road load; the skidpad relation."""
from dataclasses import replace

import pytest

from autosdv_rc_sim.drivetrain import road_load
from autosdv_rc_sim.fit import fit_coastdown, skidpad_mu
from autosdv_rc_sim.params import load_vehicle
from autosdv_rc_sim.plant import Plant


def test_coastdown_fit_recovers_the_road_load():
    p = load_vehicle()
    plant = Plant(p)
    w = 9.0 / p.wheel_radius
    plant.state = replace(plant.state, vx=9.0, omega_f=w, omega_r=w)
    t, v = [], []
    for k in range(8000):
        plant.step(p.physics_dt, 370, 489)
        if k % 10 == 0 and plant.state.vx > 1.0:
            t.append(k * p.physics_dt)
            v.append(plant.state.vx)
    a0, a2 = fit_coastdown(t, v, p.effective_mass)
    for speed in (2.0, 6.0, 9.0):
        assert a0 + a2 * speed * speed == pytest.approx(road_load(p, speed), rel=0.05)


def test_skidpad_mu():
    assert skidpad_mu(2.0, 4.0) == pytest.approx(16.0 / (9.81 * 2.0))
