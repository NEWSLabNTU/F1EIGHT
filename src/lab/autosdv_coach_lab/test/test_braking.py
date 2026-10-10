"""lab2_braking_report: reaction and deceleration per stop, and the envelope."""
import math

import numpy as np
import pytest

from autosdv_vehicle_interface.motor_plant import HallSpeedSensor

from autosdv_coach_lab import pursuit
from autosdv_coach_lab.calibration import braking as br


def stop_trace(v0=1.0, t_trig=5.0, reaction=0.3, decel=1.5, dt=0.05, end=12.0):
    t = np.arange(0.0, end, dt)
    v = np.where(t < t_trig + reaction, v0, np.maximum(v0 - decel * (t - t_trig - reaction), 0.0))
    return t, v


def test_ideal_stop():
    t, v = stop_trace()
    events, skipped = br.analyse(t, v, [br.Trigger(5.0, 'disengage')])
    assert not skipped
    e, = events
    assert e.v0 == pytest.approx(1.0)
    assert e.reaction == pytest.approx(0.3, abs=0.02)
    assert e.a_hinge == pytest.approx(1.5, rel=0.03)
    assert e.a_mean == pytest.approx(1.5, rel=0.05)
    # distance = v0 t_r + v0^2 / 2a; a_eff inverts exactly that.
    assert e.distance == pytest.approx(0.3 + 1.0 / 3.0, abs=0.02)
    assert e.a_eff == pytest.approx(1.5, rel=0.05)
    assert e.t_stop == pytest.approx(0.3 + 1.0 / 1.5 - 0.05 / 1.5, abs=0.06)


def test_hall_sensor_errs_on_the_safe_side():
    """Through the velocity_report model the EMA rounds the corner, so the
    hinge may put the onset earlier with a shallower slope; what must hold is
    that the pair (reaction, a_eff) the envelope uses predicts a stopping
    distance no shorter than the true one."""
    t, v = stop_trace(dt=0.001)
    s = HallSpeedSensor()
    dist = np.concatenate(([0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))))
    rt, rv = [], []
    for ti, di in zip(t, dist):
        s.update(ti, di)
        if abs(ti / 0.05 - round(ti / 0.05)) < 1e-6:
            rt.append(ti)
            rv.append(s.report(ti))
    e, = br.analyse(np.array(rt), np.array(rv), [br.Trigger(5.0, 'disengage')])[0]
    true_distance = 0.3 + 1.0 / 3.0
    predicted = e.v0 * e.reaction + e.v0 ** 2 / (2.0 * e.a_eff)
    assert predicted >= true_distance
    assert predicted < true_distance + 0.5      # but not absurdly so
    assert e.a_eff <= 1.5 * 1.03


def test_skips_and_followed_by():
    t, v = stop_trace()
    trig = [br.Trigger(2.0, 'board_drop'), br.Trigger(5.0, 'disengage'),
            br.Trigger(5.4, 'emergency', 'MRM'), br.Trigger(11.0, 'board_drop')]
    v = v.copy()
    v[t < 3.0] = 0.0          # parked until 3 s
    events, skipped = br.analyse(t, v, trig)
    assert [s.trigger.t for s in skipped] == [2.0, 11.0]
    assert 'not moving' in skipped[0].reason
    e, = events
    assert [f.kind for f in e.followed_by] == ['emergency']


def test_recommendation_and_speed_cap():
    t, v = stop_trace(reaction=0.3, decel=1.5)
    t2, v2 = stop_trace(t_trig=5.0, reaction=0.5, decel=0.8, v0=0.8)
    e1, = br.analyse(t, v, [br.Trigger(5.0, 'disengage')])[0]
    e2, = br.analyse(t2, v2, [br.Trigger(5.0, 'board_stop')])[0]
    rec = br.recommend([e1, e2], reaction_margin=0.2, decel_margin=0.75)
    # Reaction from every stop, deceleration from the commanded ones only.
    assert rec.reaction_time == pytest.approx(0.7, abs=0.051)
    assert rec.decel_from == ['disengage']
    assert rec.envelope_decel == pytest.approx(math.floor(e1.a_eff * 0.75 / 0.05) * 0.05)
    only_board = br.recommend([e2])
    assert only_board.notes and only_board.decel_from == ['board_stop']
    v_cap = br.braking_speed(1.0, rec.envelope_decel, rec.reaction_time)
    p = pursuit.PlannerParams(envelope_decel=rec.envelope_decel, reaction_time=rec.reaction_time)
    assert v_cap == pytest.approx(pursuit.braking_speed(1.0, p))
    # The range at which a speed is allowed inverts the cap.
    assert br.range_for_speed(v_cap, rec.envelope_decel, rec.reaction_time, 1.0) == \
        pytest.approx(2.0)
    current = {'standoff_min': 1.0, 'v_max': 1.5, 'envelope_decel': 0.6, 'reaction_time': 0.5}
    text = br.format_report([e1, e2], [], rec, current)
    assert 'speed cap at 2.0 m' in text and 'reaction_time:' in text


def test_board_stop_from_positions():
    """A board walking at 0.8 m/s that stops dead at 10 s, seen at 10 Hz."""
    t = np.arange(0.0, 15.0, 0.1)
    s = np.where(t < 10.0, 0.8 * t, 8.0)
    rng = np.random.default_rng(0)
    x, y = s + rng.normal(0, 0.01, len(t)), rng.normal(0, 0.01, len(t))
    sp = br.window_speed(t, x, y)
    stops = br.board_stop_times(t, sp)
    assert len(stops) == 1 and stops[0] == pytest.approx(10.0, abs=0.1)
    # A board that only slows down is not a stop.
    s2 = np.where(t < 10.0, 0.8 * t, 8.0 + 0.5 * (t - 10.0))
    assert br.board_stop_times(t, br.window_speed(t, s2, 0 * t)) == []


def test_board_in_world_uses_the_ego_pose():
    ex = np.array([0.0, 1.0])
    det = br.board_in_world([0.0, 1.0], [[2.0, 0.0], [2.0, 0.0]],
                            [0.0, 1.0], np.column_stack((ex, [0.0, 0.0])), [0.0, math.pi / 2])
    assert np.allclose(det[0], [2.0, 1.0]) and np.allclose(det[1], [0.0, 2.0])


def test_edges():
    assert br.edges([0, 1, 2, 3], [True, True, False, True], rising=False) == [2.0]
    assert br.edges([0, 1, 2, 3], [True, True, False, True], rising=True) == [3.0]
