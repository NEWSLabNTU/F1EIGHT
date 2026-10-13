import pytest

from autosdv_waypoint_lab.loc_check_node import PoseHistory


def test_history_interpolates_at_the_fix_stamp():
    h = PoseHistory()
    for i in range(11):
        h.add(10.0 + 0.1 * i, 0.08 * i, 0.0)        # 0.8 m/s along x
    assert h.at(10.25) == pytest.approx((0.2, 0.0))
    assert h.at(10.0) == pytest.approx((0.0, 0.0))
    assert h.at(11.0) == pytest.approx((0.8, 0.0))


def test_history_outside_its_span_is_none():
    h = PoseHistory()
    assert h.at(1.0) is None
    h.add(1.0, 0.0, 0.0)
    h.add(2.0, 1.0, 0.0)
    assert h.at(0.5) is None and h.at(2.5) is None
    assert h.at(2.05) == (1.0, 0.0)                  # just past the newest: the newest


def test_history_forgets_beyond_its_horizon_and_restarts_on_a_clock_jump():
    h = PoseHistory(horizon=1.0)
    for i in range(30):
        h.add(0.1 * i, float(i), 0.0)
    assert h.at(1.0) is None and h.at(2.5) is not None
    h.add(0.0, 5.0, 5.0)                             # clock went back
    assert h.at(0.0) == (5.0, 5.0) and h.at(2.5) is None
