import math
from types import SimpleNamespace as NS

import pytest

from autosdv_waypoint_lab import grading as g


def odo(x, y, v):
    return NS(pose=NS(pose=NS(position=NS(x=x, y=y))), twist=NS(twist=NS(linear=NS(x=v))))


def test_metrics_from_synthetic_messages():
    msgs = {
        g.MODE: [(0.0, NS(mode=1)), (1.0, NS(mode=g.AUTONOMOUS))],
        g.PROGRESS: [(t, NS(laps_done=min(int(t // 3), 3), laps=3, finished=t >= 9.0))
                     for t in range(0, 11)],
        g.TRACKER: [(1.0 + 0.1 * k, NS(speed=0.8, cross_track=0.1 if k % 2 else -0.1))
                    for k in range(50)],
        g.CONTROL: [(1.0 + 0.1 * k, NS(longitudinal=NS(velocity=0.8))) for k in range(50)],
        g.VELOCITY: [(0.95 + 0.1 * k, NS(longitudinal_velocity=0.7)) for k in range(50)],
        g.MRM: [(0.0, NS(state=g.MRM_NORMAL)), (9.5, NS(state=2))],
    }
    m = g.metrics_from_messages('run', msgs)
    assert m.laps == 3 and m.finished
    assert m.xt_rmse == pytest.approx(0.1) and m.xt_max == pytest.approx(0.1)
    assert m.speed_rmse == pytest.approx(0.1)
    assert m.mrm == 0          # after the finish: a parked car, not counted
    assert m.disengages == 0
    assert g.verdict('xt_rmse', m.xt_rmse).startswith('pass')


def test_disengage_before_the_finish_counts():
    msgs = {g.MODE: [(0.0, NS(mode=g.AUTONOMOUS)), (5.0, NS(mode=1))],
            g.PROGRESS: [(4.0, NS(laps_done=0, laps=3, finished=False))]}
    assert g.metrics_from_messages('run', msgs).disengages == 1
