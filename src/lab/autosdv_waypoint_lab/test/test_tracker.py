import math

import pytest

from autosdv_waypoint_lab import kinematic_bench as kb
from autosdv_waypoint_lab.faulty_path_tracker import NanPathTracker, RaisingPathTracker
from autosdv_waypoint_lab.path import CoursePath, Path
from autosdv_waypoint_lab.reference.path_tracker import PathTracker as Reference
from autosdv_waypoint_lab.stub.path_tracker import PathTracker as Stub
from autosdv_waypoint_lab.tracker_api import (
    TrackerCommand, TrackerShell, VehicleGeometry, VehicleState, load_tracker)

V = VehicleGeometry()
STATE = VehicleState(0.0, 0.0, 0.0, 0.5)
PATH = Path([0.0, 5.0], [0.0, 0.0], [0.6, 0.6])


class Fixed:
    def __init__(self, steer, speed):
        self.cmd = TrackerCommand(steer, speed)

    def reset(self):
        pass

    def step(self, state, path, dt):
        return self.cmd


def test_shell_clamps_steering_and_path_speed():
    res = TrackerShell(Fixed(1.0, 3.0), V).step(STATE, PATH, 0.03)
    assert res.command.steering_angle == pytest.approx(V.max_steer_angle)
    assert res.command.speed == pytest.approx(0.6)
    assert 'steer' in res.limit and 'path_speed' in res.limit


@pytest.mark.parametrize('cls', [RaisingPathTracker, NanPathTracker])
def test_shell_stops_then_disables_after_three_faults(cls):
    shell = TrackerShell(cls({}, V), V, max_faults=3)
    for k in range(3):
        res = shell.step(STATE, PATH, 0.03)
        assert res.command.speed == 0.0 and res.fault
    assert res.disabled
    shell.reset()
    assert shell.step(STATE, PATH, 0.03).disabled


def test_load_tracker_by_import_path():
    t = load_tracker('autosdv_waypoint_lab.reference.path_tracker:PathTracker', {}, V)
    assert isinstance(t, Reference)
    with pytest.raises(ValueError):
        load_tracker('no_colon', {}, V)


def circle(r=2.0, n=120):
    return ([r * math.sin(2 * math.pi * k / n) for k in range(n)],
            [r - r * math.cos(2 * math.pi * k / n) for k in range(n)], [0.8] * n)


# Part 1 acceptance, against the reference law


def test_reference_follows_a_2m_circle_at_0_8():
    cp = CoursePath(*circle(), closed=True, laps=2)
    r = kb.run(Reference({}, V), cp, (0.0, 0.0, 0.0))
    assert r.finished and r.laps == 2
    assert r.cross_track_rmse <= 0.10


def test_reference_recovers_from_a_half_metre_offset_within_3m():
    cp = CoursePath([0.0, 15.0], [0.0, 0.0], [0.8, 0.8], closed=False)
    r = kb.run(Reference({}, V), cp, (0.0, 0.5, 0.0))
    settled = next(x for x, e in zip(r.x, r.cross_track) if abs(e) < 0.05)
    assert settled <= 3.0
    assert r.finished
    assert math.hypot(r.x[-1] - 15.0, r.y[-1]) < 0.3     # stopped at the end


def test_reference_never_asks_beyond_the_steering_limit():
    cp = CoursePath(*circle(1.0), closed=True, laps=1)    # tighter than the car can turn
    r = kb.run(Reference({}, V), cp, (0.0, 0.0, 0.0))
    assert r.max_steer_cmd <= V.max_steer_angle + 1e-9


def test_the_stub_does_not_move():
    cp = CoursePath(*circle(), closed=True)
    r = kb.run(Stub({}, V), cp, (0.0, 0.0, 0.0))
    assert not r.finished and max(r.speed) == 0.0


def test_reference_tolerates_localization_noise():
    cp = CoursePath(*circle(), closed=True, laps=1)
    r = kb.run(Reference({}, V), cp, (0.0, 0.0, 0.0),
               params=kb.BenchParams(pose_noise_std=0.03, yaw_noise_std=math.radians(1.0)))
    assert r.finished and r.cross_track_rmse <= 0.10


def test_reference_lookahead_does_not_shrink_at_the_path_end():
    """The target past the end continues along the last heading (car C, lab2_straight_2m_3:
    full lock 0.5 m before the stop, then the opposite lock past it)."""
    path = Path([0.0, 1.0, 2.0], [0.0, 0.0, 0.0], [0.8, 0.8, 0.8])
    ref = Reference({}, V)
    # 5 cm left of the line, 0.2 m before the end, at speed: a gentle correction to the right
    near = ref.step(VehicleState(1.8, 0.05, 0.0, 0.8), path, 0.03).steering_angle
    assert -0.1 < near < 0.0
    # past the end, still left of it: the same direction, not the opposite lock
    past = ref.step(VehicleState(2.1, 0.05, 0.0, 0.8), path, 0.03).steering_angle
    assert -0.1 < past < 0.0
    # before the end the target is the path's own point
    from autosdv_waypoint_lab.reference.path_tracker import target_point
    assert target_point(path, 1.5) == pytest.approx((1.5, 0.0))
    assert target_point(path, 2.6) == pytest.approx((2.6, 0.0))
