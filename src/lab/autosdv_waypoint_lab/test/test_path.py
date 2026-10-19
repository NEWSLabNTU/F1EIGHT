import math

import pytest

from autosdv_waypoint_lab.path import CoursePath, Path, load_csv, resample


def test_projection_sign_and_arc_length():
    p = Path([0.0, 10.0], [0.0, 0.0], [1.0, 1.0])
    left = p.project(3.0, 0.5)
    assert left.s == pytest.approx(3.0)
    assert left.cross_track == pytest.approx(0.5)
    assert p.project(3.0, -0.2).cross_track == pytest.approx(-0.2)
    assert p.point_at(4.0)[:2] == pytest.approx((4.0, 0.0))


def test_curvature_of_a_circle_is_one_over_radius_and_left_positive():
    n = 100
    xs = [2.0 * math.cos(2 * math.pi * k / n) for k in range(n)]
    ys = [2.0 * math.sin(2 * math.pi * k / n) for k in range(n)]
    p = Path(xs, ys, [1.0] * n)
    assert p.kappa[50] == pytest.approx(0.5, rel=1e-3)


def test_resample_spacing_and_endpoints():
    x, y, v = resample([0.0, 1.0, 1.0], [0.0, 0.0, 1.0], [0.5, 1.0, 1.5], 0.1)
    assert (x[0], y[0], x[-1], y[-1]) == (0.0, 0.0, 1.0, 1.0)
    gaps = [math.hypot(x[i + 1] - x[i], y[i + 1] - y[i]) for i in range(len(x) - 1)]
    assert max(gaps) <= 0.1 + 1e-9


def test_load_csv_header_comments_and_default_speed(tmp_path):
    f = tmp_path / 'p.csv'
    f.write_text('# a comment\nx,y,v\n0,0\n1,0,0.5\n')
    x, y, v = load_csv(str(f), 0.8)
    assert x == [0.0, 1.0] and v == [0.8, 0.5]


def square_loop():
    return [0.0, 4.0, 4.0, 0.0], [0.0, 0.0, 4.0, 4.0], [1.0] * 4


def test_lap_counting_and_finish():
    cp = CoursePath(*square_loop(), closed=True, laps=2)
    assert cp.lap_length == pytest.approx(16.0, rel=1e-3)
    for k in range(int(2 * 16.0 / 0.2) + 1):
        s = (k * 0.2) % 16.0
        side, t = int(s // 4), s % 4
        x, y = [(t, 0), (4, t), (4 - t, 4), (0, 4 - t)][side]
        cp.update_progress(x, y)
    assert cp.laps_done == 2
    assert cp.finished


def test_window_stops_at_the_end_of_the_run():
    cp = CoursePath(*square_loop(), closed=True, laps=1, decel=0.3)
    cp.progress = cp.run_length - 1.0
    w = cp.window(0.5, 3.0)
    assert w.v[-1] == 0.0
    # every point can still stop by the end at decel
    for s_i, v_i in zip(w.s, w.v):
        remaining = cp.run_length - (cp.progress - 0.5 + s_i)
        assert v_i <= math.sqrt(2 * 0.3 * max(remaining, 0.0)) + 0.05


def test_window_is_all_stop_once_finished():
    # an open 10 m straight; the car reaches the end, then its localized
    # pose settles 0.1 m short (the EKF pulling back an overshoot)
    cp = CoursePath([0.0, 10.0], [0.0, 0.0], [0.8, 0.8], closed=False, decel=0.3)
    cp.update_progress(0.0, 0.0)
    for k in range(1, 101):
        cp.update_progress(k * 0.1, 0.0)
    assert cp.finished
    cp.update_progress(9.9, 0.0)
    assert cp.finished
    w = cp.window(1.0, 6.0)
    assert max(w.v) == 0.0
    # before the finish the ramp short of the end is not zero
    fresh = CoursePath([0.0, 10.0], [0.0, 0.0], [0.8, 0.8], closed=False, decel=0.3)
    fresh.update_progress(9.9 - 2.0, 0.0)
    for k in range(21):
        fresh.update_progress(7.9 + k * 0.1, 0.0)
    assert not fresh.finished
    w = fresh.window(1.0, 6.0)
    assert w.speed_at(w.project(9.9, 0.0).s) > 0.1


def test_progress_does_not_jump_across_a_crossing():
    # a figure-8 through the origin: the car on one lobe must not be matched
    # to the other lobe's segment through the same point
    n = 200
    xs = [3.0 * math.sin(2 * math.pi * k / n) for k in range(n)]
    ys = [1.5 * math.sin(4 * math.pi * k / n) for k in range(n)]
    cp = CoursePath(xs, ys, [1.0] * n, closed=True, laps=1)
    prev = None
    for k in range(0, n, 2):
        prog, _ = cp.update_progress(xs[k], ys[k])
        if prev is not None:
            assert 0.0 <= prog - prev < 1.0
        prev = prog


@pytest.mark.parametrize('closed,laps', [(False, 1), (True, 2)])
def test_a_finished_run_counts_every_lap(closed, laps):
    # Speeds are 0 once the run is finished (within 0.05 m of the end), so the
    # car can come to rest short of the end, as the Lab 2 gate saw on
    # the earlier 24.71 m lab2_uturn: progress 24.6999 of 24.7100, finished, "0/1 laps".
    pts = square_loop() if closed else ([0.0, 10.0], [0.0, 0.0], [0.8, 0.8])
    cp = CoursePath(*pts, closed=closed, laps=laps)
    cp.progress = cp.run_length - 0.01
    assert cp.finished
    assert cp.laps_done == cp.laps
    cp.progress = cp.run_length - 0.10
    assert not cp.finished
    assert cp.laps_done == cp.laps - 1
