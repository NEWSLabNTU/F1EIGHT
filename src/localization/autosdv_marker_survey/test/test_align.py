"""survey-align (stage 2): the start-box frame from one straight drive."""
import math
import os
import random

import numpy as np
import pytest
import yaml

from autosdv_marker_survey import align as al
from autosdv_marker_survey.simulate import stage1_map

HERE = os.path.dirname(os.path.abspath(__file__))
# a route of this package's own (no Lab 2 package needed); Lab 2's routes are tested
# in autosdv_waypoint_lab/test/test_survey_align_routes.py
ROUTE = os.path.join(HERE, 'routes', 'test_route')


def drive(x0=1.0, y0=2.0, yaw0=0.3, rest=2.0, distance=3.0, speed=0.3, rate=50.0, noise=0.005,
          wiggle=0.0, heading_off=0.0, slope=0.0, seed=1, wheel=True, settle=0.0):
    """Poses of a car at rest at (x0, y0, yaw0) for `rest` s, then driving `distance`
    m straight at `speed` along yaw0 + heading_off, then stopping. `wiggle`: a
    lateral sine of that amplitude; `slope`: rad the path climbs."""
    rng = random.Random(seed)
    out = []
    t = 0.0
    dt = 1.0 / rate
    h = yaw0 + heading_off
    c, s = math.cos(h), math.sin(h)
    while t < rest:
        e = settle * math.exp(-t / 0.3)
        out.append(al.Sample(t, x0 + rng.gauss(0, noise) + e, y0 + rng.gauss(0, noise), 0.0,
                             yaw0 + rng.gauss(0, 0.003), 0.0, 0.0, 0.0 if wheel else math.nan))
        t += dt
    d = 0.0
    while d < distance:
        d = min(distance, d + speed * dt)
        lat = wiggle * math.sin(2 * math.pi * d / 1.5)
        out.append(al.Sample(t, x0 + c * d - s * lat + rng.gauss(0, noise),
                             y0 + s * d + c * lat + rng.gauss(0, noise), d * math.tan(slope),
                             h + rng.gauss(0, 0.003), 0.0, 0.0, speed if wheel else math.nan))
        t += dt
    for _ in range(int(rate)):
        out.append(al.Sample(t, x0 + c * d + rng.gauss(0, noise), y0 + s * d + rng.gauss(0, noise),
                             d * math.tan(slope), h, 0.0, 0.0, 0.0 if wheel else math.nan))
        t += dt
    return out


def test_line_fit_recovers_the_direction():
    rng = np.random.default_rng(0)
    th = math.radians(37.0)
    s = np.linspace(0, 3, 200)
    xy = np.column_stack([1 + s * math.cos(th), -2 + s * math.sin(th)]) + rng.normal(0, 0.01, (200, 2))
    c, u = al.fit_line(xy)
    assert abs(math.atan2(u[1], u[0]) % math.pi - th) < math.radians(0.3)


def test_motion_onset_by_wheel_and_by_displacement():
    for wheel in (True, False):
        ss = drive(rest=2.0, wheel=wheel)
        i = al.motion_onset(ss)
        assert ss[i].t == pytest.approx(2.0, abs=0.25)


def test_a_clean_drive_gives_the_start_frame():
    ss = drive(x0=1.0, y0=2.0, yaw0=0.3)
    a = al.align(ss)
    assert a.ok, a.problems
    assert (a.start_x, a.start_y) == pytest.approx((1.0, 2.0), abs=0.005)
    assert a.heading == pytest.approx(0.3, abs=math.radians(0.3))
    assert a.distance == pytest.approx(3.0, abs=0.03)
    assert a.lateral_max < 0.03 and a.heading_sd_deg < 0.5
    # a point 1 m ahead of the start and 0.5 m to its left
    x, y, yaw = a.to_frame(1.0 + math.cos(0.3) - 0.5 * math.sin(0.3),
                           2.0 + math.sin(0.3) + 0.5 * math.cos(0.3), 0.3 + 1.0)
    assert (x, y, yaw) == pytest.approx((1.0, 0.5, 1.0), abs=0.01)


def test_the_line_not_the_car_heading_defines_x():
    # the car stood 4 deg off the line it then drove: x follows the drive
    a = al.align(drive(yaw0=0.0, heading_off=math.radians(4.0)))
    assert a.ok, a.problems
    assert math.degrees(a.heading) == pytest.approx(4.0, abs=0.3)
    assert a.start_yaw_diff_deg == pytest.approx(4.0, abs=0.3)


@pytest.mark.parametrize('kw,why', [
    (dict(distance=0.8), 'drove 0.'),
    (dict(wiggle=0.15), 'not straight'),
    (dict(rest=0.2), 'at rest for'),
    (dict(heading_off=math.radians(20.0)), 'pointed'),
])
def test_refusals(kw, why):
    a = al.align(drive(**kw))
    assert not a.ok
    assert any(why in p for p in a.problems), a.problems
    f = al.align(drive(**kw), force=True)
    assert f.ok and any(w.startswith('FORCED') for w in f.warnings)


def test_a_car_that_never_moved_is_refused_even_forced():
    ss = drive(distance=0.0, rest=3.0)
    a = al.align(ss, force=True)
    assert not a.ok and any('never moved' in p for p in a.problems)


def test_short_but_legal_drive_warns():
    a = al.align(drive(distance=1.5))
    assert a.ok and any('recommended' in w or 'fixes the heading' in w for w in a.warnings)


def test_a_sloping_drive_warns_and_a_settling_start_warns():
    a = al.align(drive(slope=math.radians(3.0)))
    assert a.ok and a.slope_deg == pytest.approx(3.0, abs=0.2)
    assert any('climbs' in w for w in a.warnings)
    b = al.align(drive(settle=0.0, noise=0.03))
    assert any('spread' in w for w in b.warnings) or not b.ok


def _truth():
    """The route frame's boards (the template), with the anchor board 24 misplaced
    on the floor: turned 3 deg, 0.12 m off, leaning 10 deg back."""
    with open(os.path.join(ROUTE, 'course.yaml')) as f:
        d = yaml.safe_load(f)
    t = {m['id']: (m['x'], m['y'], 0.25, m['yaw'], 0.0, 0.0) for m in d['markers']}
    x, y, z, yaw, _, _ = t[24]
    t[24] = (x + 0.12, y - 0.07, z, yaw + math.radians(3.0), math.radians(10.0), 0.0)
    return t


def test_end_to_end_a_misframed_stage1_is_recovered():
    truth = _truth()
    stage1, T_w_s1 = stage1_map(truth, 24)
    # stage 1 is level although its origin board leans: z is the floor's, not the board's
    assert T_w_s1[:3, 2] == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)
    # the car really starts a little off the box: 4 cm, -3 cm, 1.5 deg
    start = (0.04, -0.03, math.radians(1.5))
    T_s1_w = np.linalg.inv(T_w_s1)
    p = T_s1_w @ [start[0], start[1], 0.0, 1.0]
    yaw_s1 = start[2] + math.atan2(T_s1_w[1, 0], T_s1_w[0, 0])
    a = al.align(drive(x0=p[0], y0=p[1], yaw0=yaw_s1, noise=0.004))
    assert a.ok, a.problems
    got = {m['id']: m for m in al.transform_markers(stage1, a)}
    c, s = math.cos(start[2]), math.sin(start[2])
    for mid, (x, y, z, yaw, pitch, _) in truth.items():
        dx, dy = x - start[0], y - start[1]
        ex, ey = c * dx + s * dy, -s * dx + c * dy
        m = got[mid]
        # a leaning board's centre stands where it is; stage 1 drops it onto the floor
        assert math.hypot(m['x'] - ex, m['y'] - ey) < 0.02, (mid, m, ex, ey)
        assert abs(al.wrap(m['yaw'] - (yaw - start[2]))) < math.radians(0.5)
        assert m['z'] == pytest.approx(z, abs=1e-3)
        assert m['pitch'] == pytest.approx(pitch, abs=1e-3)


def test_written_course_is_the_route_with_the_aligned_boards(tmp_path):
    truth = _truth()
    stage1, T_w_s1 = stage1_map(truth, 24)
    d = tmp_path / 'survey'
    d.mkdir()
    (d / 'course.yaml').write_text(yaml.safe_dump(
        {'path': 'path.csv', 'closed': True, 'laps': 3, 'v_ref': 0.8,
         'start_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'marker_size': 0.38,
         'dictionary': 'DICT_5X5_1000', 'survey': {'origin_marker': 24}, 'markers': stage1}))
    assert not al.is_aligned(str(d))
    s1 = al.prepare_stage1(str(d))
    assert os.path.isfile(os.path.join(s1, 'course.yaml'))
    T_s1_w = np.linalg.inv(T_w_s1)
    a, written = al.solve_dir(str(d), ROUTE, drive(x0=T_s1_w[0, 3], y0=T_s1_w[1, 3],
                                                   yaw0=math.atan2(T_s1_w[1, 0], T_s1_w[0, 0])),
                              log=lambda *x: None)
    assert a.ok and written
    assert al.is_aligned(str(d))
    # the route's path, byte for byte, and its keys
    assert (d / 'path.csv').read_bytes() == open(os.path.join(ROUTE, 'path.csv'), 'rb').read()
    from autosdv_aruco_localization.marker_map import load_marker_map
    c = load_marker_map(str(d))
    assert (c.start_pose.x, c.start_pose.y, c.start_pose.yaw) == (0.0, 0.0, 0.0)
    y = yaml.safe_load((d / 'course.yaml').read_text())
    assert len(c.markers) == len(truth) and y['closed'] is False and y['laps'] == 1
    assert y['alignment']['route'] == 'test_route'
    assert (d / 'align_report.txt').read_text().startswith('survey-align ALIGNED')
    # aligned: stage 2 again localizes on the kept stage-1 map, never on its own output
    assert al.prepare_stage1(str(d)) == s1
    assert yaml.safe_load(open(os.path.join(s1, 'course.yaml')))['markers'] == stage1
    # a new stage 1 into the same directory drops the alignment
    from autosdv_marker_survey import output
    from autosdv_marker_survey.survey import SurveyedMarker
    output.write_course(str(d), [SurveyedMarker(24, 0, 0, 0.25, 0, 0, 5, 0.1)], 24, 'DICT_5X5_1000',
                        0.38, None, {'keyframes': 5, 'reproj_rms_px': 0.1})
    assert not al.is_aligned(str(d))


def test_samples_round_trip(tmp_path):
    ss = drive()[:20]
    p = str(tmp_path / 'p.csv')
    al.write_samples(p, ss)
    back = al.read_samples(p)
    assert len(back) == len(ss) and back[5].x == pytest.approx(ss[5].x, abs=1e-5)


def _project(T_map_base, T_base_cam, K, pts):
    Tcm = np.linalg.inv(T_map_base @ T_base_cam)
    pc = (Tcm[:3, :3] @ pts.T).T + Tcm[:3, 3]
    uv = (K @ pc.T).T
    return uv[:, :2] / uv[:, 2:3]


def test_pose_from_markers_recovers_the_car_and_its_uncertainty():
    from autosdv_marker_survey.simulate import T_BASE_CAM
    from autosdv_marker_survey.survey import Detection, make_T
    from autosdv_aruco_localization.marker_map import Marker
    K = np.array([[403.0, 0, 480], [0, 403.0, 300], [0, 0, 1]])
    ms = [Marker(1, 4.0, 1.5, 0.25, math.pi), Marker(2, 4.5, -1.5, 0.25, math.pi),
          Marker(3, 9.0, 0.5, 0.25, math.pi)]
    corners = al.marker_corner_points(ms, 0.38)
    yaw = 0.05
    T = make_T(np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]),
               [0.3, -0.2, 0.0])
    rng = np.random.default_rng(2)
    dets = [Detection(i, _project(T, T_BASE_CAM, K, c) + rng.normal(0, 0.3, (4, 2)))
            for i, c in corners.items()]
    guess = make_T(np.eye(3), [0.0, 0.0, 0.0])
    Tb, rms, used, sd = al.pose_from_markers(dets, K, corners, T_BASE_CAM, guess)
    assert sorted(used) == [1, 2, 3] and rms < 1.0
    assert math.hypot(Tb[0, 3] - 0.3, Tb[1, 3] + 0.2) < 0.02
    assert math.atan2(Tb[1, 0], Tb[0, 0]) == pytest.approx(yaw, abs=math.radians(0.3))
    assert 0 < sd < 0.02
    # one far board alone: far less certain
    far = al.pose_from_markers([d for d in dets if d.id == 3], K, corners, T_BASE_CAM, guess)
    assert far is not None and far[3] > 3 * sd
    assert al.pose_from_markers([Detection(99, dets[0].corners)], K, corners, T_BASE_CAM, guess) is None


def test_multi_board_poses_beat_a_biased_ekf():
    # the EKF sits 7 cm off at the start box (one board's orientation, as on the bench) and
    # settles onto the truth along the drive; multi-board poses are unbiased
    ss = drive(x0=0.0, y0=0.0, yaw0=0.0, noise=0.001)
    biased = []
    for s in ss:
        k = math.exp(-max(0.0, s.x) / 1.0)
        biased.append(al.Sample(s.t, s.x - 0.05 * k, s.y + 0.05 * k, 0.0, s.yaw, 0.0, 0.0, s.speed))
    pnp = [al.PnpSample(s.t, s.x, s.y, s.yaw, 3, 0.4, 0.005) for s in ss[::5]]
    a = al.align(biased, pnp=pnp)
    assert a.ok and 'multi-board' in a.start_source and 'multi-board' in a.line_source
    assert math.hypot(a.start_x, a.start_y) < 0.005 and abs(math.degrees(a.heading)) < 0.1
    assert a.ekf_start_offset == pytest.approx(0.0707, abs=0.005)
    b = al.align(biased)            # without them: the EKF's, warned
    assert math.hypot(b.start_x, b.start_y) > 0.06
    assert any('multi-board poses at the start' in w for w in b.warnings)
    # uncertain multi-board poses are left out
    c = al.align(biased, pnp=[al.PnpSample(p.t, p.x, p.y + 0.1, p.yaw, 3, 0.4, 0.05) for p in pnp])
    assert 'EKF' in c.start_source


def test_solve_main_reuses_the_recorded_multi_board_poses(tmp_path):
    """marker_survey_align_solve re-solves as survey-align solved: with SURVEY_DIR/align_pnp.csv
    when the drive recorded one (--no-pnp: the EKF poses alone)."""
    truth = _truth()
    stage1, T_w_s1 = stage1_map(truth, 24)
    d = tmp_path / 'survey'
    d.mkdir()
    (d / 'course.yaml').write_text(yaml.safe_dump(
        {'path': 'path.csv', 'closed': False, 'laps': 1, 'v_ref': 0.8,
         'start_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'marker_size': 0.38,
         'dictionary': 'DICT_5X5_1000', 'survey': {'origin_marker': 24}, 'markers': stage1}))
    T = np.linalg.inv(T_w_s1)
    ss = drive(x0=T[0, 3], y0=T[1, 3], yaw0=math.atan2(T[1, 0], T[0, 0]))
    al.write_samples(str(d / al.POSES_FILE), ss)
    al.write_pnp(str(d / al.PNP_FILE), [al.PnpSample(s.t, s.x, s.y, s.yaw, 3, 0.4, 0.004) for s in ss])
    assert al.solve_main([str(d), '--route', ROUTE]) == 0
    assert 'line fitted to   the EKF' not in (d / al.REPORT_FILE).read_text()
    assert al.solve_main([str(d), '--route', ROUTE, '--no-pnp']) == 0
    assert 'line fitted to   the EKF' in (d / al.REPORT_FILE).read_text()
