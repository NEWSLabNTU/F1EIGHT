import math

import pytest

from autosdv_waypoint_lab.course import (
    Course, Marker, Pose2D, load_course, make_ar_tag_params, make_osm, parse_osm_landmarks)


def course(markers):
    return Course('.', 'path.csv', True, 1, 0.8, Pose2D(0, 0, 0), 0.25, markers)


@pytest.mark.parametrize('yaw', [0.0, 1.0, math.pi, -2.3])
def test_osm_round_trip_gives_the_marker_pose_back(yaw):
    lm = parse_osm_landmarks(make_osm(course([Marker(3, 1.0, 2.0, 0.3, yaw)])))['3'][0]
    assert lm.position == pytest.approx((1.0, 2.0, 0.3), abs=1e-3)
    # the OSM stores coordinates to 0.1 mm
    assert math.cos(lm.facing_yaw - yaw) == pytest.approx(1.0, abs=1e-6)
    # landmark_manager's axes are the ArUco marker's: x to the viewer's right,
    # y up, z out of the marker
    assert lm.y_axis == pytest.approx((0.0, 0.0, 1.0), abs=1e-3)
    viewer_right = (-math.sin(yaw), math.cos(yaw), 0.0)
    assert lm.x_axis == pytest.approx(viewer_right, abs=1e-3)


def test_duplicate_ids_are_refused(tmp_path):
    (tmp_path / 'course.yaml').write_text(
        'markers:\n  - {id: 1, x: 0, y: 0, yaw: 0}\n  - {id: 1, x: 1, y: 0, yaw: 0}\n')
    with pytest.raises(ValueError):
        load_course(str(tmp_path))


def test_unknown_dictionary_is_refused(tmp_path):
    (tmp_path / 'course.yaml').write_text('dictionary: DICT_4X4_50\nmarkers: []\n')
    with pytest.raises(ValueError):
        load_course(str(tmp_path))


def test_ar_tag_params_name_a_shipped_dictionary_file():
    import yaml
    c = course([Marker(24, 3.0, 0.0, 0.5, math.pi)])
    c.marker_size, c.dictionary = 0.38, 'DICT_5X5_1000'
    p = yaml.safe_load(make_ar_tag_params(c))['/**']['ros__parameters']
    assert p['dictionary'] == ('$(find-pkg-share autosdv_waypoint_lab)'
                               '/config/dictionaries/DICT_5X5_1000.dict')
    assert p['marker_size'] == 0.38
    assert p['target_tag_ids'] == ['24']


def test_ar_tag_params_pass_an_aruco_name_through():
    c = course([Marker(1, 0, 0, 0, 0)])
    assert 'dictionary: "ALL_DICTS"' in make_ar_tag_params(c)


def test_shipped_dictionary_files_exist():
    import os
    from autosdv_waypoint_lab.course import SHIPPED_DICTIONARIES
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for name in SHIPPED_DICTIONARIES:
        assert os.path.isfile(os.path.join(here, 'config', 'dictionaries', name + '.dict'))


SHIPPED_COURSES = ('hall_demo', 'lab2_straight', 'lab2_circle_cw', 'lab2_circle_ccw')


def _course_dir(name):
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(here, 'config', 'courses', name)


@pytest.mark.parametrize('name', SHIPPED_COURSES)
def test_shipped_course_map_is_up_to_date(name):
    import os
    from autosdv_waypoint_lab import course as c
    d = _course_dir(name)
    with open(os.path.join(d, 'lanelet2_map.osm')) as f:
        assert f.read() == c.make_osm(c.load_course(d)), 'run: just lab2 course-map'
    with open(os.path.join(d, 'ar_tag.param.yaml')) as f:
        assert f.read() == c.make_ar_tag_params(c.load_course(d)), 'run: just lab2 course-map'


@pytest.mark.parametrize('pitch,roll', [(0.1, 0.0), (0.0, -0.08), (-0.05, 0.12)])
def test_a_tilted_marker_round_trips_with_its_tilt(pitch, roll):
    m = Marker(5, 1.0, 2.0, 0.4, 0.7, pitch, roll)
    lm = parse_osm_landmarks(make_osm(course([m])))['5'][0]
    xa, ya, za = m.axes()
    assert lm.position == pytest.approx((1.0, 2.0, 0.4), abs=1e-3)
    assert lm.x_axis == pytest.approx(xa, abs=2e-3)
    assert lm.y_axis == pytest.approx(ya, abs=2e-3)
    assert lm.z_axis == pytest.approx(za, abs=2e-3)
    # leaning back tips the normal upward
    if pitch > 0:
        assert za[2] > 0


def test_an_upright_marker_is_unchanged_by_the_tilt_fields():
    a = Marker(1, 0.5, -1.0, 0.25, 2.0)
    b = Marker(1, 0.5, -1.0, 0.25, 2.0, 0.0, 0.0)
    assert a.corners(0.25) == b.corners(0.25)


# ------------------------------------------------ the Lab 2 grading routes

LAB2_ROUTES = ('lab2_straight', 'lab2_circle_cw', 'lab2_circle_ccw')


@pytest.mark.parametrize('name', LAB2_ROUTES)
def test_lab2_route_is_covered_from_start_box_to_finish(name):
    from autosdv_waypoint_lab.marker_sim import CameraModel, coverage
    from autosdv_waypoint_lab.path import CoursePath, Path, load_csv
    c = load_course(_course_dir(name))
    x, y, v = load_csv(c.path_file, c.v_ref)
    cp = CoursePath(x, y, v, closed=c.closed)
    sp = c.start_pose
    cov = coverage(Path(cp.x, cp.y, cp.v), parse_osm_landmarks(make_osm(c)),
                   (sp.x, sp.y, sp.yaw), CameraModel(marker_size=c.marker_size))
    assert cov.fraction == 1.0
    assert cov.start_box_min >= 1


@pytest.mark.parametrize('name', LAB2_ROUTES)
def test_lab2_route_is_in_the_origin_boards_frame(name):
    # `just lab2 survey DIR ORIGIN` puts the origin board at (0, 0) facing +x
    # and keeps path.csv: the template must already be in that frame.
    from autosdv_waypoint_lab.path import load_csv
    c = load_course(_course_dir(name))
    origin = [m for m in c.markers if (m.x, m.y) == (0.0, 0.0)]
    assert len(origin) == 1 and origin[0].yaw == pytest.approx(0.0)
    assert all(m.z == pytest.approx(0.25) for m in c.markers)   # floor-standing 500 mm board
    x, y, _ = load_csv(c.path_file, c.v_ref)
    assert (c.start_pose.x, c.start_pose.y) == pytest.approx((x[0], y[0]), abs=1e-3)


def test_lab2_circles_share_one_ring_about_3_5_0():
    cw = load_course(_course_dir('lab2_circle_cw'))
    ccw = load_course(_course_dir('lab2_circle_ccw'))
    assert cw.markers == ccw.markers
    for m in cw.markers:
        assert math.hypot(m.x - 3.5, m.y) == pytest.approx(3.5, abs=1e-3)
        # each board faces the centre
        assert math.cos(math.atan2(-m.y, 3.5 - m.x) - m.yaw) == pytest.approx(1.0, abs=1e-6)


@pytest.mark.parametrize('name,radius,laps', [('lab2_circle_cw', 1.5, 3),
                                              ('lab2_circle_ccw', 2.5, 3),
                                              ('lab2_straight', None, 1)])
def test_reference_tracker_completes_the_lab2_route(name, radius, laps):
    from autosdv_waypoint_lab import kinematic_bench as kb
    from autosdv_waypoint_lab.path import CoursePath, load_csv
    from autosdv_waypoint_lab.reference.path_tracker import PathTracker
    from autosdv_waypoint_lab.tracker_api import VehicleGeometry
    c = load_course(_course_dir(name))
    x, y, v = load_csv(c.path_file, c.v_ref)
    cp = CoursePath(x, y, v, closed=c.closed, v_max=0.8, decel=0.3, laps=c.laps)
    veh = VehicleGeometry()
    r = kb.run(PathTracker({}, veh), cp, (c.start_pose.x, c.start_pose.y, c.start_pose.yaw), veh,
               kb.BenchParams(pose_noise_std=0.05, yaw_noise_std=math.radians(2.0)))
    assert r.finished and r.laps == laps
    assert r.cross_track_max < 0.1
    if radius is None:
        assert math.hypot(r.x[-1] - x[-1], r.y[-1] - y[-1]) < 0.15
    else:
        rad = [math.hypot(px - 3.5, py) for px, py, s in zip(r.x, r.y, r.speed) if s > 0.7]
        assert sum(rad) / len(rad) == pytest.approx(radius, abs=0.05)
