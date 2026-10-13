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
    c.marker_size, c.dictionary = 0.48, 'DICT_5X5_1000'
    p = yaml.safe_load(make_ar_tag_params(c))['/**']['ros__parameters']
    assert p['dictionary'] == ('$(find-pkg-share autosdv_waypoint_lab)'
                               '/config/dictionaries/DICT_5X5_1000.dict')
    assert p['marker_size'] == 0.48
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


def test_shipped_course_map_is_up_to_date():
    import os
    from autosdv_waypoint_lab import course as c
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    d = os.path.join(here, 'config', 'courses', 'hall_demo')
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
