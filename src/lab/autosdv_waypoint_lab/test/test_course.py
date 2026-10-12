import math

import pytest

from autosdv_waypoint_lab.course import (
    Course, Marker, Pose2D, load_course, make_osm, parse_osm_landmarks)


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


def test_shipped_course_map_is_up_to_date():
    import os
    from autosdv_waypoint_lab import course as c
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    d = os.path.join(here, 'config', 'courses', 'hall_demo')
    with open(os.path.join(d, 'lanelet2_map.osm')) as f:
        assert f.read() == c.make_osm(c.load_course(d)), 'run: just lab2 course-map'
