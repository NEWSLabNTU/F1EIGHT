import functools
import math

import pytest

from autosdv_waypoint_lab.course import load_course, make_osm, parse_osm_landmarks

# The marker-map tests (the OSM, ar_tag.param.yaml, the dictionaries) moved with the code
# to autosdv_aruco_localization/test/test_marker_map.py; these are Lab 2's courses.


SHIPPED_COURSES = ('hall_demo', 'lab2_uturn')


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


# ------------------------------------------------ the Lab 2 graded route (lab2_uturn)

def _uturn_spec():
    import yaml
    with open(_course_dir('lab2_uturn') + '/course.yaml') as f:
        d = yaml.safe_load(f)
    return float(d['uturn']['radius']), float(d['uturn']['leg'])


@functools.lru_cache(maxsize=None)
def _layout(radius, leg):
    from autosdv_waypoint_lab import uturn
    return uturn.make_layout(radius, leg)


def test_lab2_uturn_path_is_the_declared_geometry():
    # path.csv is the route course.yaml declares (radius, leg), in the start-box
    # frame: a right U-turn, legs 2R apart, the stop point level with the start
    # box in the other lane.
    from autosdv_waypoint_lab import uturn
    from autosdv_waypoint_lab.path import load_csv, turn_spans
    radius, leg = _uturn_spec()
    c = load_course(_course_dir('lab2_uturn'))
    x, y, _ = load_csv(c.path_file, c.v_ref)
    xt, yt = uturn.route_polyline(radius, leg)
    assert x == pytest.approx([leg - v for v in xt], abs=1e-3)
    assert y == pytest.approx([-v for v in yt], abs=1e-3)
    assert not c.closed and c.laps == 1 and c.v_ref == pytest.approx(0.8)
    assert (c.start_pose.x, c.start_pose.y, c.start_pose.yaw) == (0.0, 0.0, 0.0)
    assert (x[0], y[0]) == (0.0, 0.0) and (x[1], y[1]) == pytest.approx((0.25, 0.0))
    assert (x[-1], y[-1]) == pytest.approx((0.0, -2 * radius), abs=1e-3)
    (s0, s1), = turn_spans(x, y)
    assert (s0, s1) == pytest.approx((leg, leg + math.pi * radius), abs=0.02)
    # turning right: driving +x, the car's right and the turn centre are on the -y side
    assert max(y) == pytest.approx(0.0) and min(y) == pytest.approx(-2 * radius)
    assert max(x) == pytest.approx(leg + radius, abs=1e-3)


def test_lab2_uturn_is_in_the_start_box_frame():
    # `just lab2 survey-align` writes the boards in the start-box frame (origin at
    # the start box, x along the first leg) beside this path.csv: the template's
    # boards are a placement guide measured from the start box. The anchor board
    # stands on the first leg's line beyond the turn, facing the start box.
    c = load_course(_course_dir('lab2_uturn'))
    radius, leg = _uturn_spec()
    anchor = c.markers[0]
    assert anchor.id == 24 and anchor.y == 0.0 and anchor.x > leg + radius
    assert abs(anchor.yaw) == pytest.approx(math.pi, abs=1e-3)
    assert all(m.z == pytest.approx(0.25) for m in c.markers)   # floor-standing 500 mm board
    assert sorted(m.id for m in c.markers) == list(range(24, 24 + len(c.markers)))


def _check_coverage(xs, ys, boards, start, stop):
    from autosdv_waypoint_lab import uturn
    from autosdv_waypoint_lab.course import Course, Marker, Pose2D
    c = Course('.', 'path.csv', False, 1, 0.8, Pose2D(*start), 0.38,
               [Marker(i, bx, by, 0.25, byaw) for i, (bx, by, byaw) in enumerate(boards)], 'DICT_5X5_1000')
    cov, turn, _, _, _ = uturn.course_coverage(c, (xs, ys))
    assert cov.start_box_min >= 2                  # a fix (two) before the car moves
    assert cov.longest_gap <= 3.2                  # ~4 s at 0.8 m/s, under error_age 5 s
    assert turn == 1.0                             # a board in view through the whole turn
    cam = uturn.CameraModel(marker_size=0.38)
    lms = [lm for v in parse_osm_landmarks(make_osm(c)).values() for lm in v]
    assert min(sum(1 for lm in lms if uturn.visible(cam, *p, lm))
               for p in uturn._start_box((stop[0], stop[1], math.pi))) >= 1    # a fix at the stop
    d = uturn._point_path_distance([b[0] for b in boards], [b[1] for b in boards],
                                   *uturn.car_track(xs, ys))
    assert d.min() >= 1.4 - 1e-6                   # 1 m clear of the car


def test_lab2_uturn_shipped_boards_cover_the_route():
    from autosdv_waypoint_lab.path import load_csv
    c = load_course(_course_dir('lab2_uturn'))
    x, y, _ = load_csv(c.path_file, c.v_ref)
    _check_coverage(x, y, [(m.x, m.y, m.yaw) for m in c.markers],
                    (c.start_pose.x, c.start_pose.y, c.start_pose.yaw), (x[-1], y[-1]))


def test_lab2_uturn_shipped_files_are_what_make_uturn_writes():
    # every file of the course is generated: edit the radius, not the files
    import os
    from autosdv_waypoint_lab import uturn
    radius, leg = _uturn_spec()
    d = _course_dir('lab2_uturn')
    for name, text in uturn.files(_layout(radius, leg), d).items():
        with open(os.path.join(d, name)) as f:
            assert f.read() == text, f'{name}: run just lab2 make-uturn {radius:g} --write'


# other radii and leg lengths: the generator must meet the goals there too
def test_spec_tex_states_the_shipped_geometry():
    # the spec's \\input: one macro per number it prints, and every board for its map
    import re
    from autosdv_waypoint_lab import uturn
    radius, leg = _uturn_spec()
    lay = _layout(radius, leg)
    tex = uturn.spec_tex(lay)
    macros = dict(re.findall(r'\\def\\(uturn\w+)\{([^}]*)\}', tex))
    assert float(macros['uturnR']) == radius and float(macros['uturnLeg']) == leg
    assert float(macros['uturnStartX']) == pytest.approx(lay.start[0], abs=0.01)
    assert (float(macros['uturnStopX']), float(macros['uturnStopY'])) == pytest.approx(lay.stop, abs=0.01)
    boards = [b.split('/') for b in macros['uturnBoards'].split(', ')]
    assert len(boards) == int(macros['uturnBoardCount']) == len(lay.boards)
    assert [int(b[3]) for b in boards] == [b.id for b in lay.boards]
    assert sum(int(b[4]) for b in boards) == 1


@pytest.mark.parametrize('radius,leg', [(1.0, 6.0), (2.5, 14.0)])
def test_make_uturn_meets_the_goals_at_other_sizes(radius, leg, tmp_path):
    from autosdv_waypoint_lab import bag_meta as bm
    from autosdv_waypoint_lab import route_grade as rg
    from autosdv_waypoint_lab import uturn
    lay = _layout(radius, leg)
    assert lay.ok, lay.problems
    assert (lay.radius, lay.leg) == (radius, leg)
    assert lay.start == (0.0, 0.0, 0.0)
    assert lay.boards[0].y == 0.0 and lay.boards[0].x > leg + radius
    assert lay.stop == pytest.approx((0.0, -2 * radius))
    assert lay.path_length == pytest.approx(2 * leg + math.pi * radius, abs=0.01)
    _check_coverage(lay.xs, lay.ys, [(b.x, b.y, b.yaw) for b in lay.boards], lay.start, lay.stop)
    for name, text in uturn.files(lay, str(tmp_path)).items():
        (tmp_path / name).write_text(text)
    c = load_course(str(tmp_path))
    assert (tmp_path / 'lanelet2_map.osm').read_text() == make_osm(c)
    assert bm.classify_route(str(tmp_path)) == 'uturn'
    geo = rg.geometry_from_course(str(tmp_path), 'uturn')
    assert geo.turns == [pytest.approx((leg, leg + math.pi * radius), abs=0.02)]
    import yaml
    assert yaml.safe_load((tmp_path / 'course.yaml').read_text())['uturn'] == {'radius': radius, 'leg': leg}
    assert (geo.dir_x, geo.dir_y) == pytest.approx((-1.0, 0.0))


def test_reference_tracker_completes_the_lab2_uturn():
    from autosdv_waypoint_lab import kinematic_bench as kb
    from autosdv_waypoint_lab.path import CoursePath, load_csv
    from autosdv_waypoint_lab.reference.path_tracker import PathTracker
    from autosdv_waypoint_lab.tracker_api import VehicleGeometry
    c = load_course(_course_dir('lab2_uturn'))
    x, y, v = load_csv(c.path_file, c.v_ref)
    cp = CoursePath(x, y, v, closed=c.closed, v_max=0.8, decel=0.3, laps=c.laps)
    veh = VehicleGeometry()
    r = kb.run(PathTracker({}, veh), cp, (c.start_pose.x, c.start_pose.y, c.start_pose.yaw), veh,
               kb.BenchParams(pose_noise_std=0.05, yaw_noise_std=math.radians(2.0)))
    assert r.finished and r.laps == 1
    # R 1.1 m: the default lookahead at 0.8 m/s (1.08 m) is about the radius, so the
    # car cuts the turn entry: 0.097 m without noise, 0.089-0.116 m over 10 noise
    # seeds (R 1.5 m gave 0.066-0.077). The lookahead is the teams' knob, so the
    # default stays; the grade's validity limit is PATH_DEV_MAX 0.30 m.
    assert r.cross_track_max < 0.15
    assert math.hypot(r.x[-1] - x[-1], r.y[-1] - y[-1]) < 0.15


def test_the_old_dictionary_path_still_resolves():
    # ar_tag.param.yaml files generated before the ArUco localization moved to
    # autosdv_aruco_localization (surveys/ on the cars) name this package's copy
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    p = os.path.join(here, 'config', 'dictionaries', 'DICT_5X5_1000.dict')
    assert os.path.isfile(p) and open(p).readline().strip() == 'name DICT_5X5_1000'
