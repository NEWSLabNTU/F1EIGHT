import math
import os

import pytest
import yaml

from autosdv_aruco_localization.marker_map import (
    SHIPPED_DICTIONARIES, Marker, MarkerMap, Pose2D, load_marker_map, make_ar_tag_params,
    make_osm, parse_osm_landmarks, write_map_files)

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def mmap(markers):
    return MarkerMap('.', Pose2D(0, 0, 0), 0.25, markers)


@pytest.mark.parametrize('yaw', [0.0, 1.0, math.pi, -2.3])
def test_osm_round_trip_gives_the_marker_pose_back(yaw):
    lm = parse_osm_landmarks(make_osm(mmap([Marker(3, 1.0, 2.0, 0.3, yaw)])))['3'][0]
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
        load_marker_map(str(tmp_path))


def test_unknown_dictionary_is_refused(tmp_path):
    (tmp_path / 'course.yaml').write_text('dictionary: DICT_4X4_50\nmarkers: []\n')
    with pytest.raises(ValueError):
        load_marker_map(str(tmp_path))


def test_ar_tag_params_name_a_shipped_dictionary_file():
    c = mmap([Marker(24, 3.0, 0.0, 0.5, math.pi)])
    c.marker_size, c.dictionary = 0.38, 'DICT_5X5_1000'
    p = yaml.safe_load(make_ar_tag_params(c))['/**']['ros__parameters']
    assert p['dictionary'] == ('$(find-pkg-share autosdv_aruco_localization)'
                               '/config/dictionaries/DICT_5X5_1000.dict')
    assert p['marker_size'] == 0.38
    assert p['target_tag_ids'] == ['24']


def test_ar_tag_params_pass_an_aruco_name_through():
    c = mmap([Marker(1, 0, 0, 0, 0)])
    assert 'dictionary: "ALL_DICTS"' in make_ar_tag_params(c)


def test_shipped_dictionary_files_exist():
    for name in SHIPPED_DICTIONARIES:
        assert os.path.isfile(os.path.join(HERE, 'config', 'dictionaries', name + '.dict'))


@pytest.mark.parametrize('pitch,roll', [(0.1, 0.0), (0.0, -0.08), (-0.05, 0.12)])
def test_a_tilted_marker_round_trips_with_its_tilt(pitch, roll):
    m = Marker(5, 1.0, 2.0, 0.4, 0.7, pitch, roll)
    lm = parse_osm_landmarks(make_osm(mmap([m])))['5'][0]
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


def test_a_bare_marker_map_needs_no_route(tmp_path):
    # a survey without a route: start_pose, the boards, nothing else
    (tmp_path / 'course.yaml').write_text(
        'start_pose: {x: -1.0, y: 0.5, yaw: 0.2}\nmarker_size: 0.38\ndictionary: DICT_5X5_1000\n'
        'markers:\n  - {id: 24, x: 0.0, y: 0.0, z: 0.25, yaw: 0.0}\n'
        '  - {id: 25, x: 2.0, y: 1.0, z: 0.25, yaw: 3.0}\n')
    written = write_map_files(str(tmp_path))
    assert {os.path.basename(p) for p in written} == {
        'lanelet2_map.osm', 'ar_tag.param.yaml', 'map_projector_info.yaml'}
    m = load_marker_map(str(tmp_path))
    assert (m.start_pose.x, m.start_pose.y, m.start_pose.yaw) == (-1.0, 0.5, 0.2)
    assert sorted(parse_osm_landmarks((tmp_path / 'lanelet2_map.osm').read_text())) == ['24', '25']


def test_the_package_needs_nothing_of_lab_2():
    """This package is lab-independent: no import, launch include or manifest entry names
    a lab package (Lab 2 depends on it, never the reverse)."""
    lab = ('autosdv_waypoint_lab', 'autosdv_coach_lab', 'autosdv_lab_msgs', 'lab2_config', 'lab2_run')
    offenders = []
    for root, _, files in os.walk(HERE):
        if os.sep + 'test' in root[len(HERE):]:
            continue
        for f in files:
            if not f.endswith(('.py', '.xml', '.yaml', '.cfg')):
                continue
            p = os.path.join(root, f)
            for n, line in enumerate(open(p, errors='replace'), 1):
                code = line.split('#', 1)[0] if f.endswith('.py') else line
                if any(x in code for x in lab) and ('import' in code or '<' in code or 'pkg' in code):
                    offenders.append(f'{os.path.relpath(p, HERE)}:{n}: {line.strip()}')
    assert not offenders, offenders
