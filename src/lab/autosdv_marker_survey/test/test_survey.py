import math

import numpy as np
import pytest

from autosdv_marker_survey.finish import finish, load_dump, save_dump
from autosdv_marker_survey.simulate import drive, marker_T, simulate
from autosdv_marker_survey.survey import (
    Detection, Detector, course_frame, inv_T, make_T, opencv_dictionary, to_course)

# a small room: six upright 0.48 m boards around a 6 m x 3 m loop
BOARDS = {
    24: (8.0, 1.5, math.pi),
    25: (6.0, -2.0, math.radians(120)),
    26: (2.0, -2.0, math.radians(120)),
    27: (-2.0, 1.5, 0.0),
    28: (2.0, 5.0, math.radians(-60)),
    29: (6.0, 5.0, math.radians(-120)),
}
LOOP = [(0, 0), (6, 0), (7.5, 1.5), (6, 3), (0, 3), (-1, 1.5), (0, 0)]


# boards standing on the floor but not quite upright: (pitch, roll), rad
TILT = {25: (0.12, 0.0), 27: (0.0, -0.10), 29: (-0.06, 0.05)}


@pytest.fixture(scope='module')
def result():
    truth = {mid: marker_T(x, y, 0.40, yaw, *TILT.get(mid, (0.0, 0.0)))
             for mid, (x, y, yaw) in BOARDS.items()}
    poses = drive(LOOP, step=0.05, sweeps={2: math.radians(50), 5: math.radians(50)})
    sv = simulate(truth, poses, 0.48, seed=3)
    return truth, sv, finish(sv, 24, 'DICT_5X5_1000', None, log=lambda *a: None)


def test_every_marker_is_found(result):
    truth, sv, res = result
    assert {m.id for m in res['markers']} == set(BOARDS)


def test_markers_land_within_a_few_centimetres(result):
    truth, _, res = result
    fr = course_frame(truth[24], np.zeros(3), np.array([0, 0, 1.0]))
    Ti = inv_T(fr.T_world_course)
    for m in res['markers']:
        Tt = Ti @ truth[m.id]
        assert math.hypot(m.x - Tt[0, 3], m.y - Tt[1, 3]) < 0.05, m
        yaw_t = math.atan2(Tt[1, 2], Tt[0, 2])
        assert abs(math.atan2(math.sin(m.yaw - yaw_t), math.cos(m.yaw - yaw_t))) < math.radians(1.5)
        assert abs(m.z - 0.40) < 0.03
        p, r = TILT.get(m.id, (0.0, 0.0))
        assert m.pitch == pytest.approx(p, abs=math.radians(1.5)), m
        assert m.roll == pytest.approx(r, abs=math.radians(1.5)), m


def test_the_origin_marker_defines_the_frame(result):
    _, _, res = result
    o = next(m for m in res['markers'] if m.id == 24)
    assert (o.x, o.y, o.yaw) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_the_start_pose_is_where_the_drive_began(result):
    truth, _, res = result
    # the drive starts at the world origin heading +x; marker 24 at (8, 1.5) faces -x,
    # so the course x axis is world -x and its y axis world -y
    sp = res['start_pose']
    assert sp['x'] == pytest.approx(8.0, abs=0.05)
    assert sp["y"] == pytest.approx(1.5, abs=0.05)
    assert abs(math.atan2(math.sin(sp['yaw'] - math.pi), math.cos(sp['yaw'] - math.pi))) < 0.02


def test_dump_round_trip_solves_the_same(result, tmp_path):
    _, sv, res = result
    p = tmp_path / 'survey_data.npz'
    save_dump(str(p), sv, 'DICT_5X5_1000', 24)
    sv2, dic, origin = load_dump(str(p))
    assert (dic, origin) == ('DICT_5X5_1000', 24)
    res2 = finish(sv2, origin, dic, None, log=lambda *a: None)
    a = {m.id: (m.x, m.y) for m in res['markers']}
    b = {m.id: (m.x, m.y) for m in res2['markers']}
    for k in a:
        assert a[k] == pytest.approx(b[k], abs=0.01)


def test_writes_a_course_the_lab_can_load(result, tmp_path):
    from autosdv_waypoint_lab.course import load_course, make_osm, parse_osm_landmarks
    truth, sv, _ = result
    res = finish(sv, 24, 'DICT_5X5_1000', str(tmp_path), log=lambda *a: None)
    c = load_course(str(tmp_path))
    assert {m.id for m in c.markers} == set(BOARDS)
    # the localizer's map carries the measured tilt: its marker normals match the truth
    fr = course_frame(truth[24], np.zeros(3), np.array([0, 0, 1.0]))
    Ti = inv_T(fr.T_world_course)
    lms = parse_osm_landmarks(make_osm(c))
    for mid in BOARDS:
        n_true = (Ti @ truth[mid])[:3, 2]
        assert np.dot(lms[str(mid)][0].z_axis, n_true) > math.cos(math.radians(1.5)), mid
    assert c.dictionary == 'DICT_5X5_1000' and c.marker_size == 0.48
    for f in ('lanelet2_map.osm', 'ar_tag.param.yaml', 'survey_report.yaml', 'map_projector_info.yaml'):
        assert (tmp_path / f).exists()


def test_an_existing_course_keeps_its_path_and_settings(result, tmp_path):
    (tmp_path / 'course.yaml').write_text('path: my_path.csv\nlaps: 5\nv_ref: 0.6\n'
                                          'start_pose: {x: 1.0, y: 2.0, yaw: 0.5}\nmarkers: []\n')
    _, sv, _ = result
    finish(sv, 24, 'DICT_5X5_1000', str(tmp_path), log=lambda *a: None)
    import yaml
    d = yaml.safe_load((tmp_path / 'course.yaml').read_text())
    assert d['path'] == 'my_path.csv' and d['laps'] == 5 and d['v_ref'] == 0.6
    assert d['start_pose'] == {'x': 1.0, 'y': 2.0, 'yaw': 0.5}
    assert len(d['markers']) == len(BOARDS)


def test_an_unseen_origin_is_refused(result):
    _, sv, _ = result
    with pytest.raises(ValueError, match='never seen'):
        finish(sv, 99, 'DICT_5X5_1000', None, log=lambda *a: None)


def test_dictionaries():
    opencv_dictionary('DICT_5X5_1000')
    with pytest.raises(ValueError):
        opencv_dictionary('ALL_DICTS')


def test_the_detector_reads_a_rendered_marker():
    import cv2
    d = opencv_dictionary('DICT_5X5_1000')
    gen = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    img = np.full((400, 400), 255, np.uint8)
    img[100:300, 100:300] = gen(d, 24, 200)
    dets = Detector('DICT_5X5_1000').detect(img)
    assert [x.id for x in dets] == [24]
    # corners TL, TR, BR, BL
    c = dets[0].corners
    assert c[0][0] < c[1][0] and c[0][1] < c[3][1]
