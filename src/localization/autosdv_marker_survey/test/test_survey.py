import math

import numpy as np
import pytest

from autosdv_marker_survey.finish import finish, load_dump, save_dump
from autosdv_marker_survey.simulate import drive, marker_T, simulate
from autosdv_marker_survey.survey import (
    Detection, Detector, course_frame, inv_T, make_T, opencv_dictionary, to_course)

# a small room: six upright 0.38 m boards around a 6 m x 3 m loop
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
    sv = simulate(truth, poses, 0.38, seed=3)
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


def test_writes_a_marker_map_the_localization_can_load(result, tmp_path):
    from autosdv_aruco_localization.marker_map import load_marker_map, make_osm, parse_osm_landmarks
    truth, sv, _ = result
    res = finish(sv, 24, 'DICT_5X5_1000', str(tmp_path), log=lambda *a: None)
    c = load_marker_map(str(tmp_path))
    assert {m.id for m in c.markers} == set(BOARDS)
    # the localizer's map carries the measured tilt: its marker normals match the truth
    fr = course_frame(truth[24], np.zeros(3), np.array([0, 0, 1.0]))
    Ti = inv_T(fr.T_world_course)
    lms = parse_osm_landmarks(make_osm(c))
    for mid in BOARDS:
        n_true = (Ti @ truth[mid])[:3, 2]
        assert np.dot(lms[str(mid)][0].z_axis, n_true) > math.cos(math.radians(1.5)), mid
    assert c.dictionary == 'DICT_5X5_1000' and c.marker_size == 0.38
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


# ------------------------------------------------------------------ OpenCV 4.5 (JetPack 6)

def test_the_detector_works_without_arucodetector(monkeypatch):
    """OpenCV 4.5.4 has no ArucoDetector/DetectorParameters: the function API must serve."""
    import cv2
    real = cv2.aruco
    calls = {}

    class Legacy:
        pass

    legacy = Legacy()
    for name in dir(real):
        if name in ('ArucoDetector', 'DetectorParameters'):
            continue
        setattr(legacy, name, getattr(real, name))

    def detect_markers(gray, dictionary, parameters=None):
        calls['params'] = parameters
        det = real.ArucoDetector(dictionary, parameters)
        return det.detectMarkers(gray)

    def create():
        return real.DetectorParameters()

    legacy.DetectorParameters_create = create
    legacy.detectMarkers = detect_markers
    monkeypatch.setattr(cv2, 'aruco', legacy)
    assert not hasattr(cv2.aruco, 'ArucoDetector')
    gen = getattr(real, 'generateImageMarker', None) or real.drawMarker
    img = np.full((400, 400), 255, np.uint8)
    img[100:300, 100:300] = gen(real.getPredefinedDictionary(real.DICT_5X5_1000), 24, 200)
    det = Detector('DICT_5X5_1000')
    assert det.legacy
    assert [d.id for d in det.detect(img)] == [24]
    assert calls['params'].cornerRefinementMethod == real.CORNER_REFINE_SUBPIX


# ------------------------------------------------------------------ a stationary save

def test_a_single_keyframe_is_saved_without_adjustment(tmp_path):
    truth = {mid: marker_T(x, y, 0.40, yaw) for mid, (x, y, yaw) in {
        24: (3.0, 0.0, math.pi), 25: (3.0, 1.2, math.pi)}.items()}
    sv = simulate(truth, [drive([(0, 0), (0.05, 0)], step=0.05)[0]], 0.38, seed=2)
    assert len(sv.keyframes) == 1
    with pytest.raises(ValueError, match='not enough data'):
        sv.solve()
    logs = []
    res = finish(sv, 24, 'DICT_5X5_1000', str(tmp_path), log=logs.append)
    assert any('single keyframe' in str(x) and 'quick look' in str(x) for x in logs)
    assert res['info']['status'] == 'single keyframe, no bundle adjustment'
    assert all(m.sd_xy is None and m.sd_yaw_deg is None for m in res['markers'])
    import yaml
    rep = yaml.safe_load((tmp_path / 'survey_report.yaml').read_text())
    assert rep['solve']['status'] == 'single keyframe, no bundle adjustment'
    assert rep['markers'][0]['sd_xy_cm'] is None
    m = {x.id: x for x in res['markers']}
    assert m[24].x == pytest.approx(0.0, abs=0.02) and m[24].z == pytest.approx(0.40, abs=0.03)
    assert m[25].y == pytest.approx(-1.2, abs=0.05) or m[25].y == pytest.approx(1.2, abs=0.05)


# ------------------------------------------------------------------ tilt vs in-plane rotation

def _course_markers(Ts, origin_T=None):
    frame = course_frame(origin_T if origin_T is not None else Ts[24], np.zeros(3), np.array([0, 0, 1.0]))
    return to_course(Ts, {}, frame)


def test_rotating_a_board_in_its_plane_is_not_a_tilt():
    Ts = {24: marker_T(0, 0, 0.4, 0.0), 30: marker_T(2, 1, 0.4, 0.7, 0.0, math.pi / 2),
          31: marker_T(2, -1, 0.4, -0.4, 0.0, math.radians(-90))}
    out = {m.id: m for m in _course_markers(Ts)}
    for mid in (30, 31):
        assert out[mid].tilt_deg == pytest.approx(0.0, abs=1e-6)
        assert not any('tilted' in f for f in out[mid].flags)
    assert math.degrees(out[30].roll) == pytest.approx(90.0, abs=1e-6)
    assert math.degrees(out[30].pitch) == pytest.approx(0.0, abs=1e-6)


def test_tilt_is_the_lean_of_the_normal():
    Ts = {24: marker_T(0, 0, 0.4, 0.0), 30: marker_T(2, 1, 0.4, 0.7, math.radians(12), math.radians(90)),
          31: marker_T(2, -1, 0.4, -0.4, math.radians(-12), math.radians(37))}
    out = {m.id: m for m in _course_markers(Ts)}
    assert out[30].tilt_deg == pytest.approx(12.0, abs=1e-6)
    assert out[31].tilt_deg == pytest.approx(12.0, abs=1e-6)
    assert any('tilted' in f for f in out[30].flags)
    assert math.degrees(out[30].pitch) == pytest.approx(12.0, abs=1e-6)
    assert math.degrees(out[30].roll) == pytest.approx(90.0, abs=1e-6)


def test_a_rotated_marker_round_trips_through_the_lanelet_map(tmp_path):
    import yaml
    from autosdv_aruco_localization import marker_map as course_lib
    Ts = {24: marker_T(0, 0, 0.4, 0.0), 30: marker_T(2, 1, 0.4, 0.7, math.radians(5), math.pi / 2),
          31: marker_T(2, -1, 0.4, -0.4, math.radians(-4), math.radians(-90))}
    surveyed = _course_markers(Ts)
    from autosdv_marker_survey import output
    output.write_course(str(tmp_path), surveyed, 24, 'DICT_5X5_1000', 0.38, None,
                        {'keyframes': 1, 'reproj_rms_px': 0.0})
    c = course_lib.load_marker_map(str(tmp_path))
    osm = (tmp_path / 'lanelet2_map.osm').read_text()
    parsed = course_lib.parse_osm_landmarks(osm)
    frame = course_frame(Ts[24], np.zeros(3), np.array([0, 0, 1.0]))
    Tci = inv_T(frame.T_world_course)
    h = 0.19
    assert {mk.id for mk in c.markers} == {24, 30, 31}
    for mk in c.markers:
        Tc = Tci @ Ts[mk.id]
        truth = [Tc[:3, :3] @ np.array(p) + Tc[:3, 3] for p in
                 ([-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0])]
        for a, b in zip(mk.corners(0.38), truth):
            assert np.allclose(a, b, atol=2e-3), (mk.id, a, b)
        lm = parsed[str(mk.id)][0]
        assert np.allclose(lm.position, Tc[:3, 3], atol=2e-3)
        for got, col in ((lm.x_axis, 0), (lm.y_axis, 1), (lm.z_axis, 2)):
            assert np.allclose(got, Tc[:3, col], atol=2e-3), (mk.id, col)
    assert yaml.safe_load((tmp_path / 'course.yaml').read_text())['markers']


# ------------------------------------------------------------------ a car on a stand

def _stand_survey(h):
    truth = {mid: marker_T(x, y, 0.25, yaw) for mid, (x, y, yaw) in {
        24: (3.0, 0.0, math.pi), 25: (3.0, 1.2, math.pi)}.items()}
    pose = drive([(0, 0), (0.05, 0)], step=0.05)[0].copy()
    pose[2, 3] = h                                   # base_link h above the floor: wheels off
    return simulate(truth, [pose], 0.38, seed=2)


def test_floor_offset_corrects_a_car_on_a_stand(tmp_path):
    logs = []
    res = finish(_stand_survey(0.15), 24, 'DICT_5X5_1000', str(tmp_path), log=logs.append)
    z = {m.id: m.z for m in res['markers']}
    assert z[24] == pytest.approx(0.10, abs=0.03)           # true 0.25 - 0.15
    assert all(any('half the marker size' in f for f in m.flags) for m in res['markers'])
    assert any('floor_offset' in str(x) and 'stand' in str(x) for x in logs)
    import yaml
    rep = yaml.safe_load((tmp_path / 'survey_report.yaml').read_text())
    assert any('floor_offset' not in f and 'half the marker size' in f for f in rep['markers'][0]['flags'])
    fixed = finish(_stand_survey(0.15), 24, 'DICT_5X5_1000', None, log=lambda *a: None,
                   floor_offset=0.15)
    for m in fixed['markers']:
        assert m.z == pytest.approx(0.25, abs=0.03)
        assert not any('half the marker size' in f for f in m.flags)
    # a pure vertical offset leaves x, y and yaw alone
    for a, b in zip(res['markers'], fixed['markers']):
        assert (a.x, a.y, a.yaw) == pytest.approx((b.x, b.y, b.yaw), abs=1e-6)


def test_a_marker_below_the_floor_is_flagged():
    res = finish(_stand_survey(0.40), 24, 'DICT_5X5_1000', None, log=lambda *a: None)
    assert all(m.z < 0 and any('BELOW the floor' in f for f in m.flags) for m in res['markers'])
