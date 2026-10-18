"""lab2_audit and the mock camera on synthetic data. No ROS (matplotlib Agg, cv2)."""
import math
import os
from types import SimpleNamespace as NS

import numpy as np
import pytest

from autosdv_waypoint_lab import audit as au
from autosdv_waypoint_lab import bag_meta as bm
from autosdv_waypoint_lab import grading as g
from autosdv_waypoint_lab import route_grade as rg

from test_route_grade import UTURN, msgs_for

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
OUT = os.path.join(REPO, 'tmp', 'test_audit')
COURSE = rg.template_dir('uturn')


def verdict(ok=True, reasons=()):
    return NS(ok=ok and not reasons, reasons=list(reasons), warnings=[])


def run(msgs, verdict_=None, meta=None):
    m = rg.route_metrics('lab2_uturn_9', msgs, 'uturn', UTURN)
    sc = rg.score_route(m)
    s = au.series_from_messages(msgs, UTURN)
    a = au.build_audit('/x/lab2_uturn_9', verdict_ or verdict(), m, sc, s,
                       meta or {'car_id': 'A', 'team_id': '3'})
    return m, sc, s, a


def test_spans_where():
    ts = [0, 1, 2, 3, 4, 5]
    assert au.spans_where(ts, [False, True, True, False, True, True]) == [(1, 2), (4, 5)]
    assert au.spans_where(ts, [False] * 6) == []


def test_clean_run_is_valid_with_no_violations():
    m, sc, s, a = run(msgs_for())
    assert a.valid and a.verdict == 'VALID'
    assert a.violations == []
    assert a.total == pytest.approx(45.0)
    text = au.format_audit(a, m)
    assert 'VALID' in text and 'violations: none' in text and 'total  45.00 / 45' in text


def test_off_path_stretch_is_a_violation_with_its_time():
    msgs = msgs_for()
    # push the localized pose 0.45 m sideways between 10 s and 12 s
    kin = []
    for t, k in msgs[g.KINEMATIC]:
        p = k.pose.pose.position
        if 10.0 <= t <= 12.0:
            k = NS(pose=NS(pose=NS(position=NS(x=p.x, y=p.y + 0.45))))
        kin.append((t, k))
    msgs[g.KINEMATIC] = kin
    m, sc, s, a = run(msgs)
    assert not a.valid
    spans = au.off_path_spans(s, m.t_engage, m.t_rest)
    assert len(spans) == 1
    t0, t1, worst = spans[0]
    assert 9.9 <= t0 <= 10.1 and 11.9 <= t1 <= 12.1 and worst > 0.3
    assert any('off the path' in v and 'after engage' in v for v in a.violations)


def test_mrm_and_points_lost_are_violations():
    m, sc, s, a = run(msgs_for(mrm=True, stop_off=0.4))
    assert not a.valid
    assert any(v.startswith('grade:') and 'MRM' in v for v in a.violations)
    assert any(v.startswith('MRM at') for v in a.violations)
    assert any(v.startswith('stop error') and 'full-marks line' in v for v in a.violations)
    assert s.mrm == [10.0]


def test_check_bags_rejection_makes_it_invalid():
    m, sc, s, a = run(msgs_for(), verdict(False, ['topic /lab2/camera/image/compressed is empty']))
    assert a.grade_valid and not a.check_ok and not a.valid
    assert a.violations == ['check-bags: topic /lab2/camera/image/compressed is empty']
    assert 'total  0 / 45' in au.format_audit(a, m)


def test_speed_violations_are_the_costly_samples():
    m, sc, s, a = run(msgs_for(wobble=0.1))
    bad = au.speed_violations(m, sc, s, UTURN)
    assert bad and all(abs(v - m.target) > 0.04 for _, v in bad)
    m, sc, s, a = run(msgs_for())
    assert len(au.speed_violations(m, sc, s, UTURN)) < 5      # the first samples of the window only


def test_segments_follow_the_route():
    m, sc, s, a = run(msgs_for())
    segs = au.segment_spans(s, UTURN, m.t_engage, m.t_rest)
    assert [k for _, _, k in segs] == ['straight', 'U-turn', 'straight']


def test_plot_writes_a_png():
    os.makedirs(OUT, exist_ok=True)
    for name, msgs in (('clean', msgs_for()), ('bad', msgs_for(mrm=True, lateral=0.4))):
        m, sc, s, a = run(msgs)
        png = au.plot_audit(os.path.join(OUT, f'{name}.png'), a, m, sc, s, UTURN, COURSE)
        assert os.path.getsize(png) > 20000
        with open(png, 'rb') as f:
            assert f.read(8) == b'\x89PNG\r\n\x1a\n'


def stamped(t, x, y):
    return NS(header=NS(stamp=NS(sec=int(t), nanosec=int((t % 1) * 1e9))),
              pose=NS(pose=NS(position=NS(x=x, y=y))))


def test_localization_quality_against_the_true_pose():
    ts = [0.05 * k for k in range(400)]                   # 20 s
    truth = [(t, stamped(t, 0.8 * t, 0.0)) for t in ts]
    fixes = [(t + 0.05, stamped(t, 0.8 * t + 0.1, 0.0)) for t in ts[::5]]   # 4/s, 0.1 m off
    ekf = [(t, stamped(t, 0.8 * t, 0.03)) for t in ts]
    msgs = {bm.TRUE_POSE: truth, bm.MARKER_FIXES: fixes, g.KINEMATIC: ekf}
    q = au.localization_quality(msgs, 1.0, 19.0)
    assert q['ok'], q['failures']
    assert q['fix_err_median'] == pytest.approx(0.1, abs=1e-6)
    assert q['ekf_err_p95'] == pytest.approx(0.03, abs=1e-6)
    assert 3.5 < q['fix_rate'] < 4.5 and q['gap_max_s'] < 0.5
    # a 5 s blind stretch and a 0.3 m EKF offset fail it
    msgs[bm.MARKER_FIXES] = [f for f in fixes if not 5.0 <= f[0] <= 10.0]
    msgs[g.KINEMATIC] = [(t, stamped(t, 0.8 * t, 0.3)) for t in ts]
    q = au.localization_quality(msgs, 1.0, 19.0)
    assert not q['ok']
    assert any('without a fix' in f for f in q['failures'])
    assert any('EKF error' in f for f in q['failures'])
    assert au.localization_quality({g.KINEMATIC: ekf}) is None     # no true pose: a car bag


# ---- the mock camera

def test_mock_camera_renders_boards_the_detector_reads():
    cv2 = pytest.importorskip('cv2')
    from autosdv_waypoint_lab import mock_camera_node as mc
    from autosdv_waypoint_lab.course import load_course
    c = load_course(COURSE)
    d = mc.aruco_dictionary(c.dictionary)
    pats = mc.marker_patterns(d, [m.id for m in c.markers])
    cam = mc.Camera(960, 600, 403.0)
    pose = (c.start_pose.x, c.start_pose.y, 0.0, c.start_pose.yaw)
    img, drawn = mc.render(cam, pose, c.markers, c.marker_size, pats)
    assert img.shape == (600, 960) and set(drawn) >= {25, 26}
    corners, ids, _ = cv2.aruco.ArucoDetector(d).detectMarkers(img)
    assert ids is not None and set(ids.ravel()) >= {25, 26}
    k = np.array(cam.info_k()).reshape(3, 3)
    h = c.marker_size / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
    for cs, i in zip(corners, ids.ravel()):
        mk = next(m for m in c.markers if m.id == i)
        true = cam.to_optical(pose, np.array([[mk.x, mk.y, mk.z]]))[0]
        if true[2] > 4.0:
            continue
        _, _, tv = cv2.solvePnP(obj, cs[0], k, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        assert np.linalg.norm(tv.ravel() - true) < 0.1, (i, tv.ravel(), true)


def test_mock_camera_skips_boards_behind_or_facing_away():
    from autosdv_waypoint_lab import mock_camera_node as mc
    from autosdv_waypoint_lab.course import Marker
    cam = mc.Camera(960, 600, 403.0)
    ahead = Marker(1, 3.0, 0.0, 0.25, math.pi)       # 3 m ahead, facing the car
    away = Marker(2, 3.0, 0.0, 0.25, 0.0)            # 3 m ahead, facing away
    behind = Marker(3, -3.0, 0.0, 0.25, 0.0)         # behind, facing the car
    pose = (0.0, 0.0, 0.0, 0.0)
    assert mc.board_in_view(cam, pose, ahead, 12.0)
    assert not mc.board_in_view(cam, pose, away, 12.0)
    assert not mc.board_in_view(cam, pose, ahead, 2.0)
    pats = mc.marker_patterns(mc.aruco_dictionary('DICT_5X5_1000'), [1, 2, 3])
    _, drawn = mc.render(cam, pose, [ahead, away, behind], 0.38, pats)
    assert drawn == [1]


def test_bench_frame_is_the_recorded_size():
    cv2 = pytest.importorskip('cv2')
    from autosdv_waypoint_lab.record import bench_frame
    data = bench_frame('12:00:00  frame 1')
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    assert (img.shape[1], img.shape[0]) == bm.CAMERA_SIZE
