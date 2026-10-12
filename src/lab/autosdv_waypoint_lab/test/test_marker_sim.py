import math
import random

import pytest

from autosdv_waypoint_lab.course import Course, Marker, Pose2D, make_osm, parse_osm_landmarks
from autosdv_waypoint_lab.marker_sim import CameraModel, NoiseModel, simulate_fix, visible


def landmarks(*markers):
    c = Course('.', 'p.csv', True, 1, 0.8, Pose2D(0, 0, 0), 0.25, list(markers))
    return parse_osm_landmarks(make_osm(c))


AHEAD = Marker(0, 3.0, 0.0, 0.25, math.pi)   # 3 m ahead, facing the car


def test_visible_ahead_and_not_behind_or_beyond_range():
    cam = CameraModel()
    lm = landmarks(AHEAD)['0'][0]
    assert visible(cam, 0.0, 0.0, 0.0, lm)
    assert not visible(cam, 0.0, 0.0, math.pi, lm)
    assert not visible(cam, -4.0, 0.0, 0.0, lm)


def test_not_visible_from_behind_its_back_or_edge_on():
    cam = CameraModel()
    back = landmarks(Marker(0, 3.0, 0.0, 0.25, 0.0))['0'][0]
    assert not visible(cam, 0.0, 0.0, 0.0, back)
    oblique = landmarks(Marker(0, 3.0, 0.0, 0.25, math.pi / 2 + 0.2))['0'][0]
    assert not visible(cam, 0.0, 0.0, 0.0, oblique)


def test_a_marker_cut_by_the_image_border_is_not_detected():
    cam = CameraModel()
    lm = landmarks(Marker(0, 2.0, 2.3, 0.25, -2.3))['0'][0]
    # its centre is inside 50 deg, its far edge is not
    assert not visible(cam, 0.2 - 0.2, 0.0, 0.0, lm)


def test_noiseless_fix_is_the_true_pose():
    noise = NoiseModel(pos_std=0, pos_std_per_m=0, yaw_std=0, dropout=0)
    truth = (0.3, -0.2, 0.15)
    fix = simulate_fix(CameraModel(), noise, random.Random(1), truth, (0.0, 0.0, 0.0),
                       landmarks(AHEAD), 1.0)
    assert (fix.x, fix.y, fix.yaw) == pytest.approx(truth, abs=1e-9)
    assert fix.covariance_scale == 1.0


def test_no_ekf_pose_no_fix_and_the_gate_drops_far_fixes():
    noise = NoiseModel(pos_std=0, pos_std_per_m=0, yaw_std=0, dropout=0)
    lms = landmarks(AHEAD)
    assert simulate_fix(CameraModel(), noise, random.Random(1), (0, 0, 0), None, lms, 1.0) is None
    assert simulate_fix(CameraModel(), noise, random.Random(1), (0, 0, 0), (1.5, 0, 0),
                        lms, 1.0) is None


def test_orientation_error_moves_the_fix_sideways_by_range_times_angle():
    noise = NoiseModel(pos_std=0, pos_std_per_m=0, yaw_std=math.radians(2.0), dropout=0)
    rng = random.Random(3)
    lat = []
    for _ in range(400):
        f = simulate_fix(CameraModel(), noise, rng, (0, 0, 0), (0, 0, 0), landmarks(AHEAD), 1.0)
        lat.append(f.y)
    sd = math.sqrt(sum(v * v for v in lat) / len(lat))
    assert sd == pytest.approx(3.0 * math.radians(2.0), rel=0.2)
