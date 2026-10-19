"""A synthetic survey: a camera on a car driving among markers, tracking with drift. No ROS.

For tests and for predicting a survey's accuracy before going to the hall.
Corners are projected (no image rendering) with Gaussian pixel noise; the
tracking pose is the true pose with a drifting error integrated along the
drive. Returns the Survey (ready to finish()) and the true marker poses.
"""
import math
from typing import Dict, List, Tuple

import numpy as np

from autosdv_marker_survey.survey import (
    Detection, Survey, SurveySettings, inv_T, make_T, marker_object_points, rotvec_to_matrix)

# base_link -> camera optical frame: camera 0.20 m ahead, 0.25 m up, looking forward
R_BASE_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)   # columns: optical axes in base
T_BASE_CAM = make_T(R_BASE_OPT, [0.20, 0.0, 0.25])


def marker_T(x, y, z, yaw, pitch=0.0, roll=0.0) -> np.ndarray:
    """A board at (x, y, z) facing yaw; tilted like course.yaml's pitch and roll."""
    from autosdv_waypoint_lab.course import Marker
    xa, ya, za = Marker(0, x, y, z, yaw, pitch, roll).axes()
    return make_T(np.column_stack([xa, ya, za]), [x, y, z])


def zed_K(width=1920, height=1200, hfov_deg=102.0):
    f = width / 2 / math.tan(math.radians(hfov_deg / 2))
    return np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1.0]]), width, height


def drive(path_xy: List[Tuple[float, float]], step=0.05, sweeps: Dict[int, float] = None):
    """Base poses along a polyline, every `step` m; at listed waypoint indices the
    car swings its heading by +-amp rad (pushed by hand)."""
    poses = []
    sweeps = sweeps or {}
    for i in range(len(path_xy) - 1):
        (x0, y0), (x1, y1) = path_xy[i], path_xy[i + 1]
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        for k in range(n):
            f = k / n
            poses.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0), yaw))
        if i + 1 in sweeps:
            amp = sweeps[i + 1]
            for a in list(np.linspace(0, amp, 15)) + list(np.linspace(amp, -amp, 30)) + list(np.linspace(-amp, 0, 15)):
                poses.append((x1, y1, yaw + a))
    return [make_T(rotvec_to_matrix([0, 0, yaw]), [x, y, 0.0]) for x, y, yaw in poses]


def simulate(markers: Dict[int, np.ndarray], base_poses: List[np.ndarray], marker_size: float,
             settings: SurveySettings = None, pixel_sd=0.5, drift=0.01, yaw_drift_deg_per_m=0.3,
             seed=1, hfov_deg=102.0):
    rng = np.random.default_rng(seed)
    K, W, H = zed_K(hfov_deg=hfov_deg)
    s = settings or SurveySettings(marker_size=marker_size)
    sv = Survey(s, K)
    sv.T_cam_base = inv_T(T_BASE_CAM)
    obj = marker_object_points(marker_size)
    # tracking: integrate noisy relative motion
    T_odo = base_poses[0].copy()
    prev = base_poses[0]
    for k, Tb in enumerate(base_poses):
        if k > 0:
            rel = inv_T(prev) @ Tb
            d = np.linalg.norm(rel[:3, 3])
            noise_t = rng.normal(0, drift * d, 3)
            noise_r = rng.normal(0, math.radians(yaw_drift_deg_per_m) * d + 1e-5, 3) * np.array([0.2, 0.2, 1.0])
            rel_n = make_T(rel[:3, :3] @ rotvec_to_matrix(noise_r), rel[:3, 3] + noise_t)
            T_odo = T_odo @ rel_n
            prev = Tb
        T_wc_true = Tb @ T_BASE_CAM
        dets = []
        for mid, Tm in markers.items():
            Xw = (Tm[:3, :3] @ obj.T).T + Tm[:3, 3]
            Xc = (inv_T(T_wc_true)[:3, :3] @ Xw.T).T + inv_T(T_wc_true)[:3, 3]
            if np.any(Xc[:, 2] < 0.3):
                continue
            v = T_wc_true[:3, 3] - Tm[:3, 3]
            if (v / np.linalg.norm(v)) @ Tm[:3, 2] < math.cos(math.radians(65)):
                continue
            uv = (K @ Xc.T).T
            uv = uv[:, :2] / uv[:, 2:]
            if uv.min() < 2 or uv[:, 0].max() > W - 2 or uv[:, 1].max() > H - 2:
                continue
            dets.append(Detection(mid, uv + rng.normal(0, pixel_sd, uv.shape)))
        sv.add(k * 0.1, T_odo @ T_BASE_CAM, dets)
    return sv


def stage1_map(truth: Dict[int, Tuple[float, ...]], origin_id: int):
    """What stage 1 (`just aruco-map scan ORIGIN`) makes of boards whose TRUE poses are
    `truth` = {id: (x, y, z, yaw, pitch, roll)} in a level world frame, noise aside:
    the floor normal from the car's poses (the world's z), the origin below the
    origin board's centre, x the horizontal part of the way it faces. Returns
    (course.yaml marker entries, T_world_stage1)."""
    from autosdv_marker_survey.survey import course_frame, to_course
    Ts = {mid: marker_T(*p) for mid, p in truth.items()}
    frame = course_frame(Ts[origin_id], np.zeros(3), np.array([0.0, 0.0, 1.0]))
    surveyed = to_course(Ts, {}, frame)
    markers = [{'id': m.id, 'x': round(m.x, 4), 'y': round(m.y, 4), 'z': round(m.z, 4),
                'yaw': round(m.yaw, 5), 'pitch': round(m.pitch, 5), 'roll': round(m.roll, 5)}
               for m in surveyed]
    return markers, frame.T_world_course
