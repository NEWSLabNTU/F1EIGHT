"""What the camera and the stock AR-tag localizer would output, from the true pose. No ROS.

The bench has no images, so `virtual_marker_localizer` replaces the ZED and
`ar_tag_based_localizer` together. This module is its model, written to match
what the stock node does (autoware_ar_tag_based_localizer 0.48.0, read from
source) rather than an idealised detector:

- A marker is detected when the whole of it is inside the camera's
  horizontal field of view (both vertical edges, with a small margin: a
  marker cut by the image border is not decoded -- checked against the real
  localizer with rendered images), within [range_min, range_max] of the
  camera, and seen within `max_incidence` of head-on (a marker seen edge-on
  is not decoded).
- The detection -- the marker's pose in base_link -- carries noise: position
  noise growing with range, and an orientation (yaw) error. With
  consider_orientation: true the stock node computes the car's pose as
  marker_on_map * inverse(detection), so the yaw error moves the car's
  position by range * yaw error, sideways. That lever arm is why a far
  marker's fix is worse than its position noise suggests.
- Among the detected markers, the stock node keeps the one whose mapped
  position is closest to where the EKF pose puts the detection, and only if
  the resulting pose is within `ekf_position_tolerance` of the EKF pose.
  Without an EKF pose it publishes nothing at all.
- Its covariance is base_covariance * max(1, (d / 5)^3) where d is that
  EKF-to-fix distance -- not the marker range, whatever the upstream comment
  says. Below the tolerance used here, d < 5 m, so the covariance is constant.
"""

from dataclasses import dataclass
import math
import random
from typing import Dict, List, Optional, Tuple

from autosdv_waypoint_lab.course import Landmark


@dataclass
class CameraModel:
    x: float = 0.20              # [m] camera position in base_link
    y: float = 0.0
    hfov: float = math.radians(100.0)
    range_min: float = 0.4
    range_max: float = 5.0
    max_incidence: float = math.radians(60.0)
    marker_size: float = 0.25    # [m] for the field-of-view test on both edges
    fov_margin: float = math.radians(2.0)


@dataclass
class NoiseModel:
    pos_std: float = 0.02        # [m] at 1 m range
    pos_std_per_m: float = 0.01  # [m] added per metre of range
    yaw_std: float = math.radians(1.5)   # [rad] marker orientation error
    dropout: float = 0.1         # probability a frame yields nothing


@dataclass
class Fix:
    marker_id: str
    x: float
    y: float
    yaw: float
    marker_range: float
    gate_distance: float
    covariance_scale: float


def visible(cam: CameraModel, car_x: float, car_y: float, car_yaw: float,
            lm: Landmark) -> Optional[Tuple[float, float]]:
    """(range, bearing) from the camera if the marker would be detected, else None."""
    c, s = math.cos(car_yaw), math.sin(car_yaw)
    cam_x = car_x + c * cam.x - s * cam.y
    cam_y = car_y + s * cam.x + c * cam.y
    dx, dy = lm.position[0] - cam_x, lm.position[1] - cam_y
    rng = math.hypot(dx, dy)
    if not (cam.range_min <= rng <= cam.range_max):
        return None
    bearing = math.atan2(dy, dx) - car_yaw
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
    h = cam.marker_size / 2.0
    for sgn in (-1.0, 1.0):
        ex = lm.position[0] + sgn * h * lm.x_axis[0] - cam_x
        ey = lm.position[1] + sgn * h * lm.x_axis[1] - cam_y
        b = math.atan2(ey, ex) - car_yaw
        b = math.atan2(math.sin(b), math.cos(b))
        if abs(b) > cam.hfov / 2.0 - cam.fov_margin:
            return None
    # incidence: the marker normal against the direction from marker to camera
    nx, ny = lm.z_axis[0], lm.z_axis[1]
    nn = math.hypot(nx, ny)
    if nn < 1e-6:
        return None
    cos_inc = (-dx * nx - dy * ny) / (rng * nn)
    if cos_inc < math.cos(cam.max_incidence):
        return None
    return rng, bearing


def _compose(a, b):
    """2-D pose composition a * b, poses as (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def _inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def simulate_fix(cam: CameraModel, noise: NoiseModel, rng: random.Random,
                 true_pose: Tuple[float, float, float],
                 ekf_pose: Optional[Tuple[float, float, float]],
                 landmarks: Dict[str, List[Landmark]],
                 ekf_position_tolerance: float) -> Optional[Fix]:
    """One camera frame through the stock localizer. None: nothing published."""
    if ekf_pose is None:
        return None
    if rng.random() < noise.dropout:
        return None
    best = None
    for mid, lms in landmarks.items():
        for lm in lms:
            vis = visible(cam, *true_pose, lm)
            if vis is None:
                continue
            r = vis[0]
            # the true detection: the marker in base_link
            lm_pose = (lm.position[0], lm.position[1], lm.facing_yaw)
            det = _compose(_inverse(true_pose), lm_pose)
            std = noise.pos_std + noise.pos_std_per_m * r
            det = (det[0] + rng.gauss(0.0, std), det[1] + rng.gauss(0.0, std),
                   det[2] + rng.gauss(0.0, noise.yaw_std))
            # the stock node's matching: detection placed with the EKF pose,
            # nearest mapped marker of that id
            det_on_map = _compose(ekf_pose, det)
            match, match_d = None, math.inf
            for cand in lms:
                d = math.hypot(cand.position[0] - det_on_map[0], cand.position[1] - det_on_map[1])
                if d < match_d:
                    match, match_d = cand, d
            if best is None or match_d < best[0]:
                best = (match_d, mid, match, det, r)
    if best is None:
        return None
    _, mid, match, det, r = best
    mapped = (match.position[0], match.position[1], match.facing_yaw)
    new_pose = _compose(mapped, _inverse(det))
    gate = math.hypot(new_pose[0] - ekf_pose[0], new_pose[1] - ekf_pose[1])
    if gate > ekf_position_tolerance:
        return None
    yaw = math.atan2(math.sin(new_pose[2]), math.cos(new_pose[2]))
    return Fix(mid, new_pose[0], new_pose[1], yaw, r, gate, max(1.0, (gate / 5.0) ** 3))


@dataclass
class Coverage:
    fraction: float          # of the path's points with a marker in view
    longest_gap: float       # [m] of path with none
    start_box_min: int       # fewest markers in view over start-box placements


def coverage(path, landmarks: Dict[str, List[Landmark]], start, cam: CameraModel = None,
             box: float = 0.3, box_yaw: float = math.radians(10.0)) -> Coverage:
    """How well a course's markers cover its path, for a forward camera.

    `path` is a Path (x, y, yaw, s); `start` an (x, y, yaw) start box, tried
    at +-box metres and +-box_yaw around it.
    """
    cam = cam or CameraModel()
    flat = [lm for lms in landmarks.values() for lm in lms]
    seen = 0
    gap = longest = 0.0
    for i in range(len(path.x)):
        if any(visible(cam, path.x[i], path.y[i], path.yaw[i], lm) for lm in flat):
            seen += 1
            gap = 0.0
        else:
            gap += (path.s[i] - path.s[i - 1]) if i else 0.0
            longest = max(longest, gap)
    sx, sy, syaw = start
    c, s_ = math.cos(syaw), math.sin(syaw)
    worst = None
    for dx in (-box, 0.0, box):
        for dy in (-box, 0.0, box):
            for dyaw in (-box_yaw, 0.0, box_yaw):
                x, y = sx + c * dx - s_ * dy, sy + s_ * dx + c * dy
                n = sum(1 for lm in flat if visible(cam, x, y, syaw + dyaw, lm))
                worst = n if worst is None else min(worst, n)
    return Coverage(seen / len(path.x), longest, worst or 0)
