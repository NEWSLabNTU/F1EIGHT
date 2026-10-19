"""Stage 2 of the Lab 2 survey: put a surveyed board map into the START-BOX frame. No ROS.

Stage 1 (`just lab2 survey ID`) measures where the boards stand relative to one
another, in a frame anchored on one board: origin on the floor below it, x the
way it faces, z the floor's normal fitted from the car's own poses (so a board
that leans does not tilt that frame; a board that is turned or misplaced does
turn and shift it). The taped route does not care about that board. Stage 2
(`just lab2 survey-align`) runs the localization stack on the stage-1 map, the
TA drives the car STRAIGHT from the taped start box along the taped first leg,
and this module turns the localized poses of that drive into the route's frame:

    origin  the car's start pose (base_link, the rear axle centre), averaged
            over the REST_WINDOW before it started to move;
    x       the direction of a least-squares line through every pose of the
            drive (not just its ends), in the floor plane;

            "pose" is preferably a MULTI-BOARD pose: one PnP over every corner of
            every mapped board in a camera frame (pose_from_markers). The EKF's
            own pose follows the AR-tag localizer, which takes each fix from one
            board's orientation; at 4 m that is a degree or so off, a few cm of
            position bias that depends on where the car stands (7 cm at the
            bench's start box). Two boards metres apart pin the pose to about a
            centimetre. Without enough of them (two boards in view), the EKF's
            poses are used, with a warning;
    z       the stage-1 floor normal: stage 1 fitted it to every survey
            keyframe's base_link pose, which is better conditioned than one
            straight drive (a line does not fix a plane's roll about itself).
            The drive checks it: its slope and the EKF's mean roll and pitch
            are reported, and a slope over FLOOR_SLOPE_WARN_DEG is a warning;
    y       z cross x: the car's left.

Since both frames share the floor plane, the change of frame is a planar rigid
motion (x, y, yaw), and every board keeps its height, pitch and roll.

Refused (unless forced, with a warning) when the drive is shorter than
MIN_DISTANCE, wanders more than MAX_LATERAL off its own line, the car was not
at rest for MIN_REST before it moved, or the car pointed more than
MAX_START_YAW_DIFF away from where it then drove.

Writes, into the survey directory, the ALIGNED course: course.yaml (the route's
path, start_pose (0, 0, 0), laps, v_ref from the route template; the measured
boards in the start-box frame; an `alignment` block), path.csv (the template's,
byte for byte, so check-bags' route hash matches), the lanelet2 map and
ar_tag.param.yaml (autosdv_waypoint_lab.course, as `just lab2 course-map`),
map_projector_info.yaml, and align_report.txt. The stage-1 result is kept in
<survey>/stage1/.
"""

from dataclasses import dataclass, field
import datetime
import math
import os
import shutil
from typing import List, Optional, Sequence, Tuple

import numpy as np
import yaml

# ------------------------------------------------------------ the rules
MIN_DISTANCE = 1.0              # m: refuse a shorter drive
RECOMMENDED_DISTANCE = 2.0      # m: warn below this
MAX_LATERAL = 0.10              # m: refuse a drive that leaves its own line by more
# 5 cm refused every drive on car B (2026-10-16): the fixes were 1.4 cm RMS about the
# line, but one board set disagreed with another by ~8 cm, a survey error, not a curve.
MAX_START_YAW_DIFF = math.radians(10.0)   # refuse: the car pointed elsewhere than it drove
MIN_REST = 0.5                  # s at rest before the drive, at least
REST_WINDOW = 1.0               # s averaged for the start pose
REST_GUARD = 0.15               # s left out just before the motion onset
MOTION_SPEED = 0.05             # m/s of wheel speed that counts as moving
MOTION_DISPLACEMENT = 0.05      # m from the rest position that counts as moving
START_SPREAD_WARN = 0.03        # m: the start pose wandered at rest (EKF settling)
FLOOR_SLOPE_WARN_DEG = 2.0      # deg: the drive climbs against the stage-1 floor
CORRELATION_LENGTH = 0.25       # m: EKF pose errors along the drive are this correlated

STAGE1_DIR = 'stage1'
STAGE1_FILES = ('course.yaml', 'lanelet2_map.osm', 'ar_tag.param.yaml', 'map_projector_info.yaml',
                'survey_report.yaml')
POSES_FILE = 'align_poses.csv'
PNP_FILE = 'align_pnp.csv'
PNP_HEADER = 't,x,y,yaw,markers,rms_px,sd'
PNP_MIN_MARKERS = 2             # boards in one frame for a multi-board pose
PNP_MAX_RMS = 1.5               # px
PNP_MAX_SD = 0.015              # m: a multi-board pose less certain than this is not used
PNP_PIXEL_FLOOR = 0.3           # px: corner noise assumed at least this, for its uncertainty
PNP_MIN_REST = 3                # multi-board frames in the rest window to use them for the start
PNP_MIN_DRIVE = 10              # ... during the drive to fit the line to them
REPORT_FILE = 'align_report.txt'
POSES_HEADER = 't,x,y,z,yaw,roll,pitch,speed,std_xy'


@dataclass
class Sample:
    t: float
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    speed: float = math.nan     # wheel speed (velocity_status), nan if none
    std_xy: float = math.nan    # EKF sqrt(var_x + var_y)


@dataclass
class PnpSample:
    """A base_link pose from every board corner in one camera frame at once."""
    t: float
    x: float
    y: float
    yaw: float
    markers: int
    rms_px: float
    sd: float = math.nan          # m, 1-sigma position, worst direction (from the PnP's Jacobian)


@dataclass
class Alignment:
    ok: bool = False
    problems: List[str] = field(default_factory=list)     # refusals
    warnings: List[str] = field(default_factory=list)
    # the frame, in stage-1 coordinates
    start_x: float = math.nan
    start_y: float = math.nan
    start_z: float = math.nan
    start_yaw: float = math.nan       # the car's own heading at rest
    heading: float = math.nan         # x of the new frame: the fitted line's direction
    start_source: str = ''
    line_source: str = ''
    line_points: int = 0
    ekf_start_offset: float = math.nan   # m, the EKF's start against the chosen one
    ekf_lateral_rms: float = math.nan    # the EKF's poses about the line
    ekf_lateral_max: float = math.nan
    # the drive
    onset_t: float = math.nan
    rest_s: float = 0.0
    rest_samples: int = 0
    drive_samples: int = 0
    distance: float = 0.0             # start to the farthest pose, along the line
    lateral_rms: float = math.nan
    lateral_max: float = math.nan
    start_off_line: float = math.nan  # the start pose's distance from the fitted line
    heading_sd_deg: float = math.nan
    start_yaw_diff_deg: float = math.nan
    start_spread: float = math.nan    # m, RMS of the rest positions about their mean
    start_yaw_spread_deg: float = math.nan
    # the floor
    slope_deg: float = math.nan       # the fitted 3-D line against the stage-1 floor
    z_rms: float = math.nan           # drive poses' height about the stage-1 floor
    mean_roll_deg: float = math.nan
    mean_pitch_deg: float = math.nan
    forced: bool = False

    def to_frame(self, x: float, y: float, yaw: float = 0.0) -> Tuple[float, float, float]:
        """A stage-1 point (and heading) in the start-box frame."""
        c, s = math.cos(self.heading), math.sin(self.heading)
        dx, dy = x - self.start_x, y - self.start_y
        return c * dx + s * dy, -s * dx + c * dy, wrap(yaw - self.heading)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def circular_mean(angles: Sequence[float]) -> float:
    a = np.asarray(angles, float)
    return float(math.atan2(np.sin(a).mean(), np.cos(a).mean()))


# ------------------------------------------------------------ the drive

def motion_onset(samples: Sequence[Sample]) -> Optional[int]:
    """Index of the first sample of the drive: wheel speed above MOTION_SPEED on
    three samples in a row, or the position MOTION_DISPLACEMENT from where the car
    stood at the beginning, whichever comes first. None if it never moved."""
    n = len(samples)
    if n < 2:
        return None
    by_speed = None
    run = 0
    for i, s in enumerate(samples):
        run = run + 1 if (not math.isnan(s.speed) and abs(s.speed) > MOTION_SPEED) else 0
        if run >= 3:
            by_speed = i - 2
            break
    # the rest position: the median of the first quarter second
    t0 = samples[0].t
    head = [s for s in samples if s.t - t0 <= 0.25] or [samples[0]]
    x0 = float(np.median([s.x for s in head]))
    y0 = float(np.median([s.y for s in head]))
    by_disp = next((i for i, s in enumerate(samples)
                    if math.hypot(s.x - x0, s.y - y0) > MOTION_DISPLACEMENT), None)
    if by_disp is not None:
        # back up to where the displacement started to grow: the last sample still
        # within a fifth of the threshold
        j = by_disp
        while j > 0 and math.hypot(samples[j - 1].x - x0, samples[j - 1].y - y0) > 0.2 * MOTION_DISPLACEMENT:
            j -= 1
        by_disp = j
    cands = [i for i in (by_speed, by_disp) if i is not None]
    return min(cands) if cands else None


def fit_line(xy: np.ndarray, w: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
    """Total least squares, optionally weighted: (centroid, unit direction) of the
    points' best line."""
    w = np.ones(len(xy)) if w is None else np.asarray(w, float)
    c = (xy * w[:, None]).sum(axis=0) / w.sum()
    _, _, vt = np.linalg.svd((xy - c) * np.sqrt(w)[:, None], full_matrices=False)
    return c, vt[0] / np.linalg.norm(vt[0])


def heading_sd(lateral_rms: float, along: np.ndarray) -> float:
    """1-sigma of a fitted line's direction [rad] for lateral noise `lateral_rms`
    and the points' spread `along` it. EKF poses are correlated along the drive, so
    the count is the drive length over CORRELATION_LENGTH, not the sample count."""
    if len(along) < 2:
        return math.nan
    length = float(along.max() - along.min())
    n_eff = max(2.0, length / CORRELATION_LENGTH)
    # var(theta) = sigma^2 / sum((s - mean)^2); for n_eff evenly spread points that
    # sum is n_eff * length^2 / 12
    return float(lateral_rms / math.sqrt(n_eff * length ** 2 / 12.0)) if length > 0 else math.nan


def align(samples: Sequence[Sample], min_distance: float = MIN_DISTANCE,
          max_lateral: float = MAX_LATERAL, force: bool = False,
          pnp: Optional[Sequence[PnpSample]] = None) -> Alignment:
    """The start-box frame from one straight drive, in stage-1 coordinates.

    `samples` are the EKF's poses (with the wheel speed); they find the drive (the
    motion onset, the stop) and stand in for `pnp` where that is missing. `pnp` are
    multi-board poses (pose_from_markers): where there are enough of them, the
    start pose and the line come from them, because the EKF follows the AR-tag
    localizer, which takes each fix from ONE board's orientation and so carries a
    few centimetres of bias that depends on where the car stands."""
    a = Alignment(forced=force)
    samples = sorted(samples, key=lambda s: s.t)
    if len(samples) < 10:
        a.problems.append(f'{len(samples)} poses recorded: the localization did not run '
                          '(no marker fix?)')
        return _finish(a, force)
    onset = motion_onset(samples)
    if onset is None:
        a.problems.append('the car never moved: drive it straight along the first leg')
        return _finish(a, force)
    a.onset_t = samples[onset].t
    # the start: at rest, the REST_WINDOW before the onset
    t_end = a.onset_t - REST_GUARD
    rest = [s for s in samples if t_end - REST_WINDOW <= s.t <= t_end]
    a.rest_samples = len(rest)
    a.rest_s = (rest[-1].t - rest[0].t) if len(rest) > 1 else 0.0
    if a.rest_s < MIN_REST or len(rest) < 3:
        a.problems.append(f'the car was at rest for {max(0.0, a.rest_s):.2f} s before it moved, '
                          f'{MIN_REST} s needed: wait for READY, then drive')
        if not rest:
            return _finish(a, force)
    ekf_x = float(np.mean([s.x for s in rest]))
    ekf_y = float(np.mean([s.y for s in rest]))
    good = sorted((p for p in (pnp or []) if p.markers >= PNP_MIN_MARKERS and p.rms_px <= PNP_MAX_RMS
                   and not p.sd > PNP_MAX_SD), key=lambda p: p.t)
    pnp_rest = [p for p in good if t_end - REST_WINDOW <= p.t <= t_end]
    if len(pnp_rest) >= PNP_MIN_REST:
        a.start_source = f'{len(pnp_rest)} multi-board poses'
        a.start_x = float(np.mean([p.x for p in pnp_rest]))
        a.start_y = float(np.mean([p.y for p in pnp_rest]))
        a.start_yaw = circular_mean([p.yaw for p in pnp_rest])
        pts = [(p.x, p.y) for p in pnp_rest]
        yaws = [p.yaw for p in pnp_rest]
    else:
        a.start_source = f'the EKF ({len(rest)} poses)'
        a.warnings.append(f'{len(pnp_rest)} multi-board poses at the start ({PNP_MIN_REST} wanted: two or '
                          'more boards in view of the start box): the start is the EKF\'s, which can '
                          'be a few cm off')
        a.start_x, a.start_y = ekf_x, ekf_y
        a.start_yaw = circular_mean([s.yaw for s in rest])
        pts = [(s.x, s.y) for s in rest]
        yaws = [s.yaw for s in rest]
    a.start_z = float(np.mean([s.z for s in rest]))
    a.ekf_start_offset = math.hypot(ekf_x - a.start_x, ekf_y - a.start_y)
    a.start_spread = float(np.sqrt(np.mean([(x - a.start_x) ** 2 + (y - a.start_y) ** 2 for x, y in pts])))
    a.start_yaw_spread_deg = math.degrees(float(np.sqrt(np.mean([wrap(y - a.start_yaw) ** 2 for y in yaws]))))
    if a.start_spread > START_SPREAD_WARN:
        a.warnings.append(f'the start pose spread {100 * a.start_spread:.1f} cm at rest '
                          f'(over {100 * START_SPREAD_WARN:.0f} cm): the localization was still settling')

    # the drive: from the onset until the wheel stops (or, without a wheel speed, to
    # the pose farthest from the start)
    after = samples[onset:]
    moving = [i for i, s in enumerate(after) if not math.isnan(s.speed) and abs(s.speed) > MOTION_SPEED]
    if moving:
        drive = after[:moving[-1] + 1]
    else:
        d = np.array([math.hypot(s.x - a.start_x, s.y - a.start_y) for s in after])
        drive = after[:int(np.argmax(d)) + 1]
    a.drive_samples = len(drive)
    if len(drive) < 5:
        a.problems.append(f'{len(drive)} poses during the drive: too few to fit a line')
        return _finish(a, force)
    ekf_xy = np.array([[s.x, s.y] for s in drive])
    t0, t1 = drive[0].t, drive[-1].t
    pnp_drive = [p for p in good if t0 <= p.t <= t1]
    p_xy = np.array([[p.x, p.y] for p in pnp_drive]).reshape(-1, 2)
    span = float(np.max(np.hypot(p_xy[:, 0] - a.start_x, p_xy[:, 1] - a.start_y))) if len(p_xy) else 0.0
    ekf_far = float(np.max(np.hypot(ekf_xy[:, 0] - a.start_x, ekf_xy[:, 1] - a.start_y)))
    if len(pnp_drive) >= PNP_MIN_DRIVE and span >= min(MIN_DISTANCE, 0.5 * ekf_far):
        xy = p_xy
        sd = np.array([p.sd if p.sd > 0 else PNP_MAX_SD for p in pnp_drive])
        w = 1.0 / np.maximum(sd, 0.002) ** 2
        a.line_source = f'{len(pnp_drive)} multi-board poses over {span:.2f} m'
    else:
        xy = ekf_xy
        w = np.ones(len(xy))
        a.line_source = f'the EKF ({len(drive)} poses)'
        a.warnings.append(f'{len(pnp_drive)} multi-board poses during the drive, over {span:.2f} m: the '
                          'line is fitted to the EKF\'s poses, whose lateral scatter is the localizer\'s')
    # the start, as one point weighted like the line's best
    c, u = fit_line(np.vstack([xy, [[a.start_x, a.start_y]]]), np.concatenate([w, [w.max()]]))
    if (xy[-1] - [a.start_x, a.start_y]) @ u < 0:
        u = -u
    a.heading = math.atan2(u[1], u[0])
    n = np.array([-u[1], u[0]])
    lat = (xy - c) @ n
    along = (xy - [a.start_x, a.start_y]) @ u
    a.line_points = len(xy)
    a.distance = float(max(along.max(), ((ekf_xy - [a.start_x, a.start_y]) @ u).max()))
    a.lateral_rms = float(np.sqrt(np.mean(lat ** 2)))
    a.lateral_max = float(np.abs(lat).max())
    ekf_lat = (ekf_xy - c) @ n
    a.ekf_lateral_rms = float(np.sqrt(np.mean(ekf_lat ** 2)))
    a.ekf_lateral_max = float(np.abs(ekf_lat).max())
    a.start_off_line = float(abs((np.array([a.start_x, a.start_y]) - c) @ n))
    a.heading_sd_deg = math.degrees(heading_sd(a.lateral_rms, along))
    a.start_yaw_diff_deg = math.degrees(wrap(a.heading - a.start_yaw))
    # the floor: the drive against the stage-1 floor (z = 0)
    zs = np.array([s.z for s in drive])
    ekf_along = (ekf_xy - [a.start_x, a.start_y]) @ u
    a.z_rms = float(np.sqrt(np.mean(zs ** 2)))
    if np.ptp(ekf_along) > 0.2:
        a.slope_deg = math.degrees(math.atan(np.polyfit(ekf_along, zs, 1)[0]))
    a.mean_roll_deg = math.degrees(circular_mean([s.roll for s in drive]))
    a.mean_pitch_deg = math.degrees(circular_mean([s.pitch for s in drive]))

    if a.distance < min_distance:
        a.problems.append(f'drove {a.distance:.2f} m, at least {min_distance:g} m needed '
                          f'({RECOMMENDED_DISTANCE:g} m or more recommended)')
    elif a.distance < RECOMMENDED_DISTANCE:
        a.warnings.append(f'drove {a.distance:.2f} m; {RECOMMENDED_DISTANCE:g} m or more fixes the '
                          'heading better')
    if a.lateral_max > max_lateral:
        a.problems.append(f'not straight: {100 * a.lateral_max:.1f} cm off the fitted line at worst, '
                          f'{100 * max_lateral:.0f} cm allowed (RMS {100 * a.lateral_rms:.1f} cm)')
    if abs(a.start_yaw_diff_deg) > math.degrees(MAX_START_YAW_DIFF):
        a.problems.append(f'the car pointed {a.start_yaw_diff_deg:+.1f} deg off the way it drove '
                          f'(over {math.degrees(MAX_START_YAW_DIFF):.0f} deg): place it along the '
                          'taped first leg and drive straight ahead')
    if not math.isnan(a.slope_deg) and abs(a.slope_deg) > FLOOR_SLOPE_WARN_DEG:
        a.warnings.append(f'the drive climbs {a.slope_deg:+.1f} deg against the stage-1 floor '
                          f'(over {FLOOR_SLOPE_WARN_DEG:g} deg): a sloping floor, or a stage-1 map '
                          'that is tilted (check the survey report)')
    return _finish(a, force)


def _finish(a: Alignment, force: bool) -> Alignment:
    a.ok = not a.problems
    if a.problems and force and not math.isnan(a.heading):
        a.warnings += [f'FORCED past: {p}' for p in a.problems]
        a.ok = True
    return a


# ------------------------------------------------------------ multi-board pose

def marker_corner_points(markers, size: float) -> dict:
    """{id: 4x3 corners in the map, in the detector's order TL, TR, BR, BL} for
    autosdv_waypoint_lab.course.Marker objects (whose corners() are BL, BR, TR, TL)."""
    out = {}
    for m in markers:
        bl, br, tr, tl = m.corners(size)
        out[int(m.id)] = np.array([tl, tr, br, bl], float)
    return out


def pose_from_markers(detections, K, corners: dict, T_base_cam: np.ndarray,
                      T_map_base_guess: np.ndarray):
    """(T_map_base, reprojection RMS [px], boards used) from every corner of every
    mapped board in one image, or None. One board alone gives its orientation from
    its own small square, which at a few metres is off by a degree or so (and the
    position by range x that); two or more boards metres apart pin it far better."""
    import cv2
    obj, img, used = [], [], []
    for d in detections:
        c = corners.get(int(d.id))
        if c is None:
            continue
        obj.append(c)
        img.append(np.asarray(d.corners, float))
        used.append(int(d.id))
    if not used:
        return None
    obj = np.concatenate(obj).astype(np.float64)
    img = np.concatenate(img).astype(np.float64)
    T_cam_map = np.linalg.inv(T_map_base_guess @ T_base_cam)
    rvec, _ = cv2.Rodrigues(T_cam_map[:3, :3])
    tvec = T_cam_map[:3, 3].reshape(3, 1).copy()
    ok, rvec, tvec = cv2.solvePnP(obj, img, np.asarray(K, float), None, rvec, tvec,
                                  useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj, jac = cv2.projectPoints(obj, rvec, tvec, np.asarray(K, float), None)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))

    def base_of(rv, tv):
        R, _ = cv2.Rodrigues(rv)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, np.asarray(tv).ravel()
        return np.linalg.inv(T) @ np.linalg.inv(T_base_cam)

    T_map_base = base_of(rvec, tvec)
    # the position's uncertainty: corner noise (the fit's own RMS, at least
    # PNP_PIXEL_FLOOR) through the PnP's Jacobian to base_link's x, y
    J = jac[:, :6]
    sigma = max(rms, PNP_PIXEL_FLOOR)
    try:
        cov = sigma ** 2 * np.linalg.inv(J.T @ J)
        theta = np.concatenate([rvec.ravel(), tvec.ravel()])
        G = np.zeros((2, 6))
        for k in range(6):
            dt = np.zeros(6)
            dt[k] = 1e-6
            th = theta + dt
            G[:, k] = (base_of(th[:3].reshape(3, 1), th[3:])[:2, 3] - T_map_base[:2, 3]) / 1e-6
        sd = float(math.sqrt(max(np.linalg.eigvalsh(G @ cov @ G.T).max(), 0.0)))
    except np.linalg.LinAlgError:
        sd = math.inf
    return T_map_base, rms, used, sd


# ------------------------------------------------------------ the boards

def transform_markers(markers: Sequence[dict], a: Alignment) -> List[dict]:
    """course.yaml marker entries (stage 1) in the start-box frame: x, y and yaw
    move; z, pitch and roll are relative to the floor and the board's own yaw, and
    stay."""
    out = []
    for m in markers:
        x, y, yaw = a.to_frame(float(m['x']), float(m['y']), float(m.get('yaw', 0.0)))
        e = {'id': int(m['id']), 'x': round(x, 4), 'y': round(y, 4)}
        if 'z' in m:
            e['z'] = round(float(m['z']), 4)
        e['yaw'] = round(yaw, 4)
        for k in ('pitch', 'roll'):
            if k in m:
                e[k] = round(float(m[k]), 4)
        out.append(e)
    return out


# ------------------------------------------------------------ files

def write_pnp(path: str, pnp: Sequence[PnpSample]) -> None:
    with open(path, 'w') as f:
        f.write(PNP_HEADER + '\n')
        for p in pnp:
            f.write(f'{p.t:.4f},{p.x:.5f},{p.y:.5f},{p.yaw:.6f},{p.markers},{p.rms_px:.3f},{p.sd:.5f}\n')


def read_pnp(path: str) -> List[PnpSample]:
    out = []
    if not os.path.isfile(path):
        return out
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(('#', 't,')):
                continue
            v = line.split(',')
            out.append(PnpSample(float(v[0]), float(v[1]), float(v[2]), float(v[3]), int(v[4]), float(v[5]),
                                 float(v[6]) if len(v) > 6 else math.nan))
    return out


def read_samples(path: str) -> List[Sample]:
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(('#', 't,')):
                continue
            v = [float(x) for x in line.split(',')]
            out.append(Sample(*v[:9]))
    return out


def write_samples(path: str, samples: Sequence[Sample]) -> None:
    with open(path, 'w') as f:
        f.write(POSES_HEADER + '\n')
        for s in samples:
            f.write(f'{s.t:.4f},{s.x:.5f},{s.y:.5f},{s.z:.5f},{s.yaw:.6f},{s.roll:.6f},'
                    f'{s.pitch:.6f},{s.speed:.4f},{s.std_xy:.5f}\n')


def is_aligned(course_dir: str) -> bool:
    p = os.path.join(course_dir, 'course.yaml')
    if not os.path.isfile(p):
        return False
    with open(p) as f:
        d = yaml.safe_load(f) or {}
    return isinstance(d.get('alignment'), dict)


def prepare_stage1(survey_dir: str) -> str:
    """The stage-1 course directory to localize on: <survey>/stage1. A survey that is
    not aligned yet is copied there first (again, if a new stage 1 replaced an older
    alignment); an aligned one already has it."""
    survey_dir = os.path.abspath(survey_dir)
    s1 = os.path.join(survey_dir, STAGE1_DIR)
    if is_aligned(survey_dir):
        if not os.path.isfile(os.path.join(s1, 'course.yaml')):
            raise ValueError(f'{survey_dir} is aligned but has no {STAGE1_DIR}/course.yaml: '
                             'survey again (`just lab2 survey ID`)')
        return s1
    src = os.path.join(survey_dir, 'course.yaml')
    if not os.path.isfile(src):
        raise ValueError(f'{survey_dir} has no course.yaml: run `just lab2 survey ID` first')
    with open(src) as f:
        d = yaml.safe_load(f) or {}
    if not d.get('markers'):
        raise ValueError(f'{src} has no markers: run `just lab2 survey ID` first')
    os.makedirs(s1, exist_ok=True)
    for name in STAGE1_FILES:
        p = os.path.join(survey_dir, name)
        if os.path.isfile(p):
            shutil.copy2(p, os.path.join(s1, name))
    if not os.path.isfile(os.path.join(s1, 'map_projector_info.yaml')):
        with open(os.path.join(s1, 'map_projector_info.yaml'), 'w') as f:
            f.write('projector_type: local\n')
    return s1


ROUTE_KEYS = ('path', 'closed', 'laps', 'v_ref', 'start_pose', 'marker_height', 'uturn')


def write_aligned(survey_dir: str, a: Alignment, route_dir: str, report: str,
                  min_distance: float = MIN_DISTANCE, max_lateral: float = MAX_LATERAL) -> List[str]:
    """The aligned course in survey_dir (see the module docstring). Returns the files."""
    survey_dir = os.path.abspath(survey_dir)
    s1 = os.path.join(survey_dir, STAGE1_DIR)
    with open(os.path.join(s1, 'course.yaml')) as f:
        stage1 = yaml.safe_load(f) or {}
    with open(os.path.join(route_dir, 'course.yaml')) as f:
        route = yaml.safe_load(f) or {}
    path_name = str(route.get('path', 'path.csv'))
    sp = route.get('start_pose') or {}
    if any(abs(float(sp.get(k, 0.0))) > 1e-9 for k in ('x', 'y', 'yaw')):
        raise ValueError(f'route {route_dir}: start_pose is not (0, 0, 0); the aligned frame '
                         'puts the start box at the origin (regenerate it: just lab2 make-uturn)')
    stamp = datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    course = {k: route[k] for k in ROUTE_KEYS if k in route}
    course['marker_size'] = stage1.get('marker_size')
    course['dictionary'] = stage1.get('dictionary')
    if 'marker_height' in stage1 and 'marker_height' not in course:
        course['marker_height'] = stage1['marker_height']
    if 'survey' in stage1:
        course['survey'] = stage1['survey']
    course['alignment'] = {
        'date': stamp,
        'route': os.path.basename(os.path.normpath(route_dir)),
        'frame': 'origin at the start box (base_link at rest), x along the drive, z the stage-1 floor normal',
        'stage1_start': {'x': round(a.start_x, 4), 'y': round(a.start_y, 4),
                         'heading': round(a.heading, 5)},
        'distance_m': round(a.distance, 3),
        'lateral_rms_m': round(a.lateral_rms, 4),
        'lateral_max_m': round(a.lateral_max, 4),
        'heading_sd_deg': round(a.heading_sd_deg, 3),
        'start_spread_m': round(a.start_spread, 4),
        'start_from': a.start_source,
        'line_from': a.line_source,
        'forced': bool(a.forced and a.problems),
    }
    course['markers'] = transform_markers(stage1.get('markers') or [], a)
    header = (f'# ALIGNED {stamp} by `just lab2 survey-align` (autosdv_marker_survey/align.py).\n'
              f'# Route {course["alignment"]["route"]}: path, start_pose, laps and v_ref are the\n'
              '# route template\'s (path.csv is its copy, byte for byte); the markers are the\n'
              f'# stage-1 survey ({STAGE1_DIR}/course.yaml) in the START-BOX frame: origin at the\n'
              '# car\'s start pose, x along the straight drive, y to its left, z up.\n'
              f'# {REPORT_FILE} says how well the drive fixed it. Re-surveying (`just lab2 survey`)\n'
              '# into this directory drops the alignment; run survey-align again after it.\n')
    written = []
    p = os.path.join(survey_dir, 'course.yaml')
    with open(p, 'w') as f:
        f.write(header)
        yaml.safe_dump(course, f, sort_keys=False, default_flow_style=None)
    written.append(p)
    p = os.path.join(survey_dir, path_name)
    shutil.copyfile(os.path.join(route_dir, path_name), p)
    written.append(p)
    from autosdv_waypoint_lab import course as course_lib
    c = course_lib.load_course(survey_dir)
    for p, text in ((c.map_file, course_lib.make_osm(c)),
                    (c.ar_tag_param_file, course_lib.make_ar_tag_params(c))):
        with open(p, 'w') as f:
            f.write(text)
        written.append(p)
    p = os.path.join(survey_dir, 'map_projector_info.yaml')
    with open(p, 'w') as f:
        f.write('projector_type: local\n')
    written.append(p)
    p = os.path.join(survey_dir, REPORT_FILE)
    with open(p, 'w') as f:
        f.write(report)
    written.append(p)
    return written


def report_text(a: Alignment, survey_dir: str = '', route: str = '',
                markers: Optional[List[dict]] = None, stage1_markers: Optional[List[dict]] = None) -> str:
    def cm(v):
        return 'n/a' if math.isnan(v) else f'{100 * v:.1f} cm'

    def deg(v, signed=False):
        return 'n/a' if math.isnan(v) else (f'{v:+.2f} deg' if signed else f'{v:.2f} deg')

    verdict = ('ALIGNED' if a.ok and not a.forced else 'ALIGNED (FORCED)' if a.ok else 'REFUSED')
    lines = [
        f'survey-align {verdict}' + (f'  {survey_dir}' if survey_dir else ''),
        f'  route            {route}' if route else None,
        '',
        'the frame (in stage-1 coordinates)',
        f'  start pose       ({a.start_x:.3f}, {a.start_y:.3f}) m, car heading '
        f'{math.degrees(a.start_yaw):.2f} deg' if not math.isnan(a.start_x) else '  start pose       n/a',
        f'  x axis           {math.degrees(a.heading):.2f} deg (the fitted line)' if not math.isnan(a.heading)
        else '  x axis           n/a',
        f'  start from       {a.start_source or "n/a"}, at rest {a.rest_s:.2f} s; spread '
        f'{cm(a.start_spread)}, yaw {deg(a.start_yaw_spread_deg)}',
        f'  EKF start        {cm(a.ekf_start_offset)} from it (the localizer\'s own fix there)',
        '',
        'the drive',
        f'  distance         {a.distance:.2f} m (refused under {MIN_DISTANCE:g} m, '
        f'{RECOMMENDED_DISTANCE:g} m or more recommended), {a.drive_samples} poses',
        f'  line fitted to   {a.line_source or "n/a"}',
        f'  straightness     lateral RMS {cm(a.lateral_rms)}, max {cm(a.lateral_max)} '
        f'(refused over {100 * MAX_LATERAL:.0f} cm)',
        f'  EKF about line   RMS {cm(a.ekf_lateral_rms)}, max {cm(a.ekf_lateral_max)} (not checked)',
        f'  heading          1-sigma {deg(a.heading_sd_deg)} (from the lateral scatter; '
        + ('n/a' if math.isnan(a.heading_sd_deg) else
           f'{100 * math.tan(math.radians(a.heading_sd_deg)) * 10:.1f} cm at 10 m') + ')',
        f'  car vs line      {deg(a.start_yaw_diff_deg, True)} (the car\'s heading at rest against '
        f'the drive; refused over {math.degrees(MAX_START_YAW_DIFF):.0f} deg)',
        f'  start off line   {cm(a.start_off_line)}',
        '',
        'the floor (z = the stage-1 floor normal, fitted to the stage-1 keyframes)',
        f'  drive slope      {deg(a.slope_deg, True)} (warned over {FLOOR_SLOPE_WARN_DEG:g} deg)',
        f'  drive height     RMS {cm(a.z_rms)} about the stage-1 floor',
        f'  EKF attitude     mean roll {deg(a.mean_roll_deg, True)}, pitch {deg(a.mean_pitch_deg, True)}',
    ]
    if a.problems:
        lines += ['', 'REFUSED' if not a.ok else 'refusals overridden by --force:']
        lines += [f'  - {p}' for p in a.problems]
    if a.warnings:
        lines += ['', 'warnings']
        lines += [f'  - {w}' for w in a.warnings]
    if markers is not None:
        old = {int(m['id']): m for m in (stage1_markers or [])}
        lines += ['', 'boards (start-box frame)          stage 1',
                  '    id      x [m]    y [m]  yaw [deg]      x [m]    y [m]  yaw [deg]']
        for m in markers:
            o = old.get(int(m['id']), {})
            lines.append(f'  {m["id"]:4d} {m["x"]:9.3f} {m["y"]:8.3f} {math.degrees(m["yaw"]):9.1f}  '
                         f'{float(o.get("x", math.nan)):9.3f} {float(o.get("y", math.nan)):8.3f} '
                         f'{math.degrees(float(o.get("yaw", math.nan))):9.1f}')
    return '\n'.join(ln for ln in lines if ln is not None) + '\n'


def solve_dir(survey_dir: str, route_dir: str, samples: Sequence[Sample], force: bool = False,
              min_distance: float = MIN_DISTANCE, max_lateral: float = MAX_LATERAL,
              log=print, pnp: Optional[Sequence[PnpSample]] = None) -> Tuple[Alignment, List[str]]:
    """Align, print the report, and write the aligned course if it passed."""
    a = align(samples, min_distance, max_lateral, force, pnp)
    with open(os.path.join(survey_dir, STAGE1_DIR, 'course.yaml')) as f:
        stage1 = yaml.safe_load(f) or {}
    markers = transform_markers(stage1.get('markers') or [], a) if not math.isnan(a.heading) else None
    route = os.path.basename(os.path.normpath(route_dir))
    text = report_text(a, survey_dir, route, markers, stage1.get('markers'))
    log(text)
    written: List[str] = []
    if a.ok:
        written = write_aligned(survey_dir, a, route_dir, text, min_distance, max_lateral)
        for w in written:
            log(f'wrote {w}')
    else:
        p = os.path.join(survey_dir, REPORT_FILE)
        with open(p, 'w') as f:
            f.write(text)
        log(f'nothing aligned; the report is in {p} (--force writes it anyway)')
    return a, written


def route_template(route: str) -> str:
    """A route name (a packaged course) or a course directory."""
    if os.path.isdir(route):
        return os.path.abspath(route)
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('autosdv_waypoint_lab'), 'config', 'courses', route)
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    raise ValueError(f'no route {route!r}: name a course under autosdv_waypoint_lab/config/courses '
                     'or a course directory')


def solve_main(argv=None) -> int:
    """lab2_survey_align_solve SURVEY_DIR [--route lab2_uturn] [--force]: align again from the
    recorded drive (SURVEY_DIR/align_poses.csv), without the car."""
    import argparse
    ap = argparse.ArgumentParser(description=solve_main.__doc__)
    ap.add_argument('survey_dir')
    ap.add_argument('--route', default='lab2_uturn')
    ap.add_argument('--poses', default=None, help=f'default SURVEY_DIR/{POSES_FILE}')
    ap.add_argument('--force', action='store_true')
    ap.add_argument('--min-distance', type=float, default=MIN_DISTANCE)
    ap.add_argument('--max-lateral', type=float, default=MAX_LATERAL)
    args = ap.parse_args(argv)
    d = os.path.abspath(args.survey_dir)
    prepare_stage1(d)
    samples = read_samples(args.poses or os.path.join(d, POSES_FILE))
    a, _ = solve_dir(d, route_template(args.route), samples, args.force, args.min_distance,
                     args.max_lateral)
    return 0 if a.ok else 1
