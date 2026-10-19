"""Marker survey from a moving camera: detection, live map, bundle adjustment. No ROS.

The camera (the car's ZED) moves through a room of ArUco markers. Every frame
comes with a camera pose from the camera's own tracking (the ZED SDK's visual-
inertial odometry), which drifts slowly but links frames in which the markers
are seen apart. The survey:

  1. detects markers in each keyframe (OpenCV ArUco, sub-pixel corners);
  2. keeps a LIVE estimate of every marker as soon as it is first seen
     (tracking pose x the marker's single-frame pose), for feedback while
     driving;
  3. at the end, solves a bundle adjustment over every keyframe and marker:
     marker corner reprojection (robust) + the tracking's relative motion
     between consecutive keyframes (weighted for ~1 % drift). Markers seen
     together correct the drift; markers seen alone still land in the frame;
  4. expresses the result in the COURSE frame defined by one marker, the
     origin: z up (the floor's normal, from the car's base_link poses), the
     origin on the floor below the origin marker's centre, x along the
     direction the origin marker faces.

Conventions. A marker's frame is OpenCV's: x to the viewer's right, y up, z out
of the marker towards the viewer; its corners, as ArucoDetector returns them,
are top-left, top-right, bottom-right, bottom-left. Poses are 4x4 matrices;
T_a_b maps b-frame points into frame a.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

# OpenCV names for the course.yaml dictionary names (autosdv_aruco_localization.marker_map).
OPENCV_DICTIONARY = {
    'ARUCO_MIP_36h12': 'DICT_ARUCO_MIP_36h12',
    'ARUCO': 'DICT_ARUCO_ORIGINAL',
    'TAG16h5': 'DICT_APRILTAG_16h5',
    'TAG25h9': 'DICT_APRILTAG_25h9',
    'TAG36h10': 'DICT_APRILTAG_36h10',
    'TAG36h11': 'DICT_APRILTAG_36h11',
}


def opencv_dictionary(name: str):
    """cv2.aruco dictionary for a course.yaml dictionary name, or an OpenCV DICT_* name."""
    import cv2
    if name == 'ALL_DICTS':
        raise ValueError('a survey needs one dictionary, not ALL_DICTS: name the printed family '
                         '(e.g. DICT_5X5_1000 or ARUCO_MIP_36h12)')
    cv_name = name if name.startswith('DICT_') else OPENCV_DICTIONARY.get(name)
    if cv_name is None or not hasattr(cv2.aruco, cv_name):
        raise ValueError(f'dictionary {name!r} is not available in this OpenCV '
                         f'({cv2.__version__}); DICT_ARUCO_MIP_36h12 needs OpenCV >= 4.7')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cv_name))


def marker_object_points(size: float) -> np.ndarray:
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


# ------------------------------------------------------------------ SE(3)

def rotvec_to_matrix(v) -> np.ndarray:
    v = np.asarray(v, float)
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * Kx + (1 - math.cos(th)) * (Kx @ Kx)


def matrix_to_rotvec(R) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_rotvec()


def make_T(R, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inv_T(T) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def T_from_params(p) -> np.ndarray:
    return make_T(rotvec_to_matrix(p[:3]), p[3:6])


def params_from_T(T) -> np.ndarray:
    return np.concatenate([matrix_to_rotvec(T[:3, :3]), T[:3, 3]])


def quat_to_matrix(x, y, z, w) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def interpolate_T(Ta, Tb, f: float) -> np.ndarray:
    """Pose a fraction f of the way from Ta to Tb (slerp + lerp)."""
    from scipy.spatial.transform import Rotation, Slerp
    rots = Rotation.from_matrix(np.stack([Ta[:3, :3], Tb[:3, :3]]))
    R = Slerp([0.0, 1.0], rots)([f]).as_matrix()[0]
    return make_T(R, (1 - f) * Ta[:3, 3] + f * Tb[:3, 3])


# ------------------------------------------------------------------ detection

@dataclass
class Detection:
    id: int
    corners: np.ndarray          # 4x2 px, TL TR BR BL


class Detector:
    def __init__(self, dictionary: str):
        import cv2
        self.cv2 = cv2
        self.dictionary = opencv_dictionary(dictionary)
        # OpenCV >= 4.7 has ArucoDetector; 4.5.x (JetPack 6) has the function API only.
        self.legacy = not hasattr(cv2.aruco, 'ArucoDetector')
        if self.legacy:
            self.params = cv2.aruco.DetectorParameters_create()
        else:
            self.params = cv2.aruco.DetectorParameters()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = None if self.legacy else cv2.aruco.ArucoDetector(self.dictionary, self.params)

    def detect(self, image) -> List[Detection]:
        gray = image if image.ndim == 2 else self.cv2.cvtColor(image, self.cv2.COLOR_BGR2GRAY)
        if self.legacy:
            corners, ids, _ = self.cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        else:
            corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return []
        return [Detection(int(i), c.reshape(4, 2).astype(float)) for i, c in zip(ids.ravel(), corners)]


def marker_pose_in_camera(det: Detection, K, size: float) -> Tuple[np.ndarray, float]:
    """T_cam_marker from one detection (IPPE, the better of its two solutions) and its
    RMS reprojection error [px]."""
    import cv2
    obj = marker_object_points(size)
    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
        obj, det.corners.reshape(-1, 1, 2), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    best = int(np.argmin(np.asarray(errs).ravel())) if n > 1 else 0
    R, _ = cv2.Rodrigues(rvecs[best])
    return make_T(R, tvecs[best].ravel()), float(np.asarray(errs).ravel()[best])


# ------------------------------------------------------------------ the survey

@dataclass
class Keyframe:
    stamp: float
    T_world_cam: np.ndarray            # from the camera's tracking
    detections: List[Detection]


@dataclass
class MarkerEstimate:
    id: int
    T_world_marker: np.ndarray
    views: int = 0
    best_area: float = 0.0


@dataclass
class SurveySettings:
    marker_size: float
    keyframe_distance: float = 0.15    # [m] new keyframe after this much motion...
    keyframe_angle: float = math.radians(5.0)   # ...or this much rotation
    min_marker_px: float = 20.0        # ignore detections smaller than this (edge length)
    max_reproj_px: float = 3.0         # ...or with a worse single-frame fit
    odom_drift: float = 0.01           # tracking: translation sd per metre travelled
    odom_floor_t: float = 0.003        # [m] and its floor
    odom_rot_per_rad: float = 0.01     # rotation sd per radian turned
    odom_floor_r: float = math.radians(0.2)
    pixel_sd: float = 0.5              # [px] corner noise
    huber_px: float = 1.5              # robust loss scale, in pixel sds


class Survey:
    """Accumulates keyframes and keeps a live marker map."""

    def __init__(self, settings: SurveySettings, K: np.ndarray):
        self.s = settings
        self.K = np.asarray(K, float)
        self.keyframes: List[Keyframe] = []
        self.markers: Dict[int, MarkerEstimate] = {}
        self.T_cam_base: Optional[np.ndarray] = None    # camera optical frame -> base_link

    def wants_keyframe(self, T_world_cam: np.ndarray, detections: List[Detection]) -> bool:
        if not detections:
            return False
        if not self.keyframes:
            return True
        last = self.keyframes[-1].T_world_cam
        d = np.linalg.norm(T_world_cam[:3, 3] - last[:3, 3])
        a = np.linalg.norm(matrix_to_rotvec(last[:3, :3].T @ T_world_cam[:3, :3]))
        new_marker = any(det.id not in self.markers for det in detections)
        return new_marker or d >= self.s.keyframe_distance or a >= self.s.keyframe_angle

    def usable(self, det: Detection) -> bool:
        c = det.corners
        edge = min(np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4))
        return edge >= self.s.min_marker_px

    def add(self, stamp: float, T_world_cam: np.ndarray, detections: List[Detection]) -> List[int]:
        """Add a frame if it is a keyframe. Returns the ids seen for the first time."""
        dets = [d for d in detections if self.usable(d)]
        if not self.wants_keyframe(T_world_cam, dets):
            return []
        kept, new = [], []
        for d in dets:
            T_cm, err = marker_pose_in_camera(d, self.K, self.s.marker_size)
            if err > self.s.max_reproj_px:
                continue
            kept.append(d)
            area = abs(np.cross(d.corners[2] - d.corners[0], d.corners[3] - d.corners[1])) / 2
            m = self.markers.get(d.id)
            if m is None:
                self.markers[d.id] = MarkerEstimate(d.id, T_world_cam @ T_cm, 1, area)
                new.append(d.id)
            else:
                m.views += 1
                if area > m.best_area:       # the live estimate follows the best view
                    m.T_world_marker = T_world_cam @ T_cm
                    m.best_area = area
        if kept:
            self.keyframes.append(Keyframe(stamp, T_world_cam.copy(), kept))
        return new

    # -------------------------------------------------------------- the solve
    def solve(self, max_nfev: int = 300):
        """Bundle adjustment. Returns (markers {id: T_world_marker}, keyframe poses,
        per-marker stats {id: dict}) in the tracking's world frame."""
        from scipy.optimize import least_squares
        from scipy.sparse import lil_matrix

        kfs = self.keyframes
        ids = sorted(self.markers)
        if len(kfs) < 2 or not ids:
            raise ValueError('not enough data: drive past the markers first')
        mi = {mid: n for n, mid in enumerate(ids)}
        nk, nm = len(kfs), len(ids)
        obj = marker_object_points(self.s.marker_size)
        obs = [(k, mi[d.id], d.corners) for k, kf in enumerate(kfs) for d in kf.detections]
        o_k = np.array([o[0] for o in obs])
        o_m = np.array([o[1] for o in obs])
        o_uv = np.stack([o[2] for o in obs])            # n x 4 x 2
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]

        # odometry factors between consecutive keyframes
        odo = []
        for a in range(nk - 1):
            T_ab = inv_T(kfs[a].T_world_cam) @ kfs[a + 1].T_world_cam
            dist = np.linalg.norm(T_ab[:3, 3])
            ang = np.linalg.norm(matrix_to_rotvec(T_ab[:3, :3]))
            st = self.s.odom_floor_t + self.s.odom_drift * dist
            sr = self.s.odom_floor_r + self.s.odom_rot_per_rad * ang
            odo.append((a, T_ab, st, sr))

        # unknowns: keyframes 1..nk-1 (0 is the gauge), then markers
        x0 = np.concatenate([params_from_T(kf.T_world_cam) for kf in kfs[1:]]
                            + [params_from_T(self.markers[mid].T_world_marker) for mid in ids])
        T0 = kfs[0].T_world_cam

        def kf_T(x, k):
            return T0 if k == 0 else T_from_params(x[6 * (k - 1):6 * k])

        def rotm_batch(v):
            from scipy.spatial.transform import Rotation
            return Rotation.from_rotvec(v).as_matrix()

        # odometry factors, batched: measured relative poses and their weights
        from scipy.spatial.transform import Rotation as _Rot
        o_a = np.array([o[0] for o in odo], int)
        R_ab = np.stack([o[1][:3, :3] for o in odo]) if odo else np.zeros((0, 3, 3))
        t_ab = np.stack([o[1][:3, 3] for o in odo]) if odo else np.zeros((0, 3))
        w_t = np.array([1.0 / o[2] for o in odo])
        w_r = np.array([1.0 / o[3] for o in odo])
        p0 = params_from_T(T0)

        def residuals(x):
            kp = np.vstack([p0[None, :], x[:6 * (nk - 1)].reshape(-1, 6)])
            mp = x[6 * (nk - 1):].reshape(nm, 6)
            Rk = rotm_batch(kp[:, :3]); tk = kp[:, 3:]
            Rk[0] = T0[:3, :3]; tk[0] = T0[:3, 3]
            Rm = rotm_batch(mp[:, :3]); tm = mp[:, 3:]
            Xw = np.einsum('nij,cj->nci', Rm[o_m], obj) + tm[o_m][:, None, :]      # n x 4 x 3
            Xc = np.einsum('nji,ncj->nci', Rk[o_k], Xw - tk[o_k][:, None, :])     # R^T (X - t)
            z = np.maximum(Xc[..., 2], 1e-3)
            u = fx * Xc[..., 0] / z + cx
            v = fy * Xc[..., 1] / z + cy
            r_proj = (np.stack([u, v], -1) - o_uv).reshape(-1) / self.s.pixel_sd
            if not len(o_a):
                return r_proj
            # estimated relative pose a->b, then its error against the measured one
            Ra, ta, Rb, tb = Rk[o_a], tk[o_a], Rk[o_a + 1], tk[o_a + 1]
            R_est = np.einsum('nji,njk->nik', Ra, Rb)                      # Ra^T Rb
            t_est = np.einsum('nji,nj->ni', Ra, tb - ta)                   # Ra^T (tb - ta)
            R_err = np.einsum('nji,njk->nik', R_ab, R_est)                 # R_ab^T R_est
            t_err = np.einsum('nji,nj->ni', R_ab, t_est - t_ab)            # R_ab^T (t_est - t_ab)
            r_rot = _Rot.from_matrix(R_err).as_rotvec() * w_r[:, None]
            r_tr = t_err * w_t[:, None]
            return np.concatenate([r_proj, np.hstack([r_rot, r_tr]).reshape(-1)])

        npar = x0.size
        nres = 8 * len(obs) + 6 * len(odo)
        sp = lil_matrix((nres, npar), dtype=np.int8)
        for n, (k, m, _) in enumerate(obs):
            rows = slice(8 * n, 8 * n + 8)
            if k > 0:
                sp[rows, 6 * (k - 1):6 * k] = 1
            sp[rows, 6 * (nk - 1) + 6 * m:6 * (nk - 1) + 6 * m + 6] = 1
        base = 8 * len(obs)
        for n, (a, _, _, _) in enumerate(odo):
            rows = slice(base + 6 * n, base + 6 * n + 6)
            for k in (a, a + 1):
                if k > 0:
                    sp[rows, 6 * (k - 1):6 * k] = 1
        sol = least_squares(residuals, x0, jac_sparsity=sp, loss='huber', f_scale=self.s.huber_px,
                            x_scale='jac', max_nfev=max_nfev, method='trf', ftol=1e-6, xtol=1e-6,
                            tr_solver='lsmr', tr_options={'atol': 1e-10, 'btol': 1e-10, 'maxiter': 3000})
        x = sol.x
        r = residuals(x)
        proj = r[:8 * len(obs)].reshape(len(obs), 8) * self.s.pixel_sd
        markers = {mid: T_from_params(x[6 * (nk - 1) + 6 * mi[mid]:6 * (nk - 1) + 6 * mi[mid] + 6])
                   for mid in ids}
        poses = [kf_T(x, k) for k in range(nk)]
        # marker covariance from the Gauss-Newton normal matrix
        cov = marker_covariances(sol.jac, nk, nm)
        stats = {}
        for mid in ids:
            sel = o_m == mi[mid]
            rms = float(np.sqrt(np.mean(proj[sel] ** 2))) if sel.any() else float('nan')
            stats[mid] = {'views': int(sel.sum()), 'reproj_rms_px': rms,
                          'cov_world': cov[mi[mid]] if cov is not None else None}
        info = {'keyframes': nk, 'observations': len(obs), 'cost': float(sol.cost),
                'reproj_rms_px': float(np.sqrt(np.mean(proj ** 2))), 'status': sol.message}
        return markers, poses, stats, info


def marker_covariances(jac, nk: int, nm: int):
    """6x6 covariance (rotvec, translation) of every marker, from J^T J, or None if
    the problem is too large to invert densely."""
    npar = jac.shape[1]
    if npar > 9000:
        return None
    JtJ = (jac.T @ jac)
    JtJ = JtJ.toarray() if hasattr(JtJ, 'toarray') else np.asarray(JtJ)
    try:
        C = np.linalg.pinv(JtJ)
    except np.linalg.LinAlgError:
        return None
    off = 6 * (nk - 1)
    return [C[off + 6 * m:off + 6 * m + 6, off + 6 * m:off + 6 * m + 6] for m in range(nm)]


# ------------------------------------------------------------------ the course frame

@dataclass
class CourseFrame:
    T_world_course: np.ndarray       # course frame in the tracking's world frame
    floor_normal_world: np.ndarray


def floor_from_base_poses(T_world_base: List[np.ndarray]):
    """The floor: mean base_link z axis as the normal, mean base_link origin on it."""
    zs = np.array([T[:3, 2] for T in T_world_base])
    n = zs.mean(axis=0)
    n /= np.linalg.norm(n)
    p = np.mean([T[:3, 3] for T in T_world_base], axis=0)
    return p, n


def course_frame(T_world_origin_marker: np.ndarray, floor_point, floor_normal) -> CourseFrame:
    """z = floor normal; origin = the origin marker's centre dropped onto the floor;
    x = the horizontal part of the direction the origin marker faces."""
    u = np.asarray(floor_normal, float)
    c = T_world_origin_marker[:3, 3]
    origin = c - ((c - floor_point) @ u) * u
    facing = T_world_origin_marker[:3, 2]
    xh = facing - (facing @ u) * u
    if np.linalg.norm(xh) < 1e-3:
        raise ValueError('the origin marker lies flat: it must stand upright')
    xh /= np.linalg.norm(xh)
    y = np.cross(u, xh)
    return CourseFrame(make_T(np.column_stack([xh, y, u]), origin), u)


@dataclass
class SurveyedMarker:
    id: int
    x: float
    y: float
    z: float                      # height of the centre above the floor
    yaw: float                    # direction it faces, in the course frame
    tilt_deg: float               # lean: angle between the marker's normal and the horizontal plane
    views: int
    reproj_rms_px: float
    pitch: float = 0.0            # top leaning back (course.yaml's convention)
    roll: float = 0.0             # turned in its own plane, counter-clockwise from the front
    sd_xy: Optional[float] = None   # [m] 1-sigma, worst horizontal direction
    sd_yaw_deg: Optional[float] = None
    flags: List[str] = field(default_factory=list)


def tilt_angles(R, yaw: float):
    """(pitch, roll) of a marker orientation R in the course frame, given its yaw: the
    inverse of autosdv_aruco_localization.marker_map.Marker.axes (upright at yaw, leaned back by
    pitch about x, turned by roll about the normal)."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    R0 = np.array([[-sy, 0.0, cy], [cy, 0.0, sy], [0.0, 1.0, 0.0]])   # columns: right, up, facing
    M = R0.T @ R                       # = Rx(-pitch) @ Rz(roll)
    a = math.atan2(-M[1, 2], M[2, 2])
    b = math.atan2(-M[0, 1], M[0, 0])
    return -a, b


def to_course(markers: Dict[int, np.ndarray], stats: dict, frame: CourseFrame,
              min_views: int = 3, max_sd_xy: float = 0.05, max_sd_yaw_deg: float = 1.0,
              max_tilt_deg: float = 10.0) -> List[SurveyedMarker]:
    Tci = inv_T(frame.T_world_course)
    out = []
    for mid, Tw in sorted(markers.items()):
        Tc = Tci @ Tw
        facing = Tc[:3, 2]
        yaw = math.atan2(facing[1], facing[0])
        # lean from vertical = angle between the normal and the horizontal plane; turning the
        # board in its own plane (roll) does not change it
        tilt = math.degrees(math.asin(max(0.0, min(1.0, abs(facing[2])))))
        pitch, roll = tilt_angles(Tc[:3, :3], yaw)
        st = stats.get(mid, {})
        sm = SurveyedMarker(mid, float(Tc[0, 3]), float(Tc[1, 3]), float(Tc[2, 3]), yaw, tilt,
                            int(st.get('views', 0)), float(st.get('reproj_rms_px', float('nan'))),
                            pitch=pitch, roll=roll)
        C = st.get('cov_world')
        if C is not None:
            R = Tci[:3, :3]
            Ct = R @ C[3:, 3:] @ R.T
            sm.sd_xy = float(math.sqrt(max(np.linalg.eigvalsh(Ct[:2, :2]).max(), 0.0)))
            Cr = R @ C[:3, :3] @ R.T
            sm.sd_yaw_deg = float(math.degrees(math.sqrt(max(Cr[2, 2], 0.0))))
        if sm.views < min_views:
            sm.flags.append(f'only {sm.views} view(s)')
        if sm.sd_xy is not None and sm.sd_xy > max_sd_xy:
            sm.flags.append(f'position sd {sm.sd_xy * 100:.1f} cm')
        if sm.sd_yaw_deg is not None and sm.sd_yaw_deg > max_sd_yaw_deg:
            sm.flags.append(f'yaw sd {sm.sd_yaw_deg:.1f} deg')
        if tilt > max_tilt_deg:
            sm.flags.append(f'tilted {tilt:.0f} deg from vertical')
        out.append(sm)
    return out
