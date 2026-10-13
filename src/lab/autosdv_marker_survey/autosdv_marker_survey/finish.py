"""Turn an accumulated survey into a course: solve, choose the frame, write. No ROS.

Shared by the live node (on save / on exit) and the offline re-solver, which
reads the survey's own dump (survey_data.npz) so a recording can be solved
again with other settings without the car.
"""

import math
from typing import Optional

import numpy as np

from autosdv_marker_survey import output
from autosdv_marker_survey.survey import (
    Detection, Keyframe, Survey, SurveySettings, course_frame, floor_from_base_poses, inv_T,
    to_course)


def finish(survey: Survey, origin_id: int, dictionary: str, course_dir: Optional[str],
           log=print, max_nfev: int = 300) -> dict:
    if origin_id not in survey.markers:
        seen = sorted(survey.markers)
        raise ValueError(f'origin marker {origin_id} was never seen (seen: {seen})')
    if survey.T_cam_base is None:
        raise ValueError('no camera -> base_link transform: the floor cannot be found')
    log(f'solving: {len(survey.keyframes)} keyframes, {len(survey.markers)} markers ...')
    markers, poses, stats, info = survey.solve(max_nfev=max_nfev)
    log(f'solved: reprojection RMS {info["reproj_rms_px"]:.2f} px ({info["status"]})')
    base = [T @ survey.T_cam_base for T in poses]
    p, n = floor_from_base_poses(base)
    frame = course_frame(markers[origin_id], p, n)
    surveyed = to_course(markers, stats, frame)
    Tb0 = inv_T(frame.T_world_course) @ base[0]
    start = {'x': round(float(Tb0[0, 3]), 3), 'y': round(float(Tb0[1, 3]), 3),
             'yaw': round(math.atan2(Tb0[1, 0], Tb0[0, 0]), 4)}
    log(output.format_table(surveyed))
    written = []
    if course_dir:
        written = output.write_course(course_dir, surveyed, origin_id, dictionary,
                                      survey.s.marker_size, start, info)
        for w in written:
            log(f'wrote {w}')
    return {'markers': surveyed, 'info': info, 'start_pose': start, 'written': written,
            'frame': frame, 'poses': poses}


# ------------------------------------------------------------------ the dump

def save_dump(path: str, survey: Survey, dictionary: str, origin_id: int) -> None:
    kf_T = np.stack([k.T_world_cam for k in survey.keyframes]) if survey.keyframes else np.zeros((0, 4, 4))
    rows = [(i, d.id, *d.corners.ravel()) for i, k in enumerate(survey.keyframes) for d in k.detections]
    np.savez_compressed(
        path, K=survey.K, T_cam_base=survey.T_cam_base if survey.T_cam_base is not None else np.eye(4),
        has_base=survey.T_cam_base is not None, kf_T=kf_T,
        kf_stamp=np.array([k.stamp for k in survey.keyframes]),
        detections=np.array(rows, dtype=float).reshape(-1, 10),
        marker_size=survey.s.marker_size, dictionary=dictionary, origin_id=origin_id)


def load_dump(path: str, settings: Optional[SurveySettings] = None):
    d = np.load(path, allow_pickle=False)
    s = settings or SurveySettings(marker_size=float(d['marker_size']))
    s.marker_size = float(d['marker_size'])
    sv = Survey(s, d['K'])
    if bool(d['has_base']):
        sv.T_cam_base = d['T_cam_base']
    dets = d['detections']
    for i, (T, stamp) in enumerate(zip(d['kf_T'], d['kf_stamp'])):
        rows = dets[dets[:, 0] == i]
        sv.keyframes.append(Keyframe(float(stamp), T, [
            Detection(int(r[1]), r[2:].reshape(4, 2)) for r in rows]))
    # live estimates as the node had them, for the solver's initial guess
    from autosdv_marker_survey.survey import MarkerEstimate, marker_pose_in_camera
    for kf in sv.keyframes:
        for det in kf.detections:
            T_cm, _ = marker_pose_in_camera(det, sv.K, s.marker_size)
            m = sv.markers.get(det.id)
            area = abs(np.cross(det.corners[2] - det.corners[0], det.corners[3] - det.corners[1])) / 2
            if m is None:
                sv.markers[det.id] = MarkerEstimate(det.id, kf.T_world_cam @ T_cm, 1, area)
            else:
                m.views += 1
                if area > m.best_area:
                    m.T_world_marker, m.best_area = kf.T_world_cam @ T_cm, area
    return sv, str(d['dictionary']), int(d['origin_id'])


def resolve_main(argv=None) -> int:
    """lab2_marker_survey_solve DUMP [--course DIR] [--origin ID]: solve a saved survey again."""
    import argparse
    ap = argparse.ArgumentParser(description=resolve_main.__doc__)
    ap.add_argument('dump', help='survey_data.npz written by the survey node')
    ap.add_argument('--course', help='course directory to write (default: none, print only)')
    ap.add_argument('--origin', type=int, help='override the origin marker id')
    ap.add_argument('--odom-drift', type=float, help='tracking drift, sd per metre (default 0.01)')
    ap.add_argument('--max-nfev', type=int, default=300, help='solver iteration cap')
    args = ap.parse_args(argv)
    settings = SurveySettings(marker_size=0.0)
    if args.odom_drift is not None:
        settings.odom_drift = args.odom_drift
    sv, dictionary, origin = load_dump(args.dump, settings)
    finish(sv, args.origin if args.origin is not None else origin, dictionary, args.course,
           max_nfev=args.max_nfev)
    return 0
