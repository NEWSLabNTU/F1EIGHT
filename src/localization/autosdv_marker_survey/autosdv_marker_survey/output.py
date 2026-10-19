"""Write a survey into a course directory. No ROS.

    course.yaml          created, or updated in place: `markers`, `marker_size`,
                         `dictionary` and `survey` are replaced; everything else
                         (start_pose, and a Lab 2 course's path, laps, v_ref, ...)
                         is kept. A new map gets the recording's start as its
                         start_pose, and nothing of a route: a bare survey is a
                         marker map in the origin board's frame.
    lanelet2_map.osm,    regenerated through autosdv_aruco_localization.marker_map
    ar_tag.param.yaml,   (make_marker_map)
    map_projector_info.yaml
    survey_report.yaml   per marker: views, reprojection, uncertainty, tilt, flags
"""

import datetime
import math
import os
from typing import List, Optional

import yaml

from autosdv_marker_survey.survey import SurveyedMarker


def write_course(course_dir: str, markers: List[SurveyedMarker], origin_id: int,
                 dictionary: str, marker_size: float, start_pose: Optional[dict],
                 info: dict) -> List[str]:
    os.makedirs(course_dir, exist_ok=True)
    path = os.path.join(course_dir, 'course.yaml')
    if os.path.exists(path):
        with open(path) as f:
            course = yaml.safe_load(f) or {}
    else:
        course = {'start_pose': start_pose or {'x': 0.0, 'y': 0.0, 'yaw': 0.0}}
    if course.pop('alignment', None) is not None:
        # survey-align had put the markers into the start-box frame; these are in the
        # origin board's again, so the alignment no longer holds
        print(f'WARNING: {path} was aligned (`just aruco-map align`); this survey replaces its '
              'markers in the origin board\'s frame and drops the alignment. Run '
              '`just aruco-map align` again before any run.')
    stamp = datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    course['marker_size'] = float(marker_size)
    course['dictionary'] = dictionary
    course['survey'] = {
        'origin_marker': int(origin_id), 'date': stamp,
        'frame': 'origin on the floor below the origin marker centre, x the way it faces '
                 '(horizontal), z up (the floor normal), y = z cross x',
        'keyframes': info.get('keyframes'), 'reproj_rms_px': round(info.get('reproj_rms_px', 0.0), 3),
    }
    course['markers'] = [
        {'id': m.id, 'x': round(m.x, 4), 'y': round(m.y, 4), 'z': round(m.z, 4),
         'yaw': round(m.yaw, 4), 'pitch': round(m.pitch, 4), 'roll': round(m.roll, 4)}
        for m in markers]
    order = ['path', 'closed', 'laps', 'v_ref', 'start_pose', 'marker_size', 'dictionary',
             'marker_height', 'survey', 'markers']
    ordered = {k: course[k] for k in order if k in course}
    ordered.update({k: v for k, v in course.items() if k not in ordered})
    header = (f'# Markers surveyed {stamp} by autosdv_marker_survey (origin marker {origin_id}).\n'
              '# Re-running the survey replaces `markers`, `marker_size`, `dictionary` and\n'
              '# `survey`; the rest of this file is yours.\n')
    with open(path, 'w') as f:
        f.write(header)
        yaml.safe_dump(ordered, f, sort_keys=False, default_flow_style=None)

    report = {
        'date': stamp, 'origin_marker': int(origin_id), 'dictionary': dictionary,
        'marker_size': float(marker_size), 'solve': info,
        'recording_start_pose': start_pose,
        'markers': [{
            'id': m.id, 'x': round(m.x, 4), 'y': round(m.y, 4), 'z': round(m.z, 4),
            'yaw_deg': round(math.degrees(m.yaw), 2), 'tilt_deg': round(m.tilt_deg, 2),
            'pitch_deg': round(math.degrees(m.pitch), 2), 'roll_deg': round(math.degrees(m.roll), 2),
            'views': m.views, 'reproj_rms_px': round(m.reproj_rms_px, 3),
            'sd_xy_cm': None if m.sd_xy is None else round(m.sd_xy * 100, 2),
            'sd_yaw_deg': None if m.sd_yaw_deg is None else round(m.sd_yaw_deg, 3),
            'flags': m.flags} for m in markers],
    }
    with open(os.path.join(course_dir, 'survey_report.yaml'), 'w') as f:
        yaml.safe_dump(report, f, sort_keys=False, default_flow_style=None)

    written = [path, os.path.join(course_dir, 'survey_report.yaml')]
    from autosdv_aruco_localization.marker_map import write_map_files
    written += write_map_files(course_dir)
    return written


def format_table(markers: List[SurveyedMarker]) -> str:
    rows = ['  id      x [m]    y [m]    z [m]  yaw [deg]  tilt  views  reproj  sd_xy  sd_yaw  flags']
    for m in markers:
        sdxy = '   -  ' if m.sd_xy is None else f'{m.sd_xy * 100:4.1f}cm'
        sdy = '   -  ' if m.sd_yaw_deg is None else f'{m.sd_yaw_deg:4.2f}d'
        rows.append(f'  {m.id:3d} {m.x:9.3f} {m.y:8.3f} {m.z:8.3f} {math.degrees(m.yaw):9.1f} {m.tilt_deg:4.1f}d'
                    f' {m.views:6d} {m.reproj_rms_px:6.2f}px {sdxy} {sdy}  {"; ".join(m.flags)}')
    return '\n'.join(rows)
