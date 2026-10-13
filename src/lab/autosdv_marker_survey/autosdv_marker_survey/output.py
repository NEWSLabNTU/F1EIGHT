"""Write a survey into a course directory. No ROS.

    course.yaml          created, or updated in place: `markers`, `marker_size`,
                         `dictionary` and `survey` are replaced; everything else
                         (path, laps, v_ref, start_pose, ...) is kept. A new
                         course gets the recording's start as its start_pose.
    lanelet2_map.osm,    regenerated through autosdv_waypoint_lab.course, the
    ar_tag.param.yaml,   same code `just lab2 course-map` runs
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
        course = {'path': 'path.csv', 'closed': True, 'laps': 3, 'v_ref': 0.8,
                  'start_pose': start_pose or {'x': 0.0, 'y': 0.0, 'yaw': 0.0}}
    stamp = datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    course['marker_size'] = float(marker_size)
    course['dictionary'] = dictionary
    course['survey'] = {
        'origin_marker': int(origin_id), 'date': stamp,
        'frame': 'origin on the floor below the origin marker centre, x the way it faces, z up',
        'keyframes': info.get('keyframes'), 'reproj_rms_px': round(info.get('reproj_rms_px', 0.0), 3),
    }
    course['markers'] = [
        {'id': m.id, 'x': round(m.x, 4), 'y': round(m.y, 4), 'z': round(m.z, 4),
         'yaw': round(m.yaw, 4)} for m in markers]
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
            'views': m.views, 'reproj_rms_px': round(m.reproj_rms_px, 3),
            'sd_xy_cm': None if m.sd_xy is None else round(m.sd_xy * 100, 2),
            'sd_yaw_deg': None if m.sd_yaw_deg is None else round(m.sd_yaw_deg, 3),
            'flags': m.flags} for m in markers],
    }
    with open(os.path.join(course_dir, 'survey_report.yaml'), 'w') as f:
        yaml.safe_dump(report, f, sort_keys=False, default_flow_style=None)

    written = [path, os.path.join(course_dir, 'survey_report.yaml')]
    from autosdv_waypoint_lab import course as course_lib
    c = course_lib.load_course(course_dir)
    for p, text in ((c.map_file, course_lib.make_osm(c)),
                    (c.ar_tag_param_file, course_lib.make_ar_tag_params(c))):
        with open(p, 'w') as f:
            f.write(text)
        written.append(p)
    proj = os.path.join(course_dir, 'map_projector_info.yaml')
    if not os.path.exists(proj):
        with open(proj, 'w') as f:
            f.write('projector_type: local\n')
        written.append(proj)
    return written


def format_table(markers: List[SurveyedMarker]) -> str:
    rows = ['  id      x [m]    y [m]    z [m]  yaw [deg]  views  reproj  sd_xy  sd_yaw  flags']
    for m in markers:
        sdxy = '   -  ' if m.sd_xy is None else f'{m.sd_xy * 100:4.1f}cm'
        sdy = '   -  ' if m.sd_yaw_deg is None else f'{m.sd_yaw_deg:4.2f}d'
        rows.append(f'  {m.id:3d} {m.x:9.3f} {m.y:8.3f} {m.z:8.3f} {math.degrees(m.yaw):9.1f}'
                    f' {m.views:6d} {m.reproj_rms_px:6.2f}px {sdxy} {sdy}  {"; ".join(m.flags)}')
    return '\n'.join(rows)
