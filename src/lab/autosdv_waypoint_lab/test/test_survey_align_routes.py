"""An aligned survey combined with a Lab 2 route is that route to Lab 2: same route hash,
same class, the route named in the bag's metadata.

The alignment itself is autosdv_marker_survey's (its test/test_align.py); this
checks what Lab 2 relies on when `just lab2 run` combines surveys/latest with a route.
"""
import math
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTE = os.path.normpath(os.path.join(HERE, '..', 'config', 'courses', 'lab2_uturn'))


def _drive(x0, y0, yaw0, rest=2.0, distance=3.0, speed=0.3, rate=50.0):
    from autosdv_marker_survey import align as al
    out, t, dt = [], 0.0, 1.0 / rate
    c, s = math.cos(yaw0), math.sin(yaw0)
    while t < rest:
        out.append(al.Sample(t, x0, y0, 0.0, yaw0, 0.0, 0.0, 0.0))
        t += dt
    d = 0.0
    while d < distance:
        d = min(distance, d + speed * dt)
        out.append(al.Sample(t, x0 + c * d, y0 + s * d, 0.0, yaw0, 0.0, 0.0, speed))
        t += dt
    for _ in range(int(rate)):
        out.append(al.Sample(t, x0 + c * d, y0 + s * d, 0.0, yaw0, 0.0, 0.0, 0.0))
        t += dt
    return out


def test_the_run_course_keeps_the_route_hash(tmp_path):
    from autosdv_marker_survey import align as al
    from autosdv_marker_survey.simulate import stage1_map
    from autosdv_waypoint_lab import bag_meta as bm
    from autosdv_waypoint_lab.course import load_course
    with open(os.path.join(ROUTE, 'course.yaml')) as f:
        d = yaml.safe_load(f)
    truth = {m['id']: (m['x'], m['y'], 0.25, m['yaw'], 0.0, 0.0) for m in d['markers']}
    stage1, T_w_s1 = stage1_map(truth, 24)
    s = tmp_path / 'survey'
    s.mkdir()
    (s / 'course.yaml').write_text(yaml.safe_dump(
        {'start_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'marker_size': 0.38,
         'dictionary': 'DICT_5X5_1000', 'survey': {'origin_marker': 24}, 'markers': stage1}))
    al.prepare_stage1(str(s))
    T = np.linalg.inv(T_w_s1)
    a, written = al.solve_dir(str(s), _drive(T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0])),
                              log=lambda *x: None)
    assert a.ok and written
    run = tmp_path / 'run'
    al.write_route_course(str(s), al.route_template(ROUTE), str(run))
    assert bm.course_hashes(str(run))['route_hash'] == bm.course_hashes(ROUTE)['route_hash']
    assert bm.classify_route(str(run)) == 'uturn'
    assert bm.course_route_name(str(run)) == 'lab2_uturn'
    c = load_course(str(run))
    assert (c.start_pose.x, c.start_pose.y, c.start_pose.yaw) == (0.0, 0.0, 0.0)
    assert len(c.markers) == len(truth) and not c.closed and c.laps == 1
