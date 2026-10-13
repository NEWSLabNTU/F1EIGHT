"""Route grading and the bag checker on synthetic message streams. No ROS."""
import math
from types import SimpleNamespace as NS

import pytest

from autosdv_waypoint_lab import bag_meta as bm
from autosdv_waypoint_lab import check_bags as cb
from autosdv_waypoint_lab import grading as g
from autosdv_waypoint_lab import route_grade as rg

TARGET = 0.8


def speed_profile(t, t_ramp=20.0, t_stop=24.0, rise=2.0, overshoot=0.0, wobble=0.0):
    """Measured speed: ramp up over `rise` s, plateau, linear stop between t_ramp and t_stop."""
    if t < rise:
        v = TARGET * t / rise
    elif t < t_ramp:
        v = TARGET + wobble * math.sin(8 * t) + overshoot * math.exp(-(t - rise) / 1.0)
    elif t < t_stop:
        v = TARGET * (t_stop - t) / (t_stop - t_ramp)
    else:
        v = 0.0
    return v


def cmd_profile(t, t_ramp=20.0, t_stop=24.0):
    if t < 0.5:
        return 0.0
    if t < t_ramp:
        return TARGET
    return max(0.0, TARGET * (t_stop - t) / (t_stop - t_ramp))


def kin(x, y):
    return NS(pose=NS(pose=NS(position=NS(x=x, y=y))))


def msgs_for(route, *, x_stop=0.0, overshoot=0.0, wobble=0.0, radius_off=0.0, mrm=False,
             laps=3, finished=True, engaged=True, t_ramp=20.0, t_stop=24.0, rise=2.0):
    ts = [0.1 * k for k in range(0, 300)]
    modes = [(0.0, NS(mode=1))] + ([(1.0, NS(mode=g.AUTONOMOUS))] if engaged else [])
    # shift: engaged at t=1; the profile starts at t=1
    vel = [(t, NS(longitudinal_velocity=speed_profile(t - 1.0, t_ramp, t_stop, rise, overshoot, wobble)))
           for t in ts if t >= 1.0]
    cmd = [(t, NS(longitudinal=NS(velocity=cmd_profile(t - 1.0, t_ramp, t_stop)))) for t in ts if t >= 1.0]
    if route == 'straight':
        # drive along -x from 11.5; stop point at (0, 0) heading -x; car rests at x_stop
        pos = []
        s = 0.0
        for t, v in vel:
            s += v.longitudinal_velocity * 0.1
            pos.append((t, kin(11.5 - s * (11.5 - x_stop) / max(1e-9, 11.5), 0.0)))
        # rescale so the final position is exactly x_stop
        final_s = s
        pos = []
        s = 0.0
        for t, v in vel:
            s += v.longitudinal_velocity * 0.1
            pos.append((t, kin(11.5 + (x_stop - 11.5) * s / final_s, 0.0)))
    else:
        R = 1.5 if route == 'circle_cw' else 2.5
        w = 1 if route == 'circle_ccw' else -1
        pos = [(t, kin((R + radius_off) * math.cos(w * 0.5 * t), (R + radius_off) * math.sin(w * 0.5 * t)))
               for t, _ in vel]
    prog = [(t, NS(laps_done=min(int(t // 8), laps), laps=laps if finished or True else 3,
                   finished=finished and t >= t_stop + 1.0)) for t in ts]
    out = {g.MODE: modes, g.VELOCITY: vel, g.CONTROL: cmd, g.KINEMATIC: pos, g.PROGRESS: prog,
           g.MRM: [(0.0, NS(state=g.MRM_NORMAL))] + ([(10.0, NS(state=2))] if mrm else [])}
    return out


STRAIGHT = rg.Geometry('straight', end_x=0.0, end_y=0.0, dir_x=-1.0, dir_y=0.0)


def circle_geo(R):
    return rg.Geometry('circle', cx=0.0, cy=0.0, radius=R)


def test_window_and_speed_rmse_ignore_the_start_transient():
    # a wobble of 0.05 m/s amplitude -> RMS 0.05/sqrt(2); the slow start must not count
    m = rg.route_metrics('b', msgs_for('straight', wobble=0.05), 'straight', STRAIGHT)
    assert m.speed_rmse == pytest.approx(0.05 / math.sqrt(2), rel=0.1)
    assert 1.0 < m.t90 < 4.0            # 90 % reached on the ramp (t=1+1.8)
    assert m.t_ramp == pytest.approx(1.0 + 19.9, abs=0.2)
    assert m.window_s > 15


def test_perfect_speed_scores_zero_rmse_and_overshoot():
    m = rg.route_metrics('b', msgs_for('straight'), 'straight', STRAIGHT)
    assert m.speed_rmse == pytest.approx(0.0, abs=0.01)
    assert m.overshoot == pytest.approx(0.0, abs=0.01)


def test_start_overshoot_is_the_peak_above_the_target():
    m = rg.route_metrics('b', msgs_for('straight', overshoot=0.15), 'straight', STRAIGHT)
    assert m.overshoot == pytest.approx(0.15, abs=0.03)


def test_slow_speed_gives_rmse_of_the_gap():
    ms = msgs_for('straight')
    ms[g.VELOCITY] = [(t, NS(longitudinal_velocity=min(v.longitudinal_velocity, 0.74)))
                      for t, v in ms[g.VELOCITY]]
    m = rg.route_metrics('b', ms, 'straight', STRAIGHT)
    assert m.speed_rmse == pytest.approx(0.06, abs=0.01)


def test_stop_error_short_and_past():
    short = rg.route_metrics('b', msgs_for('straight', x_stop=0.2), 'straight', STRAIGHT)
    assert short.stop_signed == pytest.approx(-0.2, abs=0.01)     # 0.2 m short of the stop point
    assert short.stop_error == pytest.approx(0.2, abs=0.01)
    past = rg.route_metrics('b', msgs_for('straight', x_stop=-0.2), 'straight', STRAIGHT)
    assert past.stop_signed == pytest.approx(0.2, abs=0.01)
    assert past.stop_error == pytest.approx(0.4, abs=0.02)        # past counts double


def test_stop_error_when_the_car_never_stops():
    ms = msgs_for('straight')
    ms[g.VELOCITY] = [(t, NS(longitudinal_velocity=max(v.longitudinal_velocity, 0.1)))
                      for t, v in ms[g.VELOCITY]]
    m = rg.route_metrics('b', ms, 'straight', STRAIGHT)
    assert any('never came to rest' in n for n in m.notes)


def test_circle_radius_error_and_gate():
    ok = rg.route_metrics('b', msgs_for('circle_cw', radius_off=0.1), 'circle_cw', circle_geo(1.5))
    assert ok.radius_rms == pytest.approx(0.1, abs=1e-6) and ok.radius_max == pytest.approx(0.1, abs=1e-6)
    assert not ok.gate_failures
    bad = rg.route_metrics('b', msgs_for('circle_ccw', radius_off=0.4), 'circle_ccw', circle_geo(2.5))
    assert any('radius RMS' in f for f in bad.gate_failures)
    assert any('radius max' in f for f in bad.gate_failures)
    assert rg.score_route(bad).total == 0.0


def test_laps_gate():
    m = rg.route_metrics('b', msgs_for('circle_cw', laps=2), 'circle_cw', circle_geo(1.5))
    assert any('lap' in f for f in m.gate_failures)


def test_mrm_before_finish_zeroes_the_bag():
    m = rg.route_metrics('b', msgs_for('straight', mrm=True), 'straight', STRAIGHT)
    assert m.mrm == 1
    assert rg.score_route(m).total == 0.0


def test_not_engaged_and_unfinished():
    m = rg.route_metrics('b', msgs_for('straight', engaged=False), 'straight', STRAIGHT)
    assert not m.engaged and rg.score_route(m).total == 0.0
    m = rg.route_metrics('b', msgs_for('straight', finished=False), 'straight', STRAIGHT)
    assert any('did not finish' in f for f in m.gate_failures)


def test_fraction_curve_against_a_reference():
    ref = 0.04
    assert rg.fraction(0.04, ref, 'speed_rmse') == 1.0
    assert rg.fraction(1.5 * ref, ref, 'speed_rmse') == 1.0
    assert rg.fraction(4 * ref, ref, 'speed_rmse') == 0.0
    mid = (rg.FULL_AT + rg.ZERO_AT) / 2 * ref
    assert rg.fraction(mid, ref, 'speed_rmse') == pytest.approx(0.5)
    assert rg.fraction(1.0, ref, 'speed_rmse') == 0.0
    assert rg.fraction(math.nan, ref, 'speed_rmse') == 0.0


def test_reference_floor_and_absolute_fallback():
    # a perfect reference does not make full marks unreachable
    assert rg.fraction(0.02 * rg.FULL_AT, 0.0, 'speed_rmse') == 1.0
    full, zero = rg.ABSOLUTE['stop_error']
    assert rg.fraction(full, None, 'stop_error') == 1.0
    assert rg.fraction(zero, None, 'stop_error') == 0.0


def test_scoring_reference_gets_full_marks_and_degraded_less():
    ref = rg.route_metrics('ref', msgs_for('straight', wobble=0.03, x_stop=0.05), 'straight', STRAIGHT)
    s_ref = rg.score_route(ref, ref)
    assert s_ref.total == pytest.approx(20.0)
    bad = rg.route_metrics('bad', msgs_for('straight', wobble=0.12, overshoot=0.2, x_stop=0.4),
                           'straight', STRAIGHT)
    s_bad = rg.score_route(bad, ref)
    assert s_bad.total < 12.0
    assert s_bad.points['speed_rmse'] < s_ref.points['speed_rmse']


def test_circle_points_and_team_total():
    ref = rg.route_metrics('r', msgs_for('circle_cw', wobble=0.03), 'circle_cw', circle_geo(1.5))
    s1 = rg.score_route(ref, ref)
    assert s1.total == pytest.approx(12.5)
    ref2 = rg.route_metrics('r', msgs_for('circle_ccw', wobble=0.03), 'circle_ccw', circle_geo(2.5))
    s2 = rg.score_route(ref2, ref2)
    refs = rg.route_metrics('r', msgs_for('straight'), 'straight', STRAIGHT)
    s3 = rg.score_route(refs, refs)
    assert rg.team_total([s1, s2, s3]) == (pytest.approx(45.0), 45.0)
    assert rg.team_total([s1, s2]) == (pytest.approx(25.0), 45.0)      # a missing bag scores 0


def test_geometry_from_templates():
    for route in bm.ROUTES:
        d = rg.template_dir(route)
        assert d, route
        geo = rg.geometry_from_course(d, route)
        if route == 'straight':
            assert math.hypot(geo.dir_x, geo.dir_y) == pytest.approx(1.0)
        else:
            assert geo.radius == pytest.approx(1.5 if route == 'circle_cw' else 2.5, abs=0.02)
        assert bm.classify_route(d) == route


# ----------------------------------------------------------- the bag checker

def good_meta(route='straight'):
    d = rg.template_dir(route)
    return bm.build_meta(d, 'A', '3', 'abc123')


def topics_all(rate=10.0, dur=25.0):
    t = {name: [100, 0.0, dur] for name in bm.REQUIRED_TOPICS + bm.OPTIONAL_TOPICS}
    t[bm.CAMERA_TOPIC] = [int(rate * dur) + 1, 0.0, dur]
    return t


def check(msgs=None, topics=None, meta='default', route='straight', **kw):
    msgs = msgs if msgs is not None else msgs_for(route)
    return cb.check_messages('bag', topics if topics is not None else topics_all(),
                             msgs, good_meta(route) if meta == 'default' else meta, **kw)


def test_good_bag_is_accepted():
    v = check()
    assert v.ok, v.reasons
    assert v.route == 'straight'


def test_missing_topic():
    t = topics_all()
    del t[bm.PWM]
    v = check(topics=t)
    assert any(bm.PWM in r for r in v.reasons)


def test_camera_too_slow_or_wrong_size():
    v = check(topics=topics_all(rate=5.0))
    assert any('camera at' in r for r in v.reasons)
    v = check(cam_size=(640, 480))
    assert any('640x480' in r for r in v.reasons)
    t = topics_all()
    t[bm.CAMERA_TOPIC] = [60, 0.0, 6.0]          # 10 Hz, but only the first 6 s
    v = check(topics=t)
    assert any('covers' in r for r in v.reasons)


def test_modified_course_hash():
    m = good_meta()
    m['route_hash'] = '0' * 64
    v = check(meta=m)
    assert any('modified course' in r for r in v.reasons)


def test_survey_may_change_markers_but_not_the_route(tmp_path):
    import shutil
    src = rg.template_dir('straight')
    d = tmp_path / 'surveyed'
    shutil.copytree(src, d)
    text = (d / 'course.yaml').read_text()
    (d / 'course.yaml').write_text(text.replace('id: 24', 'id: 99'))
    assert bm.course_hashes(str(d))['route_hash'] == bm.course_hashes(src)['route_hash']
    assert bm.course_hashes(str(d))['course_hash'] != bm.course_hashes(src)['course_hash']
    # a doctored path or speed is a different route
    e = tmp_path / 'doctored'
    shutil.copytree(src, e)
    (e / 'path.csv').write_text((e / 'path.csv').read_text() + '0.0,0.5\n')
    assert bm.course_hashes(str(e))['route_hash'] != bm.course_hashes(src)['route_hash']
    f = tmp_path / 'fast'
    shutil.copytree(src, f)
    (f / 'course.yaml').write_text((f / 'course.yaml').read_text().replace('v_ref: 0.8', 'v_ref: 1.2'))
    assert bm.course_hashes(str(f))['route_hash'] != bm.course_hashes(src)['route_hash']
    # a bag recorded on the doctored course is rejected against the template, and
    # against the surveyed copy it is accepted
    m = bm.build_meta(str(d), 'A', '3', 'x')
    assert check(meta=m, course_dirs=[str(d)]).ok
    assert not check(meta=bm.build_meta(str(e), 'A', '3', 'x'), course_dirs=[str(d)]).ok


def test_wrong_speed():
    m = good_meta()
    m['route'] = dict(m['route'], v_ref=1.2)
    v = check(meta=m)
    assert any('v_ref' in r for r in v.reasons)
    ms = msgs_for('straight')
    ms[g.CONTROL] = [(t, NS(longitudinal=NS(velocity=c.longitudinal.velocity * 1.5)))
                     for t, c in ms[g.CONTROL]]
    v = check(msgs=ms)
    assert any('commanded speed peaks' in r for r in v.reasons)


def test_too_few_laps_mrm_not_engaged():
    v = check(msgs=msgs_for('circle_cw', laps=2), route='circle_cw')
    assert any('lap' in r for r in v.reasons)
    v = check(msgs=msgs_for('straight', mrm=True))
    assert any('MRM' in r for r in v.reasons)
    v = check(msgs=msgs_for('straight', engaged=False))
    assert any('never engaged' in r for r in v.reasons)
    v = check(msgs=msgs_for('straight', finished=False))
    assert any('did not finish' in r for r in v.reasons)


def test_no_metadata_and_missing_ids():
    v = check(meta=None)
    assert any('no metadata' in r for r in v.reasons)
    m = good_meta()
    m['car_id'] = ''
    assert any('car or team' in r for r in check(meta=m).reasons)
    assert any('expected' in r for r in check(expect_car='B').reasons)


def test_set_checks():
    a = cb.Verdict('a', 'straight', 'A', '3')
    b = cb.Verdict('b', 'circle_cw', 'A', '3')
    c = cb.Verdict('c', 'circle_cw', 'B', '4')
    assert cb.check_set([a, b, c]) != []
    d = cb.Verdict('d', 'circle_ccw', 'A', '3')
    assert cb.check_set([a, b, d]) == []


def test_meta_from_topic_then_sidecar(tmp_path):
    import json
    meta = {'car_id': 'A'}
    assert bm.meta_from_messages({bm.META_TOPIC: [(0, NS(data=json.dumps(meta)))]}) == meta
    (tmp_path / bm.SIDECAR).write_text(json.dumps({'car_id': 'B'}))
    assert bm.meta_from_messages({}, str(tmp_path)) == {'car_id': 'B'}


def test_failed_circle_gate_is_invalid_not_penalised():
    bad = rg.route_metrics('b', msgs_for('circle_ccw', radius_off=0.4), 'circle_ccw', circle_geo(2.5))
    s = rg.score_route(bad)
    assert s.invalid and s.total == 0.0
    assert 'INVALID' in rg.format_route(bad, s)
    st = rg.route_metrics('b', msgs_for('straight', mrm=True), 'straight', STRAIGHT)
    assert not rg.score_route(st).invalid


def _centred(ms, route):
    geo = rg.geometry_from_course(rg.template_dir(route), route)
    ms[g.KINEMATIC] = [(t, kin(k.pose.pose.position.x + geo.cx, k.pose.pose.position.y + geo.cy))
                       for t, k in ms[g.KINEMATIC]]
    return ms


def test_checker_rejects_circle_radius_gate():
    ms = _centred(msgs_for('circle_cw', radius_off=0.5), 'circle_cw')
    v = check(msgs=ms, route='circle_cw')
    assert any('radius error' in r for r in v.reasons)
    assert check(msgs=_centred(msgs_for('circle_cw', radius_off=0.05), 'circle_cw'), route='circle_cw').ok
