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


UTURN = rg.geometry_from_course(rg.template_dir('uturn'), 'uturn')
T_RAMP, T_STOP = 31.0, 35.0     # the plateau covers the route up to its end ramp


def place(geo, s, lateral=0.0):
    """(x, y) at arc length s on the route (run on straight past its end), moved
    `lateral` metres to the left."""
    path = geo.path
    if s <= path.length:
        x, y, yaw = path.point_at(s)
    else:
        x, y = geo.end_x + (s - path.length) * geo.dir_x, geo.end_y + (s - path.length) * geo.dir_y
        yaw = math.atan2(geo.dir_y, geo.dir_x)
    return x - lateral * math.sin(yaw), y + lateral * math.cos(yaw)


def msgs_for(route='uturn', *, stop_off=0.0, overshoot=0.0, wobble=0.0, turn_dip=0.0, lateral=0.0,
             mrm=False, laps=1, finished=True, engaged=True, t_ramp=T_RAMP, t_stop=T_STOP, rise=2.0,
             geo=UTURN):
    """A run along the route: engaged at t = 1, speed profile from t = 1, the car
    coming to rest stop_off metres past the stop point (negative: short). In the
    turn the measured speed is turn_dip lower (the position is integrated from
    the profile, so the dip moves no sample)."""
    ts = [0.1 * k for k in range(0, 450)]
    modes = [(0.0, NS(mode=1))] + ([(1.0, NS(mode=g.AUTONOMOUS))] if engaged else [])
    prof = [(t, speed_profile(t - 1.0, t_ramp, t_stop, rise, overshoot, wobble)) for t in ts if t >= 1.0]
    final = sum(v * 0.1 for _, v in prof)
    target_s = geo.path.length + stop_off
    vel, pos, s = [], [], 0.0
    for t, v in prof:
        s += v * 0.1
        sp = s * target_s / final
        pos.append((t, kin(*place(geo, sp, lateral))))
        on_plateau = 1.0 + rise <= t <= 1.0 + t_ramp
        vel.append((t, NS(longitudinal_velocity=v - (turn_dip if on_plateau and geo.in_turn(sp) else 0.0))))
    cmd = [(t, NS(longitudinal=NS(velocity=cmd_profile(t - 1.0, t_ramp, t_stop)))) for t in ts if t >= 1.0]
    prog = [(t, NS(laps_done=laps if (finished and t >= t_stop + 1.0) else 0, laps=laps,
                   finished=finished and t >= t_stop + 1.0)) for t in ts]
    return {g.MODE: modes, g.VELOCITY: vel, g.CONTROL: cmd, g.KINEMATIC: pos, g.PROGRESS: prog,
            g.MRM: [(0.0, NS(state=g.MRM_NORMAL))] + ([(10.0, NS(state=2))] if mrm else [])}


def metrics(**kw):
    return rg.route_metrics('b', msgs_for(**kw), 'uturn', UTURN)


def test_window_and_speed_rmse_ignore_the_start_transient():
    # a wobble of 0.05 m/s amplitude -> RMS 0.05/sqrt(2); the slow start must not count
    m = metrics(wobble=0.05)
    assert m.speed_rmse == pytest.approx(0.05 / math.sqrt(2), rel=0.1)
    assert m.speed_rmse_straight == pytest.approx(0.05 / math.sqrt(2), rel=0.15)
    assert m.speed_rmse_turn == pytest.approx(0.05 / math.sqrt(2), rel=0.25)
    assert 1.0 < m.t90 < 4.0            # 90 % reached on the ramp (t=1+1.8)
    assert m.t_ramp == pytest.approx(1.0 + T_RAMP - 0.1, abs=0.2)
    assert m.window_s > 25


def test_perfect_speed_scores_zero_errors_and_overshoot():
    m = metrics()
    assert m.speed_rmse_straight == pytest.approx(0.0, abs=0.01)
    assert m.speed_rmse_turn == pytest.approx(0.0, abs=0.01)
    assert m.overshoot == pytest.approx(0.0, abs=0.01)
    assert m.path_dev_max == pytest.approx(0.0, abs=0.01)
    assert not m.gate_failures


def test_samples_split_into_straight_and_turn_by_arc_length():
    m = metrics(turn_dip=0.1)
    assert m.speed_rmse_turn == pytest.approx(0.1, abs=0.005)
    # the straight keeps only the window's first samples, still on the start ramp
    assert m.speed_rmse_straight == pytest.approx(metrics().speed_rmse_straight, abs=1e-6)
    assert m.speed_rmse_straight < 0.01
    (s0, s1), = UTURN.turns
    # 10 Hz samples over the arc at about 0.8 m/s
    assert m.samples_turn == pytest.approx((s1 - s0) / 0.8 / 0.1, rel=0.1)
    assert m.samples_straight > 2 * m.samples_turn


def test_the_turn_is_read_from_the_course():
    from autosdv_waypoint_lab import uturn
    (s0, s1), = UTURN.turns
    assert (s0, s1) == pytest.approx((10.0, 10.0 + 1.5 * math.pi), abs=0.02)
    for radius, leg in ((1.0, 6.0), (2.5, 14.0)):
        geo = rg.geometry_from_points('uturn', *uturn.route_polyline(radius, leg))
        assert geo.turns == [pytest.approx((leg, leg + radius * math.pi), abs=0.02)]
        m = rg.route_metrics('b', msgs_for(turn_dip=0.1, geo=geo, t_ramp=(2 * leg + math.pi * radius) / 0.8 - 1.0,
                                           t_stop=(2 * leg + math.pi * radius) / 0.8 + 3.0), 'uturn', geo)
        assert m.speed_rmse_turn == pytest.approx(0.1, abs=0.005)
        assert m.speed_rmse_straight < 0.01


def test_start_overshoot_is_the_peak_above_the_target():
    m = metrics(overshoot=0.15)
    assert m.overshoot == pytest.approx(0.15, abs=0.03)


def test_slow_speed_gives_rmse_of_the_gap():
    ms = msgs_for()
    ms[g.VELOCITY] = [(t, NS(longitudinal_velocity=min(v.longitudinal_velocity, 0.74)))
                      for t, v in ms[g.VELOCITY]]
    m = rg.route_metrics('b', ms, 'uturn', UTURN)
    assert m.speed_rmse_straight == pytest.approx(0.06, abs=0.01)
    assert m.speed_rmse_turn == pytest.approx(0.06, abs=0.01)


def test_stop_error_short_and_past():
    short = metrics(stop_off=-0.2)
    assert short.stop_signed == pytest.approx(-0.2, abs=0.01)     # 0.2 m short of the stop point
    assert short.stop_error == pytest.approx(0.2, abs=0.01)
    past = metrics(stop_off=0.2)
    assert past.stop_signed == pytest.approx(0.2, abs=0.01)
    assert past.stop_error == pytest.approx(0.4, abs=0.02)        # past counts double


def test_stop_error_when_the_car_never_stops():
    ms = msgs_for()
    ms[g.VELOCITY] = [(t, NS(longitudinal_velocity=max(v.longitudinal_velocity, 0.1)))
                      for t, v in ms[g.VELOCITY]]
    m = rg.route_metrics('b', ms, 'uturn', UTURN)
    assert any('never came to rest' in n for n in m.notes)


def test_off_the_path_is_invalid():
    ok = metrics(lateral=0.2)
    assert ok.path_dev_max == pytest.approx(0.2, abs=0.01) and not ok.gate_failures
    bad = metrics(lateral=0.35)
    assert any('off the path' in f for f in bad.gate_failures)
    s = rg.score_route(bad, bad)
    assert s.invalid and s.total == 0.0
    assert 'INVALID' in rg.format_route(bad, s)


def test_mrm_before_finish_zeroes_the_bag():
    m = metrics(mrm=True)
    assert m.mrm == 1
    assert rg.score_route(m).total == 0.0 and rg.score_route(m).invalid


def test_not_engaged_and_unfinished():
    m = metrics(engaged=False)
    assert not m.engaged and rg.score_route(m).total == 0.0
    m = metrics(finished=False)
    assert any('did not finish' in f for f in m.gate_failures)
    assert rg.score_route(m).total == 0.0


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
    ref = metrics(wobble=0.03, stop_off=0.05)
    s_ref = rg.score_route(ref, ref)
    assert s_ref.total == pytest.approx(45.0) and s_ref.max_total == 45.0
    assert set(s_ref.points) == {'speed_rmse_straight', 'speed_rmse_turn', 'overshoot', 'stop_error'}
    bad = metrics(wobble=0.12, overshoot=0.2, stop_off=0.4)
    s_bad = rg.score_route(bad, ref)
    assert s_bad.total < 25.0
    assert s_bad.points['speed_rmse_straight'] < s_ref.points['speed_rmse_straight']
    # a slow turn costs only the turn's marks
    slow = rg.score_route(metrics(wobble=0.03, stop_off=0.05, turn_dip=0.15), ref)
    assert slow.points['speed_rmse_turn'] == 0.0
    assert slow.points['speed_rmse_straight'] == pytest.approx(15.0)
    assert rg.team_total([s_ref]) == (pytest.approx(45.0), 45.0)
    assert rg.team_total([]) == (0.0, 45.0)                       # no bag scores 0


def test_geometry_from_the_template():
    d = rg.template_dir('uturn')
    assert d and bm.ROUTES == ('uturn',)
    geo = rg.geometry_from_course(d, 'uturn')
    assert (geo.dir_x, geo.dir_y) == pytest.approx((1.0, 0.0))
    assert bm.classify_route(d) == 'uturn'
    assert bm.classify_route(rg.template_dir('uturn').replace('lab2_uturn', 'hall_demo')) is None


# ----------------------------------------------------------- the bag checker

def good_meta(route='uturn'):
    d = rg.template_dir(route)
    return bm.build_meta(d, 'A', '3', 'abc123')


def topics_all(rate=10.0, dur=40.0):
    t = {name: [100, 0.0, dur] for name in bm.REQUIRED_TOPICS + bm.OPTIONAL_TOPICS}
    t[bm.CAMERA_TOPIC] = [int(rate * dur) + 1, 0.0, dur]
    return t


def check(msgs=None, topics=None, meta='default', route='uturn', **kw):
    msgs = msgs if msgs is not None else msgs_for(route)
    return cb.check_messages('bag', topics if topics is not None else topics_all(),
                             msgs, good_meta(route) if meta == 'default' else meta, **kw)


def test_good_bag_is_accepted():
    v = check()
    assert v.ok, v.reasons
    assert v.route == 'uturn'


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
    src = rg.template_dir('uturn')
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
    ms = msgs_for()
    ms[g.CONTROL] = [(t, NS(longitudinal=NS(velocity=c.longitudinal.velocity * 1.5)))
                     for t, c in ms[g.CONTROL]]
    v = check(msgs=ms)
    assert any('commanded speed peaks' in r for r in v.reasons)


def test_laps_mrm_not_engaged_unfinished():
    v = check(msgs=msgs_for(laps=3))
    assert any('course set to 3 lap(s)' in r for r in v.reasons)
    v = check(msgs=msgs_for(mrm=True))
    assert any('MRM' in r for r in v.reasons)
    v = check(msgs=msgs_for(engaged=False))
    assert any('never engaged' in r for r in v.reasons)
    v = check(msgs=msgs_for(finished=False))
    assert any('did not finish' in r for r in v.reasons)


def test_checker_rejects_off_the_path():
    v = check(msgs=msgs_for(lateral=0.4))
    assert any('off the path' in r for r in v.reasons)
    assert check(msgs=msgs_for(lateral=0.1)).ok


def test_checker_rejects_the_old_routes():
    # a bag of the first draft's straight or circles is not a graded route
    for kind, name in (('straight', 'lab2_straight'), ('circle_cw', 'lab2_circle_cw')):
        m = good_meta()
        m['route_kind'], m['course_name'] = kind, name
        v = check(meta=m)
        assert v.route is None
        assert any('not graded' in r for r in v.reasons), v.reasons
    m = good_meta()
    m['route_kind'] = None          # an old recorder: the course name decides
    assert bm.route_from_meta(m) == 'uturn'


def test_no_metadata_and_missing_ids():
    v = check(meta=None)
    assert any('no metadata' in r for r in v.reasons)
    m = good_meta()
    m['car_id'] = ''
    assert any('car or team' in r for r in check(meta=m).reasons)
    assert any('expected' in r for r in check(expect_car='B').reasons)


def test_set_checks():
    a = cb.Verdict('a', 'uturn', 'A', '3')
    b = cb.Verdict('b', 'uturn', 'A', '3')
    c = cb.Verdict('c', None, 'B', '4')
    assert any('2 times' in p for p in cb.check_set([a, b]))
    assert any('different cars' in p for p in cb.check_set([a, c]))
    assert cb.check_set([a]) == []


def test_meta_from_topic_then_sidecar(tmp_path):
    import json
    meta = {'car_id': 'A'}
    assert bm.meta_from_messages({bm.META_TOPIC: [(0, NS(data=json.dumps(meta)))]}) == meta
    (tmp_path / bm.SIDECAR).write_text(json.dumps({'car_id': 'B'}))
    assert bm.meta_from_messages({}, str(tmp_path)) == {'car_id': 'B'}


# ----------------------------------------------------------- non-finite data, jumps, a cut bag

def test_nan_pose_is_invalid_and_rejected_not_a_crash():
    msgs = msgs_for()
    t, k = msgs[g.KINEMATIC][100]
    msgs[g.KINEMATIC][100] = (t, kin(math.nan, math.nan))
    m = rg.route_metrics('b', msgs, 'uturn', UTURN)
    assert m.bad_poses == 1
    s = rg.score_route(m)
    assert s.invalid and any('non-finite localized pose' in f for f in s.gate_failures)
    assert any('non-finite localized pose' in r for r in check(msgs).reasons)
    with pytest.raises(ValueError):
        UTURN.path.project(math.nan, 0.0)


def test_nan_speed_is_named():
    msgs = msgs_for()
    for i in (50, 51, 52):
        t, _ = msgs[g.VELOCITY][i]
        msgs[g.VELOCITY][i] = (t, NS(longitudinal_velocity=math.nan))
    m = rg.route_metrics('b', msgs, 'uturn', UTURN)
    assert m.bad_speeds == 3
    assert any('3 non-finite speed report' in f for f in rg.score_route(m).gate_failures)
    assert any('non-finite speed report' in r for r in check(msgs).reasons)


def test_clean_run_has_no_localization_jump():
    m = rg.route_metrics('b', msgs_for(), 'uturn', UTURN)
    assert m.loc_jump < 0.05
    assert not rg.score_route(m).invalid


def test_a_jump_along_the_path_is_caught():
    msgs = msgs_for()
    kin_ = []
    for t, k in msgs[g.KINEMATIC]:
        if 10.0 <= t <= 11.0:              # 1 m ahead along the out leg (driving -x) for 1 s
            p = k.pose.pose.position
            k = kin(p.x - 1.0, p.y)
        kin_.append((t, k))
    msgs[g.KINEMATIC] = kin_
    m = rg.route_metrics('b', msgs, 'uturn', UTURN)
    assert m.path_dev_max < rg.PATH_DEV_MAX            # across the path it looks fine
    assert m.loc_jump > 0.9
    assert any('localization jump' in f for f in rg.score_route(m).gate_failures)
    assert any('localization jump' in r for r in check(msgs).reasons)


def test_bag_cut_right_after_the_stop_is_rejected():
    msgs = msgs_for()
    t_rest = rg.route_metrics('b', msgs, 'uturn', UTURN).t_rest
    cut = {k: [(t, x) for t, x in v if t <= t_rest + 0.3] for k, v in msgs.items()}
    t = topics_all(dur=t_rest + 0.3)
    v = check(cut, topics=t)
    assert any('ends 0.3 s after the car came to rest' in r for r in v.reasons)
    assert check().ok
