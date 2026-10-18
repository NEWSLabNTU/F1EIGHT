"""Lab 2 Part 1 checks on synthetic traces: every metric passes on an ideal run and fails on a
run broken in exactly that way. No ROS."""
import json
import math

import pytest
import yaml

from autosdv_waypoint_lab.part1 import metrics as M

A = 2.5        # accel, and the stack's e-stop resume ramp
D = 2.5        # decel_brake: every braking segment (lead ruling)
DT = 0.01


def make_profile_dict(speeds=(2.0, 6.0, 11.0, 6.0, 11.0), holds=(3.0, 3.0, 3.0, 8.0, 3.0),
                      runout=10.0) -> dict:
    """The spec's course: accel/decel at 2.5 m/s^2 between the holds, then brake to the line."""
    segs, s, v, k = [], 0.0, 0.0, 0
    for vh, th in zip(speeds, holds):
        kind = 'accel' if vh > v else 'decel'
        a = A if kind == 'accel' else D
        ds = abs(vh * vh - v * v) / (2.0 * a)
        segs.append({'kind': kind, 's0': s, 's1': s + ds, 'v0': v, 'v1': vh})
        s += ds
        segs.append({'kind': 'hold', 's0': s, 's1': s + vh * th, 'v0': vh, 'v1': vh,
                     'hold_index': k})
        s += vh * th
        v, k = vh, k + 1
    ds = v * v / (2.0 * D)
    segs.append({'kind': 'brake', 's0': s, 's1': s + ds, 'v0': v, 'v1': 0.0})
    stop_line = s + ds
    segs.append({'kind': 'runout', 's0': stop_line, 's1': stop_line + runout,
                 'v0': 0.0, 'v1': 0.0})
    h3 = segs[7]
    return {'speeds': list(speeds), 'holds': list(holds), 'accel': A, 'decel_brake': D,
            'runout': runout, 'segments': segs, 'stop_line_s': stop_line,
            'estop_trigger_s': h3['s0'] + 15.0, 'estop_hold_index': 3}   # P3: hold_s0 + 15 m


PROFILE = M.profile_from_dict(make_profile_dict())


def v_path(p: M.Profile, s: float) -> float:
    """The path's speed at s (the planner's profile)."""
    for seg in p.segments:
        if s < seg.s1:
            if seg.kind == 'hold':
                return seg.v0
            if seg.kind == 'accel':
                return min(seg.v1, math.sqrt(seg.v0 ** 2 + 2.0 * A * (s - seg.s0)) + 0.05)
            if seg.kind == 'decel':
                return max(seg.v1, math.sqrt(max(0.0, seg.v0 ** 2 - 2.0 * D * (s - seg.s0))))
            if seg.kind == 'brake':
                return math.sqrt(max(0.0, 2.0 * D * (p.stop_line_s - s)))
            return 0.0
    return 0.0


def ideal_rows(p: M.Profile = PROFILE, estop: bool = True, stop_offset: float = 0.0,
               creep: float = 0.0):
    """A car that drives the profile perfectly (rate-limited to the planner's a and d).

    After the e-stop clears, the stack ramps the target at A (lead ruling). `creep`: the
    rc_sim neutral creep after the final stop (true speed, m/s).
    """
    rows, t, s, v = [], 0.0, 0.0, 0.0
    phase, rest_since, t_clear, t_stopped = 'run', None, None, None
    stop_line = p.stop_line_s + stop_offset
    while True:
        em = 0
        if phase == 'run' and estop and s >= p.estop_trigger_s and t > 1.0:
            phase = 'estop'
        if phase == 'estop':
            em, target = 1, 0.0
            if v <= 1e-9:
                rest_since = t if rest_since is None else rest_since
                if t - rest_since >= 1.0:
                    phase, em, t_clear = 'done', 0, t
        if phase != 'estop':
            target = 0.0 if t < 0.5 else v_path(p, s)
            if t_clear is not None:
                target = min(target, A * (t - t_clear) + 0.05)
                if target >= v_path(p, s):
                    t_clear = None
            if s >= p.first('brake').s0:
                target = math.sqrt(max(0.0, 2.0 * D * (stop_line - s)))
        dv = max(-D * DT, min(A * DT, target - v))
        v = max(0.0, v + dv)
        if s >= p.first('brake').s0 and stop_line - s < 0.002:
            v = 0.0
        if t_stopped is None and s >= p.first('brake').s0 and v == 0.0:
            t_stopped = t
        if t_stopped is not None:
            v = creep
        s += v * DT
        rows.append({'t': round(t, 2), 's': s, 'y': 0.0, 'target': target, 'measured': v,
                     'speed': v, 'pwm': 370, 'mode': 'active', 'slip_r': 0.0, 'kappa_r': 0.0,
                     'emergency': em, 'segment': p.segment_at(s)})
        t += DT
        if t_stopped is not None and t - t_stopped >= 2.6:
            return rows


@pytest.fixture(scope='module')
def ideal():
    return ideal_rows()


def run(rows, finished=True):
    return M.evaluate(M.Trace(rows), PROFILE, finished=finished)


def edit(rows, where, **fn):
    """A copy of rows with columns changed where `where(row)`; fn maps column -> f(row)."""
    out = []
    for r in rows:
        r = dict(r)
        if where(r):
            for col, f in fn.items():
                r[col] = f(r)
        out.append(r)
    return out


def in_hold(k):
    h = PROFILE.hold(k)
    return lambda r: h.s0 <= r['s'] < h.s1


def first_t(rows, pred):
    return next(r['t'] for r in rows if pred(r))


# ---------------------------------------------------------------- the ideal run

def test_ideal_run_passes_all_fifteen(ideal):
    checks = run(ideal)
    assert list(checks) == list(M.CHECK_NAMES) and len(checks) == 16
    failed = {n: c for n, c in checks.items() if not c['pass']}
    assert not failed, failed
    assert M.summary(checks, True) == (16, 16)
    assert M.violations(checks) == []
    assert checks['stop_error']['value'] < 0.05
    assert checks['hold_rmse_11b']['value'] < 1e-6


def test_profile_reading():
    p = PROFILE
    assert p.first_up_holds == (0, 1, 2)
    assert p.hold_11b_index == 4
    assert p.hold(3).v0 == 6.0 and p.before(p.hold(3)).kind == 'decel'
    assert p.segment_at(-1.0) == 'start'
    assert p.segment_at(p.hold(0).s0 + 0.1) == '1:hold'


def test_profile_file_roundtrip(tmp_path):
    f = tmp_path / 'part1_profile.yaml'
    f.write_text(yaml.safe_dump(make_profile_dict()))
    assert M.load_profile(f) == PROFILE


def test_trace_csv_roundtrip(ideal, tmp_path):
    tr = M.Trace(ideal)
    tr.write_csv(tmp_path / 'trace.csv')
    back = M.Trace.read_csv(tmp_path / 'trace.csv')
    assert len(back) == len(tr)
    assert back['mode'][5] == 'active' and back['emergency'] == tr['emergency']
    a, b = M.evaluate(tr, PROFILE), M.evaluate(back, PROFILE)
    for n in M.CHECK_NAMES:
        assert a[n]['pass'] == b[n]['pass']
        if a[n]['value'] is not None:
            assert b[n]['value'] == pytest.approx(a[n]['value'], abs=1e-6)


# ---------------------------------------------------------------- one failure per check

def fails_only(rows, name):
    checks = run(rows)
    failed = sorted(n for n, c in checks.items() if not c['pass'])
    assert failed == [name], {n: checks[n] for n in failed}
    v = M.violations(checks)
    assert len(v) == 1 and v[0]['name'] == name and v[0]['text'].startswith(name)
    assert M.summary(checks, True) == (15, 16)
    return checks[name]


@pytest.mark.parametrize('k,name', [(0, 'hold_rmse_2'), (1, 'hold_rmse_6'), (2, 'hold_rmse_11'),
                                    (4, 'hold_rmse_11b')])
def test_hold_rmse_fails_on_oscillation(ideal, k, name):
    amp = 2.0 * M.LIMITS[name]
    t_in = first_t(ideal, in_hold(k)) + M.HOLD_SETTLE
    # below the hold speed only, after the settling second: no overshoot, no late reach
    rows = edit(ideal, lambda r: in_hold(k)(r) and r['t'] >= t_in,
                speed=lambda r: r['speed'] - amp * abs(math.sin(9.0 * r['t'])))
    c = fails_only(rows, name)
    assert c['value'] == pytest.approx(amp / math.sqrt(2.0), rel=0.15)
    h = PROFILE.hold(k)
    assert h.s0 <= c['s'] < h.s1


def test_hold_rmse_ignores_the_settling_second(ideal):
    t_in = first_t(ideal, in_hold(1))
    rows = edit(ideal, lambda r: t_in <= r['t'] < t_in + 0.9, speed=lambda r: r['speed'] + 0.2)
    assert run(rows)['hold_rmse_6']['pass']


def test_overshoot_fails_on_a_bump_in_the_top_hold(ideal):
    t_in = first_t(ideal, in_hold(2))
    rows = edit(ideal, lambda r: t_in <= r['t'] < t_in + 0.5, speed=lambda r: r['speed'] + 1.0)
    c = fails_only(rows, 'overshoot')
    assert c['value'] == pytest.approx(100.0 * 1.0 / 5.0, rel=1e-6)


def test_reach_fails_on_a_slow_step(ideal):
    h = PROFILE.hold(1)
    t_in = first_t(ideal, in_hold(1))
    level = h.v0 - 0.1 * 4.0
    # late by 0.8 s: past the reach limit, still inside the hold's settling second
    rows = edit(ideal, lambda r: PROFILE.hold(0).s1 <= r['s'] and r['t'] < t_in + 0.8,
                speed=lambda r: min(r['speed'], level - 0.05))
    c = fails_only(rows, 'reach_s')
    assert c['value'] == pytest.approx(0.8, abs=0.02)


def test_launch_slip_fails_on_wheel_spin(ideal):
    t0 = first_t(ideal, lambda r: r['speed'] > M.LAUNCH_MOVE)
    rows = edit(ideal, lambda r: t0 + 0.2 <= r['t'] < t0 + 0.4, slip_r=lambda r: 0.8)
    c = fails_only(rows, 'launch_slip')
    assert c['value'] == 0.8


def test_launch_slip_counts_a_wheel_spinning_at_rest(ideal):
    # the car has not moved yet: the window starts at the spin, not at the car's motion
    t0 = first_t(ideal, lambda r: r['speed'] > M.LAUNCH_MOVE)
    rows = edit(ideal, lambda r: t0 - 0.3 <= r['t'] < t0 - 0.1, slip_r=lambda r: 0.9)
    assert not run(rows)['launch_slip']['pass']


def test_launch_accel_fails_on_a_lurch(ideal):
    t0 = first_t(ideal, lambda r: r['speed'] > M.LAUNCH_MOVE)
    rows = edit(ideal, lambda r: t0 + 0.3 <= r['t'], speed=lambda r: r['speed'] + 0.4)
    # undo the offset past the launch window so only the lurch is new
    rows = edit(rows, lambda r: r['t'] >= t0 + 0.6,
                speed=lambda r: r['speed'] - 0.4)
    checks = run(rows)
    assert not checks['launch_accel']['pass']
    assert checks['launch_accel']['value'] > 7.0


def test_stepdown_undershoot_fails_on_a_dip(ideal):
    t_in = first_t(ideal, in_hold(3))
    rows = edit(ideal, lambda r: t_in <= r['t'] < t_in + 0.5, speed=lambda r: 5.0)
    c = fails_only(rows, 'stepdown_undershoot')
    assert c['value'] == pytest.approx(20.0)


def test_stepdown_ignores_the_estop_stop(ideal):
    # the e-stop takes the car to 0 in the same hold; that is not an undershoot
    checks = run(ideal)
    assert checks['stepdown_undershoot']['value'] == pytest.approx(0.0)


def test_stepdown_kappa_fails_on_rear_lock(ideal):
    dec = PROFILE.before(PROFILE.hold(3))
    rows = edit(ideal, lambda r: dec.s0 <= r['s'] < dec.s0 + 2.0, kappa_r=lambda r: -0.5)
    fails_only(rows, 'stepdown_kappa')


def test_estop_reverse_fails_on_reversing(ideal):
    # the car at rest under the e-stop rolls backwards
    rows = edit(ideal, lambda r: r['emergency'] and r['speed'] == 0.0, speed=lambda r: -0.2)
    c = fails_only(rows, 'estop_reverse')
    assert c['value'] == -0.2


def test_estop_resume_slip_fails_on_spin_after_the_clear(ideal):
    t_off = next(r['t'] for i, r in enumerate(ideal)
                 if i and ideal[i - 1]['emergency'] and not r['emergency'])
    rows = edit(ideal, lambda r: t_off + 0.2 <= r['t'] < t_off + 0.5, slip_r=lambda r: 0.8)
    fails_only(rows, 'estop_resume_slip')


def test_estop_resume_overshoot_fails_on_a_bump(ideal):
    t_off = next(r['t'] for i, r in enumerate(ideal)
                 if i and ideal[i - 1]['emergency'] and not r['emergency'])
    h3 = PROFILE.hold(3)
    rows = edit(ideal, lambda r: r['t'] > t_off and r['s'] < h3.s1 and r['speed'] > 5.9,
                speed=lambda r: 7.0)
    c = fails_only(rows, 'estop_resume_overshoot')
    assert c['value'] == pytest.approx(100.0 / 6.0)


def test_estop_checks_fail_when_no_estop_happened():
    checks = run(ideal_rows(estop=False))
    for n in ('estop_reverse', 'estop_resume_slip', 'estop_resume_overshoot'):
        assert checks[n]['pass'] is False and checks[n]['value'] is None


def test_stop_error_fails_past_the_line():
    rows = ideal_rows(stop_offset=0.8)
    c = fails_only(rows, 'stop_error')
    assert c['value'] == pytest.approx(0.8, abs=0.05)


def test_stop_error_fails_short_of_the_line():
    c = fails_only(ideal_rows(stop_offset=-0.7), 'stop_error')
    assert c['value'] == pytest.approx(0.7, abs=0.05)


def test_stop_error_none_when_not_at_rest(ideal):
    rows = edit(ideal, lambda r: r is ideal[-1] or r['t'] == ideal[-1]['t'],
                speed=lambda r: 0.5)
    assert run(rows)['stop_error']['value'] is None
    assert not run(rows)['stop_error']['pass']


def test_stop_kappa_fails_on_lock_braking(ideal):
    b = PROFILE.first('brake')
    rows = edit(ideal, lambda r: b.s0 + 1.0 <= r['s'] < b.s0 + 3.0, kappa_r=lambda r: -0.4)
    fails_only(rows, 'stop_kappa')


def test_standstill_kappa_artifact_is_not_a_lock(ideal):
    # the slip ratio reads < -0.2 below ~0.1 m/s at every stop (rc_sim, P2): not counted
    rows = edit(ideal, lambda r: r['speed'] < 0.1, kappa_r=lambda r: -0.9)
    checks = run(rows)
    assert checks['stop_kappa']['pass'] and checks['stepdown_kappa']['pass']
    assert all(c['pass'] for c in checks.values())


def test_lock_just_above_the_speed_gate_counts(ideal):
    b = PROFILE.first('brake')
    rows = edit(ideal, lambda r: r['s'] >= b.s0 and 0.35 < r['speed'] < 0.6,
                kappa_r=lambda r: -0.9)
    assert not run(rows)['stop_kappa']['pass']


# ---------------------------------------------------------------- live windows

def test_live_checks_open_as_segments_complete(ideal):
    h1 = PROFILE.hold(1)
    part = [r for r in ideal if r['s'] < h1.s1 + 0.5]
    checks = run(part, finished=False)
    closed = {n for n, c in checks.items() if c['pass'] is not None}
    assert closed == {'hold_rmse_2', 'hold_rmse_6', 'launch_slip', 'launch_accel'}
    assert all(c['value'] is None for n, c in checks.items() if n not in closed)
    assert not M.is_finished(M.Trace(part), PROFILE)


def test_live_estop_checks_close_after_the_resume(ideal):
    h3 = PROFILE.hold(3)
    on = M.Trace(ideal)
    i_on, i_off = M.estop_times(on)
    during = ideal[:i_on + 50]
    c = run(during, finished=False)
    assert c['stepdown_undershoot']['pass'] is True
    assert c['estop_reverse']['pass'] is None
    after = [r for i, r in enumerate(ideal) if i < i_off or r['s'] < h3.s1 + 0.2]
    c = run(after, finished=False)
    assert c['estop_resume_overshoot']['pass'] is True
    assert c['stop_error']['pass'] is None and c['hold_rmse_11b']['pass'] is None


def test_finish_detection(ideal):
    assert M.is_finished(M.Trace(ideal), PROFILE)
    # at rest during the e-stop is not the finish
    _, i_off = M.estop_times(M.Trace(ideal))
    assert not M.is_finished(M.Trace(ideal[:i_off]), PROFILE)
    # one second at rest at the line is not enough
    b = PROFILE.first('brake')
    i_rest = next(i for i, r in enumerate(ideal) if r['s'] >= b.s0 and r['speed'] == 0.0)
    assert not M.is_finished(M.Trace(ideal[:i_rest + 100]), PROFILE)
    assert M.is_finished(M.Trace(ideal[:i_rest + 201]), PROFILE)


def test_empty_trace():
    checks = M.evaluate(M.Trace(), PROFILE, finished=False)
    assert all(c['pass'] is None for c in checks.values())
    checks = M.evaluate(M.Trace(), PROFILE, finished=True)
    assert M.summary(checks, True) == (0, 16)


# ---------------------------------------------------------------- validity and limits

def test_validity_reasons():
    assert M.invalid_reason() is None
    r = M.invalid_reason(mrm='EMERGENCY_STOP', fault='update() raised')
    assert r.startswith('controller fault')
    assert M.invalid_reason(lockstep='actuator timed out').startswith('lockstep')
    assert 'aborted' in M.invalid_reason(aborted='not engaged')
    assert M.invalid_reason(estop_missing='x').startswith('e-stop not raised')
    assert M.invalid_reason(not_engaged='x').startswith('the stack did not engage')
    assert M.invalid_reason(actuator_missing='x').startswith('the actuator is not running')


def test_actuator_missing_after_the_grace():
    assert M.actuator_missing(False, 10.0, None) is None             # not engaged yet
    assert M.actuator_missing(False, 10.0 + M.ACTUATOR_GRACE, 10.0) is None
    assert M.actuator_missing(False, 10.1 + M.ACTUATOR_GRACE, 10.0) is not None
    assert M.actuator_missing(True, 100.0, 10.0) is None


def test_invalid_run_scores_zero(ideal):
    checks = run(ideal)
    assert M.summary(checks, valid=False) == (0, 16)


def test_limits_table_is_complete():
    assert set(M.LIMITS) == set(M.CHECK_NAMES)
    assert set(M.FIXED) | set(M.CALIBRATED) == set(M.CHECK_NAMES)
    assert set(M.LOWER_BOUND) <= set(M.FIXED)
    assert M.LIMITS_CALIBRATED is True


def test_fixed_limits_unchanged():
    assert M.LIMITS['launch_slip'] == 0.5 and M.LIMITS['estop_resume_slip'] == 0.5
    assert M.LIMITS['stepdown_kappa'] == M.LIMITS['stop_kappa'] == -0.2
    assert M.LIMITS['estop_reverse'] == -0.05 and M.LIMITS['stop_error'] == 0.5


def test_nonfinite_value_fails():
    assert not M.passes('reach_s', float('nan'), 1.0)
    assert not M.passes('estop_reverse', None, -0.05)
    assert M.passes('estop_reverse', 0.0, -0.05)


# ---------------------------------------------------------------- the grader node, fed directly

def _course(tmp_path):
    d = tmp_path / 'course'
    d.mkdir()
    prof = make_profile_dict()
    (d / 'part1_profile.yaml').write_text(yaml.safe_dump(prof))
    end = prof['segments'][-1]['s1'] + 5.0
    n = int(end / 0.5) + 1
    rows = ''.join(f'{i * 0.5:.3f},0.000,1.0\n' for i in range(n))
    (d / 'path.csv').write_text('# x, y, v\n' + rows)
    (d / 'course.yaml').write_text('path: path.csv\nstart_pose: {x: 0.0, y: 0.0, yaw: 0.0}\n')
    return d


def _grade_live(tmp_path, rows, mode_at=None, t_engage=0.0, params=(), drop=None,
                actuator=True):
    rclpy = pytest.importorskip('rclpy')
    import os
    from std_msgs.msg import Float64MultiArray, String
    from autoware_control_msgs.msg import Control
    os.environ.setdefault('ROS_DOMAIN_ID', '83')
    from autosdv_waypoint_lab.part1.grader_node import Part1Grader
    from rclpy.parameter import Parameter
    d = _course(tmp_path)
    out = tmp_path / 'out'
    rclpy.init()
    try:
        node = Part1Grader(parameter_overrides=[
            Parameter('profile_file', value=str(d / 'part1_profile.yaml')),
            Parameter('out_dir', value=str(out)), Parameter('variant', value='nominal'),
            Parameter('wheel_radius', value=0.05)]
            + [Parameter(k, value=v) for k, v in params])
        if actuator:
            node.on_mode(String(data='waiting'))   # the actuator's latched first mode
        engaged = False
        em = 0
        for i, r in enumerate(rows):
            if not engaged and t_engage is not None and r['t'] >= t_engage:
                node.on_event(String(data=json.dumps({'event': 'engaged', 't': r['t']})))
                engaged = True
            if drop is not None and drop[0] <= r['t'] < drop[1]:
                continue
            if r['emergency'] != em:
                em = r['emergency']
                ev = 'estop_on' if em else 'estop_off'
                node.on_event(String(data=json.dumps({'event': ev, 't': r['t']})))
            if mode_at is not None and r['t'] >= mode_at[0]:
                node.on_mode(String(data=mode_at[1]))
            cmd = Control()
            cmd.longitudinal.velocity = float(r['target'])
            node.on_cmd(cmd)
            st = [r['t'], r['s'], 0.0, 0.0, r['speed'], 0.0, 0.0, 0.0, r['speed'] / 0.05,
                  0.0, r['kappa_r'], 0.0, 0.0]
            node.on_state(Float64MultiArray(data=st))
            if i % 10 == 0:
                node.tick()
            if node.final is not None:
                break
        node.tick()
        status = json.loads(node.last_status)
        node.destroy_node()
        return out, status
    finally:
        rclpy.shutdown()


def test_grader_node_grades_an_ideal_run(tmp_path, ideal):
    out, status = _grade_live(tmp_path, ideal)
    marks = json.loads((out / 'marks.json').read_text())
    assert marks['valid'] and marks['passed'] == 16 and marks['total'] == 16, marks
    assert set(marks) >= {'variant', 'valid', 'invalid_reason', 'passed', 'total', 'checks'}
    assert status['state'] == 'finished' and status['marks'] == {'passed': 16, 'total': 16}
    back = M.Trace.read_csv(out / 'trace.csv')
    assert len(back) > 3000 and back['t'][1] - back['t'][0] == pytest.approx(0.01)
    assert sum(back['emergency']) > 100


def test_grader_node_invalid_without_the_actuator(tmp_path, ideal):
    # an actuator that crashed at start-up: the car stands still, which must not grade
    # as a valid "did not move" run
    still = [dict(r, s=0.0, speed=0.0, kappa_r=0.0) for r in ideal]
    out, status = _grade_live(tmp_path, still, actuator=False)
    marks = json.loads((out / 'marks.json').read_text())
    assert not marks['valid'] and marks['passed'] == 0
    assert marks['invalid_reason'].startswith('the actuator is not running')
    assert status['state'] == 'invalid'


def test_grader_node_invalid_on_controller_fault(tmp_path, ideal):
    out, status = _grade_live(tmp_path, ideal,
                              mode_at=(10.0, 'override_brake:controller_fault'))
    marks = json.loads((out / 'marks.json').read_text())
    assert not marks['valid'] and marks['passed'] == 0
    assert marks['invalid_reason'].startswith('controller fault')
    assert status['state'] == 'invalid'


def test_path_frame_projection():
    pytest.importorskip('rclpy')
    from autosdv_waypoint_lab.part1.grader_node import PathFrame
    xs = [0.1 * i for i in range(2001)]
    f = PathFrame(xs, [0.0] * len(xs))
    assert f.project(50.0, 0.3) == pytest.approx((50.0, 0.3))
    assert f.project(150.0, -0.2) == pytest.approx((150.0, -0.2))   # a jump past the window
    assert f.project(205.0, 0.0)[0] == pytest.approx(205.0)          # beyond the path's end
    f.s0 = 1.0
    assert f.project(10.0, 0.0)[0] == pytest.approx(9.0)
    assert f.point_at(9.0)[0] == pytest.approx(10.0)


def test_resume_window_follows_the_ramped_target(ideal):
    tr = M.Trace(ideal)
    _, off = M.estop_times(tr)
    end = M._resume_end(tr, PROFILE, off)
    t_reach = next(r['t'] for r in ideal[off:] if r['target'] >= 6.0 - M.RESUME_TARGET_TOL)
    assert t_reach - ideal[off]['t'] == pytest.approx(6.0 / A, abs=0.1)   # the 2.5 m/s^2 ramp
    assert ideal[end]['t'] == pytest.approx(t_reach + M.RESUME_SETTLE, abs=0.02)
    assert ideal[end]['s'] < PROFILE.hold(3).s1   # the 6 s hold leaves room after the resume


def test_a_bump_after_the_resume_window_is_not_a_resume_overshoot(ideal):
    tr = M.Trace(ideal)
    _, off = M.estop_times(tr)
    t_end = ideal[M._resume_end(tr, PROFILE, off)]['t']
    h3 = PROFILE.hold(3)
    rows = edit(ideal, lambda r: r['t'] > t_end + 0.1 and r['s'] < h3.s1, speed=lambda r: 7.0)
    assert run(rows)['estop_resume_overshoot']['pass']


def test_creep_below_the_rest_threshold_finishes():
    rows = ideal_rows(creep=0.03)
    tr = M.Trace(rows)
    assert M.is_finished(tr, PROFILE)
    c = run(rows)
    assert c['stop_error']['pass'] and c['stop_error']['value'] < 0.05   # where the rest began


def test_creep_above_the_rest_threshold_never_finishes():
    rows = ideal_rows(creep=0.1)
    assert not M.is_finished(M.Trace(rows), PROFILE)
    assert run(rows)['stop_error']['value'] is None


def test_braking_rate_comes_from_the_profile():
    dec = PROFILE.before(PROFILE.hold(3))
    assert dec.kind == 'decel'
    assert (dec.v0 ** 2 - dec.v1 ** 2) / (2 * (dec.s1 - dec.s0)) == pytest.approx(D)
    b = PROFILE.first('brake')
    assert b.v0 ** 2 / (2 * (b.s1 - b.s0)) == pytest.approx(D)


# ---------------------------------------------------------------- fix round 1

def test_stepdown_window_ends_at_the_trigger():
    rows = ideal_rows(estop=False)
    trig = PROFILE.estop_trigger_s
    assert trig == pytest.approx(PROFILE.hold(3).s0 + 15.0)
    after = edit(rows, lambda r: trig + 1.0 <= r['s'] < trig + 4.0, speed=lambda r: 5.0)
    assert run(after)['stepdown_undershoot']['value'] == pytest.approx(0.0)
    before = edit(rows, lambda r: trig - 4.0 <= r['s'] < trig - 1.0, speed=lambda r: 5.0)
    assert run(before)['stepdown_undershoot']['value'] == pytest.approx(20.0)


@pytest.mark.parametrize('col,where,name', [
    ('speed', lambda r: 0.6 < r['t'] < 0.9, 'launch_accel'),
    ('slip_r', lambda r: 0.6 < r['t'] < 0.9, 'launch_slip'),
    ('speed', lambda r: r['s'] >= 0.5 * (PROFILE.hold(2).s0 + PROFILE.hold(2).s1),
     'hold_rmse_11'),
    ('kappa_r', lambda r: r['s'] >= PROFILE.first('brake').s0 + 1.0
     and r['speed'] > 2.0, 'stop_kappa'),
])
def test_nan_in_a_window_fails(ideal, col, where, name):
    hit = [False]

    def nan_once(r):
        if hit[0]:
            return r[col]
        hit[0] = True
        return float('nan')

    rows = edit(ideal, where, **{col: nan_once})
    c = run(rows)[name]
    assert c['value'] is None and c['pass'] is False


def test_unclosed_checks_fail_or_stay_pending(ideal):
    part = [r for r in ideal if r['s'] < PROFILE.hold(1).s1 + 0.5]
    tr = M.Trace(part)
    pend = M.evaluate(tr, PROFILE, finished=False, unclosed='pending')
    fail = M.evaluate(tr, PROFILE, finished=False, unclosed='fail')
    assert pend['stop_error']['pass'] is None and fail['stop_error']['pass'] is False
    assert fail['stop_error']['value'] is None
    assert fail['hold_rmse_2']['pass'] is True     # closed windows keep their verdict
    # a partly driven hold must not show as passed
    assert pend['hold_rmse_11']['pass'] is None


def _stub_rows(seconds=15.0, target=2.0):
    return [{'t': round(0.01 * i, 2), 's': 0.0, 'y': 0.0, 'target': target if i > 50 else 0.0,
             'measured': 0.0, 'speed': 0.0, 'pwm': 370, 'mode': 'active', 'slip_r': 0.0,
             'kappa_r': 0.0, 'emergency': 0, 'segment': 'start'}
            for i in range(int(seconds / 0.01))]


def test_stall_did_not_move():
    msg = M.stall_message(M.Trace(_stub_rows()), 0.0)
    assert msg and msg.startswith('Your car did not move') and 'stub' in msg
    assert M.stall_message(M.Trace(_stub_rows(seconds=9.0)), 0.0) is None


def test_stall_stopped_mid_run(ideal):
    h1 = PROFILE.hold(1)
    i = next(k for k, r in enumerate(ideal) if r['s'] >= h1.s0 + 2.0)
    rows = [dict(r) for r in ideal[:i]]
    last = rows[-1]
    for k in range(1, 1101):
        rows.append(dict(last, t=round(last['t'] + 0.01 * k, 2), speed=0.0, target=6.0))
    msg = M.stall_message(M.Trace(rows), 0.0)
    assert msg and msg.startswith('Your car stopped at s =')


def test_no_stall_on_a_good_run_or_under_the_estop(ideal):
    tr = M.Trace(ideal)
    for n in range(500, len(ideal), 500):
        assert M.stall_message(M.Trace(ideal[:n]), 0.0) is None
    assert M.stall_message(tr, 0.0) is None


def test_estop_missing():
    assert M.estop_missing(M.Trace(ideal_rows(estop=False)), PROFILE)
    assert M.estop_missing(M.Trace(ideal_rows()), PROFILE) is None


def test_grader_node_stub_is_a_valid_failing_run(tmp_path):
    out, status = _grade_live(tmp_path, _stub_rows(seconds=20.0))
    marks = json.loads((out / 'marks.json').read_text())
    assert marks['valid'] is True and marks['invalid_reason'] is None
    assert 'did not move' in marks['message'] and 'stub' in marks['message']
    assert marks['passed'] < 16
    assert all(c['pass'] is False for n, c in marks['checks'].items()
               if c['value'] is None)
    assert status['state'] == 'finished' and status['message'] == marks['message']


def test_grader_node_timeout_counts_from_the_engage(tmp_path, ideal):
    boot = 200.0    # longer than run_timeout (150 s): must not count
    rest = [dict(ideal[0], t=round(0.01 * i, 2)) for i in range(int(boot / 0.01))]
    shifted = [dict(r, t=round(r['t'] + boot, 2)) for r in ideal]
    out, status = _grade_live(tmp_path, rest + shifted, t_engage=boot)
    marks = json.loads((out / 'marks.json').read_text())
    assert marks['valid'] and marks['passed'] == 16, marks['message']


def test_grader_node_never_engaged_is_invalid(tmp_path):
    rows = _stub_rows(seconds=12.0, target=0.0)
    out, status = _grade_live(tmp_path, rows, t_engage=None,
                              params=[('engage_timeout', 10.0)])
    marks = json.loads((out / 'marks.json').read_text())
    assert not marks['valid'] and marks['invalid_reason'].startswith('the stack did not engage')
    assert all(c['pass'] is not True for c in marks['checks'].values())


def test_grader_node_invalid_run_shows_no_false_passes(tmp_path, ideal):
    out, _ = _grade_live(tmp_path, ideal, mode_at=(6.0, 'override_brake:controller_fault'))
    marks = json.loads((out / 'marks.json').read_text())
    assert not marks['valid']
    assert marks['checks']['hold_rmse_11b']['pass'] is None     # never reached
    assert marks['checks']['stop_error']['pass'] is None


def test_grader_node_notes_state_gaps(tmp_path, ideal):
    out, _ = _grade_live(tmp_path, ideal, drop=(5.0, 5.5))
    marks = json.loads((out / 'marks.json').read_text())
    assert any('gap' in n for n in marks['notes']), marks['notes']


def test_stepdown_lag_fails_a_car_that_cannot_slow_down(ideal):
    dec = PROFILE.before(PROFILE.hold(3))
    rows = edit(ideal, lambda r: dec.s0 + 2.0 <= r['s'] < dec.s1,
                speed=lambda r: r['target'] + 1.5)
    c = fails_only(rows, 'stepdown_lag')
    assert c['value'] == pytest.approx(1.5)
    assert dec.s0 <= c['s'] < dec.s1


def test_stepdown_lag_ignores_the_estop(ideal):
    # the e-stop drops the target to 0 while the car is at 6 m/s: not a step-down lag
    assert run(ideal)['stepdown_lag']['value'] < 0.1


def test_lockstep_stall_before_engage_is_a_startup_fault(tmp_path, ideal):
    rclpy = pytest.importorskip('rclpy')
    import os
    from std_msgs.msg import Float64MultiArray, String
    os.environ.setdefault('ROS_DOMAIN_ID', '83')
    from autosdv_waypoint_lab.part1.grader_node import Part1Grader
    from rclpy.parameter import Parameter
    d = _course(tmp_path)
    out = tmp_path / 'out'
    rclpy.init()
    try:
        node = Part1Grader(parameter_overrides=[
            Parameter('profile_file', value=str(d / 'part1_profile.yaml')),
            Parameter('out_dir', value=str(out))])
        node.on_state(Float64MultiArray(data=[0.0] * 13))
        node.on_lockstep(String(data=json.dumps({'ok': False, 'reason': 'actuator timed out'})))
        node.tick()
        marks = json.loads((out / 'marks.json').read_text())
        assert not marks['valid'] and marks['passed'] == 0
        assert marks['invalid_reason'].startswith('the simulator stalled before the run started')
        assert 'run again' in marks['invalid_reason'] and 'actuator timed out' in marks['message']
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert M.invalid_reason(lockstep='x').startswith('lockstep invalidated')   # after engage
