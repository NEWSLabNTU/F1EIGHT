"""The Lab 2 Part 1 course (part1/course.py): the profile contract, and the committed course
is what the generator writes today."""
import math
from pathlib import Path

import pytest
import yaml

from autosdv_waypoint_lab.part1 import course as C

COMMITTED = Path(__file__).resolve().parents[1] / 'config' / 'courses' / C.COURSE_NAME


@pytest.fixture(scope='module')
def prof():
    return C.make_profile()


def test_segments_are_contiguous_and_kinematic(prof):
    segs = prof['segments']
    assert segs[0]['s0'] == 0.0
    for a, b in zip(segs, segs[1:]):
        assert b['s0'] == pytest.approx(a['s1'], abs=1e-3)
        assert b['v0'] == pytest.approx(a['v1'])
    for seg in segs:
        if seg['kind'] in ('accel', 'decel', 'brake'):
            length = abs(seg['v1'] ** 2 - seg['v0'] ** 2) / (2.0 * seg['rate'])
            assert seg['s1'] - seg['s0'] == pytest.approx(length, abs=1e-3)
        if seg['kind'] == 'hold':
            assert seg['s1'] - seg['s0'] == pytest.approx(seg['v0'] * prof['holds'][seg['hold_index']])


def test_rates_holds_and_speeds(prof):
    assert prof['speeds'] == list(C.DEFAULT_SPEEDS)
    assert prof['holds'] == list(C.DEFAULT_HOLDS)
    holds = [s for s in prof['segments'] if s['kind'] == 'hold']
    assert [h['hold_index'] for h in holds] == list(range(len(prof['speeds'])))
    assert [h['v0'] for h in holds] == prof['speeds']
    for seg in prof['segments']:
        if seg['kind'] == 'accel':
            assert seg['rate'] == prof['accel']
        if seg['kind'] in ('decel', 'brake'):
            assert seg['rate'] == prof['decel_brake']


def test_the_default_course(prof):
    assert C.DEFAULT_SPEEDS == (4.0, 11.0) and C.DEFAULT_HOLDS == (4.0, 4.0)
    kinds = [(s['kind'], s['v0'], s['v1']) for s in prof['segments']]
    assert kinds == [('accel', 0.0, 4.0), ('hold', 4.0, 4.0), ('accel', 4.0, 11.0),
                     ('hold', 11.0, 11.0), ('brake', 11.0, 0.0), ('runout', 0.0, 0.0)]
    assert prof['nominal_duration'] == pytest.approx(16.8)
    assert prof['stop_line_s'] == pytest.approx(108.4)


def test_no_estop_by_default(prof):
    assert 'estop' not in prof and 'estop_trigger_s' not in prof
    assert 'estop_hold_index' not in prof


def test_an_estop_can_still_be_asked_for():
    p = C.make_profile(estop_hold_index=1, estop_offset=10.0)
    h = [s for s in p['segments'] if s['kind'] == 'hold'][1]
    assert p['estop_trigger_s'] == pytest.approx(h['s0'] + 10.0)
    e = p['estop']
    assert (e['hold_s0'], e['hold_s1'], e['trigger_s']) == (h['s0'], h['s1'], p['estop_trigger_s'])


def test_stop_line(prof):
    assert prof['stop_line_s'] == prof['segments'][-2]['s1']
    assert prof['segments'][-2]['kind'] == 'brake' and prof['segments'][-1]['kind'] == 'runout'
    assert prof['length'] == pytest.approx(prof['stop_line_s'] + prof['runout'])


def test_path_speed_follows_the_profile(prof):
    rows = C.path_rows(prof)
    assert rows[0][2] == C.ENGAGE_VELOCITY            # never 0 under a car at rest
    for seg in prof['segments']:
        vs = [v for s, _, v in rows if seg['s0'] + 1e-6 < s < seg['s1'] - 1e-6]
        steps = [b - a for a, b in zip(vs, vs[1:])]
        if seg['kind'] == 'accel':
            assert all(d >= -1e-9 for d in steps)
        elif seg['kind'] in ('decel', 'brake'):
            assert all(d <= 1e-9 for d in steps)
        elif seg['kind'] == 'hold':
            assert all(v == seg['v0'] for v in vs)
        else:
            assert all(v == 0.0 for v in vs)
    assert max(v for *_, v in rows) == max(prof['speeds'])
    assert all(math.isfinite(v) and v >= 0.0 for *_, v in rows)


def test_bad_estop_offset_is_refused():
    with pytest.raises(C.CourseError):
        C.make_profile(estop_hold_index=1, estop_offset=1000.0)


def test_committed_course_is_the_generators_output(tmp_path):
    pytest.importorskip('autosdv_rc_sim.track')       # the lanelet writer
    C.write_course(C.make_profile(), tmp_path)
    for f in sorted(tmp_path.iterdir()):
        committed = COMMITTED / f.name
        assert committed.is_file(), f'{committed} missing: run `just lab2 make-part1-course`'
        assert committed.read_text() == f.read_text(), (
            f'{committed} is stale: run `just lab2 make-part1-course` and commit the result')


def test_committed_profile_parses():
    data = yaml.safe_load((COMMITTED / 'part1_profile.yaml').read_text())
    assert data['speeds'] == [4.0, 11.0] and data['decel_brake'] > 0 and 'estop' not in data
