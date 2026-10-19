"""A bag of a non-graded route (the straight test routes): metrics only, never a crash. No ROS."""
import json
import os
import shutil
from types import SimpleNamespace as NS

import yaml

from autosdv_waypoint_lab import audit as au
from autosdv_waypoint_lab import bag_meta as bm
from autosdv_waypoint_lab import check_bags as cb
from autosdv_waypoint_lab import grading as g
from autosdv_waypoint_lab import route_grade as rg

from test_route_grade import msgs_for, plateau_end, topics_all

NAME = 'lab2_straight_3m'
COURSE = rg.course_template(NAME)
STRAIGHT = rg.geometry_from_course(COURSE, NAME)
T_RAMP = plateau_end(STRAIGHT)
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
OUT = os.path.join(REPO, 'tmp', 'test_audit')


def straight_msgs(meta=None, **kw):
    msgs = msgs_for(geo=STRAIGHT, t_ramp=T_RAMP, t_stop=T_RAMP + 4.0, **kw)
    meta = meta if meta is not None else bm.build_meta(COURSE, 'A', '3', 'abc')
    msgs[bm.META_TOPIC] = [(0.0, NS(data=json.dumps(meta)))]
    return msgs, meta


def test_the_packaged_straights_are_found_by_name():
    assert COURSE and os.path.isfile(os.path.join(COURSE, 'course.yaml'))
    assert rg.course_template('lab2_straight') is None       # the first draft's: gone
    assert STRAIGHT.turns == []


def test_meta_names_the_route_even_for_a_survey_copy(tmp_path):
    assert bm.build_meta(COURSE, 'A', '3', 'x')['route_name'] == NAME
    assert bm.build_meta(COURSE, 'A', '3', 'x')['route_kind'] is None
    d = tmp_path / '2026-10-16_10-00-00.lab2_straight_3m'
    shutil.copytree(COURSE, d)
    t = yaml.safe_load((d / 'course.yaml').read_text())
    t['alignment'] = {'route': NAME}
    (d / 'course.yaml').write_text(yaml.safe_dump(t))
    meta = bm.build_meta(str(d), 'A', '3', 'x')
    assert meta['course_name'] == d.name and meta['route_name'] == NAME


def test_metrics_only_for_a_straight():
    msgs, _ = straight_msgs()
    m, meta = rg.read_route_bag('/x/lab2_straight_3m_1', msgs=msgs)
    assert m.route is None and m.ungraded == NAME
    assert m.notes[0] == 'not a graded route: metrics only'
    assert m.speed_rmse < 0.05 and m.stop_error < 0.05 and m.path_dev_max < 0.05
    assert not any('turn' in n for n in m.notes), m.notes


def test_an_unknown_route_still_has_no_metrics():
    msgs, meta = straight_msgs()
    meta = dict(meta, route_name='lab2_circle_cw', course_name='lab2_circle_cw')
    msgs[bm.META_TOPIC] = [(0.0, NS(data=json.dumps(meta)))]
    m, _ = rg.read_route_bag('/x/b', msgs=msgs)
    assert m.route is None and m.ungraded is None
    assert any('not a graded route' in n for n in m.notes)


def test_check_bags_accepts_a_straight_as_metrics_only():
    msgs, meta = straight_msgs()
    v = cb.check_messages('b', topics_all(), msgs, meta)
    assert v.ok, v.reasons
    assert v.route is None and v.test_route == NAME
    assert any('not a graded route: metrics only' in w for w in v.warnings)
    # its own template still guards the path and the speed
    bad = dict(meta, route_hash='0' * 64)
    assert any('modified course' in r for r in cb.check_messages('b', topics_all(), msgs, bad).reasons)
    off, _ = straight_msgs(lateral=0.6)
    assert any('off the path' in r for r in cb.check_messages('b', topics_all(), off, meta).reasons)


def test_audit_of_a_straight_is_metrics_only_with_a_plot():
    msgs, meta = straight_msgs()
    m, _ = rg.read_route_bag('/x/lab2_straight_3m_1', msgs=msgs)
    v = cb.check_messages('b', topics_all(), msgs, meta)
    s = au.series_from_messages(msgs, STRAIGHT)
    a = au.build_audit('/x/lab2_straight_3m_1', v, m, None, s, meta)
    assert a.verdict == 'METRICS ONLY' and a.valid and not a.graded
    assert a.violations == []
    text = au.format_audit(a, m)
    assert 'not a graded route: metrics only' in text and 'speed RMSE' in text
    assert 'total' not in text
    os.makedirs(OUT, exist_ok=True)
    png = au.plot_audit(os.path.join(OUT, 'straight.png'), a, m, None, s, STRAIGHT, COURSE)
    assert os.path.getsize(png) > 10000
    # a rejected bag stays INVALID
    bad = NS(ok=False, reasons=['the run did not finish'], warnings=[])
    a2 = au.build_audit('/x/b', bad, m, None, s, meta)
    assert a2.verdict == 'INVALID' and not a2.valid
    assert any('check-bags' in x for x in a2.violations)


def test_grade_score_reports_metrics_only(monkeypatch, capsys):
    msgs, _ = straight_msgs()
    orig = rg.read_route_bag
    monkeypatch.setattr(rg, 'read_route_bag',
                        lambda p, r=None, c=None, msgs=None, _m=msgs: orig(p, r, c, msgs=_m))
    assert g.main(['--score', '/x/lab2_straight_3m_1']) == 0
    out = capsys.readouterr().out
    assert 'not a graded route: metrics only' in out and 'stop error' in out


def slow_msgs():
    """The bench car on the 3 m straight: commanded 0.8 m/s, measured never above 0.6."""
    msgs, meta = straight_msgs()
    msgs[g.VELOCITY] = [(t, NS(longitudinal_velocity=0.75 * x.longitudinal_velocity))
                        for t, x in msgs[g.VELOCITY]]
    return msgs, meta


def test_no_steady_window_still_measures_the_stop_and_the_path():
    msgs, meta = slow_msgs()
    m, _ = rg.read_route_bag('/x/lab2_straight_3m_1', msgs=msgs)
    assert m.ungraded == NAME
    assert m.peak_speed == __import__('pytest').approx(0.6, abs=0.01)
    assert m.stop_error < 0.05 and m.path_dev_max < 0.05
    assert any('no steady window' in n for n in m.notes), m.notes
    v = cb.check_messages('b', topics_all(), msgs, meta)
    assert v.ok, v.reasons
    a = au.build_audit('/x/b', v, m, None, au.series_from_messages(msgs, STRAIGHT), meta)
    assert a.verdict == 'METRICS ONLY'
    assert 'peak speed' in au.format_audit(a, m)
