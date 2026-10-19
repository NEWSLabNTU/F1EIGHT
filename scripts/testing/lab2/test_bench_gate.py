"""The gate's verdict logic, without ROS: python3 -m pytest scripts/testing/lab2/test_bench_gate.py"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bench_gate import judge, load_error, localization_problem  # noqa: E402

GOOD = [0.0, 0.4, 0.8, 0.82, 0.8, 0.79, 0.4, 0.0]


def run(**kw):
    a = dict(v_ref=0.8, speeds=GOOD, finished=True, laps_done=1, laps=1, mrm=False, faults=[],
             stalled=False, at_rest=True, timed_out=False)
    a.update(kw)
    return judge(**a)


def test_clean_run_passes():
    assert run() == (True, [])


def test_fault_mrm_stall_each_fail():
    assert not run(faults=['override_neutral:controller_fault'])[0]
    assert not run(mrm=True)[0]
    assert not run(stalled=True, finished=False, speeds=[0.0])[0]


def test_mrm_reason_is_named_in_the_failure():
    ok, why = run(mrm='operating COMFORTABLE_STOP, 3.1 s after engage; not OK around it: '
                      '/adapi/mrm_request/delegate STALE')
    assert not ok
    assert why[0].startswith('MRM raised (operating COMFORTABLE_STOP')
    assert 'delegate STALE' in why[0]
    assert run(mrm=True)[1] == ['MRM raised']


def test_unfinished_and_short_laps_fail():
    assert not run(finished=False, timed_out=True)[0]
    assert 'laps' in run(laps_done=0)[1][0]


def test_garbage_speed_fails():
    assert not run(speeds=[2.7] * 50)[0]          # full throttle
    assert not run(speeds=[0.2] * 50)[0]          # crawling
    assert not run(speeds=GOOD + [1.3])[0]        # overshoot spike


def test_load_error_reads_actuator_err(tmp_path):
    d = tmp_path / 'play_log/x/node/actuator_node'
    d.mkdir(parents=True)
    (d / 'err').write_text("[1.0] [autosdv.actuator_node]: Cannot load speed controller 'n:X': No module\n")
    assert load_error(tmp_path) == "Cannot load speed controller 'n:X': No module"
    assert load_error(tmp_path / 'nothing') is None


def test_the_gate_drives_the_graded_route():
    from bench_gate import COURSES_DIR, ROUTES, course_info
    assert ROUTES == ['lab2_uturn']
    d, v_ref, laps = course_info('lab2_uturn')
    assert d == COURSES_DIR / 'lab2_uturn' and (v_ref, laps) == (0.8, 1)


def test_localization_that_never_started_is_named():
    why = localization_problem(kin_first=None, kin_last=None, launched=100.0, now=190.0)
    assert why.startswith('localization never started')
    assert 'kinematic_state' in why and '90 s' in why
    ok, reasons = run(finished=False, stalled=True, speeds=[0.0], localization=why)
    assert not ok and reasons[0] == why
    assert not any('laps' in r for r in reasons)


def test_localization_that_stopped_is_named():
    why = localization_problem(kin_first=105.0, kin_last=120.0, launched=100.0, now=150.0)
    assert why.startswith('localization stopped')
    assert '30.0 s' in why


def test_fresh_localization_is_no_problem():
    assert localization_problem(kin_first=105.0, kin_last=149.9, launched=100.0, now=150.0) is None
