"""The grader's trace alignment (RowAligner): target and emergency at the row's instant."""
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('tier4_vehicle_msgs')
from autosdv_waypoint_lab.part1.grader_node import RowAligner  # noqa: E402


def row(t, mode='active'):
    return {'t': t, 'mode': mode, 'target': None, 'emergency': None}


def test_target_is_the_command_stamped_at_or_before_the_row():
    a = RowAligner()
    for t in (1.00, 1.01, 1.02, 1.03):
        a.add(row(t))
    a.on_cmd(0.9667, 0.25)
    a.on_cmd(1.0, 0.30)                 # stamped exactly at a row
    a.on_cmd(1.0333, 0.35)
    a.on_emergency(1.05)
    out = a.ready(state_t=1.03)
    assert [r['t'] for r in out] == [1.00, 1.01, 1.02, 1.03]
    assert [r['target'] for r in out] == [0.30, 0.30, 0.30, 0.30]


def test_rows_wait_for_both_streams_or_the_lag():
    a = RowAligner(lag=0.05)
    a.add(row(2.00))
    a.add(row(2.01))
    a.on_cmd(2.0333, 1.0)               # a command after both rows ...
    assert a.ready(state_t=2.02) == []  # ... but no emergency message after them yet
    a.on_emergency(2.005)
    out = a.ready(state_t=2.02)
    assert [r['t'] for r in out] == [2.00]          # 2.01 still waits for the emergency stream
    assert [r['t'] for r in a.ready(state_t=2.06)] == [2.01]   # the lag releases it


def test_emergency_follows_the_event_times_not_their_arrival():
    a = RowAligner()
    for t in (5.00, 5.01, 5.02, 5.03):
        a.add(row(t))
    a.on_cmd(5.1, 6.0)
    a.on_emergency(5.1)
    a.on_estop(5.02, True)              # arrives late, applies from 5.02 on
    out = a.ready(state_t=5.03)
    assert [r['emergency'] for r in out] == [0, 0, 1, 1]
    a.add(row(6.00))
    a.add(row(6.01, mode='brake:emergency'))
    a.on_estop(5.995, False)
    a.on_cmd(6.1, 0.0)
    a.on_emergency(6.1)
    assert [r['emergency'] for r in a.ready(state_t=6.01)] == [0, 0]   # the event, not the mode


def test_flush_releases_everything_and_old_commands_are_dropped():
    a = RowAligner()
    for k in range(100):
        a.on_cmd(k * 0.0333, float(k))
    a.add(row(3.0))
    out = a.ready(state_t=3.0, flush=True)
    assert len(out) == 1 and out[0]['target'] == 90.0
    assert len(a.cmd_t) <= 10
    a.on_cmd(1.0, -1.0)                 # reordered, older than kept: ignored
    assert a.cmd_v[-1] == 99.0


def test_without_an_estop_rows_wait_for_the_command_alone():
    a = RowAligner(need_emergency=False)
    a.add(row(3.00))
    a.on_cmd(3.0333, 2.0)
    out = a.ready(state_t=3.00)                  # no emergency message, no lag: released
    assert [r['t'] for r in out] == [3.00] and out[0]['emergency'] == 0
