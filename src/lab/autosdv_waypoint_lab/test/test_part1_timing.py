"""Lab 2 Part 1 timing rules: the e-stop marks (scenario) and path_tracker's odometry gate."""
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from autosdv_waypoint_lab.part1.scenario_node import next_mark_ns

MS = 1_000_000
PKG = Path(__file__).resolve().parents[1]
PROFILE = PKG / 'config/courses/lab2_part1/part1_profile.yaml'


def test_next_mark_is_on_the_grid_and_at_least_the_margin_later():
    assert next_mark_ns(21_610 * MS, 100 * MS, 50 * MS) == 21_700 * MS
    assert next_mark_ns(21_650 * MS, 100 * MS, 50 * MS) == 21_700 * MS
    assert next_mark_ns(21_660 * MS, 100 * MS, 50 * MS) == 21_800 * MS
    assert next_mark_ns(21_600 * MS, 100 * MS, 50 * MS) == 21_700 * MS
    for t in range(0, 1000 * MS, 7 * MS):
        m = next_mark_ns(t, 100 * MS, 50 * MS)
        assert m % (100 * MS) == 0 and 50 * MS <= m - t < 150 * MS


def test_tracker_waits_for_the_odometry_of_its_clock_time_not_its_deadline():
    pytest.importorskip('rclpy')
    from autosdv_waypoint_lab.path_tracker_node import PathTrackerNode
    node = SimpleNamespace(odom_stamp_ns=3_890 * MS, odom_period_ns=10 * MS)
    deadline, clock = 3_899_999_961, 3_900 * MS     # every third 30 Hz deadline
    assert not PathTrackerNode.odom_awaiting(node, deadline)   # the old rule: a race
    assert PathTrackerNode.odom_awaiting(node, clock)          # waits for odometry at 3.900
    node.odom_stamp_ns = 3_900 * MS
    assert not PathTrackerNode.odom_awaiting(node, clock)
    assert not PathTrackerNode.odom_awaiting(SimpleNamespace(odom_stamp_ns=None,
                                                             odom_period_ns=None), clock)


@pytest.fixture
def scenario():
    rclpy = pytest.importorskip('rclpy')
    os.environ.setdefault('ROS_DOMAIN_ID', '73')
    from rclpy.parameter import Parameter
    from autosdv_waypoint_lab.part1.scenario_node import Part1Scenario
    rclpy.init()
    node = Part1Scenario(parameter_overrides=[
        Parameter('profile_file', value=str(PROFILE)), Parameter('wait_for_topic', value='')])
    events = []
    node.event = lambda name, t=None, **extra: events.append((name, t))
    node.publish_emergency = lambda stamp=None: None
    yield node, events
    node.destroy_node()
    rclpy.try_shutdown()


def state(node, t, s, v):
    from std_msgs.msg import Float64MultiArray
    node.on_state(Float64MultiArray(data=[t, s, 0.0, 0.0, v]))


class FakeAck:

    def __init__(self):
        self.acks = []

    def ack(self, deadline_ns, period_ns, **extra):
        self.acks.append(deadline_ns)


def tick(node, t_ns, published):
    node._hal = SimpleNamespace(timer_deadline_ns=lambda timer, now: t_ns)
    node.publish_emergency = lambda stamp=None: published.append(
        (None if stamp is None else stamp.sec * 10**9 + stamp.nanosec, node.estop))
    node.on_emergency_timer()


def test_estop_is_applied_on_the_emergency_tick_at_the_mark(scenario):
    node, events = scenario
    node.lockstep = FakeAck()
    published = []
    node.phase = 'running'
    state(node, 21.61, node.trigger_s + 0.01, 6.0)
    assert node.pending == ('estop_on', 21_700 * MS) and node.phase == 'running'
    state(node, 21.62, node.trigger_s + 0.07, 6.0)               # no second schedule
    assert node.pending == ('estop_on', 21_700 * MS)
    tick(node, 21_650 * MS, published)                           # before the mark: nothing
    assert node.phase == 'running' and published[-1] == (21_650 * MS, False)
    tick(node, 21_700 * MS, published)                           # the mark
    assert node.phase == 'estop' and published[-1] == (21_700 * MS, True)
    assert events[0][0] == 'estop_on' and events[0][1] == pytest.approx(21.7)
    assert node.lockstep.acks == [21_650 * MS, 21_700 * MS]      # acked after publishing
    for k in range(120):                                         # at rest for rest_time
        state(node, 23.0 + k * 0.01, node.trigger_s + 1.0, 0.0)
        if node.pending:
            break
    action, mark = node.pending
    assert action == 'estop_off' and mark % (100 * MS) == 0
    tick(node, mark, published)
    assert node.phase == 'resumed' and not node.estop and published[-1] == (mark, False)
    assert node.ramp_t == pytest.approx(mark * 1e-9)
    assert events[-1][0] == 'estop_off' and events[-1][1] == pytest.approx(mark * 1e-9)


def test_forwarded_gate_verdict_keeps_the_last_tick_stamp(scenario):
    from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
    node, _ = scenario
    node.lockstep = FakeAck()
    published = []
    tick(node, 5_000 * MS, published)
    verdict = VehicleEmergencyStamped(emergency=True)
    verdict.stamp.sec = 5
    verdict.stamp.nanosec = 30 * MS                              # the gate's own, later stamp
    node.on_gate(verdict)
    assert published[-1][0] == 5_000 * MS                        # not 5.030
