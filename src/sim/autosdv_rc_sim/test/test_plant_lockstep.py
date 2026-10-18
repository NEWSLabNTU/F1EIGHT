"""rc_sim_plant's clock handshake, against scripted participants. Needs rclpy.

Run under its own ROS_DOMAIN_ID and flock tmp/bench.lock.
"""
import json
import os
import time

import pytest

os.environ.setdefault('ROS_DOMAIN_ID', '73')
rclpy = pytest.importorskip('rclpy')

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from autosdv_rc_sim.plant_node import RcSimPlant  # noqa: E402
from autosdv_vehicle_interface.hal import (  # noqa: E402
    format_ack, LOCKSTEP_ACK_TOPIC, LOCKSTEP_STATUS_TOPIC)

MS = 1_000_000


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


class Harness:
    """A plant, and a probe that records /clock and status and acks as told."""

    def __init__(self, **params):
        self.plant = RcSimPlant(
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        self.probe = rclpy.create_node('rc_sim_lockstep_probe')
        self.clocks, self.status = [], []
        self.probe.create_subscription(Clock, '/clock', self.on_clock, 1000)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.probe.create_subscription(String, LOCKSTEP_STATUS_TOPIC,
                                       lambda m: self.status.append(json.loads(m.data)), latched)
        self.ack_pub = self.probe.create_publisher(
            String, LOCKSTEP_ACK_TOPIC,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.plant)
        self.executor.add_node(self.probe)
        self.on_tick = None

    def on_clock(self, msg):
        t = msg.clock.sec * 10**9 + msg.clock.nanosec
        self.clocks.append(t)
        if self.on_tick is not None:
            self.on_tick(t)

    def ack(self, name, deadline_ns, period_ns, **extra):
        self.ack_pub.publish(String(data=format_ack(name, deadline_ns, period_ns, **extra)))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def close(self):
        self.plant.stop()
        self.probe.destroy_node()
        self.plant.destroy_node()


def test_clock_waits_for_every_deadline_ack(ros):
    h = Harness(lockstep_timeout=5.0)
    period = 10 * MS
    state = {'next': None, 'acked': [], 'synced': False}

    def participant(t):
        if state['next'] is None:          # first /clock: join with this phase
            state['next'] = t + period
            h.ack('actuator', t, period)
            return
        if not state['synced'] and t > state['next']:
            # the plant ran on before it heard the join ack: catch up, as rcl's timer does
            state['next'] += (t - state['next']) // period * period
        assert t <= state['next'], f'/clock {t} passed the unacknowledged deadline {state["next"]}'
        if t == state['next']:
            time.sleep(0.003)              # the participant's work takes a while
            h.ack('actuator', t, period)
            state['synced'] = True
            state['acked'].append(t)
            state['next'] += period

    h.on_tick = participant
    try:
        h.spin(3.0)
        assert len(state['acked']) > 50
        acked = state['acked']
        assert all(b - a == period for a, b in zip(acked, acked[1:]))   # none skipped
        assert h.status and h.status[-1]['enabled'] and h.status[-1]['ok']
        assert h.status[-1]['participants']['actuator']['period'] == pytest.approx(0.01)
    finally:
        h.close()


def test_silent_participant_times_out_and_the_clock_goes_on(ros):
    h = Harness(lockstep_timeout=0.5)
    period = 10 * MS
    joined = {}

    def participant(t):
        if not joined:
            joined['t'] = t
            h.ack('actuator', t, period)    # then never again

    h.on_tick = participant
    try:
        h.spin(0.4)
        held = h.clocks[-1]
        assert held == joined['t'] + period              # parked at the first unacked deadline
        assert h.status[-1]['ok']
        h.spin(1.5)
        assert h.clocks[-1] > held + 100 * MS            # the run continued after the timeout
        bad = [s for s in h.status if not s['ok']]
        assert bad and 'actuator did not acknowledge' in bad[0]['reason']
        assert bad[0]['timeouts'] == 1
    finally:
        h.close()


def test_pwm_from_the_actuator_ack(ros):
    h = Harness(lockstep_timeout=5.0)
    try:
        h.spin(0.2)
        h.ack('actuator', h.clocks[-1], 10 * MS, pwm=420, steer=470, report=-1)
        h.spin(0.2)
        assert h.plant.pwm == [420, 470]
    finally:
        h.close()


def test_lockstep_off_ignores_acks(ros):
    h = Harness(lockstep=False)
    try:
        h.spin(0.2)
        h.ack('actuator', h.clocks[-1], 10 * MS)
        h.spin(0.5)
        clocks = h.clocks
        assert clocks[-1] - clocks[0] > 300 * MS
        assert h.status and h.status[-1]['enabled'] is False and h.status[-1]['ok']
    finally:
        h.close()


def test_every_batch_marks_its_edges_complete_up_to_the_next_clock(ros):
    from std_msgs.msg import Int64MultiArray
    from autosdv_vehicle_interface.hal import EDGE_TOPIC, edges_until_ns
    h = Harness()
    log = []
    h.probe.create_subscription(
        Int64MultiArray, EDGE_TOPIC,
        lambda m: log.append(('edges', edges_until_ns(m), list(m.data))), 1000)
    h.on_tick = lambda t: log.append(('clock', t, None))
    try:
        h.spin(1.0)
        marks = [m for kind, m, _ in log if kind == 'edges']
        clocks = [t for kind, t, _ in log if kind == 'clock']
        assert len(marks) > 100 and None not in marks
        assert set(marks[5:-5]) <= set(clocks)          # each mark is a /clock value
        for kind, mark, data in log:
            if kind == 'edges':
                assert all(e <= mark for e in data)
    finally:
        h.close()


def test_off_grid_deadline_is_rounded_up_to_the_physics_step(ros):
    h = Harness(lockstep_timeout=5.0)
    period = 33_333_333                 # path_tracker's 30 Hz
    state = {'next': None, 'acked': []}

    def participant(t):
        assert t % MS == 0, f'/clock {t} off the 1 ms physics grid'
        if state['next'] is None:
            state['next'] = t + period
            h.ack('path_tracker', t, period)
            return
        while t >= state['next']:
            assert t - state['next'] < MS or not state['acked'], \
                f'/clock {t} passed the deadline {state["next"]} by a step or more'
            h.ack('path_tracker', state['next'], period)
            state['acked'].append(state['next'])
            state['next'] += period

    h.on_tick = participant
    try:
        h.spin(2.0)
        assert len(state['acked']) > 20
    finally:
        h.close()
