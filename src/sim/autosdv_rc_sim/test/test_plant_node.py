"""rc_sim_plant: owns /clock, takes PWM, emits edges and state. Needs rclpy.

Run under ROS_DOMAIN_ID=73 and flock tmp/bench.lock.
"""
import os
import time

import pytest

os.environ.setdefault('ROS_DOMAIN_ID', '73')
rclpy = pytest.importorskip('rclpy')

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from std_msgs.msg import Float64MultiArray, Int32MultiArray, Int64MultiArray  # noqa: E402

from autosdv_rc_sim.plant_node import RcSimPlant  # noqa: E402
from autosdv_vehicle_interface.hal import EDGE_TOPIC, PWM_TOPIC  # noqa: E402


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_refuses_sim_time(ros):
    with pytest.raises(RuntimeError, match='owns /clock'):
        RcSimPlant(parameter_overrides=[Parameter('use_sim_time', value=True)])


def test_drives_on_pwm_and_publishes_clock_edges_and_state(ros):
    plant = RcSimPlant()
    probe = rclpy.create_node('rc_sim_probe')
    got = {'clock': [], 'edges': 0, 'state': None}
    probe.create_subscription(Clock, '/clock', lambda m: got['clock'].append(
        m.clock.sec + m.clock.nanosec * 1e-9), 100)
    probe.create_subscription(Int64MultiArray, EDGE_TOPIC,
                              lambda m: got.__setitem__('edges', got['edges'] + len(m.data)), 100)
    probe.create_subscription(Float64MultiArray, '/rc_sim/state',
                              lambda m: got.__setitem__('state', list(m.data)), 10)
    pwm = probe.create_publisher(Int32MultiArray, PWM_TOPIC, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(plant)
    executor.add_node(probe)
    try:
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            pwm.publish(Int32MultiArray(data=[0, 420]))
            executor.spin_once(timeout_sec=0.01)
        assert got['clock'] and got['clock'][-1] > 2.0           # simulated time advanced
        assert got['clock'] == sorted(got['clock'])
        assert got['state'] is not None and got['state'][4] > 1.0   # vx
        assert got['edges'] > 20
    finally:
        plant.stop()
        probe.destroy_node()
        plant.destroy_node()


def test_publish_order_edges_before_the_clock_that_covers_them(ros, monkeypatch):
    """Task 18c: an edge reaches velocity_report before the /clock tick at or after its time.

    velocity_report caps its estimate at 1.5 * marker_spacing / (clock - last edge): a /clock
    published before the edges it covers makes the car read ~8 m/s at most, whatever its speed
    (the hall estimate saturated and the speed PID held full throttle). Data messages keep the
    older invariant: never stamped later than the /clock already published.
    """
    log = []     # ('clock', t) / ('edges', [stamps]) / ('data', t), in publication order

    class Recorder:
        def __init__(self, inner, kind):
            self.inner, self.kind = inner, kind

        def publish(self, msg):
            if self.kind == 'clock':
                log.append(('clock', msg.clock.sec * 10**9 + msg.clock.nanosec))
            elif self.kind == 'edges':
                log.append(('edges', list(msg.data)))
            else:
                log.append(('data', msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec))
            self.inner.publish(msg)

    real = RcSimPlant.create_publisher

    def spy(self, msg_type, topic, *args, **kwargs):
        pub = real(self, msg_type, topic, *args, **kwargs)
        kind = {'/clock': 'clock', EDGE_TOPIC: 'edges',
                '/localization/kinematic_state': 'data'}.get(topic)
        return Recorder(pub, kind) if kind else pub

    monkeypatch.setattr(RcSimPlant, 'create_publisher', spy)
    plant = RcSimPlant()
    probe = rclpy.create_node('rc_sim_probe_order')
    pwm = probe.create_publisher(Int32MultiArray, PWM_TOPIC, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(plant)
    executor.add_node(probe)
    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            pwm.publish(Int32MultiArray(data=[0, 440]))
            executor.spin_once(timeout_sec=0.01)
    finally:
        plant.stop()
        probe.destroy_node()
        plant.destroy_node()
    entries = list(log)
    clocks = [(i, v) for i, (k, v) in enumerate(entries) if k == 'clock']
    n_edges = 0
    for i, (kind, v) in enumerate(entries):
        if kind == 'edges':
            for stamp in v:
                n_edges += 1
                covering = next((ci for ci, ct in clocks if ct >= stamp), None)
                assert covering is None or covering > i, f'edge {stamp} published after the /clock covering it'
        elif kind == 'data':
            latest = max((ct for ci, ct in clocks if ci < i), default=0)
            assert v <= latest, 'data stamped later than the /clock already published'
    assert n_edges > 20
