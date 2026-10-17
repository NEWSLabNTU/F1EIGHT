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
