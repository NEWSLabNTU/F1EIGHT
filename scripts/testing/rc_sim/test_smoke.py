"""smoke.py's pure helpers, without ROS: python3 -m pytest scripts/testing/rc_sim/test_smoke.py"""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parent))
from smoke import LANE_DEPARTURE_MIN_HZ, diag_rate, level_of  # noqa: E402


def test_level_of_reads_the_ros_byte():
    assert level_of(NS(level=b'\x00')) == 0
    assert level_of(NS(level=b'\x03')) == 3
    assert level_of(NS(level=2)) == 2


def test_lane_departure_rate_tells_a_routeless_checker():
    with_route = [k * 0.1 for k in range(300)]       # 10 Hz for 30 s of simulated time
    without = [float(k) for k in range(30)]          # the updater's 1 Hz fallback
    assert diag_rate(with_route) >= LANE_DEPARTURE_MIN_HZ
    assert diag_rate(without) < LANE_DEPARTURE_MIN_HZ
    assert diag_rate([]) is None and diag_rate([5.0]) is None
