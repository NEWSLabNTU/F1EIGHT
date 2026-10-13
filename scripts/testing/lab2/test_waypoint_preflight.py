"""The preflight's QoS-mismatch rule, without a running stack:
python3 -m pytest scripts/testing/lab2/test_waypoint_preflight.py"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from waypoint_preflight import qos_mismatch  # noqa: E402


def test_best_effort_publisher_to_reliable_subscriber_is_a_mismatch():
    assert qos_mismatch(['BEST_EFFORT'], ['RELIABLE']) == [('BEST_EFFORT', 'RELIABLE')]


def test_matching_or_looser_subscribers_are_fine():
    assert qos_mismatch(['RELIABLE'], ['RELIABLE']) == []
    assert qos_mismatch(['RELIABLE'], ['BEST_EFFORT']) == []
    assert qos_mismatch(['BEST_EFFORT'], ['BEST_EFFORT']) == []
    assert qos_mismatch([], ['RELIABLE']) == []
