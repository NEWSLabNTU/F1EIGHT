"""sim_pose_init waits for the route subscriber before it initializes (rc_sim's route race)."""
from types import SimpleNamespace as NS

from autosdv_coach_lab.sim_pose_init import ROUTE_TOPIC, wait_for_route_subscriber


class FakeNode:
    def __init__(self, appears_after):
        self.calls = 0
        self.appears_after = appears_after
        self.topics = []
        self.logged = []

    def get_subscriptions_info_by_topic(self, topic):
        self.topics.append(topic)
        self.calls += 1
        subs = [NS(node_name='some_other_node')]
        if self.appears_after is not None and self.calls > self.appears_after:
            subs.append(NS(node_name='lane_departure_checker_node'))
        return subs

    def get_logger(self):
        log = self.logged

        class L:
            def info(self, msg, **kw):
                log.append(('info', msg))

            def warn(self, msg, **kw):
                log.append(('warn', msg))
        return L()


def test_no_name_no_wait():
    n = FakeNode(appears_after=None)
    assert wait_for_route_subscriber(n, '', 10.0, poll=0.0)
    assert n.calls == 0


def test_waits_until_the_checker_subscribes_to_the_route():
    n = FakeNode(appears_after=3)
    assert wait_for_route_subscriber(n, 'lane_departure_checker_node', 10.0, poll=0.0)
    assert n.calls == 4
    assert set(n.topics) == {ROUTE_TOPIC} == {'/planning/mission_planning/route'}


def test_gives_up_with_a_warning():
    n = FakeNode(appears_after=None)
    assert not wait_for_route_subscriber(n, 'lane_departure_checker_node', 0.05, poll=0.01)
    assert n.logged[-1][0] == 'warn'
