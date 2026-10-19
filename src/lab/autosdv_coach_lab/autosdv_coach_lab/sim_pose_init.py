"""sim_pose_init: place the simulated vehicle once, through the ADAPI.

The planning simulator does not move until /api/localization/initialize is
called. This node calls it with a configured pose, retrying until the stack is
up enough to accept it, and then exits. Simulator only.

wait_for_route_subscriber (a node name, default empty: no wait) holds the
initialization until that node subscribes to /planning/mission_planning/route.
The route can only be set once localization is initialized, so this makes every
client (smoke.py, RViz's 2D Goal Pose) set it after that node is there. It is
for Autoware 1.5.0's lane_departure_checker, whose route subscription is
VOLATILE: the mission planner publishes the route once (TRANSIENT_LOCAL), and a
checker that subscribes after that never gets it, so it checks nothing, and its
diagnostic drops to 1 Hz against the graph's 1.0 s timeout (a STALE flap, an MRM
every second, when the phases line up). Upstream fixed the QoS (transient_local)
after 1.5.0. wait_for_route_subscriber_timeout bounds the wait: after it the
node warns and initializes anyway.
"""

import math
import time

import rclpy
from autoware_adapi_v1_msgs.srv import InitializeLocalization
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


ROUTE_TOPIC = '/planning/mission_planning/route'


def wait_for_route_subscriber(node, name, timeout, poll=0.2):
    """Return once node `name` subscribes to the route (or after `timeout` s, warning)."""
    if not name:
        return True
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if any(i.node_name == name for i in node.get_subscriptions_info_by_topic(ROUTE_TOPIC)):
            node.get_logger().info(f'{name} subscribes to {ROUTE_TOPIC} '
                                   f'({time.monotonic() - t0:.1f} s); initializing')
            return True
        node.get_logger().info(f'waiting for {name} to subscribe to {ROUTE_TOPIC}',
                               throttle_duration_sec=10.0)
        time.sleep(poll)   # the graph cache needs no spinning
    node.get_logger().warn(f'{name} never subscribed to {ROUTE_TOPIC} in {timeout:.0f} s; '
                           'initializing anyway (a route set now may never reach it)')
    return False


def main(args=None):
    rclpy.init(args=args)
    node = Node('sim_pose_init')
    for k, v in (('x', 0.0), ('y', 0.0), ('z', 0.0), ('yaw', 0.0),
                 ('frame_id', 'map'), ('retry_period', 3.0), ('give_up_after', 600.0),
                 ('wait_for_route_subscriber', ''),
                 ('wait_for_route_subscriber_timeout', 300.0)):
        node.declare_parameter(k, v)
    gp = lambda k: node.get_parameter(k).value  # noqa: E731
    cli = node.create_client(InitializeLocalization, '/api/localization/initialize')
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = gp('frame_id')
    pose.pose.pose.position.x = float(gp('x'))
    pose.pose.pose.position.y = float(gp('y'))
    pose.pose.pose.position.z = float(gp('z'))
    yaw = float(gp('yaw'))
    pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
    req = InitializeLocalization.Request()
    req.pose = [pose]
    ok = False
    # Retry by wall-clock deadline, sleeping between attempts. The API answers
    # "not success" immediately while localization is still coming up, so a
    # retry loop that does not sleep burns any attempt budget in milliseconds
    # (it did: 100 attempts in 30 ms, and the simulator never moved).
    deadline = time.monotonic() + float(gp('give_up_after'))
    attempt = 0
    try:
        wait_for_route_subscriber(node, str(gp('wait_for_route_subscriber')),
                                  float(gp('wait_for_route_subscriber_timeout')))
        while time.monotonic() < deadline:
            attempt += 1
            if cli.wait_for_service(timeout_sec=float(gp('retry_period'))):
                pose.header.stamp = node.get_clock().now().to_msg()
                fut = cli.call_async(req)
                rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
                res = fut.result()
                if res is not None and res.status.success:
                    node.get_logger().info(f'initialised after {attempt} attempt(s)')
                    ok = True
                    break
                msg = res.status.message if res is not None else 'timeout'
                node.get_logger().info(
                    f'attempt {attempt} refused ({msg!r}); retrying',
                    throttle_duration_sec=10.0)
            time.sleep(float(gp('retry_period')))
    except KeyboardInterrupt:
        pass
    if not ok:
        node.get_logger().error('gave up initialising the simulator pose')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
