"""The week-2 stack's own record of itself, for grading: `lab3_run_meta`.

Publishes one latched /lab3/run_meta (std_msgs/String, JSON) at launch:

    {"lab": "lab3", "kind": "stack", "launch_time": <ROS time, s>, "map_dir": ...,
     "map_hash": ..., "vehicle_id": ..., "autosdv": <git rev>}

Transient local, so a recorder started later still receives it. The grader reads
launch_time for the board-initialization time limit and map_hash for which map the
stack actually loaded (the team's, or the TA's fallback map).
"""

import json
import os
import sys

from autosdv_lab3 import mapdir

TOPIC = '/lab3/run_meta'


def build(map_dir: str, vehicle_id: str, launch_time: float) -> dict:
    return {'lab': 'lab3', 'kind': 'stack', 'launch_time': launch_time,
            'map_dir': os.path.abspath(map_dir), 'map_hash': mapdir.folder_hash(map_dir),
            'vehicle_id': vehicle_id, 'autosdv': mapdir.git_rev()}


def main(argv=None) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=argv)
    node = Node('lab3_run_meta')
    map_dir = node.declare_parameter('map_path', '').value
    vehicle_id = node.declare_parameter('vehicle_id', '').value
    if not map_dir or not os.path.isdir(map_dir):
        node.get_logger().error(f'map_path {map_dir!r} is not a folder')
        return 2
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(String, TOPIC, latched)
    meta = build(map_dir, vehicle_id, node.get_clock().now().nanoseconds * 1e-9)
    pub.publish(String(data=json.dumps(meta)))
    node.get_logger().info(f'map_hash {meta["map_hash"][:12]} ({meta["map_dir"]})')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
