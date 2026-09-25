#!/usr/bin/env python3
"""
Relay node: subscribes to /cf_positions (crazyflie_interfaces/PoseStampedArray)
and republishes as /cf_positions_path (nav_msgs/Path).

Both types share the same structure (Header + PoseStamped[]), so frame_id
in each PoseStamped.header is preserved.

Run this inside the Docker container where crazyflie_interfaces is available:
    python3 cf_positions_relay.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped as GeoPoseStamped

try:
    from crazyflie_interfaces.msg import PoseStampedArray
except ImportError:
    print("ERROR: crazyflie_interfaces not found. Run this in the environment "
          "where the package is installed (e.g. inside the Docker container).")
    raise SystemExit(1)


class CfPositionsRelay(Node):
    def __init__(self):
        super().__init__("cf_positions_relay")
        self.sub = self.create_subscription(
            PoseStampedArray, "/cf_positions", self.callback, 10
        )
        self.pub = self.create_publisher(Path, "/cf_positions_path", 10)
        self.get_logger().info(
            "Relaying /cf_positions (PoseStampedArray) -> /cf_positions_path (nav_msgs/Path)"
        )

    def callback(self, msg):
        path = Path()
        path.header = msg.header
        path.poses = msg.poses
        self.pub.publish(path)


def main():
    rclpy.init()
    node = CfPositionsRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
