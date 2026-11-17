from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Empty


class EmergencyClient:
    def __init__(self, node: Node, prefix: str):
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10

        self.emergency_publisher = node.create_publisher(
            Empty,
            prefix + "/emergency",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    def emergency(self) -> None:
        """Emergency stop. Cuts power, causes future commands to be ignored."""
        self.emergency_publisher.publish(Empty())
