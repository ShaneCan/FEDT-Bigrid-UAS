from abc import ABC, abstractmethod

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Empty


class EmergencyServer(ABC):
    def __init__(self, node: Node):
        self.node = node
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10

        node.create_subscription(
            Empty,
            "~/emergency",
            self._emergency,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    @abstractmethod
    def emergency(self):
        self.__warn_not_implemented(self, "emergency")

    def _emergency(self, msg: Empty) -> None:
        self.emergency()

    def __warn_not_implemented(self, function_name: str) -> None:
        self.node.get_logger().warn(
            "There is no server-side implementation for {}!".format(function_name)
        )
