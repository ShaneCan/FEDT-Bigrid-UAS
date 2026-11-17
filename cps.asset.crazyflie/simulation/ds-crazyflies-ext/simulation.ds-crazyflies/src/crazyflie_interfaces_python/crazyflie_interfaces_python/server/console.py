from abc import ABC

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String


class ConsoleServer(ABC):
    def __init__(self, node: Node):
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10

        self.console_publisher = node.create_publisher(
            String, "~/console", qos_profile=qos_profile, callback_group=callback_group
        )

    def console_message(self, message: str) -> None:
        """Sends a console message from crazyflie to the ros client

        Args:
            message (str): The message sent by the crazyflie
        """
        msg = String()
        msg.data = message
        self.console_publisher.publish(msg)
