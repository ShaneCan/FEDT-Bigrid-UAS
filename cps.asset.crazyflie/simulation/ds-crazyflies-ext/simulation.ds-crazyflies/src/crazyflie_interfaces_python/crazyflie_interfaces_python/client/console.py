from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String

from typing import Callable


class ConsoleClient:
    """The console functionality of the crazyflie.

    Console: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/crtp_console/
    The console messages from crazyflie are processed and sent as lines to the console topic.
    Add a callback in constructor to acces the messages.
    """

    def __init__(self, node: Node, prefix: str, callback: Callable[[str], None] = None):
        self.message_callback = callback
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10

        node.create_subscription(
            String,
            prefix + "/console",
            self._console_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    def _console_callback(self, msg: String) -> None:
        message = msg.data
        if self.message_callback:
            self.message_callback(message)
