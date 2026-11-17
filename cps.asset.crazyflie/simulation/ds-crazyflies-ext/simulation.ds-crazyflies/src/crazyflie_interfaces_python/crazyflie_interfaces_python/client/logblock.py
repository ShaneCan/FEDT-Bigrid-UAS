from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from crazyflie_interfaces.msg import LogDataGeneric

from typing import Callable, List


class LogBlock:
    """If a log block was created you can use this to remove it."""

    def __init__(
        self,
        node: Node,
        prefix: str,
        name: str,
        callback: Callable[[List[float]], None],
    ):
        self.log_data_subscription = node.create_subscription(
            LogDataGeneric,
            prefix + "/" + name,
            self._log_data_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.name = name
        self.callback = callback

    def _log_data_callback(self, msg: LogDataGeneric) -> None:
        self.callback(msg.values)
