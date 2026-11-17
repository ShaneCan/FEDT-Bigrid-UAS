from abc import ABC, abstractmethod

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Empty
from crazyflie_interfaces.msg import LogBlock

from crazyflie_interfaces_python.server.logblock import LogBlockServer

from typing import List


class LoggingServer(ABC):
    def __init__(self, node: Node):
        self.node = node
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10

        node.create_subscription(
            Empty,
            "~/download_logging_toc",
            self._download_toc,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            Empty,
            "~/get_logging_toc_info",
            self._get_toc_info,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            LogBlock,
            "~/create_log_block",
            self._create_log_block,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    @abstractmethod
    def download_toc(self) -> None:
        self.__warn_not_implemented("download toc")

    @abstractmethod
    def get_toc_info(self) -> None:
        self.__warn_not_implemented("get_toc_info")

    @abstractmethod
    def create_log_block(self, variables: List[str], log_block: LogBlockServer) -> None:
        self.__warn_not_implemented("create_log_block")

    def _download_toc(self, msg: Empty) -> None:
        self.download_toc()

    def _get_toc_info(self, msg: Empty) -> None:
        self.get_toc_info()

    def _create_log_block(self, msg: LogBlock) -> None:
        self.create_log_block(msg.variables, LogBlockServer(self.node, msg.name))

    def __warn_not_implemented(self, function_name: str) -> None:
        self.node.get_logger().warn(
            "There is no server-side implementation for {}!".format(function_name)
        )
