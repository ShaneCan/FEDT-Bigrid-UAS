from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crazyflie_interfaces.srv import AddLogging, RemoveLogging
from crazyflie_interfaces_python.client.logblock import LogBlock
from typing import Callable, List


class LoggingClient:
    """The logging functionality of the crazyflie.
    https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/crtp_log/

    The logging on the crazyflie is segemented into log blocks.
    Create a log block with variables from: (other implementations (webots or older crazyflie version) might not have
    all logging variables available)
    https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/

    This creates a LogBlock, which is used to remove it.
    """

    def __init__(self, node: Node, prefix: str):
        self.node = node
        self.prefix = prefix
        callback_group = MutuallyExclusiveCallbackGroup()

        self.create_log_block_client = node.create_client(
            AddLogging,
            prefix + "/add_logging",
            callback_group=callback_group,
        )

        self.remove_log_block_client = node.create_client(
            RemoveLogging,
            prefix + "/remove_logging",
            callback_group=callback_group,
        )

    def create_log_block(
        self,
        name: str,
        frequency_hz: float,
        variables: List[str],
        callback: Callable[[List[float]], None],
    ) -> LogBlock:
        """Create a log block with given variables and frequency.

        The created topic will have the specified name

        Args:
            name (str): The name of the rostopic
            frequency_hz (float): The frequency of the log block
            variables (List[str]): The logging variables (e.g. ["range.zrange", ])
            callback (Callable[[List[float]], None]): A callback function for data beeing received

        Returns:
            LogBlock: A LogBlock object with which the block can be started/stopped
        """
        req = AddLogging.Request()
        req.topic_name = name
        req.frequency = frequency_hz
        req.vars = variables
        self.create_log_block_client.call_async(req)
        return LogBlock(self.node, self.prefix, name, callback)

    def remove_log_block(self, log_block: LogBlock) -> None:
        """Remove a log block with given name.

        Args:
            log_block (LogBlock): The log block to remove
        """
        req = RemoveLogging.Request()
        req.topic_name = log_block.name
        self.remove_log_block_client.call_async(req)
