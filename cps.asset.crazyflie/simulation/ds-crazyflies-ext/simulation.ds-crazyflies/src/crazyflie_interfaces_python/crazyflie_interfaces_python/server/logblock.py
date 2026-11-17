from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crazyflie_interfaces.msg import GenericLogData
from std_msgs.msg import Int16, Empty

from typing import List, Callable

from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class LogBlockServer:

    def __init__(self, node: Node, name: str):
        self.node = node
        self._log_block_start_callback: Callable[[int], None] = None
        self._log_block_stop_callback: Callable[[], None] = None
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10
        namespace = "~/log/{}/".format(name)

        node.create_subscription(
            Int16,
            namespace + "start",
            self._start_log_block,
            qos_profile=QoSProfile(
                depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            ),
            callback_group=callback_group,
        )

        node.create_subscription(
            Empty,
            namespace + "stop",
            self._stop_log_block,
            qos_profile=QoSProfile(
                depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            ),
            callback_group=callback_group,
        )

        self.log_data_publisher = node.create_publisher(
            GenericLogData,
            namespace + "data",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    def set_log_block_start_callback(self, callback: Callable[[int], None]) -> None:
        self._log_block_start_callback = callback

    def set_log_block_stop_callback(self, callback: Callable[[], None]) -> None:
        self._log_block_stop_callback = callback

    def send_data(self, values: List[float]):
        msg = GenericLogData()
        msg.values = values
        self.log_data_publisher.publish(msg)

    def _start_log_block(self, msg):
        period_ms = msg.data
        if self._log_block_start_callback:
            return self._log_block_start_callback(period_ms)
        self.node.get_logger().warn(
            "Cannot start log block, server did not provide callback yet"
        )

    def _stop_log_block(self, msg):
        if self._log_block_stop_callback:
            return self._log_block_stop_callback()
        self.node.get_logger().warn(
            "Cannot stop log block, server didnt provide callback yet"
        )
