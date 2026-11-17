from abc import ABC, abstractmethod

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class RPYTCommanderServer(ABC):
    def __init__(self, node: Node):
        callback_group = MutuallyExclusiveCallbackGroup()

        # TODO: sendSetpoint not implemented yet
        # node.create_subscription(SendSetpoint, "~/send_setpoint", self._send_setpoint, 10,callback_group=callback_group)

    @abstractmethod
    def setpoint(self, roll: float, pitch: float, yawrate: float, thrust: int) -> None:
        pass

    def _send_setpoint(self, msg):
        self.setpoint(msg.roll, msg.pitch, msg.yawrate, msg.thrust)
