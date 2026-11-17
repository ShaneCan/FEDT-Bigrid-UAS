from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class RPYTCommanderClient:

    def __init__(self, node: Node, prefix: str):
        callback_group = MutuallyExclusiveCallbackGroup
        qos_profile = 10

        # self.rpyt_publisher = node.create_publisher(
        #    RPYT,
        #    "~/send_setpoint",
        #    qos_profile=qos_profile,
        #    callback_group=callback_group,
        # )

    def send_rpyt_setpoint(
        self, roll: float, pitch: float, yawrate: float, thrust: int
    ) -> None:
        """Send a roll pitch yawrate thrust command

        This is typically used for manual flying (easy mode) as the angles correspond to stick positions.

        Args:
            roll (float): The roll angle, positive rolls to right
            pitch (float): The pitch angle, positive pitch forward
            yawrate (float): The angular velocity of yaw, positive turn counter clockwise
            thrust (int): Thrust magnitude. Non meaningfull units corresponds to a uint16_t 0-100% [0, 2^16]

        Raises:
            NotImplementedError: Will always return this error because it is not implemented in this library.
        """

        raise NotImplementedError("There is currently no rpyt support in this library.")
