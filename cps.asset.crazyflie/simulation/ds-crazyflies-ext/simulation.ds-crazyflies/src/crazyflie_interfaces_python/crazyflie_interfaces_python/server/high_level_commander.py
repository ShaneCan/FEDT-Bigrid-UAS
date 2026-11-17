from abc import ABC, abstractmethod

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crazyflie_interfaces.msg import (
    SetGroupMask,
    Takeoff,
    Land,
    Stop,
    GoTo,
    StartTrajectory,
    UploadTrajectory,
    TakeoffWithVelocity,
    LandWithVelocity,
    Spiral,
)


class HighLevelCommanderServer(ABC):
    def __init__(self, node: Node):
        self.node = node
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10

        node.create_subscription(
            SetGroupMask,
            "~/set_group_mask",
            self._set_group_mask,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            Takeoff,
            "~/takeoff",
            self._takeoff,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            Land,
            "~/land",
            self._land,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            Stop,
            "~/stop",
            self._stop,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            GoTo,
            "~/go_to",
            self._go_to,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            StartTrajectory,
            "~/start_trajectory",
            self._start_trajectory,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            UploadTrajectory,
            "~/upload_trajectory",
            self._upload_trajectory,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            TakeoffWithVelocity,
            "~/takeoff_with_velocity",
            self._takeoff_with_velocity,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            LandWithVelocity,
            "~/land_with_velocity",
            self._land_with_velocity,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            Spiral,
            "~/spiral",
            self._spiral,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    @abstractmethod
    def set_group_mask(self, group_mask: float) -> None:
        """Gets called if a set_group_mask command is received from ros

        Args:
            group_mask (float): _description_
        """
        self.__warn_not_implemented("set_group_mask")

    @abstractmethod
    def takeoff(
        self,
        group_mask: int,
        height: float,
        yaw: float,
        use_current_yaw: bool,
        duration_seconds: float,
    ) -> None:
        """Gets called if a takeoff command is received from ros

        Args:
            group_mask (int): The crazyflies addressed
            height (float): The target height (absolute) in meters
            yaw (float): The target yaw in rad
            use_current_yaw (bool): Wheater yaw is valid or current yaw should be maintained
            duration_seconds (float): The time it should take to reach the height
        """
        self.__warn_not_implemented("takeoff")

    @abstractmethod
    def land(
        self,
        group_mask: int,
        height: float,
        yaw: float,
        use_current_yaw: bool,
        duration_seconds: float,
    ) -> None:
        """Gets called if a land command is received from ros

        Args:
            group_mask (int): _description_
            height (float): _description_
            yaw (float): _description_
            use_current_yaw (bool): _description_
            duration_seconds (float): _description_
        """
        self.__warn_not_implemented("land")

    @abstractmethod
    def stop(self, group_mask: int) -> None:
        """Gets called if a stop command is received from ros

        Stops the motors

        Args:
            group_mask (int): _description_
        """
        self.__warn_not_implemented("stop")

    @abstractmethod
    def go_to(
        self,
        group_mask: int,
        relative: bool,
        linear: bool,
        x: float,
        y: float,
        z: float,
        yaw: float,
        duration_seconds: float,
    ) -> None:
        """Gets called when a go_to command is received in ros

        Args:
            group_mask (int): _description_
            relative (bool): _description_
            linear (bool): _description_
            x (float): _description_
            y (float): _description_
            z (float): _description_
            yaw (float): _description_
            duration_seconds (float): _description_
        """
        self.__warn_not_implemented("go_to")

    @abstractmethod
    def start_trajectory(
        self,
        group_mask: int,
        trajectory_id: int,
        timescale: float,
        reversed: bool,
        relative: bool,
    ) -> None:
        self.__warn_not_implemented("start_trajectory")

    @abstractmethod
    def define_trajectory(self, trajectory_id: int, piece_offset: int) -> None:
        self.__warn_not_implemented("define_trajectory")

    @abstractmethod
    def takeoff_with_velocity(
        self,
        group_mask: int,
        height: float,
        height_is_relative: bool,
        yaw: float,
        use_current_yaw: bool,
        velocity: float,
    ) -> None:
        self.__warn_not_implemented("takeoff_with_velocity")

    @abstractmethod
    def land_with_velocity(
        self,
        group_mask: int,
        height: float,
        height_is_relative: bool,
        yaw: float,
        use_current_yaw: bool,
        velocity: float,
    ) -> None:
        self.__warn_not_implemented("land_with_velocity")

    @abstractmethod
    def spiral(
        self,
        group_mask: int,
        sidways: bool,
        clockwise: bool,
        phi: float,
        r0: float,
        rf: float,
        dz: float,
        duration_seconds: float,
    ) -> None:
        self.__warn_not_implemented("spiral")

    # Callback functions for ros2 topics:

    def _set_group_mask(self, msg: SetGroupMask) -> None:
        self.set_group_mask(msg.group_mask)

    def _takeoff(self, msg: Takeoff) -> None:
        duration_seconds = msg.duration.sec + msg.duration.nanosec * 1e-9
        self.takeoff(
            msg.group_mask, msg.height, msg.yaw, msg.use_current_yaw, duration_seconds
        )

    def _land(self, msg: Land) -> None:
        duration_seconds = msg.duration.sec + msg.duration.nanosec * 1e-9
        self.land(
            msg.group_mask, msg.height, msg.yaw, msg.use_current_yaw, duration_seconds
        )

    def _stop(self, msg: Stop) -> None:
        self.stop(msg.group_mask)

    def _go_to(self, msg: GoTo) -> None:
        duration_seconds = msg.duration.sec + msg.duration.nanosec * 1e-9
        self.go_to(
            msg.group_mask,
            msg.relative,
            msg.linear,
            msg.goal.x,
            msg.goal.y,
            msg.goal.z,
            msg.yaw,
            duration_seconds,
        )

    def _start_trajectory(self, msg: StartTrajectory) -> None:
        self.start_trajectory(
            msg.group_mask,
            msg.trajectory_id,
            msg.timescale,
            msg.reversed,
            msg.relative,
        )

    def _upload_trajectory(self, msg: UploadTrajectory) -> None:
        # TODO: Implementaion not complete
        self.define_trajectory(msg.trajectory_id, msg.piece_offset)

    def _takeoff_with_velocity(self, msg: TakeoffWithVelocity) -> None:
        self.takeoff_with_velocity(
            msg.group_mask,
            msg.height,
            msg.height_is_relative,
            msg.yaw,
            msg.use_current_yaw,
            msg.velocity,
        )

    def _land_with_velocity(self, msg: LandWithVelocity) -> None:
        self.land_with_velocity(
            msg.group_mask,
            msg.height,
            msg.height_is_relative,
            msg.yaw,
            msg.use_current_yaw,
            msg.velocity,
        )

    def _spiral(self, msg: Spiral) -> None:
        duration_seconds = msg.duration.sec + msg.duration.nanosec * 1e-9
        self.spiral(
            msg.group_mask,
            msg.sideways,
            msg.clockwise,
            msg.phi,
            msg.r0,
            msg.rf,
            msg.dz,
            duration_seconds,
        )

    def __warn_not_implemented(self, function_name: str) -> None:
        self.node.get_logger().warn(
            "There is no server-side implementation for {}!".format(function_name)
        )
