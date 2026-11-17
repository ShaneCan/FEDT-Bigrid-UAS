from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from crazyflie_interfaces.msg import (
    TrajectoryPolynomialPiece,
)

from crazyflie_interfaces.srv import (
    SetGroupMask,
    Takeoff,
    Land,
    Stop,
    GoTo,
    StartTrajectory,
    UploadTrajectory,
)

from typing import List


class HighLevelCommanderClient:
    """Send high level commands to the Crazyflie.
    For more information visit the official bitcraze website:
    https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/api/cflib/crazyflie/high_level_commander/

    These high level commands get processed onboard the Crazyflie. Polynomials are calculated in order to fly
    smooth trajectories between the commanded setpoints.
    Before switching from low-level to high-level commands the low-level command notify setpoints stop should be called.
    """

    def __init__(self, node: Node, prefix: str):
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10

        self.set_group_mask_client = node.create_client(
            SetGroupMask,
            prefix + "/set_group_mask",
            callback_group=callback_group,
        )

        self.takeoff_client = node.create_client(
            Takeoff,
            prefix + "/takeoff",
            callback_group=callback_group,
        )

        self.land_client = node.create_client(
            Land,
            prefix + "/land",
            callback_group=callback_group,
        )

        self.stop_client = node.create_client(
            Stop,
            prefix + "/stop",
            callback_group=callback_group,
        )

        self.goto_client = node.create_client(
            GoTo,
            prefix + "/go_to",
            callback_group=callback_group,
        )

        self.start_trajectory_client = node.create_client(
            StartTrajectory,
            prefix + "/start_trajectory",
            callback_group=callback_group,
        )

        self.upload_trajectory_client = node.create_client(
            UploadTrajectory,
            prefix + "/upload_trajectory",
            callback_group=callback_group,
        )

    def set_group_mask(self, group_mask: int):
        """Sets the group mask of the crazyflie

        Deprecated will be removed December 2024
        This can be used to split a swarm of Crazyflies into groups and then send high level commands via broadcasting messages.

        Args:
            group_mask (int): the group ID this CF belongs to
        """
        req = SetGroupMask.Request()
        req.group_mask = group_mask
        self.set_group_mask_client.call_async(req)

    def takeoff(
        self,
        target_height: float,
        duration_seconds: float,
        yaw: float = 0.0,
        group_mask: int = 0,
    ) -> None:
        """Vertical takeoff from current x-y position to given height (high-level)

        The Crazyflie will hover indefinetely after target_height is reached.

        Args:
            target_height (float): height to takeoff to (absolute) in meters
            duration_seconds (float): time it should take until target_height is reached in seconds
            yaw (float): Target orientation in radians
            use_current_yaw (bool): If true use ignore yaw parameter. Defaults to False.
            group_mask (int, optional): mask for which CFs this should apply to. Defaults to 0.
        """
        req = Takeoff.Request()
        req.group_mask = group_mask
        req.height = target_height
        req.yaw = yaw
        req.duration = self.__seconds_to_duration(duration_seconds)
        self.takeoff_client.call_async(req)

    def land(
        self,
        target_height: float,
        duration_seconds: float,
        yaw: float = 0.0,
        group_mask: int = 0,
    ) -> None:
        """Vertical landing from current x-y position to given height (high-level)

        The Crazyflie will hover indefinetely after target_height is reached.
        This should usually be followed by a stop command, but is not strictly required.

        Args:
            target_height (float): _description_
            duration_seconds (float): _description_
            yaw (float, optional): _description_. Defaults to 0.0.
            group_mask (int, optional): _description_. Defaults to 0.
        """
        req = Land.Request()
        req.group_mask = group_mask
        req.height = target_height
        req.yaw = yaw
        req.duration = self.__seconds_to_duration(duration_seconds)
        self.land_client.call_async(req)

    def stop(self, group_mask: int = 0) -> None:
        """Turns off the motors (high-level)

        Args:
            group_mask (int, optional): mask for which CFs this should apply to. Defaults to 0.
        """
        req = Stop.Request()
        req.group_mask = group_mask
        self.stop_client.call_async(req)

    def go_to(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        duration_seconds: float,
        relative: bool = False,
        group_mask: int = 0,
    ) -> None:
        """Move to x, y, z, yaw in duration_seconds amount of time (high-level)

        The Crazyflie will hover indefinetely afterwards.
        Calling goTo rapidly (> 1Hz) can cause instability. Consider using the cmd_position() setpoint command from
        generic_commander instead.

        Args:
            x (float): x-position of goal in meters
            y (float): y-position of goal in meters
            z (float): z-position of goal in meters
            yaw (float): target yaw in radians
            duration_seconds (float): Time in seconds it should take the CF to move to goal
            relative (bool, optional): If true the goal and yaw are interpreted as relative to current position. Defaults to False.
            linear (bool, optional): If true a linear interpolation is used for trajectory instead of a smooth polynomial . Defaults to False.
            group_mask (int, optional): mask for which CFs this should apply to. Defaults to 0.
        """
        req = GoTo.Request()
        req.group_mask = group_mask
        req.goal = Point(x=x, y=y, z=z)
        req.yaw = yaw
        req.relative = relative
        req.duration = self.__seconds_to_duration(duration_seconds)
        self.goto_client.call_async(req)

    def start_trajectory(
        self,
        trajectory_id: int,
        timescale: float = 1.0,
        reversed: bool = False,
        relative: bool = True,
        group_mask: int = 0,
    ) -> None:
        """Begin executing an uploaded trajectory (high-level)

        Args:
            trajectory_id (int): ID of trajectory as uploaded
            timescale (float, optional): Scales the duration of trajectory by this factor (if 2.0, trajectory twice as long). Defaults to 1.0.
            reversed (bool, optional): Execute the trajectory in reverse order. Defaults to False.
            relative (bool, optional): Of true, the position of the trajectory is shifted such that it begins at the current position setpoint. Defaults to True.
            group_mask (int, optional): mask for which Crazyflies this should apply to. Defaults to 0.
        """
        req = StartTrajectory.Request()
        req.group_mask = group_mask
        req.trajectory_id = trajectory_id
        req.timescale = timescale
        req.reversed = reversed
        req.relative = relative
        self.start_trajectory_client.call_async(req)

    def upload_trajectory(
        self, trajectory_id: int, piece_offset: int, pieces: List
    ) -> None:
        """Uploads a piecewise polynomial trajectory for later execution.

        This feature is currently not fully supported.

        TODO: How do poly pieces work, should we also use uav_trajectory.py
        See https://crazyswarm.readthedocs.io/en/latest/api.html or ask whoenig for further information.

        Args:
            trajectory_id (int): The trajectory id to reference in start_trajectory
            piece_offset (int): TODO
            pieces (List): TODO
        """
        req = UploadTrajectory.Request()
        req.trajectory_id = trajectory_id
        req.piece_offset = piece_offset
        for _piece in pieces:
            piece = TrajectoryPolynomialPiece()
            piece.duration = self.__seconds_to_duration(_piece.duration_seconds)
            piece.poly_x = _piece.poly_x
            piece.poly_y = _piece.poly_y
            piece.poly_z = _piece.poly_z
            piece.poly_yaw = _piece.poly_yaw
            req.pieces.append(piece)
        self.upload_trajectory_client.call_async(req)

    def __seconds_to_duration(self, seconds: float) -> Duration:
        i_seconds = int(seconds)
        fractional_seconds = seconds - i_seconds
        nanoseconds = int(fractional_seconds * 1_000_000_000)
        duration = Duration()
        duration.sec = i_seconds
        duration.nanosec = nanoseconds
        return duration
