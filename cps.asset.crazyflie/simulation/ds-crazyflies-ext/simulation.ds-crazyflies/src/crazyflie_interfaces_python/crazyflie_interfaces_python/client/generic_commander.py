from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Vector3, Pose, Twist, Point, Quaternion
from crazyflie_interfaces.msg import (
    VelocityWorld,
    Hover,
    FullState,
    Position,
)

from crazyflie_interfaces.srv import NotifySetpointsStop

from typing import List


class GenericCommanderClient:
    """Send control setpoints to the Crazyflie.
    For more information on the functionality of the setpoints visit the official bitcraze website:
    https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/api/cflib/crazyflie/commander/

    This commander sends low-level setpoints directly to the PID/Mellinger controller of the Crazyflie.
    Sending setpoints to far from current state can crash the crazyflie.
    All low-level commands must be sent out regulary (>= 5Hz) in order not to trigger fail safe mechanisms in the Crazyflie.
    """

    def __init__(self, node: Node, prefix: str):
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10  # Should this be passed?

        self.notify_setpoints_stop_client = node.create_client(
            NotifySetpointsStop,
            prefix + "/notify_setpoints_stop",
            callback_group=callback_group,
        )

        self.velocity_world_publisher = node.create_publisher(
            VelocityWorld,
            prefix + "/cmd_vel",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.hover_publisher = node.create_publisher(
            Hover,
            prefix + "/cmd_hover",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.full_state_publisher = node.create_publisher(
            FullState,
            prefix + "/cmd_full_state",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.position_publisher = node.create_publisher(
            Position,
            prefix + "/cmd_position",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    def notify_setpoints_stop(
        self, remain_valid_milliseconds: int = 100, group_mask: int = 0
    ) -> None:
        """Send a notify setpoints command
        Informs that streaming low-level setpoint packets are about to stop.
        A common use case is to send this after the last low-level setpoint is sent but before the first high-level setpoint is sent.

        Args:
            remain_valid_milliseconds (int, optional): Artefact of pull-based hl-commander architecture no longer needed. Defaults to 100.
            group_mask (int, optional): The group this should apply to. Deprecated Dec2024. Defaults to 0.
        """
        req = NotifySetpointsStop.Request()
        req.group_mask = group_mask
        req.remain_valid_millisecs = remain_valid_milliseconds
        self.notify_setpoints_stop_client.call_async(req)

    def cmd_velocity_world(self, velocity: List[float], yawrate: float = 0.0) -> None:
        """Send a velocity world setpoint to controller (low-level)

        This control-command does not work with Mellinger controller.
        Switch to PID controller if you are using this setpoint command.

        Args:
            velocity (List[float]): Velocity in m/s in world coordinates (vx, vy, vz).
            yawrate (float, optional): Angular velocity in degrees/s . Defaults to 0.0.
        """
        msg = VelocityWorld()
        x, y, z = velocity
        msg.vel = Vector3(x, y, z)
        msg.yaw_rate = yawrate
        self.velocity_world_publisher.publish(msg)

    def cmd_hover(
        self,
        z_distance: float,
        velocity_x: float = 0.0,
        velocity_y: float = 0.0,
        yawrate: float = 0.0,
    ) -> None:
        """Send a hover setpoint to controller (low-level)

        Sets the crazyflie absolute height and velocity in body coordinate system.

        Args:
            z_distance (float): Absolute height in m
            velocity_x (float, optional): Velocity in x direction (body coordinates) in m/s. Defaults to 0.0.
            velocity_y (float, optional): Velocity in y direction (body coordinates) in m/s. Defaults to 0.0.
            yawrate (float, optional): Angular velocity in deg/s. Defaults to 0.0.
        """
        msg = Hover()
        msg.z_distance = z_distance
        msg.vx = velocity_x
        msg.vy = velocity_y
        msg.yaw_rate = yawrate
        self.hover_publisher.publish(msg)

    def cmd_full_state(
        self,
        position: List[float],
        velocity: List[float],
        acceleration: List[float],
        orientation: List[float],
        angular_rate: List[float],
    ) -> None:
        """Send a full-state setpoint to controller (low-level)

        Can be used for aggressive maneuvers

        Args:
            position (List[float]): Position [x, y, z] in m
            velocity (List[float]): Velocity [vx, vy, vz] in m/s
            acceleration (List[float]): Acceleration [ax, ay, az] in m/s^2
            orientation (List[float]): Orientation in quaternion components [qx, qy, qz, qw]
            omega (List[float]): Angular rates [Rollrate, Pitchrate, Yawrate] in degrees/s (TODO: rad/s ???)
        """
        pose = Pose()
        (pose.position.x, pose.position.y, pose.position.z) = position
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = orientation

        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = velocity
        twist.angular.x, twist.angular.y, twist.angular.z = angular_rate

        acceleration = Vector3()
        acceleration.x, acceleration.y, acceleration.z = acceleration

        msg = FullState()
        msg.pose = pose
        msg.twist = twist
        msg.acc = acceleration
        self.full_state_publisher.publish(msg)

    def cmd_position(self, position: List[float], yaw: float) -> None:
        """Send a position setpoint to controller (low-level)

        Args:
            position (List[float]): Position [x, y, z] in m
            yaw (float): Orientation in degrees
        """
        msg = Position()
        msg.x, msg.y, msg.z = position
        msg.yaw = yaw
        self.position_publisher.publish(msg)
