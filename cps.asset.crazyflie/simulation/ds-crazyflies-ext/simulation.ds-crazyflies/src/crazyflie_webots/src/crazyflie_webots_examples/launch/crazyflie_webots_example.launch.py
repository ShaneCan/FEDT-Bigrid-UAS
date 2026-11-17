from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():
    gateway = Node(
        package="crazyflie_webots_gateway",
        executable="gateway",
        output="screen",
    )

    spawn_crazyflie = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/crazyflie_webots_gateway/add_crazyflie  ",
                "crazyflie_webots_gateway_interfaces/srv/WebotsCrazyflie ",
                '"{id: 0}"',
            ]
        ],
        shell=True,
    )

    return LaunchDescription(
        [
            gateway,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=gateway,
                    on_start=[
                        LogInfo(msg="Gateway started, spawning crazyflie..."),
                        spawn_crazyflie,
                    ],
                )
            ),
        ]
    )
