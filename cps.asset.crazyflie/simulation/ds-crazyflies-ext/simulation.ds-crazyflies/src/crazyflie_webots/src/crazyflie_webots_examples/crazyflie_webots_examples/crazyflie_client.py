import rclpy
from rclpy.node import Node
from crazyflie_interfaces.srv import Takeoff, Land, GoTo
import numpy as np

import threading


class Crazyflie(Node):

    def __init__(self, cf_id: int):
        rclpy.init()
        super().__init__("crazyflie_client")

        prefix = "cf" + str(cf_id)

        self.takeoff_client = self.create_client(Takeoff, prefix + "/takeoff")
        self.land_client = self.create_client(Land, prefix + "/land")
        self.go_to_client = self.create_client(GoTo, prefix + "/go_to")

        # Spin in a separate thread
        self.thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.thread.start()

    def __del__(self):
        self.thread.join()
        rclpy.shutdown()

    def goto(self, position: list[float], secs: int):
        req = GoTo.Request()
        req.relative = False
        req.goal.x = position[0]
        req.goal.y = position[1]
        req.goal.z = position[2]
        req.yaw = 0.0
        req.duration.sec = secs
        self.go_to_client.call_async(req)

    def takeoff(self, height: float, secs: int):
        req = Takeoff.Request()
        req.height = height
        req.yaw = 0.0
        req.duration.sec = secs
        self.takeoff_client.call_async(req)

    def land(self, secs: int):
        msg = Land.Request()
        msg.height = 0.0
        msg.yaw = 0.0
        msg.duration.sec = secs
        self.land_client.call_async(msg)
