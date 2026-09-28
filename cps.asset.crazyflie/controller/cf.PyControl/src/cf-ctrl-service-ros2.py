#!venv/bin/python
import logging
import sys
import time
from typing import Optional
from threading import Event
import threading
import argparse
import asyncio
import websockets
import json
import math

from flask import Flask, jsonify, request
from routes import drone_blueprint
from werkzeug.routing import BaseConverter, ValidationError
from statemachine import StateMachine, State, exceptions
from statemachine.contrib.diagram import DotGraphMachine

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from crazyflie_interfaces.msg import Hover, FullState, Position, TrajectoryPolynomialPiece
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

import cf_positioning
from cf_positioning import Point3D, PositionEstimationStrategy, KalmanEstimatePositionStrategy, StateEstimatePositionStrategy
from cf_drone_ops import CFOperationStrategy, HlCommanderCFOperationImpl, DebugLoggingCFOperationImpl
import cf_sm
from cf_logger import FileLogger
from scipy.spatial.transform import Rotation as R
from wall_following import WallFollowingConfig, WallFollowingOrchestrator
#################################################################################

# Global variables
websocketserver_started = threading.Event()
flask_started = threading.Event()
ISRUNNING = True

# Multi-drone support: log_values is a dict keyed by drone_id
log_values = {}

def LOG(msg: str):
    if DEBUG:
        print(msg)

#################################################################################

def create_arg_parser():
    parser = argparse.ArgumentParser(description='Crazyflie 2.x RESTful Drone Controller Service (ROS2 Version)')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='The host of the web server.')
    parser.add_argument('--port', type=int, default=5000, help='Port of the web server.')
    parser.add_argument('--drone_id', type=str, default='cf231', help='The ID of the drone.')
    parser.add_argument(
        '--dssim',
        action='store_true',
        help='Use ds-crazyflies simulation mode (takeoff/land/go_to/stop are topics, not services).',
    )
    parser.add_argument('--debug', action='store_true', help='Outputs many additional debug messages to the console.')
    parser.add_argument('--logging', action='store_true', help='Add a logger that writes CF states to a file.')
    ws_group = parser.add_argument_group('WebSocket settings')
    ws_group.add_argument('--wsendpoint', action='store_true', help='Add a websocket that publishes CF state.')
    ws_group.add_argument('--wshost', type=str, default='0.0.0.0', help='The host of the websocket server.')
    ws_group.add_argument('--wsport', type=int, default=8765, help='Port of the websocket server.')
    return parser

#################################################################################

## Flask Webserver
def start_flask_app(app, host, port):
    flask_started.set()
    # DEV
    # app.run(host=host, port=port)
    # PROD
    from waitress import serve
    serve(app, host=host, port=port)

async def send_pos_data(websocket, path, wsrate_ms=1000):
    while True:
        data = {
            "message": "crazyflie_position",
            "value": log_values,  # push the data of every drone
        }
        await websocket.send(json.dumps(data))
        await asyncio.sleep(wsrate_ms / 1000)

def start_websocket_server(host, port, wsrate_ms=1000):
    async def server():
        websocketserver_started.set()
        async def handler(websocket, path):
            await send_pos_data(websocket, path, wsrate_ms)
        async with websockets.serve(handler, host, port):
            print(f"WebSocket server started on ws://{host}:{port}")
            await asyncio.Future()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(server())

############## This class is the lowest layer: it talks to ROS2 directly and performs the actual hardware control. `controller` is an instance of it. ####################

class ROS2CrazyflieController(Node):
    def __init__(self, drone_id: str, dssim: bool = False):
        super().__init__('crazyflie_controller')
        self.drone_id = drone_id
        self.namespace = f'/{drone_id}'
        self.dssim = dssim
        self.executor: Optional[MultiThreadedExecutor] = None
        
        # Create the ROS2 publishers and subscribers
        self.hover_pub = self.create_publisher(Hover, f'{self.namespace}/cmd_hover', 10)
        self.full_state_pub = self.create_publisher(FullState, f'{self.namespace}/cmd_full_state', 10)
        self.trajectory_pub = self.create_publisher(TrajectoryPolynomialPiece, f'{self.namespace}/cmd_trajectory', 10)
        self.velocity_pub = self.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10)
        
        # Subscribe to the position information
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{self.namespace}/odom',
            self.odom_callback,
            10)
        
        # Subscribe to the scan data, for debugging
        from sensor_msgs.msg import LaserScan
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'{self.namespace}/scan',
            self.scan_callback,
            10)
        self.scan_ranges = [0.0, 0.0, 0.0, 0.0]
        self.last_scan_log_time = 0.0
        
        # Status subscription (assumes a Status message exists)
        try:
            from crazyflie_interfaces.msg import Status
            self.status_sub = self.create_subscription(
                Status,
                f'{self.namespace}/status',
                self.status_topic_callback,
                10)
        except ImportError:
            pass
        
        # In the ds-crazyflies simulator, takeoff/land/go_to/notify_setpoints_stop are topics, not services
        if self.dssim:
            from rosidl_runtime_py.utilities import get_message

            try:
                self._TakeoffMsgType = get_message("crazyflie_interfaces/msg/Takeoff")
                self._LandMsgType = get_message("crazyflie_interfaces/msg/Land")
                self._GoToMsgType = get_message("crazyflie_interfaces/msg/GoTo")
            except Exception as e:
                raise ImportError(
                    "Using --dssim requires the message types "
                    "`crazyflie_interfaces/msg/Takeoff`, `.../Land`, `.../GoTo`。\n"
                    "The `crazyflie_interfaces` in your environment does not appear to contain these msg types (only srv).\n"
                ) from e

            self.takeoff_pub = self.create_publisher(
                self._TakeoffMsgType, f"{self.namespace}/takeoff", 10
            )
            self.land_pub = self.create_publisher(
                self._LandMsgType, f"{self.namespace}/land", 10
            )
            self.go_to_pub = self.create_publisher(
                self._GoToMsgType, f"{self.namespace}/go_to", 10
            )
            self.stop_pub = self.create_publisher(
                Empty, f'{self.namespace}/notify_setpoints_stop', 10
            )
            self.takeoff_client = None
            self.land_client = None
            self.go_to_client = None
            self.stop_client = None
        else:
            # Real hardware / crazyswarm2: use services, resolved at runtime so the import does not fail in a dssim-only environment
            from rosidl_runtime_py.utilities import get_service

            try:
                self._TakeoffSrvType = get_service("crazyflie_interfaces/srv/Takeoff")
                self._LandSrvType = get_service("crazyflie_interfaces/srv/Land")
                self._GoToSrvType = get_service("crazyflie_interfaces/srv/GoTo")
                self._NotifySetpointsStopSrvType = get_service(
                    "crazyflie_interfaces/srv/NotifySetpointsStop"
                )
            except Exception as e:
                raise ImportError(
                    "Non---dssim mode requires the service types "
                    "`crazyflie_interfaces/srv/Takeoff`, `.../Land`, `.../GoTo`, "
                    "`.../NotifySetpointsStop`。\n"
                    "Make sure you have sourced a workspace that provides these srv types (for example the crazyswarm2 or mapping_demo overlay)."
                ) from e

            self.takeoff_client = self.create_client(
                self._TakeoffSrvType, f"{self.namespace}/takeoff"
            )
            self.land_client = self.create_client(
                self._LandSrvType, f"{self.namespace}/land"
            )
            self.go_to_client = self.create_client(
                self._GoToSrvType, f"{self.namespace}/go_to"
            )
            self.stop_client = self.create_client(
                self._NotifySetpointsStopSrvType, f"{self.namespace}/notify_setpoints_stop"
            )
        self.wall_following_orchestrator: Optional[WallFollowingOrchestrator] = None
        self.wall_following_config: Optional[WallFollowingConfig] = None
        
        self.current_position = Point3D(0, 0, 0)
        self.current_orientation = None  # Current quaternion
        self.is_flying = False
        self.battery = 0.0
        self.batteryState = 0
        
        
        self.get_logger().info(f'Crazyflie controller initialized for drone {drone_id}')
        
    def odom_callback(self, msg):
        lv = log_values.setdefault(self.drone_id, {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'battery': 0.0, 'batteryLevel': 0.0, 'batteryState': 0,
            'acc_x': 0.0, 'acc_y': 0.0, 'acc_z': 0.0,
        })
        # Write the position into log_values
        lv['x'] = msg.pose.pose.position.x
        lv['y'] = msg.pose.pose.position.y
        lv['z'] = msg.pose.pose.position.z
        
        # Important: also update self.current_position
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        
        # Keep the current quaternion for FullState control
        q = msg.pose.pose.orientation
        self.current_orientation = q  # keep the raw quaternion
        
        r = R.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        lv['roll'] = roll
        lv['pitch'] = pitch
        lv['yaw'] = yaw

    def scan_callback(self, msg):
        """Receives scan data and logs it periodically."""
        self.scan_ranges = list(msg.ranges)
        current_time = time.time()
        
        # Log the scan data once per second
        if current_time - self.last_scan_log_time > 1.0:
            # if len(self.scan_ranges) >= 4:
            #     self.get_logger().info(
            #         f'[{self.drone_id}] SCAN DATA -> back: {self.scan_ranges[0]:.3f}m, '
            #         f'right: {self.scan_ranges[1]:.3f}m, front: {self.scan_ranges[2]:.3f}m, '
            #         f'left: {self.scan_ranges[3]:.3f}m'
            #     )
            self.last_scan_log_time = current_time
    
    def status_topic_callback(self, msg):
        lv = log_values.setdefault(self.drone_id, {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'battery': 0.0, 'batteryLevel': 0.0, 'batteryState': 0,
            'acc_x': 0.0, 'acc_y': 0.0, 'acc_z': 0.0,
        })
        lv['battery'] = getattr(msg, 'battery_voltage', 0.0)
        lv['batteryLevel'] = getattr(msg, 'battery_level', 0.0)
        lv['batteryState'] = getattr(msg, 'pm_state', 0)
        if lv['battery'] < 3.2:
            print(f"[{self.drone_id}] ⚠️ Battery low! Consider landing soon.")
            
    def take_off(self, velocity=0.2):
        """Take-off command, asynchronous so it does not block the executor."""
        self.get_logger().info(f'Taking off drone {self.drone_id}')
        try:
            # ds-crazyflies simulation: publish topic once (not a service)
            if self.dssim:
                msg = self._TakeoffMsgType()
                msg.group_mask = 0
                msg.height = 0.5
                msg.duration.sec = 2
                msg.duration.nanosec = 0
                # yaw/use_current_yaw can keep their defaults (0/False) or be set explicitly
                msg.yaw = 0.0
                msg.use_current_yaw = False
                self.takeoff_pub.publish(msg)
                self.get_logger().info('Takeoff topic published successfully (dssim)')
                self.is_flying = True
                return True

            # Build the take-off request
            request = self._TakeoffSrvType.Request()
            request.height = 0.5  # take-off altitude
            request.duration.sec = 2  # duration
            request.duration.nanosec = 0
            
            # Wait for the service to become available
            if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('takeoff service not available')
                return False
                
            # Call the service asynchronously
            future = self.takeoff_client.call_async(request)
            
            # Important: do not use spin_once; wait for the future to complete directly
            timeout = 5.0  # 5 second timeout
            start_time = time.time()
            
            while (time.time() - start_time) < timeout:
                if future.done():
                    result = future.result()
                    if result is not None:
                        self.get_logger().info('Takeoff command sent successfully')
                        self.is_flying = True
                        return True
                    else:
                        self.get_logger().error('Failed to send takeoff command')
                        return False
                time.sleep(0.05)  # short sleep, to avoid busy-waiting
                
            self.get_logger().error('Takeoff command timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Error in take_off: {str(e)}')
            return False
            
    def land(self):
        """Landing command, asynchronous so it does not block the executor."""
        self.get_logger().info(f'Landing drone {self.drone_id}')
        try:
            # ds-crazyflies simulation: land is a topic; no need to send stop before land
            if self.dssim:
                msg = self._LandMsgType()
                msg.group_mask = 0
                msg.height = 0.0
                msg.duration.sec = 2
                msg.duration.nanosec = 0
                msg.yaw = 0.0
                msg.use_current_yaw = False
                self.land_pub.publish(msg)
                self.get_logger().info('Land topic published successfully (dssim)')
                self.is_flying = False
                return True

            # Build the landing request. NOTE: low-level control is stopped first.
            request = self._NotifySetpointsStopSrvType.Request() #which is indicating that we won't be sending low level commands anymore
            self.stop_client.call_async(request)
            request = self._LandSrvType.Request()
            request.height = 0.0  # landing altitude
            request.duration.sec = 2  # duration
            request.duration.nanosec = 0
            
            # Wait for the service to become available
            if not self.land_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('land service not available')
                return False
                
            # Call the service asynchronously
            future = self.land_client.call_async(request)
            
            # Important: do not use spin_once; wait for the future to complete directly
            timeout = 5.0  # 5 second timeout
            start_time = time.time()
            
            while (time.time() - start_time) < timeout:
                if future.done():
                    result = future.result()
                    if result is not None:
                        self.get_logger().info('Land command sent successfully')
                        self.is_flying = False
                        return True
                    else:
                        self.get_logger().error('Failed to send land command')
                        return False
                time.sleep(0.05)  # short sleep, to avoid busy-waiting
                
            self.get_logger().error('Land command timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Error in land: {str(e)}')
            return False
            
    # def stop(self):
    #     """Stop command, synchronous version."""
    #     self.get_logger().info(f'Stopping drone {self.drone_id}')
    #     try:
    #         # Build the stop request
    #         request = NotifySetpointsStop.Request()
            
    #         # Wait for the service to become available
    #         if not self.stop_client.wait_for_service(timeout_sec=1.0):
    #             self.get_logger().error('stop service not available')
    #             return False
                
    #         # Call the service synchronously
    #         future = self.stop_client.call_async(request)
            
    #         # Non-blocking poll with a timeout
    #         timeout = 5.0  # 5 second timeout
    #         start_time = time.time()
            
    #         while (time.time() - start_time) < timeout:
    #             rclpy.spin_once(self, timeout_sec=0.1)
    #             if future.done():
    #                 result = future.result()
    #                 if result is not None:
    #                     self.get_logger().info('Stop command sent successfully')
    #                     return True
    #                 else:
    #                     self.get_logger().error('Failed to send stop command')
    #                     return False
    #             time.sleep(0.1)
                
    #         self.get_logger().error('Stop command timed out')
    #         return False
    #     except Exception as e:
    #         self.get_logger().error(f'Error in stop: {str(e)}')
    #         return False
            
    def go_to(self, x, y, z, velocity=0.1): # velocity!!!!!!
        """Navigates to the given position, asynchronous so it does not block the executor."""
        self.get_logger().info(f'Navigating drone {self.drone_id} to ({x}, {y}, {z})')
        try:
            # ds-crazyflies simulation: publish topic once (not a service)
            if self.dssim:
                msg = self._GoToMsgType()
                msg.group_mask = 0
                msg.relative = False
                msg.linear = False
                msg.goal.x = float(x)
                msg.goal.y = float(y)
                msg.goal.z = float(z)
                msg.yaw = 0.0
                msg.duration.sec = 10
                msg.duration.nanosec = 0
                self.go_to_pub.publish(msg)
                self.get_logger().info('GoTo topic published successfully (dssim)')
                return True

            # Build the navigation request
            request = self._GoToSrvType.Request()
            request.goal.x = x
            request.goal.y = y
            request.goal.z = z

            # Originally the speed could not be controlled
            request.duration.sec = 10 # longer duration lowers the speed
            request.duration.nanosec = 0

            # Wait for the service to become available
            if not self.go_to_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('go_to service not available')
                return False
                
            # Call the service asynchronously
            future = self.go_to_client.call_async(request)
            
            # Important: do not use spin_once; wait for the future to complete directly
            timeout = 5.0  # 5 second timeout
            start_time = time.time()
            
            while (time.time() - start_time) < timeout:
                if future.done():
                    result = future.result()
                    if result is not None:
                        self.get_logger().info('Go to command sent successfully')
                        return True
                    else:
                        self.get_logger().error('Failed to send go to command')
                        return False
                time.sleep(0.05)  # short sleep, to avoid busy-waiting
                
            self.get_logger().error('Go to command timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Error in go_to: {str(e)}')
            return False

#################################################################################
    def hover_at_position(self, x=None, y=None, z=None, yaw=0.0, emergency_stop=True):
        """Stops immediately at the given altitude and hovers, by continuously publishing Hover messages.
        X/Y are no longer forced to a position; only zero velocity is sent."""
        try:
            # Use the supplied coordinates if given, otherwise the current position
            if x is None or y is None or z is None:
                x = self.current_position.x
                y = self.current_position.y
                z = self.current_position.z
                self.get_logger().info(f'HOVER: drone {self.drone_id} at current position ({x:.3f}, {y:.3f}, {z:.3f})')
            else:
                self.get_logger().info(f'HOVER: drone {self.drone_id} at specified position ({x:.3f}, {y:.3f}, {z:.3f})')
            
            # Enforce a safe altitude
            safe_z = max(float(z), 0.5)  # minimum safe altitude of 0.5 m
            if safe_z != z:
                self.get_logger().warning(f'Adjusting hover height from {z:.3f}m to safe height {safe_z:.3f}m')
                z = safe_z
            
            # Record the target altitude (Hover controls z)
            self.hover_z = float(z)

            # Stop every timer
            if hasattr(self, 'move_timer') and self.move_timer is not None:
                self.get_logger().info(f'Stopping previous move timer')
                self.move_timer.destroy()
                self.move_timer = None
            
            if hasattr(self, 'hover_timer') and self.hover_timer is not None:
                self.hover_timer.destroy()
                self.hover_timer = None

            # Create a timer that keeps sending the hover command at 50 Hz
            self.hover_timer = self.create_timer(0.02, self._hover_timer_callback)
            
            # Send one Hover command immediately, to avoid a control gap
            msg = Hover()
            msg.vx = 0.0
            msg.vy = 0.0
            msg.yaw_rate = 0.0
            msg.z_distance = self.hover_z
            self.hover_pub.publish(msg)
            
            self.get_logger().info(f'Hover started at position ({x:.3f}, {y:.3f}, {z:.3f}) with continuous control')
            return True
        except Exception as e:
            self.get_logger().error(f'Error in hover_at_position: {str(e)}')
            return False
    
    def _hover_timer_callback(self):
        """Hover timer callback: keeps sending Hover commands (zero velocity, fixed altitude)."""
        if hasattr(self, 'hover_z'):
            msg = Hover()
            msg.vx = 0.0
            msg.vy = 0.0
            msg.yaw_rate = 0.0
            msg.z_distance = self.hover_z
            self.hover_pub.publish(msg)

    def move_to_position_realtime(self, x, y, z, velocity=0.2, timeout=10.0, skip_stop=False):
        """Moves to the given position in real time, publishing velocity (vx, vy) via Hover messages while holding the target altitude z."""
        try:
            self.get_logger().info(f'Moving drone {self.drone_id} to ({x}, {y}, {z}) in realtime with velocity={velocity}')
            
            # Short wait, so the position data is up to date
            time.sleep(0.05)
            
            # Get the current position
            start_x, start_y, start_z = self.current_position.x, self.current_position.y, self.current_position.z
            
            # Compute the movement distance
            dx = x - start_x
            dy = y - start_y
            dz = z - start_z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            self.get_logger().info(f'Distance to target: {distance:.3f}m, start=({start_x:.3f},{start_y:.3f},{start_z:.3f})')
            
            if distance < 0.05:  # if the target is very close, just hover
                self.get_logger().info(f'Target very close (distance: {distance:.3f}m), using hover instead of movement')
                return self.hover_at_position(x, y, z, emergency_stop=False)
            
            # Compute the motion parameters (speed capped at 0.15 m/s)
            max_velocity = min(velocity, 0.2)
            start_time = time.time()
            self.get_logger().info(f'Max velocity: {max_velocity:.3f}m/s')
            
            # Initialise the movement state (under Hover control only the target and speed are needed)
            self.move_start_time = start_time
            self.move_start_pos = (start_x, start_y, start_z)
            self.move_target_pos = (x, y, z)
            self.move_max_velocity = max_velocity
            self.move_timeout = timeout
            self.move_completed = False
            
            # Now it is safe to stop the previous timer
            if hasattr(self, 'hover_timer') and self.hover_timer is not None:
                self.get_logger().info(f'Stopping hover timer before movement')
                self.hover_timer.destroy()
                self.hover_timer = None
            
            if hasattr(self, 'move_timer') and self.move_timer is not None:
                self.get_logger().info(f'Stopping previous move timer')
                self.move_timer.destroy()
                self.move_timer = None
            
            # Create the new movement timer immediately, for a seamless switch
            self.move_timer = self.create_timer(0.02, self._move_timer_callback)
            
            # Run the movement callback once immediately, to avoid a control gap
            try:
                self._move_timer_callback()
            except Exception as e:
                self.get_logger().warning(f'Immediate move callback failed: {str(e)}')
            
            self.get_logger().info(f'Movement started with continuous control')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error in move_to_position_realtime: {str(e)}')
            return False
    
    def _move_timer_callback(self):
        """Movement timer callback: publishes a Hover velocity command along the direction from the current position to the target."""
        try:
            if not hasattr(self, 'move_start_time') or self.move_completed:
                return
            
            current_time = time.time()
            elapsed_time = current_time - self.move_start_time
            
            # Check for a timeout
            if elapsed_time > self.move_timeout:
                self.get_logger().warning(f'Movement timeout after {elapsed_time:.2f}s, stopping')
                self.move_completed = True
                # Stop the movement timer
                if hasattr(self, 'move_timer') and self.move_timer is not None:
                    self.move_timer.destroy()
                    self.move_timer = None
                # Hover at the current position
                self.hover_at_position()
                return
            
            # Check the current position
            current_x = self.current_position.x
            current_y = self.current_position.y
            current_z = self.current_position.z
            
            # Compute the remaining distance (in the horizontal plane only)
            remaining_dx = self.move_target_pos[0] - current_x
            remaining_dy = self.move_target_pos[1] - current_y
            remaining_distance_xy = math.hypot(remaining_dx, remaining_dy)
            
            # Close to the target: stop moving and hover at the target altitude
            if remaining_distance_xy < 0.1:
                self.get_logger().info(f'Reached target (distance_xy: {remaining_distance_xy:.3f}m), stopping movement')
                self.move_completed = True
                if hasattr(self, 'move_timer') and self.move_timer is not None:
                    self.move_timer.destroy()
                    self.move_timer = None
                self.hover_at_position(z=self.move_target_pos[2])
                return
            
            # Compute the velocity direction and clamp it
            if remaining_distance_xy > 0:
                unit_x = remaining_dx / remaining_distance_xy
                unit_y = remaining_dy / remaining_distance_xy
            else:
                unit_x = 0.0
                unit_y = 0.0
            
            # The closer to the target, the lower the speed, down to a minimum threshold
            speed = min(self.move_max_velocity, max(0.1, remaining_distance_xy))
            vx = unit_x * speed
            vy = unit_y * speed
            
            # Publish the Hover velocity command, holding the target altitude
            msg = Hover()
            msg.vx = float(vx)
            msg.vy = float(vy)
            msg.yaw_rate = 0.0
            msg.z_distance = float(self.move_target_pos[2])
            self.hover_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in _move_timer_callback: {str(e)}')
            self.move_completed = True
            # Stop the movement timer
            if hasattr(self, 'move_timer') and self.move_timer is not None:
                self.move_timer.destroy()
                self.move_timer = None
            # Hover at the current position
            self.hover_at_position()
    

    def start_wall_following(self, config: WallFollowingConfig) -> bool:
        """Starts wall-following control."""
        if self.executor is None:
            self.get_logger().error('Executor not configured, cannot start wall following')
            return False
        if self.wall_following_orchestrator is None:
            self.wall_following_orchestrator = WallFollowingOrchestrator(self.executor)
        success = self.wall_following_orchestrator.start(config, logger=self.get_logger())
        if success:
            self.wall_following_config = config
        return success

    def stop_wall_following(self) -> bool:
        if self.wall_following_orchestrator is None:
            return True
        success = self.wall_following_orchestrator.stop(logger=self.get_logger())
        if success:
            self.wall_following_config = None
        return success

    def set_executor(self, executor):
        self.executor = executor
        executor.add_node(self)

    def stop(self):
        """Stops every control command, including streaming setpoints, without blocking the executor."""
        self.get_logger().info(f'Stopping drone {self.drone_id} (including FullState streaming)')
        try:
            # Stop wall following first, if it is running
            if self.wall_following_orchestrator is not None:
                self.get_logger().info('Stopping wall following before stopping controller')
                self.stop_wall_following()
            
            # Stop all local repeating timers first, so no further Hover/FullState messages are published
            try:
                if hasattr(self, 'hover_timer') and self.hover_timer is not None:
                    self.get_logger().info('Destroying hover_timer')
                    self.hover_timer.destroy()
                    self.hover_timer = None
                if hasattr(self, 'move_timer') and self.move_timer is not None:
                    self.get_logger().info('Destroying move_timer')
                    self.move_timer.destroy()
                    self.move_timer = None
                if hasattr(self, 'land_timer') and self.land_timer is not None:
                    self.get_logger().info('Destroying land_timer')
                    self.land_timer.destroy()
                    self.land_timer = None
            except Exception as e:
                self.get_logger().warning(f'Error destroying timers: {str(e)}')

            # Clear the real-time control target so the callbacks do not use it again
            for attr in ['hover_z', 'move_start_time', 'move_start_pos', 'move_target_pos', 'move_completed']:
                if hasattr(self, attr):
                    try:
                        delattr(self, attr)
                    except Exception:
                        pass

            # ds-crazyflies simulation: stop is a std_msgs/Empty topic
            if self.dssim:
                self.stop_pub.publish(Empty())
                self.get_logger().info('Stop topic published successfully (dssim)')
                return True

            if not self.stop_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning('NotifySetpointsStop service not available')
                return True  # streaming has been stopped even if the service is unavailable
                
            request = self._NotifySetpointsStopSrvType.Request()
            future = self.stop_client.call_async(request)
            
            # Important: do not use spin_once; wait for the future to complete directly
            timeout = 2.0
            start_time = time.time()
            
            while (time.time() - start_time) < timeout:
                if future.done():
                    result = future.result()
                    if result is not None:
                        self.get_logger().info('Stop command sent successfully')
                        return True
                    else:
                        self.get_logger().error('Failed to send stop command')
                        return False
                time.sleep(0.05)  # short sleep, to avoid busy-waiting
                
            self.get_logger().warning('Stop command timed out')
            return True  # streaming has been stopped, so return True even if the service timed out
        except Exception as e:
            self.get_logger().error(f'Error in stop: {str(e)}')
            return False

#################################################################################

def print_log_values():
    for drone_id, lv in log_values.items():
        print(
            f"[{drone_id}] "
            f"X: {lv['x']:.2f} Y: {lv['y']:.2f} Z: {lv['z']:.2f} | "
            f"Roll: {lv['roll']:.2f} Pitch: {lv['pitch']:.2f} Yaw: {lv['yaw']:.2f} | "
            f"Battery: {lv['battery']:.2f}V State: {'Charging' if lv['batteryState'] else 'Discharging'}"
        )

def main():
    parser = create_arg_parser()
    args = parser.parse_args()
    global DEBUG
    DEBUG = args.debug
    
    # Set the log level
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger(__name__)
    
    # Add the error-handling decorator
    class AppWrapper(Flask):
        def handle_exception(self, e):
            """Overrides the default exception handling."""
            logger.error(f"Unhandled exceptions: {str(e)}", exc_info=True)
            return jsonify({"error": str(e)}), 500
            
        def handle_user_exception(self, e):
            """Overrides the user exception handling."""
            if isinstance(e, exceptions.TransitionNotAllowed):
                logger.error(f"state transition error: {str(e)}", exc_info=True)
                return jsonify({"error": str(e)}), 400
            return super().handle_user_exception(e)
    
    try:
        # Initialise ROS2
        logger.info("initializing ROS2...")
        rclpy.init(args=None)

        #！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        # Initialise ROS2CrazyflieController; `controller` is the instance of that ROS2 node
        # Its behaviour is defined by the class above. ROS2CrazyflieController extends Node.
        controller = ROS2CrazyflieController(args.drone_id, dssim=args.dssim)
        #！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！

        # Run ROS2 on a separate thread
        executor = MultiThreadedExecutor()
        controller.set_executor(executor)
        ros_thread = threading.Thread(target=lambda: executor.spin())
        ros_thread.daemon = True
        ros_thread.start()
        
        # Create the state-machine instance
        logger.info("creating state machine instance...")
        drone = cf_sm.StateMachineDrone(args.drone_id, debug=args.debug)
        
        # Set the drone operation strategy implementation
        logger.info("setting UAV operation strategy...")
        from cf_drone_ops import HlCommanderCFOperationImpl
        
        #！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        # Set the drone operation strategy implementation, passing the controller to HlCommanderCFOperationImpl
        drone.set_uavOpsImpl(HlCommanderCFOperationImpl(scf=None, controller=controller, debug=DEBUG))
        #！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        
        # Initialise the state machine
        logger.info("initializing state machine...")
        try:
            drone.install()  # INSTALLED -> RESOLVED
            logger.info("state machine installed")
            time.sleep(0.2)  # short delay
            
            drone.start()    # RESOLVED -> STARTING
            logger.info("state machine started")
            time.sleep(0.2)  # short delay
            
            drone.initialize()  # STARTING -> ACTIVE
            logger.info("state machine initialized")
            logger.info(f"state machine current state: {drone.get_current_state()}")
            
            # Wait for the state machine to finish initialising
            time.sleep(0.5)
            
        except Exception as e:
            logger.error(f"initializing state machine failed: {str(e)}", exc_info=True)
            raise
        
        # Create the Flask application
        logger.info("creating Flask app...")
        app = AppWrapper(__name__)
        app.register_blueprint(drone_blueprint)
        app.config['DRONE'] = drone
        
        # Configure the static-file directory
        app.static_folder = '../webview'
        app.static_url_path = ''
        
        # Add error handling
        @app.errorhandler(500)
        def internal_error(error):
            logger.error(f"server internal error: {str(error)}", exc_info=True)
            return jsonify({"error": str(error)}), 500
        
        @app.errorhandler(Exception)
        def handle_exception(e):
            logger.error(f"unhandled exception: {str(e)}", exc_info=True)
            return jsonify({"error": str(e)}), 500
        
        # Start the Flask server
        logger.info("starting Flask server...")
        flask_thread = threading.Thread(
            target=start_flask_app,
            args=(app, args.host, args.port)
        )
        flask_thread.daemon = True
        flask_thread.start()
        
        # Wait for the Flask server to start
        flask_started.wait()
        logger.info("Flask WebServer started [OK]")
        
        # Start the WebSocket server, if enabled
        if args.wsendpoint:
            logger.info("starting WebSocket server...")
            ws_thread = threading.Thread(
                target=start_websocket_server,
                args=(args.wshost, args.wsport)
            )
            ws_thread.daemon = True
            ws_thread.start()
            websocketserver_started.wait()
            logger.info("WebSocket server started [OK]")
        
        # Main loop
        logger.info("service started, press Ctrl+C to terminate...")
        try:
            # Keep the main thread alive
            while ISRUNNING:
                #print_log_values()  # log the information of every drone
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("received termination signal, closing...")
            
    except Exception as e:
        logger.error(f"program running error: {str(e)}", exc_info=True)
    finally:
        logger.info("cleaning up resources...")
        try:
            controller.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main() 