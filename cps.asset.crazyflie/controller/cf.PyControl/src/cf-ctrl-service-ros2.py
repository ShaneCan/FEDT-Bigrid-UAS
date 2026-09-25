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

# 全局变量
websocketserver_started = threading.Event()
flask_started = threading.Event()
ISRUNNING = True

# 多机支持：log_values为dict，key为drone_id
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
            "value": log_values,  # 推送所有无人机数据
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

##############↓↓↓↓该类是最底层实现，直接与ROS2系统通信，负责实际的硬件控制，controller是该类的实例↓↓↓↓####################

class ROS2CrazyflieController(Node):
    def __init__(self, drone_id: str, dssim: bool = False):
        super().__init__('crazyflie_controller')
        self.drone_id = drone_id
        self.namespace = f'/{drone_id}'
        self.dssim = dssim
        self.executor: Optional[MultiThreadedExecutor] = None
        
        # 创建ROS2发布者和订阅者
        self.hover_pub = self.create_publisher(Hover, f'{self.namespace}/cmd_hover', 10)
        self.full_state_pub = self.create_publisher(FullState, f'{self.namespace}/cmd_full_state', 10)
        self.trajectory_pub = self.create_publisher(TrajectoryPolynomialPiece, f'{self.namespace}/cmd_trajectory', 10)
        self.velocity_pub = self.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10)
        
        # 订阅位置信息
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{self.namespace}/odom',
            self.odom_callback,
            10)
        
        # 订阅scan数据用于调试
        from sensor_msgs.msg import LaserScan
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'{self.namespace}/scan',
            self.scan_callback,
            10)
        self.scan_ranges = [0.0, 0.0, 0.0, 0.0]
        self.last_scan_log_time = 0.0
        
        # 状态订阅（假设有Status消息）
        try:
            from crazyflie_interfaces.msg import Status
            self.status_sub = self.create_subscription(
                Status,
                f'{self.namespace}/status',
                self.status_topic_callback,
                10)
        except ImportError:
            pass
        
        # ds-crazyflies 模拟器中 takeoff/land/go_to/notify_setpoints_stop 是 topic，不是 service
        if self.dssim:
            from rosidl_runtime_py.utilities import get_message

            try:
                self._TakeoffMsgType = get_message("crazyflie_interfaces/msg/Takeoff")
                self._LandMsgType = get_message("crazyflie_interfaces/msg/Land")
                self._GoToMsgType = get_message("crazyflie_interfaces/msg/GoTo")
            except Exception as e:
                raise ImportError(
                    "启用 --dssim 需要消息类型 "
                    "`crazyflie_interfaces/msg/Takeoff`, `.../Land`, `.../GoTo`。\n"
                    "你当前环境的 `crazyflie_interfaces` 似乎不包含这些 msg（只包含 srv）。\n"
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
            # 真机 / crazyswarm2：使用 service（运行时动态解析，避免在 dssim-only 环境中 import 失败）
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
                    "非 --dssim 模式需要 service 类型 "
                    "`crazyflie_interfaces/srv/Takeoff`, `.../Land`, `.../GoTo`, "
                    "`.../NotifySetpointsStop`。\n"
                    "请确认已 source 包含这些 srv 的工作区（例如 crazyswarm2 / mapping_demo 的 overlay）。"
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
        self.current_orientation = None  # 保存当前四元数
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
        # 更新位置信息到log_values
        lv['x'] = msg.pose.pose.position.x
        lv['y'] = msg.pose.pose.position.y
        lv['z'] = msg.pose.pose.position.z
        
        # ★ 关键修复：同时更新 self.current_position
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        
        # 保存当前四元数用于 FullState 控制
        q = msg.pose.pose.orientation
        self.current_orientation = q  # 保存原始四元数
        
        r = R.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        lv['roll'] = roll
        lv['pitch'] = pitch
        lv['yaw'] = yaw

    def scan_callback(self, msg):
        """接收scan数据并定期输出"""
        self.scan_ranges = list(msg.ranges)
        current_time = time.time()
        
        # 每秒输出一次scan数据
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
        """起飞命令 - 异步版本（不阻塞executor）"""
        self.get_logger().info(f'Taking off drone {self.drone_id}')
        try:
            # ds-crazyflies simulation: publish topic once (not a service)
            if self.dssim:
                msg = self._TakeoffMsgType()
                msg.group_mask = 0
                msg.height = 0.5
                msg.duration.sec = 2
                msg.duration.nanosec = 0
                # yaw/use_current_yaw 可保持默认值(0/False)，也可显式设置
                msg.yaw = 0.0
                msg.use_current_yaw = False
                self.takeoff_pub.publish(msg)
                self.get_logger().info('Takeoff topic published successfully (dssim)')
                self.is_flying = True
                return True

            # 创建起飞请求
            request = self._TakeoffSrvType.Request()
            request.height = 0.5  # 起飞高度
            request.duration.sec = 2  # 持续时间
            request.duration.nanosec = 0
            
            # 等待服务可用
            if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('takeoff service not available')
                return False
                
            # 异步调用服务
            future = self.takeoff_client.call_async(request)
            
            # ★ 关键修复：不使用spin_once，直接等待future完成
            timeout = 5.0  # 5秒超时
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
                time.sleep(0.05)  # 短暂等待，避免占用CPU
                
            self.get_logger().error('Takeoff command timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Error in take_off: {str(e)}')
            return False
            
    def land(self):
        """降落命令 - 异步版本（不阻塞executor）"""
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

            # 创建降落请求  ！！！！！！！！！！注意这里先停止了低级控制！！！！！！！！！！
            request = self._NotifySetpointsStopSrvType.Request() #which is indicating that we won't be sending low level commands anymore
            self.stop_client.call_async(request)
            request = self._LandSrvType.Request()
            request.height = 0.0  # 降落高度
            request.duration.sec = 2  # 持续时间
            request.duration.nanosec = 0
            
            # 等待服务可用
            if not self.land_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('land service not available')
                return False
                
            # 异步调用服务
            future = self.land_client.call_async(request)
            
            # ★ 关键修复：不使用spin_once，直接等待future完成
            timeout = 5.0  # 5秒超时
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
                time.sleep(0.05)  # 短暂等待，避免占用CPU
                
            self.get_logger().error('Land command timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Error in land: {str(e)}')
            return False
            
    # def stop(self):
    #     """停止命令 - 同步版本"""
    #     self.get_logger().info(f'Stopping drone {self.drone_id}')
    #     try:
    #         # 创建停止请求
    #         request = NotifySetpointsStop.Request()
            
    #         # 等待服务可用
    #         if not self.stop_client.wait_for_service(timeout_sec=1.0):
    #             self.get_logger().error('stop service not available')
    #             return False
                
    #         # 同步调用服务
    #         future = self.stop_client.call_async(request)
            
    #         # 非阻塞轮询 - 设置超时
    #         timeout = 5.0  # 5秒超时
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
        """导航到指定位置 - 异步版本（不阻塞executor）"""
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

            # 创建导航请求
            request = self._GoToSrvType.Request()
            request.goal.x = x
            request.goal.y = y
            request.goal.z = z

            # 原本无法控制速度
            request.duration.sec = 10 #增加时间用于降低速度
            request.duration.nanosec = 0

            # 等待服务可用
            if not self.go_to_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('go_to service not available')
                return False
                
            # 异步调用服务
            future = self.go_to_client.call_async(request)
            
            # ★ 关键修复：不使用spin_once，直接等待future完成
            timeout = 5.0  # 5秒超时
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
                time.sleep(0.05)  # 短暂等待，避免占用CPU
                
            self.get_logger().error('Go to command timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Error in go_to: {str(e)}')
            return False

#################################################################################
    def hover_at_position(self, x=None, y=None, z=None, yaw=0.0, emergency_stop=True):
        """立即停止在指定高度并悬停（使用 Hover 消息持续发送）。X/Y不再强制位置，仅发送零速度。"""
        try:
            # 如果提供了坐标，使用提供的坐标；否则使用当前位置
            if x is None or y is None or z is None:
                x = self.current_position.x
                y = self.current_position.y
                z = self.current_position.z
                self.get_logger().info(f'HOVER: drone {self.drone_id} at current position ({x:.3f}, {y:.3f}, {z:.3f})')
            else:
                self.get_logger().info(f'HOVER: drone {self.drone_id} at specified position ({x:.3f}, {y:.3f}, {z:.3f})')
            
            # 确保安全高度
            safe_z = max(float(z), 0.5)  # 最小安全高度0.5米
            if safe_z != z:
                self.get_logger().warning(f'Adjusting hover height from {z:.3f}m to safe height {safe_z:.3f}m')
                z = safe_z
            
            # 记录目标高度（Hover控制z）
            self.hover_z = float(z)

            # 停止所有定时器
            if hasattr(self, 'move_timer') and self.move_timer is not None:
                self.get_logger().info(f'Stopping previous move timer')
                self.move_timer.destroy()
                self.move_timer = None
            
            if hasattr(self, 'hover_timer') and self.hover_timer is not None:
                self.hover_timer.destroy()
                self.hover_timer = None

            # 创建定时器持续发送悬停命令（50Hz）
            self.hover_timer = self.create_timer(0.02, self._hover_timer_callback)
            
            # 立即发送一次Hover指令，避免控制间隙
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
        """悬停定时器回调，持续发送 Hover 命令（零速度，固定高度）。"""
        if hasattr(self, 'hover_z'):
            msg = Hover()
            msg.vx = 0.0
            msg.vy = 0.0
            msg.yaw_rate = 0.0
            msg.z_distance = self.hover_z
            self.hover_pub.publish(msg)

    def move_to_position_realtime(self, x, y, z, velocity=0.2, timeout=10.0, skip_stop=False):
        """实时移动到指定位置 - 使用 Hover 消息持续发送速度（vx, vy），并保持目标高度z。"""
        try:
            self.get_logger().info(f'Moving drone {self.drone_id} to ({x}, {y}, {z}) in realtime with velocity={velocity}')
            
            # 等待短暂时间，确保位置数据是最新的
            time.sleep(0.05)
            
            # 获取当前位置
            start_x, start_y, start_z = self.current_position.x, self.current_position.y, self.current_position.z
            
            # 计算移动距离
            dx = x - start_x
            dy = y - start_y
            dz = z - start_z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            self.get_logger().info(f'Distance to target: {distance:.3f}m, start=({start_x:.3f},{start_y:.3f},{start_z:.3f})')
            
            if distance < 0.05:  # 如果目标很近，直接悬停
                self.get_logger().info(f'Target very close (distance: {distance:.3f}m), using hover instead of movement')
                return self.hover_at_position(x, y, z, emergency_stop=False)
            
            # 计算运动参数（速度上限0.15m/s）
            max_velocity = min(velocity, 0.2)
            start_time = time.time()
            self.get_logger().info(f'Max velocity: {max_velocity:.3f}m/s')
            
            # 初始化移动状态（Hover控制下仅需目标与速度）
            self.move_start_time = start_time
            self.move_start_pos = (start_x, start_y, start_z)
            self.move_target_pos = (x, y, z)
            self.move_max_velocity = max_velocity
            self.move_timeout = timeout
            self.move_completed = False
            
            # 现在安全地停止之前的定时器
            if hasattr(self, 'hover_timer') and self.hover_timer is not None:
                self.get_logger().info(f'Stopping hover timer before movement')
                self.hover_timer.destroy()
                self.hover_timer = None
            
            if hasattr(self, 'move_timer') and self.move_timer is not None:
                self.get_logger().info(f'Stopping previous move timer')
                self.move_timer.destroy()
                self.move_timer = None
            
            # 立即创建新的移动定时器，确保无缝切换
            self.move_timer = self.create_timer(0.02, self._move_timer_callback)
            
            # 立即执行一次移动回调，避免控制间隙
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
        """移动定时器回调：基于当前位置到目标的方向，发布 Hover 速度指令。"""
        try:
            if not hasattr(self, 'move_start_time') or self.move_completed:
                return
            
            current_time = time.time()
            elapsed_time = current_time - self.move_start_time
            
            # 检查超时
            if elapsed_time > self.move_timeout:
                self.get_logger().warning(f'Movement timeout after {elapsed_time:.2f}s, stopping')
                self.move_completed = True
                # 停止移动定时器
                if hasattr(self, 'move_timer') and self.move_timer is not None:
                    self.move_timer.destroy()
                    self.move_timer = None
                # 悬停在当前位置
                self.hover_at_position()
                return
            
            # 检查当前位置
            current_x = self.current_position.x
            current_y = self.current_position.y
            current_z = self.current_position.z
            
            # 计算剩余距离（仅平面）
            remaining_dx = self.move_target_pos[0] - current_x
            remaining_dy = self.move_target_pos[1] - current_y
            remaining_distance_xy = math.hypot(remaining_dx, remaining_dy)
            
            # 如果接近目标，停止移动并悬停在目标高度
            if remaining_distance_xy < 0.1:
                self.get_logger().info(f'Reached target (distance_xy: {remaining_distance_xy:.3f}m), stopping movement')
                self.move_completed = True
                if hasattr(self, 'move_timer') and self.move_timer is not None:
                    self.move_timer.destroy()
                    self.move_timer = None
                self.hover_at_position(z=self.move_target_pos[2])
                return
            
            # 计算速度方向并限幅
            if remaining_distance_xy > 0:
                unit_x = remaining_dx / remaining_distance_xy
                unit_y = remaining_dy / remaining_distance_xy
            else:
                unit_x = 0.0
                unit_y = 0.0
            
            # 到目标越近，速度线性减小（最小速度阈值）
            speed = min(self.move_max_velocity, max(0.1, remaining_distance_xy))
            vx = unit_x * speed
            vy = unit_y * speed
            
            # 发布 Hover 速度指令（保持目标高度）
            msg = Hover()
            msg.vx = float(vx)
            msg.vy = float(vy)
            msg.yaw_rate = 0.0
            msg.z_distance = float(self.move_target_pos[2])
            self.hover_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in _move_timer_callback: {str(e)}')
            self.move_completed = True
            # 停止移动定时器
            if hasattr(self, 'move_timer') and self.move_timer is not None:
                self.move_timer.destroy()
                self.move_timer = None
            # 悬停在当前位置
            self.hover_at_position()
    

    def start_wall_following(self, config: WallFollowingConfig) -> bool:
        """启动墙面跟随控制。"""
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
        """停止所有控制指令，包括 streaming setpoints（不阻塞executor）"""
        self.get_logger().info(f'Stopping drone {self.drone_id} (including FullState streaming)')
        try:
            # 先停止墙面跟随（如果正在运行）
            if self.wall_following_orchestrator is not None:
                self.get_logger().info('Stopping wall following before stopping controller')
                self.stop_wall_following()
            
            # 先本地停止所有持续发送的计时器，避免继续发布Hover/FullState
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

            # 清理实时控制目标，避免回调再次访问
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
                return True  # 即使服务不可用，streaming 也已停止
                
            request = self._NotifySetpointsStopSrvType.Request()
            future = self.stop_client.call_async(request)
            
            # ★ 关键修复：不使用spin_once，直接等待future完成
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
                time.sleep(0.05)  # 短暂等待，避免占用CPU
                
            self.get_logger().warning('Stop command timed out')
            return True  # streaming 已停止，即使服务超时也返回 True
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
    
    # 设置日志级别
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger(__name__)
    
    # 添加错误处理装饰器
    class AppWrapper(Flask):
        def handle_exception(self, e):
            """覆盖默认异常处理"""
            logger.error(f"Unhandled exceptions: {str(e)}", exc_info=True)
            return jsonify({"error": str(e)}), 500
            
        def handle_user_exception(self, e):
            """覆盖用户异常处理"""
            if isinstance(e, exceptions.TransitionNotAllowed):
                logger.error(f"state transition error: {str(e)}", exc_info=True)
                return jsonify({"error": str(e)}), 400
            return super().handle_user_exception(e)
    
    try:
        # 初始化ROS2
        logger.info("initializing ROS2...")
        rclpy.init(args=None)

        #！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        # 初始化ROS2CrazyflieController， controller是ROS2CrazyflieController这个ROS2节点的实例化
        # controller实现的功能如上面的class类。ROS2CrazyflieController继承Node
        controller = ROS2CrazyflieController(args.drone_id, dssim=args.dssim)
        #！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！

        # 创建ROS2单独的线程
        executor = MultiThreadedExecutor()
        controller.set_executor(executor)
        ros_thread = threading.Thread(target=lambda: executor.spin())
        ros_thread.daemon = True
        ros_thread.start()
        
        # 创建状态机实例
        logger.info("creating state machine instance...")
        drone = cf_sm.StateMachineDrone(args.drone_id, debug=args.debug)
        
        # 设置无人机操作策略实现
        logger.info("setting UAV operation strategy...")
        from cf_drone_ops import HlCommanderCFOperationImpl
        
        #！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        # 设置无人机操作策略实现，将controller传递给HlCommanderCFOperationImpl
        drone.set_uavOpsImpl(HlCommanderCFOperationImpl(scf=None, controller=controller, debug=DEBUG))
        #！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        
        # 初始化状态机
        logger.info("initializing state machine...")
        try:
            drone.install()  # INSTALLED -> RESOLVED
            logger.info("state machine installed")
            time.sleep(0.2)  # 添加短暂延迟
            
            drone.start()    # RESOLVED -> STARTING
            logger.info("state machine started")
            time.sleep(0.2)  # 添加短暂延迟
            
            drone.initialize()  # STARTING -> ACTIVE
            logger.info("state machine initialized")
            logger.info(f"state machine current state: {drone.get_current_state()}")
            
            # 等待状态机完全初始化
            time.sleep(0.5)
            
        except Exception as e:
            logger.error(f"initializing state machine failed: {str(e)}", exc_info=True)
            raise
        
        # 创建Flask应用
        logger.info("creating Flask app...")
        app = AppWrapper(__name__)
        app.register_blueprint(drone_blueprint)
        app.config['DRONE'] = drone
        
        # 配置静态文件目录
        app.static_folder = '../webview'
        app.static_url_path = ''
        
        # 添加错误处理
        @app.errorhandler(500)
        def internal_error(error):
            logger.error(f"server internal error: {str(error)}", exc_info=True)
            return jsonify({"error": str(error)}), 500
        
        @app.errorhandler(Exception)
        def handle_exception(e):
            logger.error(f"unhandled exception: {str(e)}", exc_info=True)
            return jsonify({"error": str(e)}), 500
        
        # 启动Flask服务器
        logger.info("starting Flask server...")
        flask_thread = threading.Thread(
            target=start_flask_app,
            args=(app, args.host, args.port)
        )
        flask_thread.daemon = True
        flask_thread.start()
        
        # 等待Flask服务器启动
        flask_started.wait()
        logger.info("Flask WebServer started [OK]")
        
        # 启动WebSocket服务器（如果启用）
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
        
        # 主循环
        logger.info("service started, press Ctrl+C to terminate...")
        try:
            # 保持主线程运行
            while ISRUNNING:
                #print_log_values() //打印所有无人机的信息
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