#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import requests
import json
import time
import threading
from datetime import datetime
import socket
import base64
import math
import logging

import bispace_pb2
from google.protobuf import text_format
from auto_state_machine import AutoStateMachine

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class CollisionAvoidanceDemo(Node):
    def __init__(self, drone_config=None):
        super().__init__('collision_avoidance_demo')
        
        # 无人机配置 - 支持多机扩展
        if drone_config is None:
            # 默认配置：3架无人机
            self.drone_config = {
                'cf231': {'port': 5000, 'target': {'x': 0.0, 'y': 0.8, 'z': 0.6}},
                'cf232': {'port': 5001, 'target': {'x': 0.0, 'y': -0.8, 'z': 0.6}},
                'cf233': {'port': 5002, 'target': {'x': 0.8, 'y': 0.0, 'z': 0.6}}
            }
        else:
            self.drone_config = drone_config
        
        self.drone_names = list(self.drone_config.keys())
        
        # 存储无人机位置和高度数据
        self.drone_positions = {}
        self.drone_heights = {}
        
        # 创建订阅者列表
        self.odom_subs = []
        
        # 为每架无人机创建订阅者
        for drone_name in self.drone_names:
            sub = self.create_subscription(
                Odometry,
                f'/{drone_name}/odom',
                lambda msg, name=drone_name: self.odom_callback(msg, name),
                10)
            self.odom_subs.append(sub)
            self.drone_positions[drone_name] = {'x': 0.0, 'y': 0.0}
            self.drone_heights[drone_name] = 0.0
        
        # 创建自动状态机控制器
        self.drone_controllers = {}
        for drone_name, config in self.drone_config.items():
            self.drone_controllers[drone_name] = AutoStateMachine(base_url=f"http://127.0.0.1:{config['port']}")
        
        # 创建定时器，用于实时碰撞检测
        self.timer = self.create_timer(0.5, self.collision_detection_loop)  # 2Hz检测频率
        
        # 等待位置数据稳定
        time.sleep(2.0)
        
        # 演示状态
        self.demo_phase = "idle"  # idle, takeoff, flying, collision_avoidance, completed, emergency_landing
        self.demo_start_time = None
        self.last_activity_time = time.time()  # 用于超时检测
        
        # 存储quadtree信息
        self.quadtree_grid = None
        self.occupied_cells = set()
        self.available_cells = []
        
        # 避撞状态管理
        self.collision_avoidance_active = {}
        self.original_targets = {}  # 存储原始目标
        self.assigned_safe_cells = {}  # 存储分配的避撞cell
        self.emergency_stop_active = {}  # 紧急停止状态
        self.avoidance_start_times = {}  # 记录避撞开始时间
        self.last_distance_progress = {}  # 记录到安全cell的最近距离与时间 {name: (dist, ts)}
        
        # 初始化避撞状态
        for drone_name in self.drone_names:
            self.collision_avoidance_active[drone_name] = False
            self.emergency_stop_active[drone_name] = False
            self.original_targets[drone_name] = self.drone_config[drone_name]['target'].copy()
        
        self.get_logger().info('Collision avoidance demo initialized')
        
        # 初始化时检查状态机连接
        for drone_name, controller in self.drone_controllers.items():
            try:
                current_state = controller.get_current_state()
                self.get_logger().info(f'{drone_name} initial state: {current_state}')
            except Exception as e:
                self.get_logger().error(f'Failed to get initial state for {drone_name}: {str(e)}')
    
    def odom_callback(self, msg, drone_name):
        """处理里程计数据"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # 更新无人机位置和高度
        self.drone_positions[drone_name] = {'x': x, 'y': y}
        self.drone_heights[drone_name] = z
        
        # 更新最后活动时间
        self.last_activity_time = time.time()
    
    def _decode_bigrid(self, response_text):
        """Parse BiGrid protobuf from Java service JSON response"""
        try:
            payload = json.loads(response_text)
        except Exception as e:
            raise ValueError(f'Failed to parse JSON response: {e}')

        mime_type = payload.get('mimeType')
        if mime_type != 'application/x-protobuf':
            raise ValueError(f'Unsupported mimeType: {mime_type}')

        content_b64 = payload.get('content')
        if not content_b64:
            raise ValueError('Response missing content field')

        try:
            content_bytes = base64.b64decode(content_b64)
        except Exception as e:
            raise ValueError(f'Base64 decode failed: {e}')

        grid = bispace_pb2.BiGrid()
        try:
            grid.ParseFromString(content_bytes)
        except Exception as e:
            raise ValueError(f'Protobuf parse failed: {e}')

        # Parse pointsOmitted and pointsAdded
        points_omitted = payload.get('pointsOmitted', [])
        points_added = payload.get('pointsAdded', [])
        
        return grid, points_omitted, points_added

    @staticmethod
    def _point_in_cell(x, y, cell):
        """判断点(x,y)是否位于给定cell矩形范围内"""
        try:
            cx = float(cell.pose.x)
            cy = float(cell.pose.y)
            width = float(cell.size.width)
            length = float(cell.size.length)
        except Exception:
            return False

        if width <= 0.0 or length <= 0.0:
            return False

        half_w = width / 2.0
        half_l = length / 2.0

        in_x = (cx - half_w) <= x <= (cx + half_w)
        in_y = (cy - half_l) <= y <= (cy + half_l)
        return in_x and in_y

    def _find_cell_for_point(self, x, y, grid):
        """在BiGrid中查找包含点(x,y)的第一个单元格，返回cell或None"""
        for cell in grid.items:
            if self._point_in_cell(x, y, cell):
                return cell
        return None

    @staticmethod
    def _point_in_boundary(x, y, boundary):
        """判断点(x,y)是否处于 boundary 范围内"""
        try:
            bx = float(boundary["x"])
            by = float(boundary["y"])
            bw = float(boundary["width"])
            bh = float(boundary["height"])
        except Exception:
            return False

        if bw <= 0.0 or bh <= 0.0:
            return False

        in_x = bx <= x <= (bx + bw)
        in_y = by <= y <= (by + bh)
        return in_x and in_y

    def _detect_collisions_by_omitted_and_boundary(self, points_omitted, boundary):
        """检测points_omitted且位于boundary内的碰撞风险"""
        collisions = []
        for omitted_point in points_omitted:
            x, y = float(omitted_point["x"]), float(omitted_point["y"])
            if not self._point_in_boundary(x, y, boundary):
                continue

            # 查找对应的无人机 - 增加容差到0.05米
            drone_name = self._find_drone_by_position(x, y, tolerance=0.05)
            if drone_name is None:
                continue
            
            # 检查无人机是否已着陆，如果已着陆则跳过
            try:
                controller = self.drone_controllers[drone_name]
                current_state = controller.get_current_state()
                if current_state == "idle":
                    self.get_logger().debug(f'{drone_name} is landed (state: {current_state}), excluding from quadtree collision detection')
                    continue
            except Exception as e:
                self.get_logger().warning(f'Failed to get state for {drone_name} in quadtree detection: {str(e)}')
                # 如果获取状态失败，保守地包含在检测中

            collisions.append({
                'drone': drone_name,
                'position': (x, y)
            })
        return collisions
    
    def _detect_collisions_by_distance(self, safety_distance=0.4):
        """基于距离检测无人机之间的碰撞风险"""
        collisions = []
        drone_list = list(self.drone_names)
        
        # 过滤出仍在飞行的无人机（排除已着陆的）
        flying_drones = []
        for drone_name in drone_list:
            try:
                controller = self.drone_controllers[drone_name]
                current_state = controller.get_current_state()
                if current_state != "idle":  # 只有非idle状态的无人机参与避撞检测
                    flying_drones.append(drone_name)
                else:
                    self.get_logger().debug(f'{drone_name} is landed (state: {current_state}), excluding from collision detection')
            except Exception as e:
                self.get_logger().warning(f'Failed to get state for {drone_name}: {str(e)}, including in collision detection')
                flying_drones.append(drone_name)  # 如果获取状态失败，保守地包含在检测中
        
        # 如果飞行中的无人机少于2架，无需检测
        if len(flying_drones) < 2:
            return collisions
        
        for i in range(len(flying_drones)):
            for j in range(i + 1, len(flying_drones)):
                drone1 = flying_drones[i]
                drone2 = flying_drones[j]
                
                pos1 = self.drone_positions.get(drone1)
                pos2 = self.drone_positions.get(drone2)
                
                if pos1 is None or pos2 is None:
                    continue
                
                # 计算距离
                distance = math.hypot(pos1['x'] - pos2['x'], pos1['y'] - pos2['y'])
                
                # 如果距离小于安全距离，标记为碰撞风险
                if distance < safety_distance:
                    self.get_logger().warn(
                        f'Distance collision risk: {drone1} and {drone2} are {distance:.3f}m apart (safety: {safety_distance}m)'
                    )
                    
                    # 将两架无人机都加入碰撞列表
                    if not any(c['drone'] == drone1 for c in collisions):
                        collisions.append({
                            'drone': drone1,
                            'position': (pos1['x'], pos1['y'])
                        })
                    
                    if not any(c['drone'] == drone2 for c in collisions):
                        collisions.append({
                            'drone': drone2,
                            'position': (pos2['x'], pos2['y'])
                        })
        
        return collisions

    def _find_drone_by_position(self, x, y, tolerance=0.01):
        """根据坐标查找无人机名称"""
        for drone_name, pos in self.drone_positions.items():
            if abs(pos['x'] - x) < tolerance and abs(pos['y'] - y) < tolerance:
                return drone_name
        return None

    def _find_nearest_empty_cell(self, x, y, grid, exclude_cells=None):
        """寻找距离(x,y)最近且当前没有无人机处于该cell的cell中心作为避撞坐标"""
        if exclude_cells is None:
            exclude_cells = set()
            
        # 构建当前被占用的cell集合
        occupied_cell_ids = set()
        for _, pos in self.drone_positions.items():
            cell = self._find_cell_for_point(pos['x'], pos['y'], grid)
            if cell is not None and getattr(cell, 'id', None) is not None:
                occupied_cell_ids.add(cell.id)
        
        # 添加已分配的避撞cell
        occupied_cell_ids.update(exclude_cells)

        best_cell = None
        best_dist = float('inf')
        for cell in grid.items:
            # 跳过已被占用的cell
            if getattr(cell, 'id', None) in occupied_cell_ids:
                continue

            cx = float(cell.pose.x)
            cy = float(cell.pose.y)
            dist = math.hypot(cx - x, cy - y)
            if dist < best_dist:
                best_dist = dist
                best_cell = cell

        if best_cell is None:
            return None
        return (float(best_cell.pose.x), float(best_cell.pose.y), getattr(best_cell, 'id', None))
    
    def _calculate_safe_hover_position(self, drone_name):
        """计算一个安全的悬停位置（当没有grid时使用）"""
        pos = self.drone_positions[drone_name]
        
        # 找到距离最远的方向，向那个方向移动0.5米
        max_distance = 0.0
        safe_direction = (0.0, 0.0)
        
        # 检查几个候选方向
        candidates = [
            (0.5, 0.0),   # 右
            (-0.5, 0.0),  # 左
            (0.0, 0.5),   # 上
            (0.0, -0.5),  # 下
            (0.35, 0.35), # 右上
            (-0.35, 0.35),# 左上
            (0.35, -0.35),# 右下
            (-0.35, -0.35)# 左下
        ]
        
        for dx, dy in candidates:
            candidate_x = pos['x'] + dx
            candidate_y = pos['y'] + dy
            
            # 检查边界
            if abs(candidate_x) > 0.9 or abs(candidate_y) > 0.9:
                continue
            
            # 计算到所有其他无人机的最小距离
            min_dist_to_others = float('inf')
            for other_name, other_pos in self.drone_positions.items():
                if other_name == drone_name:
                    continue
                dist = math.hypot(candidate_x - other_pos['x'], candidate_y - other_pos['y'])
                min_dist_to_others = min(min_dist_to_others, dist)
            
            # 选择到其他无人机最远的位置
            if min_dist_to_others > max_distance:
                max_distance = min_dist_to_others
                safe_direction = (dx, dy)
        
        safe_x = pos['x'] + safe_direction[0]
        safe_y = pos['y'] + safe_direction[1]
        
        self.get_logger().info(f'Calculated safe hover position for {drone_name}: ({safe_x:.3f}, {safe_y:.3f}), min distance to others: {max_distance:.3f}m')
        
        return (safe_x, safe_y, None)

    def _request_quadtree_analysis(self):
        """请求quadtree分析"""
        try:
            # 准备请求数据
            data = {
                "boundary": {
                    "x": -1.0,
                    "y": -1.0,
                    "width": 2.0,
                    "height": 2.0
                },
                "pointData": {
                    "points": list(self.drone_positions.values())
                }
            }
            
            # 发送POST请求到Java服务
            response = requests.post(
                "http://172.29.224.1:8080/generate/quadtree?marginPoint=0.5&format=protobuf",
                headers={"Content-Type": "application/json"},
                data=json.dumps(data),
                timeout=5.0  # 增加超时时间
            )
            
            if response.status_code == 200:
                # 解析BiGrid
                grid, points_omitted, points_added = self._decode_bigrid(response.text)
                return grid, points_omitted, points_added
            else:
                self.get_logger().error(f'Failed to get quadtree: {response.status_code}')
                return None, [], []
                
        except Exception as e:
            self.get_logger().error(f'Error in quadtree analysis: {str(e)}')
            return None, [], []

    def _emergency_hover(self, drone_name):
        """紧急悬停"""
        try:
            controller = self.drone_controllers[drone_name]
            pos = self.drone_positions[drone_name]
            height = self.drone_heights[drone_name]
            
            self.get_logger().warn(f'EMERGENCY HOVER: {drone_name} at ({pos["x"]:.3f}, {pos["y"]:.3f}, {height:.3f})')
            
            # 使用/hover/realtime接口进行紧急悬停 - 不设置timeout，让系统使用默认值
            response = requests.post(
                f"http://127.0.0.1:{self.drone_config[drone_name]['port']}/hover/realtime",
                json={
                    "x": pos["x"],
                    "y": pos["y"], 
                    "z": max(height, 0.5),  # 确保安全高度
                    "emergency_stop": True
                }
                # 移除timeout参数，使用requests默认超时
            )
            
            if response.status_code == 200:
                self.emergency_stop_active[drone_name] = True
                self.get_logger().info(f'{drone_name} emergency hover successful')
                return True
            else:
                self.get_logger().error(f'{drone_name} emergency hover failed: {response.status_code}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error in emergency hover for {drone_name}: {str(e)}')
            return False

    def _move_to_safe_cell(self, drone_name, safe_cell_center):
        """移动到安全cell中心"""
        try:
            controller = self.drone_controllers[drone_name]
            x, y, z = safe_cell_center[0], safe_cell_center[1], 0.6  # 固定高度0.6m
            
            self.get_logger().info(f'{drone_name} moving to safe cell center ({x:.3f}, {y:.3f}, {z:.3f})')
            
            # 使用/move/realtime接口进行移动 - 不设置timeout
            response = requests.post(
                f"http://127.0.0.1:{self.drone_config[drone_name]['port']}/move/realtime",
                json={
                    "x": x,
                    "y": y,
                    "z": z,
                    "velocity": 0.1,  # 较慢的速度确保安全
                    "timeout": 15.0,
                    "skip_stop": False
                }
                # 移除timeout参数，使用requests默认超时
            )
            
            if response.status_code == 200:
                self.collision_avoidance_active[drone_name] = True
                self.assigned_safe_cells[drone_name] = safe_cell_center
                self.get_logger().info(f'{drone_name} move to safe cell command sent')
                return True
            else:
                self.get_logger().error(f'{drone_name} move to safe cell failed: {response.status_code}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error moving {drone_name} to safe cell: {str(e)}')
            return False

    def _resume_original_mission(self, drone_name):
        """恢复原始任务"""
        try:
            if drone_name not in self.original_targets:
                return False
                
            target = self.original_targets[drone_name]
            self.get_logger().info(f'{drone_name} resuming original mission to ({target["x"]:.3f}, {target["y"]:.3f}, {target["z"]:.3f})')
            
            # 使用/move/realtime接口恢复任务 - 不设置timeout
            response = requests.post(
                f"http://127.0.0.1:{self.drone_config[drone_name]['port']}/move/realtime",
                json={
                    "x": target["x"],
                    "y": target["y"],
                    "z": target["z"],
                    "velocity": 0.15,
                    "timeout": 30.0,
                    "skip_stop": True
                }
                # 移除timeout参数，使用requests默认超时
            )
            
            if response.status_code == 200:
                self.collision_avoidance_active[drone_name] = False
                self.emergency_stop_active[drone_name] = False
                if drone_name in self.assigned_safe_cells:
                    del self.assigned_safe_cells[drone_name]
                if drone_name in self.avoidance_start_times:
                    del self.avoidance_start_times[drone_name]
                if drone_name in self.last_distance_progress:
                    del self.last_distance_progress[drone_name]
                self.get_logger().info(f'{drone_name} resumed original mission')
                return True
            else:
                self.get_logger().error(f'{drone_name} resume mission failed: {response.status_code}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error resuming mission for {drone_name}: {str(e)}')
            return False

    def collision_detection_loop(self):
        """实时碰撞检测循环"""
        # 更新最后活动时间（碰撞检测本身就是活动）
        self.last_activity_time = time.time()
        
        # 检查演示是否超时（从开始时间计算，而不是最后活动时间）
        if self.demo_start_time and time.time() - self.demo_start_time > 300.0:  # 5分钟总超时
            self.get_logger().error('Demo total timeout (5min), initiating emergency landing for all drones')
            self.demo_phase = "emergency_landing"
            self._emergency_land_all()
            return
        
        # 只在飞行阶段进行碰撞检测
        if self.demo_phase not in ["flying", "collision_avoidance"]:
            return
        
        try:
            # 方法1: 基于距离的碰撞检测（主要方法）
            distance_collisions = self._detect_collisions_by_distance(safety_distance=0.4)
            
            # 方法2: 请求quadtree分析（辅助方法）
            grid, points_omitted, points_added = self._request_quadtree_analysis()
            quadtree_collisions = []
            if grid is not None:
                boundary = {"x": -1.0, "y": -1.0, "width": 2.0, "height": 2.0}
                quadtree_collisions = self._detect_collisions_by_omitted_and_boundary(points_omitted, boundary)
            
            # 合并两种方法检测到的碰撞
            all_collisions = {}
            for collision in distance_collisions + quadtree_collisions:
                drone_name = collision['drone']
                if drone_name not in all_collisions:
                    all_collisions[drone_name] = collision
            
            collisions = list(all_collisions.values())
            
            if collisions:
                # 只对还未进入避撞状态的无人机进行处理
                new_collisions = [c for c in collisions if not self.collision_avoidance_active[c['drone']]]
                
                if new_collisions:
                    self.get_logger().warn(f'COLLISION RISK DETECTED! {len(new_collisions)} NEW drones at risk (total: {len(collisions)})')
                    self.demo_phase = "collision_avoidance"
                    
                    # 收集需要处理的无人机
                    drones_to_handle = []
                    for collision in new_collisions:
                        drone_name = collision['drone']
                        pos = collision['position']
                        
                        self.get_logger().warn(f'Drone {drone_name} at ({pos[0]:.3f}, {pos[1]:.3f}) has NEW collision risk')
                        drones_to_handle.append((drone_name, pos))
                else:
                    # 所有有碰撞风险的无人机都已经在处理中
                    if self.demo_phase != "collision_avoidance":
                        self.demo_phase = "collision_avoidance"
                    # 处理进展与死锁（不重复分配，但允许重分配/强制分离）
                    self._handle_avoidance_progress_and_deadlock(grid)
                    return
                
                if drones_to_handle:
                    # 步骤1: 先对所有有风险的无人机发送紧急悬停指令
                    self.get_logger().info(f'Sending emergency hover to {len(drones_to_handle)} drones')
                    hover_success = {}
                    for drone_name, pos in drones_to_handle:
                        success = self._emergency_hover(drone_name)
                        hover_success[drone_name] = success
                        if not success:
                            self.get_logger().error(f'Failed to send emergency hover to {drone_name}')
                    
                    # 步骤2: 统一等待2秒让所有无人机稳定
                    self.get_logger().info('Waiting 2 seconds for all drones to stabilize...')
                    time.sleep(2.0)
                    
                # 记录避撞开始时间
                current_time = time.time()
                for drone_name, _ in drones_to_handle:
                    self.avoidance_start_times[drone_name] = current_time

                # 步骤3: 为所有成功悬停的无人机分配安全位置
                    assigned_cell_ids = set(cell_id for _, _, cell_id in self.assigned_safe_cells.values() if cell_id)
                    
                    for drone_name, pos in drones_to_handle:
                        if not hover_success.get(drone_name, False):
                            continue  # 跳过悬停失败的无人机
                        
                        safe_cell = None
                        
                        # 优先使用grid的安全cell
                        if grid is not None:
                            safe_cell = self._find_nearest_empty_cell(
                                pos[0], pos[1], grid, 
                                assigned_cell_ids
                            )
                        
                        # 如果找不到grid cell，计算一个安全的悬停位置
                        if safe_cell is None:
                            self.get_logger().warn(f'No grid cell found for {drone_name}, calculating safe hover position')
                            safe_cell = self._calculate_safe_hover_position(drone_name)
                        
                        if safe_cell:
                            # 移动到安全位置
                            if self._move_to_safe_cell(drone_name, safe_cell):
                                # 记录已分配的cell ID（如果有的话）
                                if safe_cell[2] is not None:
                                    assigned_cell_ids.add(safe_cell[2])
                                # 初始化进展监控
                                try:
                                    pos_now = self.drone_positions[drone_name]
                                    dist_now = math.hypot(pos_now['x'] - safe_cell[0], pos_now['y'] - safe_cell[1])
                                    self.last_distance_progress[drone_name] = (dist_now, time.time())
                                except Exception:
                                    pass
                            else:
                                self.get_logger().error(f'Failed to move {drone_name} to safe position')
                                self.collision_avoidance_active[drone_name] = True
                        else:
                            self.get_logger().error(f'No safe position found for {drone_name}, keeping in hover')
                            # 标记为避撞活动状态，即使没有移动到安全位置
                            self.collision_avoidance_active[drone_name] = True
            else:
                # 没有碰撞风险，检查是否可以恢复任务
                if self.demo_phase == "collision_avoidance":
                    self.get_logger().info('No collision risk detected, checking if drones can resume missions')
                    
                    # 统计有多少无人机在避撞状态
                    active_count = sum(1 for v in self.collision_avoidance_active.values() if v)
                    self.get_logger().info(f'{active_count} drones are in avoidance state')
                    
                    if active_count == 0:
                        # 没有无人机在避撞状态，直接返回飞行阶段
                        self.get_logger().info('No drones in avoidance state, returning to flying phase')
                        self.demo_phase = "flying"
                        return
                    
                    # 检查并按无人机逐个恢复任务（而不是等待全部安全）
                    all_safe = True
                    for drone_name in list(self.drone_names):
                        if self.collision_avoidance_active.get(drone_name, False):
                            # 检查是否已到达安全cell
                            if drone_name in self.assigned_safe_cells:
                                safe_cell = self.assigned_safe_cells[drone_name]
                                pos = self.drone_positions[drone_name]
                                distance = math.hypot(
                                    pos['x'] - safe_cell[0], 
                                    pos['y'] - safe_cell[1]
                                )
                                self.get_logger().info(f'{drone_name} distance to safe cell: {distance:.3f}m')
                                
                                # 检查异常情况：距离超过3米说明飞错方向或位置数据异常
                                if distance > 3.0:
                                    self.get_logger().error(
                                        f'{drone_name} distance to safe cell too large ({distance:.3f}m), '
                                        f'position may be incorrect. Forcing resume.'
                                    )
                                    # 强制恢复任务
                                    self._resume_original_mission(drone_name)
                                    continue
                                
                                # 若已到达安全位置（<=0.25m），立即恢复该机原任务
                                if distance <= 0.25:
                                    self.get_logger().info(f'{drone_name} reached safe position, resuming mission now')
                                    self._resume_original_mission(drone_name)
                                    continue
                                else:
                                    all_safe = False
                                    self.get_logger().info(f'{drone_name} has not reached safe position yet')
                            else:
                                # 没有分配安全cell，说明只是悬停，可以认为已经安全 -> 立即恢复
                                self.get_logger().info(f'{drone_name} is hovering without safe cell assignment, resuming mission')
                                self._resume_original_mission(drone_name)
                    
                    if all_safe:
                        # 所有无人机都安全，恢复原始任务（只对仍在飞行的无人机）
                        self.get_logger().info('All drones are safe, resuming original missions')
                        for drone_name in self.drone_names:
                            if self.collision_avoidance_active[drone_name]:
                                # 检查无人机是否已着陆，如果已着陆则跳过恢复任务
                                try:
                                    controller = self.drone_controllers[drone_name]
                                    current_state = controller.get_current_state()
                                    if current_state == "idle":
                                        self.get_logger().info(f'{drone_name} is already landed (state: {current_state}), skipping mission resume')
                                        # 清理避撞状态
                                        self.collision_avoidance_active[drone_name] = False
                                        self.emergency_stop_active[drone_name] = False
                                        if drone_name in self.assigned_safe_cells:
                                            del self.assigned_safe_cells[drone_name]
                                        continue
                                except Exception as e:
                                    self.get_logger().warning(f'Failed to get state for {drone_name} during resume: {str(e)}')
                                
                                self._resume_original_mission(drone_name)
                        self.demo_phase = "flying"
                
        except Exception as e:
            self.get_logger().error(f'Error in collision detection loop: {str(e)}')

    def _handle_avoidance_progress_and_deadlock(self, grid):
        """监控避撞进展并处理死锁：
        - 若某机对其safe cell距离无改进超过3s，则重新分配更远的cell
        - 若两机持续距离 < 0.2m 超过2s，强制分离：为其中一机重新分配更远cell
        - 若避撞持续超过20s，强制恢复原任务
        """
        try:
            current_time = time.time()
            avoidance_drones = [name for name in self.drone_names if self.collision_avoidance_active.get(name, False)]
            if not avoidance_drones:
                return

            # 1) 避撞总时长上限
            for name in list(avoidance_drones):
                start_ts = self.avoidance_start_times.get(name)
                if start_ts and (current_time - start_ts) > 20.0:
                    self.get_logger().error(f'{name} avoidance timeout >20s, forcing resume')
                    self._resume_original_mission(name)

            # 2) 进展监控与重分配
            for name in avoidance_drones:
                if name not in self.assigned_safe_cells:
                    continue
                pos = self.drone_positions.get(name)
                safe_cell = self.assigned_safe_cells.get(name)
                if not pos or not safe_cell:
                    continue
                dist_now = math.hypot(pos['x'] - safe_cell[0], pos['y'] - safe_cell[1])
                last = self.last_distance_progress.get(name)
                if last:
                    last_dist, last_ts = last
                    # 若3s内距离未缩小至少0.05m，认为无进展，重分配更远cell
                    if (current_time - last_ts) > 3.0 and (last_dist - dist_now) < 0.05:
                        self.get_logger().warning(f'{name} no progress towards safe cell (d={dist_now:.3f}m), reassigning')
                        # 选择新的cell：尽量远离其他避撞无人机
                        new_cell = None
                        if grid is not None:
                            assigned_ids = set(cell_id for _, _, cell_id in self.assigned_safe_cells.values() if cell_id)
                            new_cell = self._find_nearest_empty_cell(pos['x'], pos['y'], grid, assigned_ids)
                        if new_cell is None:
                            new_cell = self._calculate_safe_hover_position(name)
                        if new_cell:
                            if self._move_to_safe_cell(name, new_cell):
                                try:
                                    self.last_distance_progress[name] = (math.hypot(pos['x'] - new_cell[0], pos['y'] - new_cell[1]), current_time)
                                except Exception:
                                    pass
                # 更新记录
                self.last_distance_progress[name] = (dist_now, current_time)

            # 3) 两机持续太近的死锁分离
            close_pairs = []
            for i in range(len(avoidance_drones)):
                for j in range(i + 1, len(avoidance_drones)):
                    d1 = avoidance_drones[i]
                    d2 = avoidance_drones[j]
                    p1 = self.drone_positions.get(d1)
                    p2 = self.drone_positions.get(d2)
                    if p1 and p2:
                        distance = math.hypot(p1['x'] - p2['x'], p1['y'] - p2['y'])
                        if distance < 0.2:
                            close_pairs.append((d1, d2, distance))
            if close_pairs:
                # 强制让列表中的第一架重分配更远cell
                d1, d2, dist = close_pairs[0]
                self.get_logger().error(f'DEADLOCK: {d1} and {d2} {dist:.3f}m apart, forcing {d1} to reassign')
                pos1 = self.drone_positions.get(d1)
                if pos1 is not None:
                    new_cell = None
                    if grid is not None:
                        assigned_ids = set(cell_id for _, _, cell_id in self.assigned_safe_cells.values() if cell_id)
                        new_cell = self._find_nearest_empty_cell(pos1['x'], pos1['y'], grid, assigned_ids)
                    if new_cell is None:
                        new_cell = self._calculate_safe_hover_position(d1)
                    if new_cell:
                        self._move_to_safe_cell(d1, new_cell)
        except Exception as e:
            self.get_logger().warning(f'Error in avoidance progress/deadlock handler: {str(e)}')

    def _emergency_land_all(self):
        """紧急降落所有无人机"""
        self.get_logger().error('Initiating emergency landing for all drones')
        
        for drone_name, controller in self.drone_controllers.items():
            try:
                # 先紧急悬停
                self._emergency_hover(drone_name)
                time.sleep(1.0)
                
                # 停止该无人机的持续控制（Hover/Move）
                try:
                    port = self.drone_config[drone_name]['port']
                    requests.post(f"http://127.0.0.1:{port}/stop_controller", timeout=2.0)
                    self.get_logger().info(f'{drone_name} controller streaming stopped')
                except Exception as se:
                    self.get_logger().warning(f'Failed to stop controller streaming for {drone_name}: {str(se)}')
                
                # 然后降落
                if controller.begin_landing():
                    self.get_logger().info(f'{drone_name} emergency landing initiated')
                else:
                    self.get_logger().error(f'{drone_name} emergency landing failed')
            except Exception as e:
                self.get_logger().error(f'Error in emergency landing for {drone_name}: {str(e)}')

    def start_demo(self):
        """启动演示"""
        self.get_logger().info('Starting collision avoidance demo...')
        self.demo_phase = "takeoff"
        self.demo_start_time = time.time()
        self.last_activity_time = time.time()
        
        # 启动演示线程
        demo_thread = threading.Thread(target=self._run_demo)
        demo_thread.daemon = True  # 设置为守护线程
        demo_thread.start()

    def _run_demo(self):
        """运行演示序列"""
        try:
            # 阶段1: 起飞所有无人机
            self.get_logger().info('Phase 1: Taking off all drones')
            
            for drone_name, controller in self.drone_controllers.items():
                # 检查当前状态
                current_state = controller.get_current_state()
                self.get_logger().info(f'{drone_name} current state: {current_state}')
                
                if current_state == "idle":
                    # 起飞
                    self.get_logger().info(f'{drone_name} starting takeoff')
                    if not controller.begin_takeoff():
                        self.get_logger().error(f'Failed to start takeoff for {drone_name}')
                        return
                    if not controller.wait_for_state("hovering", timeout=15.0):
                        self.get_logger().error(f'Failed to reach hovering state for {drone_name}')
                        return
                    self.get_logger().info(f'{drone_name} has taken off successfully')
                elif current_state == "hovering":
                    self.get_logger().info(f'{drone_name} is already hovering')
                else:
                    # 其他状态，先激活idle
                    self.get_logger().info(f'{drone_name} activating idle state')
                    if not controller.activate_idle():
                        self.get_logger().error(f'Failed to activate idle for {drone_name}')
                        return
                    if not controller.wait_for_state("idle", timeout=5.0):
                        self.get_logger().error(f'Failed to reach idle state for {drone_name}')
                        return
                    
                    # 然后起飞
                    if not controller.begin_takeoff():
                        self.get_logger().error(f'Failed to start takeoff for {drone_name}')
                        return
                    if not controller.wait_for_state("hovering", timeout=15.0):
                        self.get_logger().error(f'Failed to reach hovering state for {drone_name}')
                        return
                    self.get_logger().info(f'{drone_name} has taken off successfully')
            
            time.sleep(3.0)  # 稳定悬停
            
            # 阶段2: 开始飞行到各自目标
            self.get_logger().info('Phase 2: Flying to target positions')
            self.demo_phase = "flying"
            
            # 为每架无人机启动飞行线程
            flight_threads = []
            for drone_name, config in self.drone_config.items():
                thread = threading.Thread(
                    target=self._fly_to_target,
                    args=(drone_name, config['target'])
                )
                flight_threads.append(thread)
                thread.start()
            
            # 阶段3: 等待飞行完成（无人机会自动降落）
            self.get_logger().info('Phase 3: Monitoring flight and collision avoidance')
            self.get_logger().info('Drones will land automatically when they reach their targets')
            
            # 等待所有飞行线程完成（设置超时）
            max_wait_time = 120.0  # 最多等待2分钟
            start_wait = time.time()
            
            for i, thread in enumerate(flight_threads):
                remaining_time = max_wait_time - (time.time() - start_wait)
                if remaining_time > 0:
                    self.get_logger().info(f'Waiting for flight thread {i+1}/{len(flight_threads)} to complete...')
                    thread.join(timeout=remaining_time)
                    if thread.is_alive():
                        self.get_logger().warning(f'Flight thread {i+1} still running after timeout')
                else:
                    self.get_logger().warning(f'Overall wait time exceeded')
                    break
            
            self.get_logger().info('All flight threads completed')
            
            # 检查所有无人机的最终状态
            landed_count = 0
            for drone_name, controller in self.drone_controllers.items():
                current_state = controller.get_current_state()
                if current_state == "idle":
                    landed_count += 1
                    self.get_logger().info(f'{drone_name} is landed (state: {current_state})')
                else:
                    self.get_logger().warning(f'{drone_name} is not landed (state: {current_state})')
            
            if landed_count == len(self.drone_config):
                self.get_logger().info(f'All {landed_count} drones have landed successfully!')
            else:
                self.get_logger().warning(f'Only {landed_count}/{len(self.drone_config)} drones have landed')
            
            self.get_logger().info('Collision avoidance demo completed!')
            
        except Exception as e:
            self.get_logger().error(f'Demo failed with error: {str(e)}')
            self.demo_phase = "error"
            self._emergency_land_all()

    def _fly_to_target(self, drone_name, target):
        """飞行到目标位置（使用可中断的实时控制），到达后立即降落"""
        try:
            target_x, target_y, target_z = target['x'], target['y'], target['z']
            
            self.get_logger().info(f'{drone_name} flying to target ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) using realtime control')
            
            # 发送移动命令 - 不设置timeout
            response = requests.post(
                f"http://127.0.0.1:{self.drone_config[drone_name]['port']}/move/realtime",
                json={
                    "x": target_x,
                    "y": target_y,
                    "z": target_z,
                    "velocity": 0.15,
                    "timeout": 30.0,
                    "skip_stop": False
                }
                # 移除timeout参数，使用requests默认超时
            )
            
            if response.status_code != 200:
                self.get_logger().error(f'Failed to send realtime navigation command to {drone_name}: {response.status_code}')
                return
            
            self.get_logger().info(f'{drone_name} move command sent successfully, monitoring progress...')
            
            # 监控飞行进度，但不阻塞主线程
            start_time = time.time()
            max_flight_time = 60.0  # 最大飞行时间60秒
            reached_target = False
            
            while time.time() - start_time < max_flight_time:
                # 检查是否已进入避撞状态
                if self.collision_avoidance_active.get(drone_name, False):
                    self.get_logger().info(f'{drone_name} avoidance triggered during flight, stopping target navigation')
                    break
                
                # 检查是否已到达目标
                pos = self.drone_positions.get(drone_name)
                z = self.drone_heights.get(drone_name)
                if pos is not None and z is not None:
                    dx = abs(pos['x'] - target_x)
                    dy = abs(pos['y'] - target_y)
                    dz = abs(z - target_z)
                    if dx <= 0.2 and dy <= 0.2 and dz <= 0.2:
                        self.get_logger().info(f'{drone_name} reached target position successfully')
                        reached_target = True
                        break
                
                time.sleep(0.5)  # 每0.5秒检查一次
            
            # 如果超时，记录警告但继续
            if time.time() - start_time >= max_flight_time:
                self.get_logger().warning(f'{drone_name} flight monitoring timed out after {max_flight_time}s')
            
            # 检查是否在避撞过程中，如果是则等待避撞完成
            if self.collision_avoidance_active.get(drone_name, False):
                self.get_logger().info(f'{drone_name} is in avoidance process, waiting for completion...')
                # 等待避撞完成，但设置最大等待时间
                wait_start = time.time()
                while self.collision_avoidance_active.get(drone_name, False) and (time.time() - wait_start) < 30.0:
                    time.sleep(0.5)
                
                if self.collision_avoidance_active.get(drone_name, False):
                    self.get_logger().warning(f'{drone_name} avoidance wait timed out')
                else:
                    self.get_logger().info(f'{drone_name} avoidance completed')
                    # 避撞完成后，重新飞往目标
                    self.get_logger().info(f'{drone_name} resuming flight to target after avoidance')
                    return self._fly_to_target(drone_name, target)  # 递归调用
            
            # 如果到达目标且没有在避撞状态，立即降落
            if reached_target and not self.collision_avoidance_active.get(drone_name, False):
                self.get_logger().info(f'{drone_name} reached target, initiating landing')
                time.sleep(1.0)  # 稳定1秒
                
                controller = self.drone_controllers[drone_name]
                # 降落前停止该无人机的持续控制（Hover/Move）
                try:
                    port = self.drone_config[drone_name]['port']
                    requests.post(f"http://127.0.0.1:{port}/stop_controller", timeout=2.0)
                    self.get_logger().info(f'{drone_name} controller streaming stopped before landing')
                except Exception as se:
                    self.get_logger().warning(f'Failed to stop controller streaming for {drone_name}: {str(se)}')
                if controller.begin_landing():
                    self.get_logger().info(f'{drone_name} landing command sent')
                    if controller.wait_for_state("idle", timeout=30.0):
                        self.get_logger().info(f'{drone_name} landed successfully')
                    else:
                        self.get_logger().error(f'{drone_name} landing timeout')
                else:
                    self.get_logger().error(f'{drone_name} failed to send landing command')
            
        except Exception as e:
            self.get_logger().error(f'Error in flight execution for {drone_name}: {str(e)}')

def main():
    rclpy.init()
    
    # 自定义配置 - 3架无人机，航线有重叠
    drone_config = {
            'cf231': {'port': 5000, 'target': {'x': 0.0, 'y': 0.8, 'z': 0.6}},
            'cf232': {'port': 5001, 'target': {'x': 0.0, 'y': -0.8, 'z': 0.6}},
            'cf233': {'port': 5002, 'target': {'x': 0.6, 'y': 0.0, 'z': 0.6}} 
    }
    node = CollisionAvoidanceDemo(drone_config)
    
    # 启动演示
    node.start_demo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
