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
import logging

import bispace_pb2
from google.protobuf import text_format

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class AutoStateMachine:
    def __init__(self, base_url: str = "http://127.0.0.1:5000"):
        """Initialize the automatic state machine controller"""
        self.base_url = base_url
        self.current_state = None
        
    def get_current_state(self) -> str:
        """Get current state"""
        try:
            response = requests.get(f"{self.base_url}/status")
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                return self.current_state
            else:
                logger.error(f"Failed to get status: {response.status_code}")
                return None
        except Exception as e:
            logger.error(f"An error occurred while getting the status: {str(e)}")
            return None
            
    def activate_idle(self) -> bool:
        """Activate idle state"""
        try:
            response = requests.post(f"{self.base_url}/activate_idle")
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Activated idle state, current state: {self.current_state}")
                return True
            else:
                logger.error(f"Failed to activate idle state: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"An error occurred while activating the idle state: {str(e)}")
            return False
            
    def begin_takeoff(self) -> bool:
        """Start takeoff"""
        try:
            response = requests.post(f"{self.base_url}/begin_takeoff")
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Takeoff started, current state: {self.current_state}")
                return True
            else:
                logger.error(f"Failed to start takeoff: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"An error occurred while starting takeoff: {str(e)}")
            return False
            
    def begin_landing(self) -> bool:
        """Start landing"""
        try:
            response = requests.post(f"{self.base_url}/begin_landing")
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Landing started, current state: {self.current_state}")
                return True
            else:
                logger.error(f"Failed to start landing: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"An error occurred while starting landing: {str(e)}")
            return False
            
    def navigate_to(self, x: float, y: float, z: float) -> bool:
        """Navigate to specified position"""
        try:
            if x < 0 or y < 0 or z < 0:
                payload = {"x": x, "y": y, "z": z}
                response = requests.post(f"{self.base_url}/navigate", json=payload)
            else:
                response = requests.post(f"{self.base_url}/navigate/{x}/{y}/{z}")
                
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Navigation command sent to position ({x}, {y}, {z}), current state: {self.current_state}")
                return True
            else:
                logger.error(f"Failed to send navigation command: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"An error occurred while sending navigation command: {str(e)}")
            return False
            
    def wait_for_state(self, target_state: str, timeout: float = 10.0) -> bool:
        """Wait for state machine to transition to specified state"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_state = self.get_current_state()
            if current_state == target_state:
                logger.info(f"Target state reached: {target_state}")
                return True
            time.sleep(0.1)
        logger.error(f"Waiting for state {target_state} timed out")
        return False

class VertiportLandingDemo(Node):
    def __init__(self, drone_config=None):
        super().__init__('vertiport_landing_demo')
        
        # 无人机配置 - 可以轻松扩展到多机
        if drone_config is None:
            # 默认配置：2架无人机
            self.drone_config = {
                'cf231': {'port': 5000, 'needs_landing': True},
                'cf232': {'port': 5001, 'needs_landing': False}
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
        
        # 创建定时器，用于在关键节点进行分析
        self.timer = self.create_timer(2.0, self.periodic_analysis_loop)
        
        # 等待一段时间让位置数据稳定
        time.sleep(2.0)
        # self.get_logger().info('Initial drone positions:')
        # for drone_name, pos in self.drone_positions.items():
        #     height = self.drone_heights[drone_name]
        #     self.get_logger().info(f'{drone_name}: ({pos["x"]:.3f}, {pos["y"]:.3f}, {height:.3f})')
        
        # 演示状态
        self.demo_phase = "idle"  # idle, analysis, takeoff, landing, completed
        self.demo_start_time = None
        
        # 存储vertiport信息
        self.vertiport_grid = None
        self.occupied_cells = set()
        self.available_cells = []
        
        self.get_logger().info('Vertiport landing demo initialized')
        
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

        return grid

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

    def _analyze_vertiport_occupancy(self):
        """分析vertiport占用情况"""
        try:
            # 准备请求数据
            bigrid_data = {
                "x": -1.0,
                "y": -1.0,
                "stepSizeX": 1.0,
                "stepSizeY": 1.0
            }
            
            self.get_logger().info('Requesting BiGrid from Java service for vertiport analysis...')
            
            response = requests.post(
                "http://172.29.224.1:8080/generate/bigrid?rows=2&cols=3&format=protobuf",
                headers={"Content-Type": "application/json"},
                data=json.dumps(bigrid_data)
            )
            
            if response.status_code == 200:
                self.get_logger().info('Successfully received BiGrid from Java service')
                
                # 只在第一次分析时打印详细信息
                if self.demo_phase == "analysis":
                    self.get_logger().info('Java service response:')
                    self.get_logger().info(response.text)

                # 解析BiGrid
                self.vertiport_grid = self._decode_bigrid(response.text)
                self.get_logger().info(f'BiGrid parsed successfully, cell count: {len(self.vertiport_grid.items)}')
                
                # 分析每个cell的占用情况
                self.occupied_cells = set()
                self.available_cells = []
                
                for cell in self.vertiport_grid.items:
                    cell_center_x = float(cell.pose.x)
                    cell_center_y = float(cell.pose.y)
                    cell_width = float(cell.size.width)
                    cell_length = float(cell.size.length)
                    
                    # 检查是否有无人机在这个cell中且高度低于0.1
                    cell_occupied = False
                    cell_drones = []
                    for drone_name, pos in self.drone_positions.items():
                        height = self.drone_heights[drone_name]
                        if self._point_in_cell(pos['x'], pos['y'], cell) and height <= 0.1:
                            cell_drones.append(drone_name)
                            cell_occupied = True
                    
                    if cell_occupied:
                        # 记录所有在这个cell中的无人机
                        if len(cell_drones) == 1:
                            self.get_logger().info(f'Cell {cell.id} occupied by {cell_drones[0]} at height {self.drone_heights[cell_drones[0]]:.3f}m')
                        else:
                            self.get_logger().info(f'Cell {cell.id} occupied by multiple drones: {cell_drones}')
                        self.occupied_cells.add(cell.id)
                    else:
                        self.available_cells.append({
                            'cell': cell,
                            'center_x': cell_center_x,
                            'center_y': cell_center_y,
                            'width': cell_width,
                            'length': cell_length
                        })
                        # 只在第一次分析时打印可用cell信息
                        if self.demo_phase == "analysis":
                            self.get_logger().info(f'Cell {cell.id} is available at center ({cell_center_x:.3f}, {cell_center_y:.3f})')
                
                # 检测碰撞情况
                collisions = self._detect_collisions(self.vertiport_grid)
                if collisions:
                    self.get_logger().warn('COLLISION RISK DETECTED! Multiple drones in same vertiport cell!')
                    for collision in collisions:
                        self.get_logger().warn(
                            f'Cell {collision["cell_id"]} contains multiple drones: {collision["drones"]}'
                        )
                        self.get_logger().warn(
                            f'ALERT: {collision["highest_drone"]} (altitude: {collision["highest_altitude"]:.3f}m) - LEAVE VERTIPORT {collision["cell_id"]}!'
                        )
                        self.get_logger().warn(
                            f'Reason: {collision["highest_drone"]} has the highest altitude and should leave vertiport {collision["cell_id"]}'
                        )
                else:
                    # 只在第一次分析时打印安全信息
                    if self.demo_phase == "analysis":
                        self.get_logger().info('No collision risk detected - vertiport cells are safe')
                
                self.get_logger().info(f'Vertiport analysis complete: {len(self.occupied_cells)} occupied, {len(self.available_cells)} available')
                return True
            else:
                self.get_logger().error(f'Failed to get BiGrid: {response.status_code}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error in vertiport analysis: {str(e)}')
            return False

    def _detect_collisions(self, grid):
        """检测多架无人机在同一vertiport区域的情况"""
        cell_occupancy = {}
        collisions = []
        
        # 检查哪些无人机在哪些cells中
        for drone_name, pos in self.drone_positions.items():
            cell = self._find_cell_for_point(pos['x'], pos['y'], grid)
            if cell is not None:
                cell_id = cell.id
                if cell_id not in cell_occupancy:
                    cell_occupancy[cell_id] = []
                cell_occupancy[cell_id].append(drone_name)
        
        # 检查有多个无人机的cells
        for cell_id, drones in cell_occupancy.items():
            if len(drones) > 1:
                # 找到高度最高的无人机（应该离开）
                highest_drone = max(drones, key=lambda drone: self.drone_heights[drone])
                collisions.append({
                    'cell_id': cell_id,
                    'drones': drones,
                    'highest_drone': highest_drone,
                    'highest_altitude': self.drone_heights[highest_drone]
                })
        
        return collisions

    def _find_nearest_available_cell(self, drone_pos, exclude_cells=None):
        """找到距离无人机最近的可用cell，可以排除已被其他无人机选择的cell"""
        if not self.available_cells:
            return None
            
        min_distance = float('inf')
        nearest_cell = None
        
        for cell_info in self.available_cells:
            # 如果这个cell已被其他无人机选择，跳过
            if exclude_cells and cell_info['cell'].id in exclude_cells:
                continue
                
            distance = np.sqrt((cell_info['center_x'] - drone_pos['x'])**2 + 
                             (cell_info['center_y'] - drone_pos['y'])**2)
            if distance < min_distance:
                min_distance = distance
                nearest_cell = cell_info
        
        return nearest_cell

    def vertiport_analysis_loop(self):
        """初始vertiport分析"""
        if self.demo_phase != "analysis":
            return
            
        # 只执行一次分析，避免重复打印
        self.get_logger().info('Starting initial vertiport analysis...')
        
        # 打印当前无人机位置
        # self.get_logger().info('Current drone positions:')
        # for drone_name, pos in self.drone_positions.items():
        #     height = self.drone_heights[drone_name]
        #     self.get_logger().info(f'{drone_name}: ({pos["x"]:.3f}, {pos["y"]:.3f}, {height:.3f})')
        
        # 分析vertiport占用情况
        if self._analyze_vertiport_occupancy():
            self.get_logger().info('Vertiport analysis completed successfully')
            self.demo_phase = "takeoff"
            
            # 启动演示线程
            demo_thread = threading.Thread(target=self._run_demo)
            demo_thread.start()
        else:
            self.get_logger().error('Vertiport analysis failed')
    
    def periodic_analysis_loop(self):
        """周期性分析，只在特定阶段执行"""
        # 只在takeoff和landing阶段进行周期性分析
        if self.demo_phase not in ["takeoff", "landing"]:
            return
            
        # 检查是否有碰撞风险
        if self.vertiport_grid is not None:
            collisions = self._detect_collisions(self.vertiport_grid)
            if collisions:
                self.get_logger().warn('COLLISION RISK DETECTED during flight!')
                for collision in collisions:
                    self.get_logger().warn(
                        f'Cell {collision["cell_id"]} contains multiple drones: {collision["drones"]}'
                    )
                    self.get_logger().warn(
                        f'ALERT: {collision["highest_drone"]} (altitude: {collision["highest_altitude"]:.3f}m) - LEAVE VERTIPORT {collision["cell_id"]}!'
                    )

    def start_demo(self):
        """启动演示"""
        self.get_logger().info('Starting vertiport landing demo...')
        self.demo_phase = "analysis"
        self.demo_start_time = time.time()
        
        # 直接执行一次分析，不使用定时器
        self.vertiport_analysis_loop()
        
    def _run_demo(self):
        """运行演示序列"""
        try:
            # 阶段1: 确定需要着陆的无人机
            self.get_logger().info('Phase 1: Determining drones that need to land')
            
            drones_to_land = []
            for drone_name, config in self.drone_config.items():
                if config['needs_landing']:
                    drones_to_land.append(drone_name)
                    self.get_logger().info(f'{drone_name} is configured to land in vertiport')
                else:
                    self.get_logger().info(f'{drone_name} is not configured to land (skipping)')
            
            if not drones_to_land:
                self.get_logger().info('No drones configured to land. Demo completed.')
                self.demo_phase = "completed"
                return
            
            # 阶段2: 让需要着陆的无人机起飞（如果还没起飞）
            self.get_logger().info('Phase 2: Taking off drones that need to land')
            self.demo_phase = "takeoff"
            
            for drone_name in drones_to_land:
                controller = self.drone_controllers[drone_name]
                
                # 检查当前状态
                current_state = controller.get_current_state()
                self.get_logger().info(f'{drone_name} current state: {current_state}')
                
                if current_state == "idle":
                    # 如果在地面，需要起飞
                    self.get_logger().info(f'{drone_name} is on ground, starting takeoff')
                    
                    # 起飞
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
            
            # 记录悬停等待开始时间
            hover_wait_start_time = time.time()
            self.get_logger().info(f'Hover wait started at: {hover_wait_start_time:.3f}')
            
            # 起飞后检测碰撞情况
            self.get_logger().info('Checking for collision risks after takeoff...')
            vertiport_analysis_start = time.time()
            if not self._analyze_vertiport_occupancy():
                self.get_logger().error('Failed to analyze vertiport occupancy after takeoff')
                return
            vertiport_analysis_end = time.time()
            self.get_logger().info(f'First vertiport analysis completed in: {vertiport_analysis_end - vertiport_analysis_start:.3f} seconds')
            
            # 阶段3: 直接导航到最近的空vertiport并着陆
            self.get_logger().info('Phase 3: Direct navigation to nearest available vertiport and landing')
            self.demo_phase = "landing"
            
            # 等待一段时间让无人机移动到新位置，然后再次分析
            # 注意：这里进行第二次分析的原因是：
            # 1. 第一次分析时无人机可能还在调整位置，数据不够稳定
            # 2. 5秒后无人机位置更稳定，基于最新数据进行vertiport分配更准确
            # 3. 两次分析可以检测位置变化，确保系统安全性
            # 如果觉得冗余，可以注释掉第一次分析，只保留这次分析
            time.sleep(5.0)
            self.get_logger().info('Re-analyzing vertiport occupancy before landing...')
            second_analysis_start = time.time()
            if not self._analyze_vertiport_occupancy():
                self.get_logger().error('Failed to re-analyze vertiport occupancy before landing')
                return
            second_analysis_end = time.time()
            self.get_logger().info(f'Second vertiport analysis completed in: {second_analysis_end - second_analysis_start:.3f} seconds')
            
            # 为所有需要着陆的无人机预先分配vertiport，避免冲突
            self.get_logger().info('Pre-allocating vertiports to avoid conflicts...')
            allocation_start = time.time()
            drone_assignments = {}  # 存储无人机分配的vertiport
            assigned_cells = set()  # 已分配的cell ID
            
            # 首先处理碰撞情况
            collisions = self._detect_collisions(self.vertiport_grid)
            if collisions:
                self.get_logger().warn('COLLISION DETECTED! Handling collision situation...')
                
                for collision in collisions:
                    highest_drone = collision['highest_drone']
                    cell_id = collision['cell_id']
                    
                    if highest_drone in drones_to_land:
                        self.get_logger().warn(f'ALERT: {highest_drone} in collision cell {cell_id} - IMMEDIATE LANDING REQUIRED!')
                        
                        pos = self.drone_positions[highest_drone]
                        nearest_cell = self._find_nearest_available_cell(pos, assigned_cells)
                        
                        if nearest_cell:
                            drone_assignments[highest_drone] = nearest_cell
                            assigned_cells.add(nearest_cell["cell"].id)
                            self.get_logger().info(f'Assigned {highest_drone} to vertiport {nearest_cell["cell"].id}')
                        else:
                            self.get_logger().error(f'No available vertiport found for emergency landing of {highest_drone}')
                            return
            
            # 为剩余的无人机分配vertiport
            for drone_name in drones_to_land:
                if drone_name not in drone_assignments:
                    pos = self.drone_positions[drone_name]
                    nearest_cell = self._find_nearest_available_cell(pos, assigned_cells)
                    
                    if nearest_cell:
                        drone_assignments[drone_name] = nearest_cell
                        assigned_cells.add(nearest_cell["cell"].id)
                        self.get_logger().info(f'Assigned {drone_name} to vertiport {nearest_cell["cell"].id}')
                    else:
                        self.get_logger().error(f'No available vertiport found for {drone_name}')
                        return
            
            allocation_end = time.time()
            self.get_logger().info(f'Vertiport allocation completed in: {allocation_end - allocation_start:.3f} seconds')
            
            # 执行着陆
            for drone_name, assigned_cell in drone_assignments.items():
                controller = self.drone_controllers[drone_name]
                
                # 第一步：导航到cell中心上方
                target_x = assigned_cell['center_x']
                target_y = assigned_cell['center_y']
                target_z = 0.6  # 初始高度
                
                # 记录发送导航指令的时间
                navigation_command_time = time.time()
                hover_to_navigation_time = navigation_command_time - hover_wait_start_time
                self.get_logger().info(f'{drone_name} navigation command sent at: {navigation_command_time:.3f}')
                self.get_logger().info(f'{drone_name} time from hover wait to navigation command: {hover_to_navigation_time:.3f} seconds')
                
                # 详细时间分解
                first_analysis_time = vertiport_analysis_end - vertiport_analysis_start
                wait_time = 5.0  # 固定的等待时间
                second_analysis_time = second_analysis_end - second_analysis_start
                allocation_time = allocation_end - allocation_start
                
                self.get_logger().info(f'{drone_name} time breakdown:')
                self.get_logger().info(f'  - First vertiport analysis: {first_analysis_time:.3f}s')
                self.get_logger().info(f'  - Wait time: {wait_time:.3f}s')
                self.get_logger().info(f'  - Second vertiport analysis: {second_analysis_time:.3f}s')
                self.get_logger().info(f'  - Vertiport allocation: {allocation_time:.3f}s')
                self.get_logger().info(f'  - Total processing time: {hover_to_navigation_time:.3f}s')
                
                self.get_logger().info(f'{drone_name} navigating to assigned vertiport center ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})')
                
                if not controller.navigate_to(target_x, target_y, target_z):
                    self.get_logger().error(f'Failed to send navigation command to {drone_name}')
                    return
                
                if not controller.wait_for_state("hovering", timeout=30.0):
                    self.get_logger().error(f'{drone_name} failed to reach vertiport center')
                    return
                
                time.sleep(2.0)  # 稳定悬停
                
                # 第二步：降低高度到0.2m并悬停2秒
                low_altitude_z = 0.2  # 降低到0.2m
                self.get_logger().info(f'{drone_name} lowering altitude to {low_altitude_z:.3f}m above vertiport {assigned_cell["cell"].id}')
                
                if not controller.navigate_to(target_x, target_y, low_altitude_z):
                    self.get_logger().error(f'Failed to send low altitude command to {drone_name}')
                    return
                
                if not controller.wait_for_state("hovering", timeout=30.0):
                    self.get_logger().error(f'{drone_name} failed to reach low altitude position')
                    return
                
                self.get_logger().info(f'{drone_name} hovering at {low_altitude_z:.3f}m for 2 seconds before landing')
                time.sleep(2.0)  # 在0.2m高度悬停2秒
                
                # 第三步：开始着陆
                self.get_logger().info(f'{drone_name} starting landing in vertiport {assigned_cell["cell"].id}')
                
                if not controller.begin_landing():
                    self.get_logger().error(f'Failed to start landing for {drone_name}')
                    return
                
                if not controller.wait_for_state("idle", timeout=30.0):
                    self.get_logger().error(f'{drone_name} failed to complete landing')
                    return
                
                self.get_logger().info(f'{drone_name} successfully landed in vertiport {assigned_cell["cell"].id}')
                
                # 从可用列表中移除这个cell
                self.available_cells.remove(assigned_cell)
                self.occupied_cells.add(assigned_cell["cell"].id)
            
            self.get_logger().info('All configured drones have successfully landed in vertiports!')
            self.demo_phase = "completed"
            
        except Exception as e:
            self.get_logger().error(f'Demo failed with error: {str(e)}')
            self.demo_phase = "error"

def main():
    rclpy.init()
    
    # 方式1: 使用默认配置（2架无人机都需要着陆）
    # node = VertiportLandingDemo()
    
    # 方式2: 自定义配置 - 指定哪些无人机需要着陆
    drone_config = {
        'cf231': {'port': 5000, 'needs_landing': True},   
        'cf232': {'port': 5001, 'needs_landing': False},  
        'cf233': {'port': 5002, 'needs_landing': False},  
        # 'cf234': {'port': 5003, 'needs_landing': False},  
        # 'cf235': {'port': 5004, 'needs_landing': True}, 
    }
    node = VertiportLandingDemo(drone_config)
    
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
