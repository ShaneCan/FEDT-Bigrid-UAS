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
        # 网格索引与分类
        self.grid_index = None  # {'rows':int,'cols':int,'xs':list,'ys':list,'id_by_rc':dict,'rc_by_id':dict,'center_by_id':dict}
        self.flying_area_cells = set()
        self.vertiport_pad_cells = set()
        # 占用映射
        self.cell_occupancy = {}
        
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
    def _cluster_sorted_values(values, default_tol=1e-3):
        """对已排序的浮点数组进行聚类去重，返回聚类中心列表。
        tol 由最小相邻差值的一半决定，若不可得则使用默认。
        """
        if not values:
            return []
        sv = sorted(values)
        diffs = [abs(sv[i+1] - sv[i]) for i in range(len(sv)-1)]
        tol = default_tol
        if diffs:
            positive_diffs = [d for d in diffs if d > 0]
            if positive_diffs:
                tol = min(positive_diffs) * 0.5
        clusters = []
        current_cluster = [sv[0]]
        for v in sv[1:]:
            if abs(v - current_cluster[-1]) <= tol:
                current_cluster.append(v)
            else:
                clusters.append(sum(current_cluster) / len(current_cluster))
                current_cluster = [v]
        clusters.append(sum(current_cluster) / len(current_cluster))
        return clusters

    def _build_grid_index(self, grid):
        """从BiGrid构建网格索引(rows, cols)，并建立(row,col)<->cell.id映射。"""
        centers_x = [float(c.pose.x) for c in grid.items]
        centers_y = [float(c.pose.y) for c in grid.items]
        print(f'centers_x: {centers_x}')
        print(f'centers_y: {centers_y}')
        xs = self._cluster_sorted_values(centers_x)
        ys = self._cluster_sorted_values(centers_y)
        rows = len(ys)
        cols = len(xs)
        id_by_rc = {}
        rc_by_id = {}
        center_by_id = {}
        for cell in grid.items:
            cx = float(cell.pose.x)
            cy = float(cell.pose.y)
            # 找到最近的索引
            col = int(np.argmin([abs(cx - vx) for vx in xs]))
            row = int(np.argmin([abs(cy - vy) for vy in ys]))
            id_by_rc[(row, col)] = cell.id
            rc_by_id[cell.id] = (row, col)
            center_by_id[cell.id] = (cx, cy)
        self.grid_index = {
            'rows': rows,
            'cols': cols,
            'xs': xs,
            'ys': ys,
            'id_by_rc': id_by_rc,
            'rc_by_id': rc_by_id,
            'center_by_id': center_by_id,
        }
        # 分类外圈为FlyingArea，中间为VertiportPad（自适应任意rows/cols）
        self.flying_area_cells = set()
        self.vertiport_pad_cells = set()
        for r in range(rows):
            for c in range(cols):
                cid = id_by_rc.get((r, c))
                if cid is None:
                    continue
                if r == 0 or c == 0 or r == rows - 1 or c == cols - 1:
                    self.flying_area_cells.add(cid)
                else:
                    self.vertiport_pad_cells.add(cid)
        print(f'flying_area_cells: {self.flying_area_cells}')
        print(f'vertiport_pad_cells: {self.vertiport_pad_cells}')

    def _cell_center(self, cell_id):
        if self.grid_index and cell_id in self.grid_index['center_by_id']:
            return self.grid_index['center_by_id'][cell_id]
        # 兜底：遍历查找
        for cell in (self.vertiport_grid.items if self.vertiport_grid else []):
            if cell.id == cell_id:
                return float(cell.pose.x), float(cell.pose.y)
        return None

    def _rc_neighbors(self, row, col):
        """返回八方向相邻格的(row,col)，边界安全。"""
        if not self.grid_index:
            return []
        rows = self.grid_index['rows']
        cols = self.grid_index['cols']
        nbrs = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                rr = row + dr
                cc = col + dc
                if 0 <= rr < rows and 0 <= cc < cols:
                    nbrs.append((rr, cc))
        return nbrs

    def _nearest_cell_of(self, cell_id_set, from_xy):
        """在给定cell集合中，查找距离点from_xy最近的cell信息返回(cell_id, center_x, center_y)。"""
        if not cell_id_set:
            return None
        min_d = float('inf')
        best = None
        for cid in cell_id_set:
            center = self._cell_center(cid)
            if center is None:
                continue
            d = np.hypot(center[0] - from_xy['x'], center[1] - from_xy['y'])
            if d < min_d:
                min_d = d
                best = (cid, center[0], center[1])
        return best

    def _nearest_available_pad(self, from_xy):
        """找到最近的空的VertiportPad cell。"""
        candidates = []
        for cid in self.vertiport_pad_cells:
            occ = self.cell_occupancy.get(cid, {})
            if not occ.get('occupied', False):
                center = self._cell_center(cid)
                if center is None:
                    continue
                d = np.hypot(center[0] - from_xy['x'], center[1] - from_xy['y'])
                candidates.append((d, cid, center))
        if not candidates:
            return None
        candidates.sort(key=lambda x: x[0])
        _, cid, center = candidates[0]
        return {'cell_id': cid, 'center_x': center[0], 'center_y': center[1]}

    def _nearest_available_pad_with_exclusions(self, from_xy, exclude_cells):
        """找到最近的空的VertiportPad cell，排除指定的cells。"""
        candidates = []
        for cid in self.vertiport_pad_cells:
            # 跳过被排除的cells
            if cid in exclude_cells:
                continue
            occ = self.cell_occupancy.get(cid, {})
            if not occ.get('occupied', False):
                center = self._cell_center(cid)
                if center is None:
                    continue
                d = np.hypot(center[0] - from_xy['x'], center[1] - from_xy['y'])
                candidates.append((d, cid, center))
        if not candidates:
            return None
        candidates.sort(key=lambda x: x[0])
        _, cid, center = candidates[0]
        return {'cell_id': cid, 'center_x': center[0], 'center_y': center[1]}

    def _cell_is_occupied_by_landing(self, cell_id):
        occ = self.cell_occupancy.get(cell_id, {})
        return occ.get('occupied', False) and occ.get('any_landing', False)

    def _cell_is_occupied_by_non_landing(self, cell_id):
        occ = self.cell_occupancy.get(cell_id, {})
        if not occ.get('occupied', False):
            return False
        # 有无人机且没有任何处于降落状态(z<1)的无人机 => 视为非降落占用
        return not occ.get('any_landing', False)

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
                "x": -2.0,
                "y": -2.0,
                "stepSizeX": 1.0,
                "stepSizeY": 1.0
            }
            
            self.get_logger().info('Requesting BiGrid from Java service for vertiport analysis...')
            
            response = requests.post(
                "http://172.29.224.1:8080/generate/bigrid?rows=4&cols=5&format=protobuf",
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
                # 构建网格索引与分类（外圈=FlyingArea，内圈=VertiportPad）
                self._build_grid_index(self.vertiport_grid)
                self.get_logger().info(f'BiGrid parsed successfully, cell count: {len(self.vertiport_grid.items)}')
                
                # 分析每个cell的占用情况
                self.occupied_cells = set()
                self.available_cells = []
                self.cell_occupancy = {}
                
                for cell in self.vertiport_grid.items:
                    cell_center_x = float(cell.pose.x)
                    cell_center_y = float(cell.pose.y)
                    cell_width = float(cell.size.width)
                    cell_length = float(cell.size.length)
                    
                    # 检查是否有无人机在这个cell中且高度低于0.1
                    cell_occupied = False
                    cell_drones = []
                    any_landing = False  # 是否有z<0.1的无人机，视为处于降落状态
                    for drone_name, pos in self.drone_positions.items():
                        height = self.drone_heights[drone_name]
                        if self._point_in_cell(pos['x'], pos['y'], cell) and height <= 0.1:
                            cell_drones.append(drone_name)
                            cell_occupied = True
                            any_landing = True
                    
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

                    # 写入占用映射
                    self.cell_occupancy[cell.id] = {
                        'occupied': cell_occupied,
                        'drones': cell_drones,
                        'any_landing': any_landing,
                        'is_pad': cell.id in self.vertiport_pad_cells,
                        'is_flying_area': cell.id in self.flying_area_cells,
                        'center': (cell_center_x, cell_center_y),
                    }
                
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

    def _get_current_cell_rc(self, drone_name):
        """返回无人机所在的(row,col)，若不在任何cell内则返回None。"""
        if not self.vertiport_grid or not self.grid_index:
            return None
        pos = self.drone_positions[drone_name]
        cell = self._find_cell_for_point(pos['x'], pos['y'], self.vertiport_grid)
        if cell is None:
            return None
        return self.grid_index['rc_by_id'].get(cell.id)

    def _choose_next_step_rc(self, current_rc, target_pad_id):
        """根据目标pad，选择下一步相邻格的rc。
        优先选择距离目标最近的可用邻居（未占用或被降落状态占用）；
        避免选择被非降落无人机占据的格。
        """
        if not self.grid_index:
            return None
        id_by_rc = self.grid_index['id_by_rc']
        rc_by_id = self.grid_index['rc_by_id']
        target_rc = rc_by_id.get(target_pad_id)
        if target_rc is None:
            return None
        neighbors = self._rc_neighbors(current_rc[0], current_rc[1])
        if not neighbors:
            return None
        def rc_center(rc):
            cid = id_by_rc.get(rc)
            return self._cell_center(cid)
        def dist_to_target(rc):
            cc = rc_center(rc)
            tc = rc_center(target_rc)
            if cc is None or tc is None:
                return float('inf')
            return np.hypot(cc[0]-tc[0], cc[1]-tc[1])
        # 过滤非法的邻居（不存在的cell）
        valid_neighbors = [rc for rc in neighbors if id_by_rc.get(rc) is not None]
        if not valid_neighbors:
            return None
        
        # 收集所有可用的邻居（未占用或被降落状态占用）
        available_nbrs = []
        for rc in valid_neighbors:
            cid = id_by_rc.get(rc)
            occ = self.cell_occupancy.get(cid, {})
            
            # 未占用
            if not occ.get('occupied', False):
                available_nbrs.append((rc, 'free'))
            # 被降落状态占用
            elif self._cell_is_occupied_by_landing(cid):
                available_nbrs.append((rc, 'landing'))
            # 被非降落状态占用 - 不可用
            # else: 跳过
        
        if not available_nbrs:
            return None
        
        # 按距离目标排序，优先选择距离最近的
        available_nbrs.sort(key=lambda x: dist_to_target(x[0]))
        
        # 返回距离最近的可用邻居
        return available_nbrs[0][0]

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

    # def vertiport_analysis_loop(self):
    #     """初始vertiport分析 - 已移除，不再在起飞前分析"""
    #     # 此方法已废弃，分析现在只在起飞后进行
    #     pass
    
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
        self.demo_phase = "takeoff"
        self.demo_start_time = time.time()
        
        # 直接启动演示线程，不再进行起飞前分析
        demo_thread = threading.Thread(target=self._run_demo)
        demo_thread.start()
        
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
            
            # 阶段3: 起飞后进行分析，然后逐格移动到最近的空VertiportPad并着陆
            self.get_logger().info('Phase 3: Analyzing vertiport occupancy after takeoff, then stepwise navigation to landing')
            self.demo_phase = "landing"
            
            # 起飞后进行分析
            self.get_logger().info('Analyzing vertiport occupancy after takeoff...')
            if not self._analyze_vertiport_occupancy():
                self.get_logger().error('Failed to analyze vertiport occupancy after takeoff')
                return
            
            # 预分配vertiport给每架无人机，避免冲突
            self.get_logger().info('Pre-allocating vertiports to avoid conflicts...')
            drone_assignments = {}  # 存储无人机分配的vertiport
            assigned_cells = set()  # 已分配的cell ID
            
            for drone_name in drones_to_land:
                pos = self.drone_positions[drone_name]
                # 寻找最近的空VertiportPad，排除已被分配的
                target_pad = self._nearest_available_pad_with_exclusions(pos, assigned_cells)
                if target_pad is None:
                    self.get_logger().error(f'No available VertiportPad found for {drone_name}')
                    return
                drone_assignments[drone_name] = target_pad
                assigned_cells.add(target_pad['cell_id'])
                self.get_logger().info(f'Assigned {drone_name} to vertiport {target_pad["cell_id"]}')
            
            # 对每架需要着陆的无人机，执行逐格移动与再分析策略
            for drone_name in drones_to_land:
                move_cycle = 0
                controller = self.drone_controllers[drone_name]
                while True:
                    move_cycle += 1
                    # 每轮先分析最新占用
                    if not self._analyze_vertiport_occupancy():
                        self.get_logger().error('Analysis failed during stepwise navigation')
                        return
                    pos = self.drone_positions[drone_name]
                    height = self.drone_heights[drone_name]
                    current_rc = self._get_current_cell_rc(drone_name)
                    # 如果不在任何cell，先前往最近的FlyingArea cell
                    if current_rc is None:
                        self.get_logger().info(f'{drone_name} is outside grid, moving to nearest FlyingArea cell first')
                        nearest_fa = self._nearest_cell_of(self.flying_area_cells, pos)
                        if nearest_fa is None:
                            self.get_logger().error('No FlyingArea cell available')
                            return
                        tx, ty = nearest_fa[1], nearest_fa[2]
                        if not controller.navigate_to(tx, ty, 0.6):
                            self.get_logger().error(f'Failed to navigate to nearest FlyingArea cell')
                            return
                        if not controller.wait_for_state("hovering", timeout=30.0):
                            self.get_logger().error(f'{drone_name} failed to reach FlyingArea cell')
                            return
                        time.sleep(2.0)
                        continue  # 再分析
                    # 使用预分配的目标pad
                    target_pad = drone_assignments[drone_name]
                    target_pad_id = target_pad['cell_id']
                    target_center = (target_pad['center_x'], target_pad['center_y'])
                    # 如果相邻格中存在空的目标pad，直接过去并着陆
                    id_by_rc = self.grid_index['id_by_rc']
                    neighbors = self._rc_neighbors(current_rc[0], current_rc[1])
                    neighbor_ids = [id_by_rc.get(rc) for rc in neighbors]
                    if target_pad_id in neighbor_ids:
                        self.get_logger().info(f'{drone_name} target pad is adjacent, moving to land')
                        tx, ty = target_center
                        if not controller.navigate_to(tx, ty, 0.6):
                            self.get_logger().error('Failed to navigate to adjacent target pad')
                            return
                        if not controller.wait_for_state("hovering", timeout=30.0):
                            self.get_logger().error('Failed to reach adjacent target pad center')
                            return
                        time.sleep(2.0)
                        # 下降并着陆
                        low_altitude_z = 0.2
                        if not controller.navigate_to(tx, ty, low_altitude_z):
                            self.get_logger().error('Failed to descend before landing')
                            return
                        if not controller.wait_for_state("hovering", timeout=30.0):
                            self.get_logger().error('Failed to reach low altitude')
                            return
                        time.sleep(2.0)
                        if not controller.begin_landing():
                            self.get_logger().error('Failed to start landing')
                            return
                        if not controller.wait_for_state("idle", timeout=30.0):
                            self.get_logger().error('Failed to complete landing')
                            return
                        self.get_logger().info(f'{drone_name} successfully landed on pad {target_pad_id}')
                        break
                    # 否则，选择一个相邻格作为下一步
                    next_rc = self._choose_next_step_rc(current_rc, target_pad_id)
                    if next_rc is None:
                        self.get_logger().warn('No valid neighboring cell to step into, waiting and retrying')
                        time.sleep(2.0)
                        continue
                    next_cid = id_by_rc.get(next_rc)
                    nc = self._cell_center(next_cid)
                    if nc is None:
                        self.get_logger().warn('Next cell has no center, skipping')
                        time.sleep(1.0)
                        continue
                    self.get_logger().info(f'{drone_name} step {move_cycle}: moving from grid {current_rc} -> {next_rc} (cell {next_cid} at position {nc[0]:.1f}, {nc[1]:.1f})')
                    if not controller.navigate_to(nc[0], nc[1], 0.6):
                        self.get_logger().error('Failed to navigate to next neighboring cell')
                        return
                    if not controller.wait_for_state("hovering", timeout=30.0):
                        self.get_logger().error('Failed to reach next neighboring cell')
                        return
                    # 如果该格被处于降落状态的无人机占用，则悬停2s再分析
                    if self._cell_is_occupied_by_landing(next_cid):
                        self.get_logger().info('Neighbor cell occupied by landing drone, hovering 2s before re-analysis')
                        time.sleep(2.0)
                    else:
                        time.sleep(1.0)
            
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
        'cf231': {'port': 5000, 'needs_landing': False},   
        'cf232': {'port': 5001, 'needs_landing': True},  
        'cf233': {'port': 5002, 'needs_landing': True},  
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
