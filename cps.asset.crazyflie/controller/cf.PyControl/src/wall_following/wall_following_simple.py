#!/usr/bin/env python3
"""
简化版墙面跟随算法 - 专为矩形场地设计

特点：
1. 只有两个状态：沿墙直行、转弯
2. 固定转弯角度90度
3. 只检测前方和跟随侧的墙面
4. 简单可靠，不会卡住
"""

import math
from enum import Enum


class SimpleWallFollowing:
    """简化的矩形墙面跟随算法"""
    
    class State(Enum):
        """简化状态"""
        FORWARD_ALONG_WALL = 1  # 沿墙直行
        TURNING_AT_CORNER = 2   # 在角落转弯
        HOVER = 3               # 悬停
    
    class Direction(Enum):
        """墙面跟随方向"""
        LEFT = 1   # 墙在左侧
        RIGHT = -1  # 墙在右侧
    
    def __init__(
        self,
        target_wall_distance=0.3,     # 目标墙面距离(m)
        max_forward_speed=0.15,       # 最大前向速度(m/s)
        max_turn_rate=0.5,            # 最大转向速率(rad/s)
        front_wall_threshold=0.35,    # 前方墙面触发转弯阈值(m)
        wall_distance_tolerance=0.05, # 墙面距离容差(m)
        turn_angle=math.pi/2,         # 转弯角度(90度)
        init_state=None
    ):
        """
        初始化简化墙面跟随算法
        
        Args:
            target_wall_distance: 与侧墙保持的目标距离
            max_forward_speed: 最大前向速度
            max_turn_rate: 最大转向速率
            front_wall_threshold: 前方墙面距离小于此值时开始转弯
            wall_distance_tolerance: 墙面距离控制容差
            turn_angle: 每次转弯的角度（默认90度）
            init_state: 初始状态
        """
        self.target_wall_distance = target_wall_distance
        self.max_forward_speed = max_forward_speed
        self.max_turn_rate = max_turn_rate
        self.front_wall_threshold = front_wall_threshold
        self.wall_distance_tolerance = wall_distance_tolerance
        self.turn_angle = turn_angle
        
        self.state = init_state or self.State.FORWARD_ALONG_WALL
        self.direction_value = 1.0
        
        # 转弯状态变量
        self.turn_start_heading = 0.0
        self.target_heading = 0.0
        self.turn_start_time = 0.0  # 转弯开始时间
        self.turn_timeout = 5.0  # 转弯超时时间（秒）
        self.time_now = 0.0
        
        # 速度控制参数
        self.lateral_speed_factor = 2.0  # 侧向修正速度因子
    
    def wrap_angle(self, angle):
        """将角度规范化到[-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def calculate_target_heading(self, current_heading, direction):
        """计算转弯后的目标航向
        
        Args:
            current_heading: 当前航向(rad)
            direction: 跟随方向(Direction枚举)
        
        Returns:
            目标航向(rad)
        """
        # 右墙跟随（墙在右侧）：向左转90度（逆时针）
        # 左墙跟随（墙在左侧）：向右转90度（顺时针）
        turn_direction = 1.0 if direction == self.Direction.RIGHT else -1.0
        target = current_heading + turn_direction * self.turn_angle
        return self.wrap_angle(target)
    
    def control_along_wall(self, side_range):
        """沿墙直行的控制逻辑
        
        Args:
            side_range: 侧方墙面距离
            
        Returns:
            (velocity_x, velocity_y): 速度命令
        """
        # 前向速度
        velocity_x = self.max_forward_speed
        
        # 侧向修正：保持与墙面的目标距离
        distance_error = side_range - self.target_wall_distance
        
        if abs(distance_error) < self.wall_distance_tolerance:
            # 距离合适，不需要侧向修正
            velocity_y = 0.0
        else:
            # 需要侧向修正
            # 机体系：vy > 0 表示向左，vy < 0 表示向右
            # 我们定义：Direction.RIGHT 表示墙在右侧；Direction.LEFT 表示墙在左侧
            # - 贴右墙：distance_error>0(离墙太远) => 向右靠近 => vy<0
            # - 贴左墙：distance_error>0(离墙太远) => 向左靠近 => vy>0
            correction_speed = min(
                self.max_forward_speed / self.lateral_speed_factor,
                abs(distance_error) * 0.5
            )
            velocity_y = self.direction_value * math.copysign(correction_speed, distance_error)
        
        return velocity_x, velocity_y
    
    def control_turning(self, current_heading):
        """转弯控制逻辑
        
        Args:
            current_heading: 当前航向
            
        Returns:
            yaw_rate: 转向速率
        """
        # 计算航向误差
        heading_error = self.wrap_angle(self.target_heading - current_heading)
        
        # 简单比例控制
        if abs(heading_error) < 0.1:  # 放宽到约6度，提高转弯完成判断容差
            # 转弯完成
            return 0.0
        else:
            # 继续转弯：直接根据误差符号决定转向方向
            # heading_error > 0: 需要逆时针转（yaw_rate > 0）
            # heading_error < 0: 需要顺时针转（yaw_rate < 0）
            # 注意：Crazyflie的yaw_rate定义和标准相反，所以取负!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            yaw_rate = -math.copysign(self.max_turn_rate, heading_error)
            return yaw_rate
    
    def update(self, front_range, side_range, current_heading, direction, time_now):
        """更新墙面跟随控制
        
        Args:
            front_range: 前方墙面距离(m)
            side_range: 侧方墙面距离(m)
            current_heading: 当前航向(rad)
            direction: 跟随方向(Direction枚举)
            time_now: 当前时间(s)
            
        Returns:
            (velocity_x, velocity_y, yaw_rate, state): 控制命令和当前状态
        """
        self.time_now = time_now
        self.direction_value = float(direction.value)
        
        # 安全检查：防止无效距离值
        if front_range <= 0 or front_range == float('inf'):
            front_range = 999.0
        if side_range <= 0 or side_range == float('inf'):
            side_range = 999.0
        
        velocity_x = 0.0
        velocity_y = 0.0
        yaw_rate = 0.0
        
        # ========== 状态机逻辑 ========== #
        
        if self.state == self.State.FORWARD_ALONG_WALL:
            # 状态：沿墙直行
            
            # 检查是否需要转弯
            if front_range < self.front_wall_threshold:
                # 前方墙面过近，进入转弯状态
                self.state = self.State.TURNING_AT_CORNER
                self.turn_start_heading = current_heading
                self.turn_start_time = time_now  # 记录转弯开始时间
                self.target_heading = self.calculate_target_heading(current_heading, direction)
                # 立即执行转弯控制
                velocity_x = 0.0
                velocity_y = 0.0
                yaw_rate = self.control_turning(current_heading)
            else:
                # 继续沿墙直行
                velocity_x, velocity_y = self.control_along_wall(side_range)
                yaw_rate = 0.0
        
        elif self.state == self.State.TURNING_AT_CORNER:
            # 状态：转弯中
            
            # 停止前进，只转向
            velocity_x = 0.0
            velocity_y = 0.0
            yaw_rate = self.control_turning(current_heading)
            
            # 检查转弯是否完成（放宽容差）
            heading_error = abs(self.wrap_angle(self.target_heading - current_heading))
            turn_duration = time_now - self.turn_start_time
            
            if heading_error < 0.1:  # 约6度（放宽容差）
                # 转弯完成，返回沿墙直行状态
                self.state = self.State.FORWARD_ALONG_WALL
            elif turn_duration > self.turn_timeout:
                # 转弯超时，强制退出转弯状态
                print(f"⚠️ Turn timeout after {turn_duration:.1f}s, forcing exit")
                self.state = self.State.FORWARD_ALONG_WALL
        
        elif self.state == self.State.HOVER:
            # 状态：悬停
            velocity_x = 0.0
            velocity_y = 0.0
            yaw_rate = 0.0
        
        return velocity_x, velocity_y, yaw_rate, self.state
    
    def hover(self):
        """切换到悬停状态"""
        self.state = self.State.HOVER
    
    def resume(self):
        """从悬停恢复"""
        if self.state == self.State.HOVER:
            self.state = self.State.FORWARD_ALONG_WALL

