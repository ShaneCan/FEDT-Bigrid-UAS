#!/usr/bin/env python3
"""
墙面跟随状态机类，集成WallFollowing算法并支持状态机图片生成
"""

import os
import time
import threading
from typing import Optional
from statemachine import StateMachine, State
from statemachine.contrib.diagram import DotGraphMachine

from .wall_following import WallFollowing


class WallFollowingStateMachine(StateMachine):
    """墙面跟随状态机，支持图片生成和实时状态跟踪"""
    
    # 定义墙面跟随状态
    idle = State('IDLE', initial=True)
    forward = State('FORWARD')
    hover = State('HOVER')
    turn_to_find_wall = State('TURN_TO_FIND_WALL')
    turn_to_align_to_wall = State('TURN_TO_ALIGN_TO_WALL')
    forward_along_wall = State('FORWARD_ALONG_WALL')
    rotate_around_wall = State('ROTATE_AROUND_WALL')
    rotate_in_corner = State('ROTATE_IN_CORNER')
    find_corner = State('FIND_CORNER')
    stopped = State('STOPPED', final=True)
    
    # 定义状态转换
    begin_forward = idle.to(forward)
    wall_detected = forward.to(turn_to_find_wall)
    wall_found = turn_to_find_wall.to(turn_to_align_to_wall)
    wall_aligned = turn_to_align_to_wall.to(forward_along_wall)
    wall_lost = forward_along_wall.to(find_corner)
    corner_detected = forward_along_wall.to(rotate_in_corner)
    corner_rotated = rotate_in_corner.to(turn_to_find_wall)
    corner_found = find_corner.to(rotate_around_wall)
    wall_rediscovered = rotate_around_wall.to(turn_to_find_wall)
    hover_command = forward.to(hover)
    hover_command_from_align = turn_to_align_to_wall.to(hover)
    hover_command_from_along = forward_along_wall.to(hover)
    hover_command_from_corner = rotate_in_corner.to(hover)
    hover_command_from_find = find_corner.to(hover)
    hover_command_from_rotate = rotate_around_wall.to(hover)
    resume_from_hover = hover.to(forward)
    stop_wall_following_transition = forward.to(stopped)
    stop_from_hover = hover.to(stopped)
    stop_from_align = turn_to_align_to_wall.to(stopped)
    stop_from_along = forward_along_wall.to(stopped)
    stop_from_corner = rotate_in_corner.to(stopped)
    stop_from_find = find_corner.to(stopped)
    stop_from_rotate = rotate_around_wall.to(stopped)
    
    def __init__(self, drone_id: str, image_output_base_dir: str = "", **wall_following_kwargs):
        """初始化墙面跟随状态机
        
        Args:
            drone_id: 无人机ID
            image_output_base_dir: 图片输出基础目录，为空则使用默认路径
            **wall_following_kwargs: 传递给WallFollowing的参数
        """
        self._drone_id = drone_id
        self._last_state = None
        self._image_numbers = {}
        self._graph_lock = threading.Lock()
        self._last_graph_update = 0
        self._graph_update_interval = 0.1
        
        # 设置图片输出目录
        if not image_output_base_dir:
            image_output_base_dir = os.path.join(
                os.path.dirname(__file__), '..', '..', 'webview', 'wall_following', 'img'
            )
        self._image_output_base_dir = os.path.abspath(image_output_base_dir)
        
        # 初始化图片编号
        if self._drone_id not in self._image_numbers:
            img_dir = self._get_image_dir()
            latest_file = os.path.join(img_dir, "latest.txt")
            if os.path.exists(latest_file):
                try:
                    with open(latest_file, 'r') as f:
                        latest_number = int(f.read().strip())
                    self._image_numbers[self._drone_id] = (latest_number % 100) + 1
                except Exception as e:
                    print(f"Error reading latest.txt: {e}")
                    self._image_numbers[self._drone_id] = 1
            else:
                self._image_numbers[self._drone_id] = 1
        
        # 确保图片目录和latest.txt文件存在
        img_dir = self._get_image_dir()
        os.makedirs(img_dir, exist_ok=True)
        latest_file = os.path.join(img_dir, "latest.txt")
        if not os.path.exists(latest_file):
            with open(latest_file, 'w') as f:
                f.write(str(self._image_numbers[self._drone_id]))
        
        # 初始化WallFollowing算法
        self.wall_following = WallFollowing(**wall_following_kwargs)
        
        # 当前状态变量
        self.current_velocity_x = 0.0
        self.current_velocity_y = 0.0
        self.current_yaw_rate = 0.0
        self.current_wall_state = None
        
        # 最后调用父类的初始化
        super().__init__()
    
    @property
    def drone_id(self):
        return self._drone_id
    
    def _get_image_dir(self) -> str:
        """获取图片输出目录"""
        return os.path.join(self._image_output_base_dir, self._drone_id)
    
    def update_wall_following(self, front_range, side_range, current_heading, 
                            wall_following_direction, time_outer_loop):
        """更新墙面跟随算法并同步状态机状态
        
        Args:
            front_range: 前方距离
            side_range: 侧方距离  
            current_heading: 当前航向
            wall_following_direction: 墙面跟随方向
            time_outer_loop: 当前时间
            
        Returns:
            tuple: (velocity_x, velocity_y, yaw_rate, wall_state)
        """
        # 调用原始墙面跟随算法
        velocity_x, velocity_y, yaw_rate, wall_state = self.wall_following.wall_follower(
            front_range, side_range, current_heading, wall_following_direction, time_outer_loop
        )
        
        # 更新当前状态变量
        self.current_velocity_x = velocity_x
        self.current_velocity_y = velocity_y
        self.current_yaw_rate = yaw_rate
        self.current_wall_state = wall_state
        
        # 根据墙面跟随状态更新状态机
        self._sync_wall_following_state(wall_state)
        
        return velocity_x, velocity_y, yaw_rate, wall_state
    
    def _sync_wall_following_state(self, wall_state):
        """根据墙面跟随算法状态同步状态机状态"""
        try:
            # 如果当前状态已经匹配，直接返回
            if wall_state == WallFollowing.StateWallFollowing.FORWARD and self.current_state == self.forward:
                return
            elif wall_state == WallFollowing.StateWallFollowing.HOVER and self.current_state == self.hover:
                return
            elif wall_state == WallFollowing.StateWallFollowing.TURN_TO_FIND_WALL and self.current_state == self.turn_to_find_wall:
                return
            elif wall_state == WallFollowing.StateWallFollowing.TURN_TO_ALIGN_TO_WALL and self.current_state == self.turn_to_align_to_wall:
                return
            elif wall_state == WallFollowing.StateWallFollowing.FORWARD_ALONG_WALL and self.current_state == self.forward_along_wall:
                return
            elif wall_state == WallFollowing.StateWallFollowing.ROTATE_AROUND_WALL and self.current_state == self.rotate_around_wall:
                return
            elif wall_state == WallFollowing.StateWallFollowing.ROTATE_IN_CORNER and self.current_state == self.rotate_in_corner:
                return
            elif wall_state == WallFollowing.StateWallFollowing.FIND_CORNER and self.current_state == self.find_corner:
                return
            
            # 需要状态转换 - 根据当前状态和目标状态决定转换路径
            current = self.current_state
            
            # 处理状态转换
            if wall_state == WallFollowing.StateWallFollowing.FORWARD:
                if current == self.idle:
                    self.begin_forward()
                elif current == self.hover:
                    self.resume_from_hover()
            elif wall_state == WallFollowing.StateWallFollowing.TURN_TO_FIND_WALL:
                if current == self.forward:
                    self.wall_detected()
                elif current == self.rotate_in_corner:
                    self.corner_rotated()
                elif current == self.rotate_around_wall:
                    self.wall_rediscovered()
            elif wall_state == WallFollowing.StateWallFollowing.TURN_TO_ALIGN_TO_WALL:
                if current == self.turn_to_find_wall:
                    self.wall_found()
            elif wall_state == WallFollowing.StateWallFollowing.FORWARD_ALONG_WALL:
                if current == self.turn_to_align_to_wall:
                    self.wall_aligned()
            elif wall_state == WallFollowing.StateWallFollowing.ROTATE_AROUND_WALL:
                if current == self.find_corner:
                    self.corner_found()
            elif wall_state == WallFollowing.StateWallFollowing.ROTATE_IN_CORNER:
                if current == self.forward_along_wall:
                    self.corner_detected()
            elif wall_state == WallFollowing.StateWallFollowing.FIND_CORNER:
                if current == self.forward_along_wall:
                    self.wall_lost()
            elif wall_state == WallFollowing.StateWallFollowing.HOVER:
                # 可以从任何状态转换到hover
                if current == self.forward:
                    self.hover_command()
                elif current == self.turn_to_align_to_wall:
                    self.hover_command_from_align()
                elif current == self.forward_along_wall:
                    self.hover_command_from_along()
                elif current == self.rotate_in_corner:
                    self.hover_command_from_corner()
                elif current == self.find_corner:
                    self.hover_command_from_find()
                elif current == self.rotate_around_wall:
                    self.hover_command_from_rotate()
        except Exception as e:
            print(f"Error syncing wall following state: {e}")
    
    def start_wall_following(self):
        """开始墙面跟随"""
        try:
            # 从idle状态转换到forward状态
            if self.current_state == self.idle:
                self.begin_forward()
        except Exception as e:
            print(f"Error starting wall following: {e}")
    
    def stop_wall_following(self):
        """停止墙面跟随"""
        try:
            if self.current_state == self.hover:
                self.stop_from_hover()
            elif self.current_state == self.forward:
                self.stop_wall_following_transition()
            elif self.current_state == self.turn_to_align_to_wall:
                self.stop_from_align()
            elif self.current_state == self.forward_along_wall:
                self.stop_from_along()
            elif self.current_state == self.rotate_in_corner:
                self.stop_from_corner()
            elif self.current_state == self.find_corner:
                self.stop_from_find()
            elif self.current_state == self.rotate_around_wall:
                self.stop_from_rotate()
        except Exception as e:
            print(f"Error stopping wall following: {e}")
    
    def hover_drone(self):
        """悬停无人机"""
        try:
            if self.current_state != self.hover:
                if self.current_state == self.forward:
                    self.hover_command()
                elif self.current_state == self.turn_to_align_to_wall:
                    self.hover_command_from_align()
                elif self.current_state == self.forward_along_wall:
                    self.hover_command_from_along()
                elif self.current_state == self.rotate_in_corner:
                    self.hover_command_from_corner()
                elif self.current_state == self.find_corner:
                    self.hover_command_from_find()
                elif self.current_state == self.rotate_around_wall:
                    self.hover_command_from_rotate()
        except Exception as e:
            print(f"Error hovering drone: {e}")
    
    def resume_from_hover(self):
        """从悬停状态恢复"""
        try:
            if self.current_state == self.hover:
                self.resume_from_hover()
        except Exception as e:
            print(f"Error resuming from hover: {e}")
    
    def write_sm_graph(self):
        """生成状态机图片"""
        with self._graph_lock:
            current_time = time.time()
            if current_time - self._last_graph_update < self._graph_update_interval:
                return
                
            current_state = self.current_state.id
            if current_state != self._last_state:
                try:
                    # 生成状态机图
                    sm_graph = DotGraphMachine(self)
                    image_number = self._image_numbers[self._drone_id]
                    
                    # 使用配置的图片目录
                    img_dir = self._get_image_dir()
                    os.makedirs(img_dir, exist_ok=True)
                    sm_graph_path = os.path.join(img_dir, f"wall_follow{image_number}.png")
                    
                    dot = sm_graph()
                    dot.write_png(sm_graph_path)
                    
                    # 更新图片编号
                    self._image_numbers[self._drone_id] = (image_number % 100) + 1
                    self._last_state = current_state
                    
                    # 更新latest.txt文件
                    latest_file = os.path.join(img_dir, "latest.txt")
                    try:
                        current_latest = 1
                        if os.path.exists(latest_file):
                            with open(latest_file, 'r') as f:
                                current_latest = int(f.read().strip())
                        
                        new_latest = max(current_latest, image_number)
                        with open(latest_file, 'w') as f:
                            f.write(str(new_latest))
                        print(f"Updated wall following latest.txt with image number: {new_latest}")
                    except Exception as e:
                        print(f"Error updating wall following latest.txt: {e}")
                        
                except Exception as e:
                    print(f"Error generating wall following state machine graph: {e}")
                
                self._last_graph_update = current_time
    
    def get_current_state(self):
        """获取当前状态"""
        return self.current_state.id
    
    def get_wall_following_state(self):
        """获取墙面跟随算法状态"""
        return self.current_wall_state
    
    def get_velocity_commands(self):
        """获取当前速度命令"""
        return self.current_velocity_x, self.current_velocity_y, self.current_yaw_rate
