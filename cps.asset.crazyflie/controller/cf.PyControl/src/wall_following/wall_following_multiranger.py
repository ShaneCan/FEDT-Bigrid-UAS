#!/usr/bin/env python3

""" This simple mapper is loosely based on both the bitcraze cflib point cloud example
https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/multiranger/multiranger_pointcloud.py
and the webots epuck simple mapper example:
https://github.com/cyberbotics/webots_ros2

Originally from https://github.com/knmcguire/crazyflie_ros2_experimental/
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf2_ros import StaticTransformBroadcaster
from std_srvs.srv import Trigger
from crazyflie_interfaces.msg import Hover

import tf_transformations
import math
import numpy as np
import os

# 使用相对导入（包内引用）
from .wall_following import WallFollowing
from .wall_following_simple import SimpleWallFollowing
from .wall_following_state_machine import WallFollowingStateMachine
import time

GLOBAL_SIZE_X = 5.0
GLOBAL_SIZE_Y = 5.0
MAP_RES = 0.5


class WallFollowingMultiranger(Node):
    def __init__(self, **kwargs):

        super().__init__('wall_following_multiranger')
        self.declare_parameter('robot_prefix', '/cf231')
        robot_prefix = self.get_parameter('robot_prefix').value
        self.declare_parameter('delay', 5.0)
        self.delay = self.get_parameter('delay').value
        self.declare_parameter('max_turn_rate', 0.5)
        max_turn_rate = self.get_parameter('max_turn_rate').value
        self.declare_parameter('max_forward_speed', 0.3)
        max_forward_speed = self.get_parameter('max_forward_speed').value
        self.declare_parameter('wall_following_direction', 'right')
        self.wall_following_direction = self.get_parameter('wall_following_direction').value
        self.declare_parameter('hover_height', 0.5)
        self.hover_height = self.get_parameter('hover_height').value
        
        # 算法模式选择：'simple'（简化版，适合矩形场地）或 'full'（完整版）
        self.declare_parameter('algorithm_mode', 'simple')
        self.algorithm_mode = self.get_parameter('algorithm_mode').value
        
        # 简化算法参数
        self.declare_parameter('target_wall_distance', 0.3)
        self.target_wall_distance = float(self.get_parameter('target_wall_distance').value)
        self.declare_parameter('front_wall_threshold', 0.35)
        self.front_wall_threshold = float(self.get_parameter('front_wall_threshold').value)
        
        # 图片输出目录配置
        self.declare_parameter('image_output_base_dir', '')
        image_output_base_dir = self.get_parameter('image_output_base_dir').value
        if not image_output_base_dir:
            # 使用默认路径
            image_output_base_dir = os.path.join(
                os.path.dirname(__file__), '..', '..', 'webview', 'wall_following', 'img'
            )
        self.image_output_base_dir = os.path.abspath(image_output_base_dir)
        
        # 垂直冲突避免参数
        self.declare_parameter('enable_vertical_avoidance', True)
        self.enable_vertical_avoidance = bool(self.get_parameter('enable_vertical_avoidance').value)
        self.declare_parameter('xy_overlap_radius', 0.2)
        self.xy_overlap_radius = float(self.get_parameter('xy_overlap_radius').value)
        self.declare_parameter('z_downwash_gap', 0.5)
        self.z_downwash_gap = float(self.get_parameter('z_downwash_gap').value)
        self.declare_parameter('avoid_lateral_step', 0.25)
        self.avoid_lateral_step = float(self.get_parameter('avoid_lateral_step').value)
        self.declare_parameter('avoid_speed', 0.2)
        self.avoid_speed = float(self.get_parameter('avoid_speed').value)
        self.declare_parameter('return_speed', 0.2)
        self.return_speed = float(self.get_parameter('return_speed').value)
        self.declare_parameter('avoid_timeout', 3.0)
        self.avoid_timeout = float(self.get_parameter('avoid_timeout').value)
        self.declare_parameter('return_timeout', 3.0)
        self.return_timeout = float(self.get_parameter('return_timeout').value)
        
        # 其他无人机前缀（逗号分隔字符串或字符串数组），但会自动排除自身
        self.declare_parameter('peer_prefixes', '/cf231,/cf232,/cf233')
        peer_param = self.get_parameter('peer_prefixes').value
        if isinstance(peer_param, (list, tuple)):
            peer_prefixes = [str(p) for p in peer_param]
        else:
            peer_prefixes = [p.strip() for p in str(peer_param).split(',') if p.strip()]
        
        # 自动排除自身，避免订阅自己的里程计
        peer_prefixes = [p for p in peer_prefixes if p != robot_prefix]
        
        # 支持通过kwargs传入参数覆盖
        if 'hover_height' in kwargs:
            self.hover_height = float(kwargs['hover_height'])
            self.get_logger().info(f"Override hover_height to {self.hover_height}m")
        if 'image_output_base_dir' in kwargs:
            self.image_output_base_dir = str(kwargs['image_output_base_dir'])
            self.get_logger().info(f"Override image_output_base_dir to {self.image_output_base_dir}")

        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_subscribe_callback, 10)

        # add service to stop wall following and make the crazyflie land
        self.srv = self.create_service(Trigger, robot_prefix + '/stop_wall_following', self.stop_wall_following_cb)

        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        # 保存最近一次LaserScan；不要假设msg.ranges的索引顺序
        self._last_scan = None

        self.position_update = False

        # 直接发布Hover消息到cmd_hover话题，不需要vel_mux
        self.hover_publisher = self.create_publisher(Hover, robot_prefix + '/cmd_hover', 10)
        
        # 添加日志记录时间戳
        self.last_log_time = self.get_clock().now().nanoseconds * 1e-9

        drone_id = robot_prefix.replace('/', '')
        self.get_logger().info(f"[{drone_id}] Wall following set for crazyflie " + robot_prefix +
                               f" using scan topic with delay of {self.delay}s, publishing to {robot_prefix}/cmd_hover")
        
        # 添加调试信息
        self.get_logger().info(f"[{drone_id}] Node name: {self.get_name()}")
        self.get_logger().info(f"[{drone_id}] Namespace: {self.get_namespace()}")
        self.get_logger().info(f"[{drone_id}] Publisher topic: {robot_prefix}/cmd_hover")
        self.get_logger().info(f"[{drone_id}] Image output dir: {self.image_output_base_dir}")

        # Create a timer to run the wall following state machine
        self.timer = self.create_timer(0.01, self.timer_callback)

        # 根据算法模式初始化墙面跟随算法
        if self.algorithm_mode == 'simple':
            # 简化算法：适合矩形场地
            self.get_logger().info(f"[{drone_id}] Using SIMPLE wall following algorithm")
            self.wall_following = SimpleWallFollowing(
                target_wall_distance=self.target_wall_distance,
                max_forward_speed=max_forward_speed,
                max_turn_rate=max_turn_rate,
                front_wall_threshold=self.front_wall_threshold,
                init_state=SimpleWallFollowing.State.FORWARD_ALONG_WALL
            )
            self.use_simple_algorithm = True
        else:
            # 完整算法：适合复杂场地
            self.get_logger().info(f"[{drone_id}] Using FULL wall following algorithm")
            self.wall_following = WallFollowing(
                max_turn_rate=max_turn_rate,
                max_forward_speed=max_forward_speed,
                init_state=WallFollowing.StateWallFollowing.FORWARD
            )
            self.use_simple_algorithm = False
            
            # 创建墙面跟随状态机（用于图片生成，仅完整算法使用）
            self.wall_following_sm = WallFollowingStateMachine(
                drone_id=robot_prefix.replace('/', ''),
                image_output_base_dir=self.image_output_base_dir,
                max_turn_rate=max_turn_rate,
                max_forward_speed=max_forward_speed,
                wall_following_direction=WallFollowing.WallFollowingDirection.RIGHT if self.wall_following_direction == 'right' else WallFollowing.WallFollowingDirection.LEFT
            )
        
        # Give a take off command but wait for the delay to start the wall following
        self.wait_for_start = True
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9

        # 同伴里程计订阅与状态
        self.peer_positions = {}  # prefix -> (x,y,z,timestamp)
        self.peer_subs = []
        self.robot_prefix = robot_prefix
        for peer in peer_prefixes:
            if peer:  # 已经在上面的代码中排除了自身，这里只需要检查非空
                sub = self.create_subscription(
                    Odometry, peer + '/odom',
                    lambda msg, p=peer: self._peer_odom_cb(p, msg), 10)
                self.peer_subs.append(sub)
        if self.enable_vertical_avoidance:
            self.get_logger().info(
                f"Vertical avoidance enabled: xy_radius={self.xy_overlap_radius}m, z_gap={self.z_downwash_gap}m, step={self.avoid_lateral_step}m")
            self.get_logger().info(f"Subscribing to peer odometry: {peer_prefixes}")
        else:
            self.get_logger().info("Vertical avoidance disabled")

        # 避让/返回模式状态
        self.avoid_mode = False
        self.return_mode = False
        self.avoid_start_time = 0.0
        self.return_start_time = 0.0
        self.home_xy = (0.0, 0.0)
        self.avoid_target_xy = (0.0, 0.0)

    def stop_wall_following_cb(self, request, response):
        drone_id = self.get_parameter('robot_prefix').value.replace('/', '')
        self.get_logger().info(f'[{drone_id}] Stopping wall following')
        
        # 销毁定时器，彻底停止hover消息发布
        if hasattr(self, 'timer') and self.timer is not None:
            self.get_logger().info(f'[{drone_id}] Destroying wall following timer')
            self.timer.destroy()
            self.timer = None
        
        # 停止状态机（仅完整算法使用）
        if not self.use_simple_algorithm and hasattr(self, 'wall_following_sm'):
            self.wall_following_sm.stop_wall_following()
        
        # 切换到悬停状态
        if hasattr(self.wall_following, 'hover'):
            self.wall_following.hover()
        
        # 发布停止命令（悬停）
        msg = Hover()
        msg.vx = 0.0
        msg.vy = 0.0
        msg.yaw_rate = 0.0
        msg.z_distance = self.hover_height  # 保持当前高度悬停
        self.hover_publisher.publish(msg)

        response.success = True

        return response

    def _peer_odom_cb(self, peer_prefix, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        t = self.get_clock().now().nanoseconds * 1e-9
        self.peer_positions[peer_prefix] = (x, y, z, t)

    def _detect_vertical_conflict(self, now_sec):
        if not self.enable_vertical_avoidance:
            return None
        my_x, my_y, my_z = self.position[0], self.position[1], self.position[2]
        conflict_peer = None
        min_xy = 1e9
        for peer, (px, py, pz, pt) in self.peer_positions.items():
            # 仅使用新鲜数据
            if now_sec - pt > 1.0:
                continue
            dx = my_x - px
            dy = my_y - py
            dxy = math.hypot(dx, dy)
            dz = pz - my_z  # 对方在上方则为正
            if dxy < self.xy_overlap_radius and 0.0 < dz <= self.z_downwash_gap:
                if dxy < min_xy:
                    min_xy = dxy
                    conflict_peer = (peer, px, py, pz)
        return conflict_peer

    def timer_callback(self): # 开始执行wall following的回调函数

        # wait for the delay to pass and then start wall following
        if self.wait_for_start:
            if self.get_clock().now().nanoseconds * 1e-9 - self.start_clock > self.delay:
                drone_id = self.get_parameter('robot_prefix').value.replace('/', '')
                self.get_logger().info(f'[{drone_id}] Starting wall following')
                self.wait_for_start = False
                # 启动状态机（仅完整算法使用）
                if not self.use_simple_algorithm and hasattr(self, 'wall_following_sm'):
                    self.wall_following_sm.start_wall_following()
            else:
                return

        # initialize variables
        velocity_x = 0.0
        velocity_y = 0.0
        yaw_rate = 0.0
        state_wf = WallFollowing.StateWallFollowing.HOVER

        # Get Yaw
        actual_yaw_rad = self.angles[2]

        # 从LaserScan按固定索引提取前/左/右/后距离
        # 根据实际scan数据：ranges = [back, right, front, left]
        # angle_min=-π, increment=π/2 → [180°, -90°, 0°, 90°]
        if self._last_scan is None or len(self._last_scan.ranges) < 4:
            return
        
        ranges = self._last_scan.ranges
        back_range = ranges[0] if not math.isinf(ranges[0]) and ranges[0] > 0 else 999.0
        right_range = ranges[1] if not math.isinf(ranges[1]) and ranges[1] > 0 else 999.0
        front_range = ranges[2] if not math.isinf(ranges[2]) and ranges[2] > 0 else 999.0
        left_range = ranges[3] if not math.isinf(ranges[3]) and ranges[3] > 0 else 999.0

        # choose here the direction that you want the wall following to turn to
        if self.wall_following_direction == 'right':
            wf_dir = WallFollowing.WallFollowingDirection.RIGHT
            # 右侧贴墙：使用右侧测距
            side_range = right_range
        else:
            wf_dir = WallFollowing.WallFollowingDirection.LEFT
            # 左侧贴墙：使用左侧测距
            side_range = left_range
        
        # 安全检查：避免除零错误
        if side_range <= 0 or side_range == float('inf'):
            side_range = 999.0  # 设置为一个很大的值
        if front_range <= 0 or front_range == float('inf'):
            front_range = 999.0  # 设置为一个很大的值

        time_now = self.get_clock().now().nanoseconds * 1e-9

        # --- 第一步，垂直冲突避免逻辑（优先于正常wall following）---------------- #
        # 根据他机里程计检测下洗风险（对方在上方且XY重叠）
        conflict = self._detect_vertical_conflict(time_now)
        if conflict and not (self.avoid_mode or self.return_mode):
            # 进入避让模式：记录home位置，选择安全侧向方向
            self.home_xy = (self.position[0], self.position[1])
            # 首选：朝远离最近侧墙的方向侧移
            # left<right -> 向右移(vy>0); 反之向左移(vy<0)
            lateral_sign = 1.0 if left_range > right_range else -1.0
            # 如果某侧距离极小，强制反向
            if left_range < 0.15:
                lateral_sign = 1.0
            if right_range < 0.15:
                lateral_sign = -1.0
            self.avoid_target_xy = (
                self.position[0],
                self.position[1] + lateral_sign * self.avoid_lateral_step
            )
            self.avoid_mode = True
            self.return_mode = False
            self.avoid_start_time = time_now
            # 状态机置于hover（图示更直观）
            # try:
            #     self.wall_following_sm.hover_drone()
            # except Exception:
            #     pass

        # 在避让模式：侧移（并在前向过近时微量后退），超时则切换到返回模式
        if self.avoid_mode:
            # 安全：前方过近则后退
            if front_range < 0.25:
                velocity_x = -min(self.avoid_speed, 0.2)
            else:
                velocity_x = 0.0
            # 动态选择更安全的侧向方向
            lateral_sign = 1.0 if left_range > right_range else -1.0
            if left_range < 0.15:
                lateral_sign = 1.0
            if right_range < 0.15:
                lateral_sign = -1.0
            velocity_y = lateral_sign * self.avoid_speed
            yaw_rate = 0.0

            # 发布并判定超时
            msg = Hover()
            msg.vx = velocity_x
            msg.vy = velocity_y
            msg.yaw_rate = yaw_rate
            msg.z_distance = self.hover_height
            self.hover_publisher.publish(msg)

            if time_now - self.avoid_start_time >= self.avoid_timeout:
                self.avoid_mode = False
                self.return_mode = True
                self.return_start_time = time_now
            return

        # 在返回模式：基于世界坐标回到home_xy，转换到机体系速度发布
        if self.return_mode:
            dx_world = self.home_xy[0] - self.position[0]
            dy_world = self.home_xy[1] - self.position[1]
            dist = math.hypot(dx_world, dy_world)
            # 阈值或超时结束返回
            if dist < 0.05 or (time_now - self.return_start_time) > self.return_timeout:
                self.return_mode = False
            else:
                # 世界->机体
                cy = math.cos(actual_yaw_rad)
                sy = math.sin(actual_yaw_rad)
                vx_body = (cy * dx_world + sy * dy_world)
                vy_body = (-sy * dx_world + cy * dy_world)
                # 归一化至速度上限
                if dist > 1e-3:
                    scale = self.return_speed / max(self.return_speed, math.hypot(vx_body, vy_body))
                    vx_body *= scale
                    vy_body *= scale
                # 安全约束：不向前撞墙
                if front_range < 0.3 and vx_body > 0:
                    vx_body = 0.0
                # 侧墙过近限制侧移
                if left_range < 0.2 and vy_body > 0:
                    vy_body = 0.0
                if right_range < 0.2 and vy_body < 0:
                    vy_body = 0.0

                msg = Hover()
                msg.vx = vx_body
                msg.vy = vy_body
                msg.yaw_rate = 0.0
                msg.z_distance = self.hover_height
                self.hover_publisher.publish(msg)
                return

        # 处理inf值：将inf视为足够远
        ranges_valid = [r if r != float('inf') and r > 0 else 999.0 for r in [back_range, right_range, front_range, left_range]]
        min_range = min(ranges_valid)
        
        # 第二步，根据算法模式更新墙面跟随
        if self.use_simple_algorithm:
            # 简化算法
            wf_dir_simple = SimpleWallFollowing.Direction.RIGHT if self.wall_following_direction == 'right' else SimpleWallFollowing.Direction.LEFT
            velocity_x, velocity_y, yaw_rate, state_wf = self.wall_following.update(
                front_range, side_range, actual_yaw_rad, wf_dir_simple, time_now)
        else:
            # 完整算法
            velocity_x, velocity_y, yaw_rate, state_wf = self.wall_following_sm.update_wall_following(
                front_range, side_range, actual_yaw_rad, wf_dir, time_now)
            # 生成状态机图片
            self.wall_following_sm.write_sm_graph()
        
        # 每秒输出一次scan数据和速度命令
        current_time = time_now
        if current_time - self.last_log_time > 1.0:
            # 从robot_prefix中提取无人机ID（去掉前缀斜杠）
            drone_id = self.get_parameter('robot_prefix').value.replace('/', '')
            if self.use_simple_algorithm:
                # 简化算法的日志
                self.get_logger().info(
                    f"[{drone_id}] SCAN -> front: {front_range:.3f}, side: {side_range:.3f} | "
                    f"CMD -> vx: {velocity_x:.3f}, vy: {velocity_y:.3f}, yaw: {yaw_rate:.3f} | "
                    f"state: {state_wf} | target_dist: {self.target_wall_distance:.2f}m"
                )
            else:
                # 完整算法的日志
                sm_state = self.wall_following_sm.get_current_state()
                self.get_logger().info(
                    f"[{drone_id}] SCAN -> back: {back_range:.3f}, right: {right_range:.3f}, front: {front_range:.3f}, left: {left_range:.3f} | "
                    f"CMD -> vx: {velocity_x:.3f}, vy: {velocity_y:.3f}, yaw: {yaw_rate:.3f} | wall_state: {state_wf} | sm_state: {sm_state}"
                )
            self.last_log_time = current_time
        
        # 直接发布Hover消息到cmd_hover话题
        msg = Hover()
        msg.vx = velocity_x
        msg.vy = velocity_y
        msg.yaw_rate = yaw_rate
        msg.z_distance = self.hover_height  # 悬停高度，可以通过参数调整
        self.hover_publisher.publish(msg)

    def odom_subscribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]
        self.position_update = True

    def _range_at_angle(self, scan: LaserScan, target_angle_rad: float, window_rad: float = 0.35) -> float:
        """
        从LaserScan中提取某个方向的距离。
        - 使用一个角度窗口取最小有效值，抗噪更稳
        - 不假设ranges顺序
        """
        if scan.angle_increment == 0.0 or not scan.ranges:
            return 999.0

        # 归一化目标角到[-pi, pi]
        def wrap(a: float) -> float:
            while a > math.pi:
                a -= 2 * math.pi
            while a < -math.pi:
                a += 2 * math.pi
            return a

        target = wrap(target_angle_rad)
        # 遍历窗口内的indices（scan角度通常从angle_min递增到angle_max）
        # 计算目标角对应的中心index
        # 注意：有的scan不覆盖全360，窗口外会被裁掉
        center_idx = int(round((target - scan.angle_min) / scan.angle_increment))
        half_span = int(max(1, round(window_rad / scan.angle_increment)))

        best = None
        n = len(scan.ranges)
        for i in range(center_idx - half_span, center_idx + half_span + 1):
            if i < 0 or i >= n:
                continue
            r = scan.ranges[i]
            if r is None or r <= 0.0 or r == float('inf'):
                continue
            # 也过滤掉超出量程的异常大值（可根据传感器调整）
            if scan.range_max > 0.0 and r > scan.range_max:
                continue
            best = r if best is None else min(best, r)

        return best if best is not None else 999.0

    def scan_subscribe_callback(self, msg):
        self._last_scan = msg

def main(args=None):

    rclpy.init(args=args)
    wall_following_multiranger = WallFollowingMultiranger()
    rclpy.spin(wall_following_multiranger)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

