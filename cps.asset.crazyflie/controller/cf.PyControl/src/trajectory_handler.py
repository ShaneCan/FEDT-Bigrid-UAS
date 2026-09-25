#!/usr/bin/env python3
import time
import logging
import numpy as np
from typing import List, Dict, Any, Optional
from scipy.spatial.transform import Rotation as R
from numpy.polynomial import Polynomial as poly

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class TrajectoryPoint:
    """轨迹点数据结构"""
    def __init__(self, time: float, position: List[float], velocity: List[float] = None, 
                 acceleration: List[float] = None):
        self.time = time
        self.position = np.array(position)
        self.velocity = np.array(velocity) if velocity else np.zeros(3)
        self.acceleration = np.array(acceleration) if acceleration else np.zeros(3)

class SplineTrajectoryGenerator:
    """样条轨迹生成器 - 基于crazychoir的实现"""
    
    def __init__(self, update_frequency: float = 10.0):
        self.update_frequency = update_frequency
        self.trajectory_points = []
        self.spline_coefficients = None
        self.start_time = None
        self.current_pose = None
        
    def set_current_pose(self, position: List[float], velocity: List[float] = None):
        """设置当前位置和速度"""
        self.current_pose = {
            'position': np.array(position),
            'velocity': np.array(velocity) if velocity else np.zeros(3)
        }
        
    def add_goal_point(self, time: float, x: float, y: float, z: float):
        """添加目标点 - 兼容四维输入（时间+xyz）"""
        point = TrajectoryPoint(time, [x, y, z])
        self.trajectory_points.append(point)
        logger.info(f"Added trajectory point: t={time}, pos=({x}, {y}, {z})")
        
    def generate_trajectory(self) -> bool:
        """生成样条轨迹"""
        if not self.trajectory_points or not self.current_pose:
            logger.error("Need trajectory points and current pose to generate trajectory")
            return False
            
        try:
            # 创建完整的轨迹参数
            self.traj_params = self._prepare_trajectory_params()
            
            # 生成样条系数
            self.spline_coefficients = self._compute_spline_coefficients()
            
            logger.info(f"Generated spline trajectory with {len(self.trajectory_points)} points")
            return True
            
        except Exception as e:
            logger.error(f"Failed to generate trajectory: {str(e)}")
            return False
    
    def _prepare_trajectory_params(self) -> Dict:
        """准备轨迹参数"""
        n_points = len(self.trajectory_points)
        
        # 如果只有一个目标点，创建中间点来平滑轨迹
        if n_points == 1:
            target = self.trajectory_points[0]
            
            # 添加中间点
            mid_time = target.time * 0.5
            mid_pos = (self.current_pose['position'] + target.position) * 0.5
            mid_vel = (self.current_pose['velocity'] + target.velocity) * 0.5
            
            traj_params = {
                'time': [0.0, mid_time, target.time],
                'position': [
                    self.current_pose['position'].tolist(),
                    mid_pos.tolist(),
                    target.position.tolist()
                ],
                'velocity': [
                    self.current_pose['velocity'].tolist(),
                    mid_vel.tolist(),
                    target.velocity.tolist()
                ],
                'acceleration': [
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    target.acceleration.tolist()
                ],
                'jerk': [
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0]
                ]
            }
        else:
            # 多点轨迹
            traj_params = {
                'time': [0.0] + [p.time for p in self.trajectory_points],
                'position': [self.current_pose['position'].tolist()] + 
                           [p.position.tolist() for p in self.trajectory_points],
                'velocity': [self.current_pose['velocity'].tolist()] + 
                           [p.velocity.tolist() for p in self.trajectory_points],
                'acceleration': [[0.0, 0.0, 0.0]] + 
                               [p.acceleration.tolist() for p in self.trajectory_points],
                'jerk': [[0.0, 0.0, 0.0] for _ in range(len(self.trajectory_points) + 1)]
            }
        
        return traj_params
    
    def _compute_spline_coefficients(self):
        """计算样条系数"""
        n_points = len(self.traj_params['time'])
        
        # 为边界条件添加额外时间点
        t_extra = [
            0.5 * (self.traj_params['time'][0] + self.traj_params['time'][1]),
            0.5 * (self.traj_params['time'][-2] + self.traj_params['time'][-1])
        ]
        
        # 插入额外时间点
        times = self.traj_params['time'].copy()
        times.insert(1, t_extra[0])
        times.insert(-1, t_extra[1])
        
        # 计算时间间隔
        T = [times[i+1] - times[i] for i in range(len(times)-1)]
        
        # 为每个轴计算样条系数
        coeffs = []
        positions = np.array(self.traj_params['position'])
        
        for axis in range(3):
            pos_axis = positions[:, axis].tolist()
            velocity_borders = [0, 0]  # 初始和最终速度边界条件
            acceleration_borders = [0, 0]  # 初始和最终加速度边界条件
            
            spline_coeffs = self._compute_spline_coefficients_1d(
                T, pos_axis, velocity_borders, acceleration_borders
            )
            coeffs.append(spline_coeffs)
        
        return np.array(coeffs)
    
    def _compute_spline_coefficients_1d(self, T, q, v_borders, a_borders):
        """计算一维样条系数"""
        N = len(q) + 1
        
        # 构建线性系统 Aw = c
        A = np.zeros((N-1, N-1))
        
        # 填充矩阵A
        for i in range(1, N-2):
            A[i, i-1] = T[i]
            A[i, i] = 2 * (T[i] + T[i+1])
            A[i, i+1] = T[i+1]
        
        A[0, 0] = 2*T[1] + T[0]*(3 + T[0]/T[1])
        A[0, 1] = T[1]
        A[1, 0] -= T[0]*T[0]/T[1]
        A[-2, -1] -= T[-1]*T[-1]/T[-2]
        A[-1, -2] = T[-2]
        A[-1, -1] = 2*T[-2] + T[-1]*(3 + T[-1]/T[-2])
        
        # 构建向量c
        c = np.zeros(N-1)
        for i in range(1, N-2):
            c[i] = (q[i+1] - q[i])/T[i+1] - (q[i] - q[i-1])/T[i]
        
        c[0] = (q[1] - q[0])/T[1] - v_borders[0]*(1 + T[0]/T[1]) - a_borders[0]*T[0]*(0.5 + T[0]/T[1]/3)
        c[1] = (q[2] - q[1])/T[2] - (q[1] - q[0])/T[0] + v_borders[0]*T[0]/T[1] + a_borders[0]*T[0]*T[0]/T[1]/3
        c[-2] = (q[-1] - q[-2])/T[-2] - (q[-2] - q[-3])/T[-3] - v_borders[1]*T[N-1]/T[N-2] + a_borders[1]*T[N-1]*T[N-1]/T[N-2]/3
        c[-1] = (q[-2] - q[-1])/T[-2] + v_borders[1]*(1 + T[N-1]/T[N-2]) - a_borders[1]*(0.5 + T[N-1]/T[N-2]/3)*T[N-1]
        c *= 6
        
        # 求解线性系统
        w = (np.linalg.inv(A) @ c).tolist()
        
        # 添加初始和最终加速度
        w.insert(0, a_borders[0])
        w.append(a_borders[1])
        
        # 计算额外点
        q_extra = [
            q[0] + T[0]*v_borders[0] + T[0]*T[0]*a_borders[0]/3 + T[0]*T[0]/6*w[1],
            q[-1] - T[-1]*v_borders[1] + T[-1]*T[-1]/3*a_borders[1] + T[-1]*T[-1]/6*w[-2]
        ]
        q.insert(1, q_extra[0])
        q.insert(N-1, q_extra[1])
        
        # 计算样条系数
        N = len(q) - 1
        a = np.zeros((N, 4))
        for k in range(N):
            a[k, 0] = q[k]
            a[k, 1] = (q[k+1] - q[k])/T[k] - T[k]/6*(w[k+1] + 2*w[k])
            a[k, 2] = w[k]/2
            a[k, 3] = (w[k+1] - w[k])/6/T[k]
        
        return a
    
    def get_reference(self, current_time: float) -> Optional[Dict]:
        """获取当前时间的参考轨迹点"""
        if self.spline_coefficients is None or self.start_time is None:
            return None
        
        time = current_time - self.start_time
        n_points = len(self.traj_params['time'])
        time_points = self.traj_params['time']
        
        # 限制时间范围
        if time >= time_points[-1]:
            time = time_points[-1] - 0.001  # 稍微提前一点避免边界问题
        elif time < 0:
            time = 0.0
        
        # 查找当前时间段
        ref = {'position': np.zeros(3), 'velocity': np.zeros(3), 'acceleration': np.zeros(3)}
        
        for q in range(n_points - 1):
            if time_points[q] <= time <= time_points[q + 1]:
                t_local = time - time_points[q]
                T_segment = time_points[q + 1] - time_points[q]
                
                # 计算位置、速度、加速度
                for axis in range(3):
                    if q < len(self.spline_coefficients[axis]):
                        coeffs = self.spline_coefficients[axis][q]
                        
                        # 位置 (三次多项式: a0 + a1*t + a2*t^2 + a3*t^3)
                        ref['position'][axis] = (coeffs[0] + coeffs[1]*t_local + 
                                               coeffs[2]*t_local**2 + coeffs[3]*t_local**3)
                        
                        # 速度 (一阶导数: a1 + 2*a2*t + 3*a3*t^2)
                        ref['velocity'][axis] = (coeffs[1] + 2*coeffs[2]*t_local + 
                                               3*coeffs[3]*t_local**2)
                        
                        # 加速度 (二阶导数: 2*a2 + 6*a3*t)
                        ref['acceleration'][axis] = (2*coeffs[2] + 6*coeffs[3]*t_local)
                
                break
        
        return ref
    
    def start_trajectory(self):
        """开始执行轨迹"""
        self.start_time = time.time()
        logger.info("Started trajectory execution")
    
    def is_trajectory_complete(self, current_time: float) -> bool:
        """检查轨迹是否完成"""
        if not self.start_time or not self.traj_params:
            return False
        
        elapsed = current_time - self.start_time
        return elapsed >= self.traj_params['time'][-1]
    
    def clear_trajectory(self):
        """清空轨迹"""
        self.trajectory_points = []
        self.spline_coefficients = None
        self.start_time = None
        logger.info("Cleared trajectory")

class TrajectoryManager:
    """轨迹管理器 - 与无人机控制系统集成"""
    
    def __init__(self, update_frequency: float = 10.0):
        self.generator = SplineTrajectoryGenerator(update_frequency)
        self.is_active = False
        self.trajectory_queue = []  # 轨迹队列
        
    def add_goal(self, time: float, x: float, y: float, z: float) -> bool:
        """添加目标点（四维输入：时间+xyz）"""
        try:
            self.generator.add_goal_point(time, x, y, z)
            return True
        except Exception as e:
            logger.error(f"Failed to add goal: {str(e)}")
            return False
    
    def set_current_state(self, position: List[float], velocity: List[float] = None):
        """设置当前状态"""
        self.generator.set_current_pose(position, velocity)
    
    def start_trajectory_execution(self) -> bool:
        """开始轨迹执行"""
        if self.is_active:
            logger.warning("TrajectoryManager: Start called when already active. Ignoring.")
            return True # 保持当前状态，不中断调用流程

        try:
            if self.generator.generate_trajectory():
                self.generator.start_trajectory()
                self.is_active = True
                logger.info("Trajectory execution started by manager") # 稍微修改日志以区分
                return True
            else:
                logger.error("Failed to generate trajectory")
                return False
        except Exception as e:
            logger.error(f"Failed to start trajectory execution: {str(e)}")
            return False
    
    def get_current_reference(self) -> Optional[Dict]:
        """获取当前参考点"""
        if not self.is_active:
            return None
        
        current_time = time.time()
        ref = self.generator.get_reference(current_time)
        
        # 检查轨迹是否完成
        if self.generator.is_trajectory_complete(current_time):
            self.is_active = False
            logger.info("Trajectory execution completed")
        
        return ref
    
    def stop_trajectory(self):
        """停止轨迹执行"""
        self.is_active = False
        self.generator.clear_trajectory()
        logger.info("Trajectory execution stopped")
    
    def is_trajectory_active(self) -> bool:
        """检查轨迹是否正在执行"""
        return self.is_active 