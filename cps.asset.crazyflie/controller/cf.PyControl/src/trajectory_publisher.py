#!/usr/bin/env python3
import threading
import time
import logging
from typing import List, Dict, Any, Optional
from trajectory_handler import TrajectoryManager

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped
    from std_msgs.msg import Empty
    ROS2_AVAILABLE = True
    logger.info("ROS2 modules imported successfully")
except ImportError as e:
    logger.warning(f"ROS2 not available: {e}")
    ROS2_AVAILABLE = False

class TrajectoryPublisher:
    """轨迹发布器 - 发布轨迹点到ROS2"""
    
    def __init__(self, agent_id: int = 0, update_frequency: float = 10.0):
        self.agent_id = agent_id
        self.update_frequency = update_frequency
        self.trajectory_manager = TrajectoryManager(update_frequency)
        self.ros_node = None
        self.is_running = False
        self.publish_thread = None
        
        # ROS2 publishers
        self.position_publisher = None
        self.velocity_publisher = None
        self.pose_publisher = None
        
        # Current state
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        
    def initialize_ros2(self) -> bool:
        """初始化ROS2节点和发布器"""
        if not ROS2_AVAILABLE:
            logger.error("ROS2 not available, cannot initialize")
            return False
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            # 创建ROS2节点
            self.ros_node = Node(f'trajectory_publisher_agent_{self.agent_id}')
            
            # 创建发布器
            self.position_publisher = self.ros_node.create_publisher(
                Vector3, f'/agent_{self.agent_id}/position', 10)
            
            self.velocity_publisher = self.ros_node.create_publisher(
                Vector3, f'/agent_{self.agent_id}/velocity', 10)
                
            self.pose_publisher = self.ros_node.create_publisher(
                PoseStamped, f'/agent_{self.agent_id}/desired_pose', 10)
            
            # 创建状态订阅器
            self.pose_subscription = self.ros_node.create_subscription(
                PoseStamped, f'/agent_{self.agent_id}/pose', 
                self._pose_callback, 10)
                
            self.velocity_subscription = self.ros_node.create_subscription(
                TwistStamped, f'/agent_{self.agent_id}/velocity',
                self._velocity_callback, 10)
            
            logger.info(f"ROS2 trajectory publisher initialized for agent {self.agent_id}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize ROS2: {str(e)}")
            return False
    
    def _pose_callback(self, msg):
        """位置回调函数"""
        self.current_position = [
            msg.pose.position.x,
            msg.pose.position.y, 
            msg.pose.position.z
        ]
    
    def _velocity_callback(self, msg):
        """速度回调函数"""
        self.current_velocity = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ]
    
    def set_current_state(self, position: List[float], velocity: List[float] = None):
        """设置当前状态"""
        self.current_position = position
        if velocity:
            self.current_velocity = velocity
        self.trajectory_manager.set_current_state(position, velocity)
    
    def add_trajectory_goal(self, time: float, x: float, y: float, z: float) -> bool:
        """添加轨迹目标点"""
        return self.trajectory_manager.add_goal(time, x, y, z)
    
    def start_publishing(self) -> bool:
        """开始轨迹参考点的ROS2发布循环"""
        if self.is_running:
            logger.warning("TrajectoryPublisher: Start publishing called when already running. Ignoring.")
            return True

        try:
            # 确保TrajectoryManager中的轨迹已经准备好或正在准备
            # 实际的轨迹启动（计算和设置start_time）由外部(AutoStateMachine -> TrajectoryManager)负责
            
            self.is_running = True
            self.publish_thread = threading.Thread(target=self._publish_loop)
            self.publish_thread.daemon = True
            self.publish_thread.start()
            
            logger.info(f"Trajectory ROS2 publisher started for agent {self.agent_id}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start trajectory publisher: {str(e)}")
            return False
    
    def stop_trajectory(self):
        """停止轨迹执行和发布"""
        self.is_running = False
        self.trajectory_manager.stop_trajectory()
        
        if self.publish_thread and self.publish_thread.is_alive():
            try:
                self.publish_thread.join(timeout=1.0)
            except Exception as e:
                logger.error(f"Error joining publish thread: {e}")

        logger.info(f"Trajectory publisher and manager stopped for agent {self.agent_id}")
    
    def _publish_loop(self):
        """发布循环"""
        dt = 1.0 / self.update_frequency
        
        while self.is_running and self.trajectory_manager.is_trajectory_active():
            try:
                # 获取当前参考点
                ref = self.trajectory_manager.get_current_reference()
                
                if ref and ROS2_AVAILABLE and self.ros_node:
                    # 发布位置参考
                    self._publish_position_reference(ref)
                    
                    # 发布速度参考
                    self._publish_velocity_reference(ref)
                    
                    # 发布姿态参考
                    self._publish_pose_reference(ref)
                
                # 处理ROS2回调
                if ROS2_AVAILABLE and self.ros_node:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.001)
                
                time.sleep(dt)
                
            except Exception as e:
                logger.error(f"Error in ROS2 publish loop for agent {self.agent_id}: {str(e)}")
                # 发生异常时，也打印轨迹管理器的状态
                logger.error(f"TrajectoryManager state: is_active={self.trajectory_manager.is_trajectory_active()}, generator_start_time={getattr(self.trajectory_manager.generator, 'start_time', 'N/A')}")
                if ROS2_AVAILABLE and self.ros_node and hasattr(e, 'print_exc'): # 更安全的traceback打印
                    import traceback
                    traceback.print_exc()
                break
        
        self.is_running = False
        logger.info(f"ROS2 publish loop ended for agent {self.agent_id}")
    
    def _publish_position_reference(self, ref: Dict):
        """发布位置参考"""
        if self.position_publisher:
            msg = Vector3()
            msg.x = float(ref['position'][0])
            msg.y = float(ref['position'][1])
            msg.z = float(ref['position'][2])
            self.position_publisher.publish(msg)
    
    def _publish_velocity_reference(self, ref: Dict):
        """发布速度参考"""
        if self.velocity_publisher:
            msg = Vector3()
            msg.x = float(ref['velocity'][0])
            msg.y = float(ref['velocity'][1])
            msg.z = float(ref['velocity'][2])
            self.velocity_publisher.publish(msg)
    
    def _publish_pose_reference(self, ref: Dict):
        """发布姿态参考"""
        if self.pose_publisher:
            msg = PoseStamped()
            msg.header.stamp = self.ros_node.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            
            # 位置
            msg.pose.position.x = float(ref['position'][0])
            msg.pose.position.y = float(ref['position'][1])
            msg.pose.position.z = float(ref['position'][2])
            
            # 简化的姿态（只考虑yaw=0）
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            
            self.pose_publisher.publish(msg)
    
    def is_trajectory_active(self) -> bool:
        """检查轨迹是否正在执行"""
        return self.is_running and self.trajectory_manager.is_trajectory_active()
    
    def cleanup(self):
        """清理资源"""
        self.stop_trajectory()
        
        if ROS2_AVAILABLE and self.ros_node:
            try:
                self.ros_node.destroy_node()
            except Exception as e:
                logger.error(f"Error destroying ROS2 node for agent {self.agent_id}: {e}")

class MultiAgentTrajectoryPublisher:
    """多智能体轨迹发布器"""
    
    def __init__(self, num_agents: int = 1, update_frequency: float = 10.0):
        self.num_agents = num_agents
        self.update_frequency = update_frequency
        self.agents = {}
        
        # 初始化每个智能体的轨迹发布器
        for agent_id in range(num_agents):
            self.agents[agent_id] = TrajectoryPublisher(agent_id, update_frequency)
    
    def initialize_all_ros2(self) -> bool:
        """初始化所有智能体的ROS2连接"""
        success = True
        for agent_id, agent in self.agents.items():
            if not agent.initialize_ros2():
                logger.error(f"Failed to initialize ROS2 for agent {agent_id}")
                success = False
        return success
    
    def set_agent_state(self, agent_id: int, position: List[float], velocity: List[float] = None):
        """设置智能体状态"""
        if agent_id in self.agents:
            self.agents[agent_id].set_current_state(position, velocity)
    
    def add_agent_goal(self, agent_id: int, time: float, x: float, y: float, z: float) -> bool:
        """为指定智能体添加目标点"""
        if agent_id in self.agents:
            return self.agents[agent_id].add_trajectory_goal(time, x, y, z)
        return False
    
    def start_agent_trajectory(self, agent_id: int) -> bool:
        """启动指定智能体的轨迹"""
        if agent_id in self.agents:
            return self.agents[agent_id].start_publishing()
        return False
    
    def start_all_trajectories(self) -> bool:
        """启动所有智能体的轨迹"""
        success = True
        for agent_id, agent in self.agents.items():
            if not agent.start_publishing():
                logger.error(f"Failed to start trajectory for agent {agent_id}")
                success = False
        return success
    
    def stop_agent_trajectory(self, agent_id: int):
        """停止指定智能体的轨迹"""
        if agent_id in self.agents:
            self.agents[agent_id].stop_trajectory()
    
    def stop_all_trajectories(self):
        """停止所有智能体的轨迹"""
        for agent in self.agents.values():
            agent.stop_trajectory()
    
    def are_all_trajectories_complete(self) -> bool:
        """检查所有轨迹是否完成"""
        return all(not agent.is_trajectory_active() for agent in self.agents.values())
    
    def cleanup(self):
        """清理所有资源"""
        for agent in self.agents.values():
            agent.cleanup()
        
        if ROS2_AVAILABLE and rclpy.ok():
            rclpy.shutdown()

# 测试函数
def test_trajectory_publisher():
    """测试轨迹发布器"""
    logger.info("Starting trajectory publisher test")
    
    # 创建单智能体轨迹发布器
    publisher = TrajectoryPublisher(agent_id=0, update_frequency=10.0)
    
    # 初始化ROS2（如果可用）
    if ROS2_AVAILABLE:
        if not publisher.initialize_ros2():
            logger.error("Failed to initialize ROS2")
            return
    
    try:
        # 设置初始状态
        publisher.set_current_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        
        # 添加轨迹点
        publisher.add_trajectory_goal(5.0, 1.0, 0.0, 0.5)  # 5秒到达(1,0,0.5)
        publisher.add_trajectory_goal(10.0, 1.0, 1.0, 0.5)  # 10秒到达(1,1,0.5)
        publisher.add_trajectory_goal(15.0, 0.0, 1.0, 0.5)  # 15秒到达(0,1,0.5)
        publisher.add_trajectory_goal(20.0, 0.0, 0.0, 0.5)  # 20秒回到(0,0,0.5)
        
        # 开始轨迹执行
        if publisher.start_publishing():
            logger.info("Trajectory started successfully")
            
            # 等待轨迹完成
            while publisher.is_trajectory_active():
                time.sleep(1.0)
                logger.info("Trajectory running...")
            
            logger.info("Trajectory completed")
        else:
            logger.error("Failed to start trajectory")
    
    finally:
        publisher.cleanup()

if __name__ == "__main__":
    test_trajectory_publisher() 