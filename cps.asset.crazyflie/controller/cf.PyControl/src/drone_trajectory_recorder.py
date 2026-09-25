#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import numpy as np
import json
import csv
import os
from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

try:
    # Crazyflie 状态消息（包含 battery_voltage, rssi 等）
    from crazyflie_interfaces.msg import Status as CfStatus
except ImportError:
    CfStatus = None

class DroneTrajectoryRecorder(Node):
    def __init__(self, experiment_name=None, sim=False):
        super().__init__('drone_trajectory_recorder')
        
        self.sim = sim
        # 无人机名称列表 - 普通模式可修改；sim 模式下会根据 /cf_positions_path 的 frame_id 动态添加
        self.drone_names = ['cf231', 'cf232', 'cf233', 'cf234', 'cf235']
        
        # 存储每架无人机的轨迹与状态数据
        self.trajectories = {}
        self.timestamps = {}
        self.status_latest = {}
        
        def _init_drone_data(drone_name):
            self.trajectories[drone_name] = {
                'x': [], 'y': [], 'z': [],
                'timestamp': [],
                'battery_voltage': [],
                'rssi': [],
            }
            self.timestamps[drone_name] = []
            self.status_latest[drone_name] = {'battery_voltage': None, 'rssi': None}
        
        for drone_name in self.drone_names:
            _init_drone_data(drone_name)
        
        self.odom_subs = []
        self.status_subs = []
        self._path_sub = None
        
        if sim:
            # sim 模式：只订阅 /cf_positions_path，根据 pose 的 frame_id 识别无人机
            self._path_sub = self.create_subscription(
                Path,
                '/cf_positions_path',
                self.path_callback,
                10,
            )
            self.get_logger().info('Sim mode: subscribing to /cf_positions_path (nav_msgs/Path)')
        else:
            # 普通模式：每架无人机单独订阅 /cfXXX/odom 和 /cfXXX/status
            for drone_name in self.drone_names:
                sub = self.create_subscription(
                    Odometry,
                    f'/{drone_name}/odom',
                    lambda msg, name=drone_name: self.odom_callback(msg, name),
                    10,
                )
                self.odom_subs.append(sub)
                if CfStatus is not None:
                    status_sub = self.create_subscription(
                        CfStatus,
                        f'/{drone_name}/status',
                        lambda msg, name=drone_name: self.status_callback(msg, name),
                        10,
                    )
                    self.status_subs.append(status_sub)
        
        self.timer = self.create_timer(5.0, self.save_data)
        
        if experiment_name is None:
            experiment_name = datetime.now().strftime('experiment_%Y%m%d_%H%M%S')
        self.experiment_name = experiment_name
        self.data_dir = os.path.join(os.getcwd(), 'experiments', experiment_name)
        os.makedirs(self.data_dir, exist_ok=True)
        self.start_time = datetime.now()
        
        self.get_logger().info(f'Drone trajectory recorder initialized for experiment: {experiment_name}')
        self.get_logger().info(f'Mode: {"sim" if sim else "normal"}')
        self.get_logger().info(f'Drones: {self.drone_names}')
        self.get_logger().info(f'Data will be saved to: {self.data_dir}')
    
    def _ensure_drone(self, drone_name):
        """sim 模式下若遇到新 frame_id，动态加入无人机列表并初始化数据结构"""
        if drone_name in self.trajectories:
            return
        self.drone_names.append(drone_name)
        self.trajectories[drone_name] = {
            'x': [], 'y': [], 'z': [],
            'timestamp': [],
            'battery_voltage': [],
            'rssi': [],
        }
        self.timestamps[drone_name] = []
        self.status_latest[drone_name] = {'battery_voltage': None, 'rssi': None}
        self.get_logger().info(f'Sim: added drone from frame_id: {drone_name}')
    
    def path_callback(self, msg):
        """sim 模式：从 /cf_positions_path 的 poses 中按 frame_id 记录各无人机位姿"""
        for pose_stamped in msg.poses:
            frame_id = pose_stamped.header.frame_id
            if not frame_id:
                continue
            self._ensure_drone(frame_id)
            p = pose_stamped.pose.position
            t = pose_stamped.header.stamp
            timestamp = float(t.sec) + float(t.nanosec) * 1e-9
            if timestamp <= 0:
                current_time = datetime.now()
                timestamp = current_time.timestamp()
            else:
                try:
                    current_time = datetime.fromtimestamp(timestamp)
                except (OSError, ValueError):
                    current_time = datetime.now()
                    timestamp = current_time.timestamp()
            self.trajectories[frame_id]['x'].append(p.x)
            self.trajectories[frame_id]['y'].append(p.y)
            self.trajectories[frame_id]['z'].append(p.z)
            self.trajectories[frame_id]['timestamp'].append(timestamp)
            self.trajectories[frame_id]['battery_voltage'].append(None)
            self.trajectories[frame_id]['rssi'].append(None)
            self.timestamps[frame_id].append(current_time)
        
    def status_callback(self, msg, drone_name):
        """接收每架无人机的状态信息（电池电压、电台信号强度等）"""
        # 只保存最近一次的数值，在里程计到来时一起记录
        if drone_name in self.status_latest:
            self.status_latest[drone_name]['battery_voltage'] = getattr(msg, 'battery_voltage', None)
            self.status_latest[drone_name]['rssi'] = getattr(msg, 'rssi', None)

    def odom_callback(self, msg, drone_name):
        """处理里程计数据并记录轨迹"""
        # 获取位置数据
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # 获取当前时间戳
        current_time = datetime.now()
        timestamp = current_time.timestamp()
        
        # 读取当前最新的电池电压 / RSSI（可能为 None）
        battery_voltage = None
        rssi = None
        if drone_name in self.status_latest:
            battery_voltage = self.status_latest[drone_name].get('battery_voltage')
            rssi = self.status_latest[drone_name].get('rssi')
        
        # 记录轨迹与状态数据
        self.trajectories[drone_name]['x'].append(x)
        self.trajectories[drone_name]['y'].append(y)
        self.trajectories[drone_name]['z'].append(z)
        self.trajectories[drone_name]['timestamp'].append(timestamp)
        self.trajectories[drone_name]['battery_voltage'].append(battery_voltage)
        self.trajectories[drone_name]['rssi'].append(rssi)
        
        # 记录时间戳
        self.timestamps[drone_name].append(current_time)
        
        # 每100个数据点输出一次状态
        if len(self.trajectories[drone_name]['x']) % 100 == 0:
            self.get_logger().info(f'{drone_name}: Recorded {len(self.trajectories[drone_name]["x"])} trajectory points')
    
    def save_data(self):
        """保存轨迹数据到文件"""
        try:
            # 生成文件名（包含时间戳）
            timestamp_str = self.start_time.strftime('%Y%m%d_%H%M%S')
            
            # 保存为JSON格式
            json_filename = os.path.join(self.data_dir, f'drone_trajectories_{timestamp_str}.json')
            with open(json_filename, 'w', encoding='utf-8') as f:
                json.dump(self.trajectories, f, indent=2, ensure_ascii=False)
            
            # 保存为CSV格式（每架无人机一个文件），包含电池与RSSI
            for drone_name in self.drone_names:
                csv_filename = os.path.join(self.data_dir, f'{drone_name}_trajectory_{timestamp_str}.csv')
                with open(csv_filename, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(['Timestamp', 'X', 'Y', 'Z', 'BatteryVoltage', 'RSSI', 'DateTime'])
                    
                    for i in range(len(self.trajectories[drone_name]['x'])):
                        writer.writerow([
                            self.trajectories[drone_name]['timestamp'][i],
                            self.trajectories[drone_name]['x'][i],
                            self.trajectories[drone_name]['y'][i],
                            self.trajectories[drone_name]['z'][i],
                            self.trajectories[drone_name]['battery_voltage'][i],
                            self.trajectories[drone_name]['rssi'][i],
                            self.timestamps[drone_name][i].strftime('%Y-%m-%d %H:%M:%S.%f')
                        ])
            
            # 保存为Excel格式
            excel_filename = os.path.join(self.data_dir, f'drone_trajectories_{timestamp_str}.xlsx')
            with pd.ExcelWriter(excel_filename, engine='openpyxl') as writer:
                for drone_name in self.drone_names:
                    df = pd.DataFrame({
                        'Timestamp': self.trajectories[drone_name]['timestamp'],
                        'X': self.trajectories[drone_name]['x'],
                        'Y': self.trajectories[drone_name]['y'],
                        'Z': self.trajectories[drone_name]['z'],
                        'BatteryVoltage': self.trajectories[drone_name]['battery_voltage'],
                        'RSSI': self.trajectories[drone_name]['rssi'],
                        'DateTime': [t.strftime('%Y-%m-%d %H:%M:%S.%f') for t in self.timestamps[drone_name]]
                    })
                    df.to_excel(writer, sheet_name=drone_name, index=False)
            
            self.get_logger().info(f'Trajectory data saved:')
            self.get_logger().info(f'  JSON: {json_filename}')
            self.get_logger().info(f'  CSV files: {len(self.drone_names)} files')
            self.get_logger().info(f'  Excel: {excel_filename}')
            
            # 输出数据统计
            for drone_name in self.drone_names:
                point_count = len(self.trajectories[drone_name]['x'])
                if point_count > 0:
                    x_range = (min(self.trajectories[drone_name]['x']), max(self.trajectories[drone_name]['x']))
                    y_range = (min(self.trajectories[drone_name]['y']), max(self.trajectories[drone_name]['y']))
                    z_range = (min(self.trajectories[drone_name]['z']), max(self.trajectories[drone_name]['z']))
                    self.get_logger().info(f'{drone_name}: {point_count} points, X: {x_range}, Y: {y_range}, Z: {z_range}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving data: {str(e)}')
    
    def plot_trajectories(self, save_plot=True):
        """绘制3D轨迹图"""
        try:
            # 创建3D图形
            fig = plt.figure(figsize=(12, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # 为每架无人机绘制轨迹
            colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
            markers = ['o', 's', '^', 'v', '<', '>', 'D', 'P']
            
            for i, drone_name in enumerate(self.drone_names):
                if len(self.trajectories[drone_name]['x']) > 0:
                    color = colors[i % len(colors)]
                    marker = markers[i % len(markers)]
                    
                    x = self.trajectories[drone_name]['x']
                    y = self.trajectories[drone_name]['y']
                    z = self.trajectories[drone_name]['z']
                    
                    # 绘制轨迹线
                    ax.plot(x, y, z, color=color, linewidth=2, label=f'{drone_name} trajectory')
                    
                    # 标记起点和终点
                    if len(x) > 0:
                        ax.scatter(x[0], y[0], z[0], color=color, marker='o', s=100, label=f'{drone_name} start')
                        ax.scatter(x[-1], y[-1], z[-1], color=color, marker='*', s=200, label=f'{drone_name} end')
            
            # 设置图形属性
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title('Drone 3D Trajectories')
            ax.legend()
            ax.grid(True)
            
            # 设置坐标轴等比例
            ax.set_box_aspect([1, 1, 1])
            
            if save_plot:
                # 保存图片
                timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
                plot_filename = os.path.join(self.data_dir, f'drone_trajectories_3d_{timestamp_str}.png')
                plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
                self.get_logger().info(f'3D trajectory plot saved: {plot_filename}')
            
            plt.show()
            
        except Exception as e:
            self.get_logger().error(f'Error plotting trajectories: {str(e)}')
    
    def plot_2d_projections(self, save_plot=True):
        """绘制2D投影图"""
        try:
            fig, axes = plt.subplots(2, 2, figsize=(15, 12))
            fig.suptitle('Drone Trajectory 2D Projections', fontsize=16)
            
            colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
            
            for i, drone_name in enumerate(self.drone_names):
                if len(self.trajectories[drone_name]['x']) > 0:
                    color = colors[i % len(colors)]
                    x = self.trajectories[drone_name]['x']
                    y = self.trajectories[drone_name]['y']
                    z = self.trajectories[drone_name]['z']
                    
                    # XY平面投影
                    axes[0, 0].plot(x, y, color=color, linewidth=2, label=drone_name)
                    axes[0, 0].scatter(x[0], y[0], color=color, marker='o', s=100)
                    axes[0, 0].scatter(x[-1], y[-1], color=color, marker='*', s=200)
                    
                    # XZ平面投影
                    axes[0, 1].plot(x, z, color=color, linewidth=2, label=drone_name)
                    axes[0, 1].scatter(x[0], z[0], color=color, marker='o', s=100)
                    axes[0, 1].scatter(x[-1], z[-1], color=color, marker='*', s=200)
                    
                    # YZ平面投影
                    axes[1, 0].plot(y, z, color=color, linewidth=2, label=drone_name)
                    axes[1, 0].scatter(y[0], z[0], color=color, marker='o', s=100)
                    axes[1, 0].scatter(y[-1], z[-1], color=color, marker='*', s=200)
                    
                    # 高度随时间变化
                    timestamps = np.array(self.trajectories[drone_name]['timestamp'])
                    relative_time = timestamps - timestamps[0]
                    axes[1, 1].plot(relative_time, z, color=color, linewidth=2, label=drone_name)
            
            # 设置子图标题和标签
            axes[0, 0].set_title('XY Projection')
            axes[0, 0].set_xlabel('X (m)')
            axes[0, 0].set_ylabel('Y (m)')
            axes[0, 0].legend()
            axes[0, 0].grid(True)
            
            axes[0, 1].set_title('XZ Projection')
            axes[0, 1].set_xlabel('X (m)')
            axes[0, 1].set_ylabel('Z (m)')
            axes[0, 1].legend()
            axes[0, 1].grid(True)
            
            axes[1, 0].set_title('YZ Projection')
            axes[1, 0].set_xlabel('Y (m)')
            axes[1, 0].set_ylabel('Z (m)')
            axes[1, 0].legend()
            axes[1, 0].grid(True)
            
            axes[1, 1].set_title('Height vs Time')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Z (m)')
            axes[1, 1].legend()
            axes[1, 1].grid(True)
            
            plt.tight_layout()
            
            if save_plot:
                timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
                plot_filename = os.path.join(self.data_dir, f'drone_trajectories_2d_{timestamp_str}.png')
                plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
                self.get_logger().info(f'2D trajectory plots saved: {plot_filename}')
            
            plt.show()
            
        except Exception as e:
            self.get_logger().error(f'Error plotting 2D projections: {str(e)}')

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='无人机轨迹记录器')
    parser.add_argument('--experiment', type=str, help='实验名称（可选，默认使用时间戳）')
    parser.add_argument('--drones', nargs='+', help='无人机名称列表（可选，默认: cf231 cf232 ...）')
    parser.add_argument('--sim', action='store_true', help='sim 模式：订阅 /cf_positions_path (nav_msgs/Path)，按 frame_id 识别无人机')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    node = DroneTrajectoryRecorder(experiment_name=args.experiment, sim=args.sim)
    
    if args.drones and not args.sim:
        node.drone_names = args.drones
        # 普通模式需补全新无人机对应的数据结构
        for name in node.drone_names:
            if name not in node.trajectories:
                node.trajectories[name] = {
                    'x': [], 'y': [], 'z': [], 'timestamp': [],
                    'battery_voltage': [], 'rssi': [],
                }
                node.timestamps[name] = []
                node.status_latest[name] = {'battery_voltage': None, 'rssi': None}
        node.get_logger().info(f'Updated drone list: {node.drone_names}')
    
    try:
        # 运行节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 当用户按Ctrl+C时，保存数据并绘制轨迹
        print("\n正在保存轨迹数据并生成可视化图表...")
        node.save_data()
        node.plot_trajectories()
        node.plot_2d_projections()
        print("数据保存和可视化完成！")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
