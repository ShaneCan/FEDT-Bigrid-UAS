#!/usr/bin/env python3
"""
无人机轨迹可视化脚本
用于加载已保存的轨迹数据并生成各种可视化图表
"""

import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import argparse
from datetime import datetime
import seaborn as sns

class TrajectoryVisualizer:
    def __init__(self, data_file=None):
        self.data_file = data_file
        self.trajectories = {}
        self.timestamps = {}
        
        # 设置英文字体
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial']
        plt.rcParams['axes.unicode_minus'] = False
        
        # 设置Seaborn样式
        sns.set_style("whitegrid")
        sns.set_palette("husl")
    
    def load_json_data(self, json_file):
        """从JSON文件加载轨迹数据"""
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                self.trajectories = json.load(f)
            
            # 转换时间戳为相对时间
            for drone_name in self.trajectories:
                if self.trajectories[drone_name]['timestamp']:
                    start_time = self.trajectories[drone_name]['timestamp'][0]
                    self.trajectories[drone_name]['relative_time'] = [
                        t - start_time for t in self.trajectories[drone_name]['timestamp']
                    ]
            
            print(f"Successfully loaded trajectory data with {len(self.trajectories)} drones")
            for drone_name, data in self.trajectories.items():
                print(f"  {drone_name}: {len(data['x'])} data points")
            
            return True
        except Exception as e:
            print(f"Failed to load JSON data: {e}")
            return False
    
    def load_csv_data(self, csv_path):
        """从CSV文件加载轨迹数据"""
        try:
            if os.path.isdir(csv_path):
                # 如果是目录，加载目录下所有CSV文件
                csv_files = [f for f in os.listdir(csv_path) if f.endswith('.csv')]
                print(f"Found {len(csv_files)} CSV files in directory {csv_path}")
                
                for csv_file in csv_files:
                    drone_name = csv_file.split('_')[0]  # 假设文件名格式为 "drone_name_trajectory_*.csv"
                    file_path = os.path.join(csv_path, csv_file)
                    self._load_single_csv(file_path, drone_name)
                    
            elif os.path.isfile(csv_path):
                # 如果是单个文件，从文件名推断无人机名称
                filename = os.path.basename(csv_path)
                drone_name = filename.split('_')[0]
                self._load_single_csv(csv_path, drone_name)
                
            else:
                print(f"Error: Path {csv_path} does not exist")
                return False
            
            print(f"Successfully loaded CSV data with {len(self.trajectories)} drones")
            return True
        except Exception as e:
            print(f"Failed to load CSV data: {e}")
            return False
    
    def _load_single_csv(self, file_path, drone_name):
        """加载单个CSV文件"""
        try:
            df = pd.read_csv(file_path)
            
            # 检查必要的列是否存在
            required_columns = ['X', 'Y', 'Z', 'Timestamp']
            missing_columns = [col for col in required_columns if col not in df.columns]
            if missing_columns:
                print(f"Warning: File {file_path} missing columns: {missing_columns}")
                return
            
            self.trajectories[drone_name] = {
                'x': df['X'].tolist(),
                'y': df['Y'].tolist(),
                'z': df['Z'].tolist(),
                'timestamp': df['Timestamp'].tolist(),
                'relative_time': (df['Timestamp'] - df['Timestamp'].iloc[0]).tolist()
            }
            
            print(f"  Successfully loaded {drone_name}: {len(df)} data points")
            
        except Exception as e:
            print(f"Failed to load file {file_path}: {e}")
    
    def plot_3d_trajectories(self, save_path=None, show_plot=True):
        """绘制3D轨迹图"""
        if not self.trajectories:
            print("No trajectory data to plot")
            return
        
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        colors = plt.cm.tab10(np.linspace(0, 1, len(self.trajectories)))
        
        for i, (drone_name, data) in enumerate(self.trajectories.items()):
            if len(data['x']) > 0:
                color = colors[i]
                
                # 绘制轨迹线
                ax.plot(data['x'], data['y'], data['z'], 
                       color=color, linewidth=3, label=f'{drone_name} Trajectory',
                       alpha=0.8)
                
                # 标记起点和终点
                ax.scatter(data['x'][0], data['y'][0], data['z'][0], 
                          color=color, marker='o', s=150, label=f'{drone_name} Start',
                          edgecolors='black', linewidth=2)
                ax.scatter(data['x'][-1], data['y'][-1], data['z'][-1], 
                          color=color, marker='*', s=300, label=f'{drone_name} End',
                          edgecolors='black', linewidth=2)
        
        ax.set_xlabel('X (m)', fontsize=26, labelpad=18)
        ax.set_ylabel('Y (m)', fontsize=26, labelpad=18)
        ax.set_zlabel('Z (m)', fontsize=26, labelpad=18)
        #ax.set_title('Drone 3D Trajectory', fontsize=16, fontweight='bold')
        ax.legend(fontsize=13)
        ax.grid(True, alpha=0.3)
        

        # 设置坐标轴刻度字体大小
        ax.tick_params(axis='x', labelsize=20, pad=8)
        ax.tick_params(axis='y', labelsize=20, pad=8)
        ax.tick_params(axis='z', labelsize=20, pad=8)
        
        # 设置Z轴从0开始
        ax.set_zlim(bottom=0)
        
        # 设置坐标轴等比例
        ax.set_box_aspect([1, 1, 1])
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"3D trajectory plot saved to: {save_path}")
        
        if show_plot:
            plt.show()
    
    def plot_2d_projections(self, save_path=None, show_plot=True):
        """绘制2D投影图"""
        if not self.trajectories:
            print("No trajectory data to plot")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        #fig.suptitle('Drone Trajectory 2D Projections', fontsize=18, fontweight='bold')
        
        colors = plt.cm.tab10(np.linspace(0, 1, len(self.trajectories)))
        
        for i, (drone_name, data) in enumerate(self.trajectories.items()):
            if len(data['x']) > 0:
                color = colors[i]
                
                # XY平面投影
                axes[0, 0].plot(data['x'], data['y'], color=color, linewidth=2, label=drone_name)
                axes[0, 0].scatter(data['x'][0], data['y'][0], color=color, marker='o', s=100)
                axes[0, 0].scatter(data['x'][-1], data['y'][-1], color=color, marker='*', s=200)
                
                # XZ平面投影
                axes[0, 1].plot(data['x'], data['z'], color=color, linewidth=2, label=drone_name)
                axes[0, 1].scatter(data['x'][0], data['z'][0], color=color, marker='o', s=100)
                axes[0, 1].scatter(data['x'][-1], data['z'][-1], color=color, marker='*', s=200)
                
                # YZ平面投影
                axes[1, 0].plot(data['y'], data['z'], color=color, linewidth=2, label=drone_name)
                axes[1, 0].scatter(data['y'][0], data['z'][0], color=color, marker='o', s=100)
                axes[1, 0].scatter(data['y'][-1], data['z'][-1], color=color, marker='*', s=200)
                
                # 高度随时间变化
                axes[1, 1].plot(data['relative_time'], data['z'], color=color, linewidth=2, label=drone_name)
        
        # 设置子图标题和标签
        axes[0, 0].set_title('XY Plane Projection', fontsize=22, fontweight='bold')
        axes[0, 0].set_xlabel('X (m)', fontsize=20)
        axes[0, 0].set_ylabel('Y (m)', fontsize=20)
        axes[0, 0].legend(fontsize=13)
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].tick_params(axis='x', labelsize=20, pad=8)
        axes[0, 0].tick_params(axis='y', labelsize=20, pad=8)

        
        axes[0, 1].set_title('XZ Plane Projection', fontsize=22, fontweight='bold')
        axes[0, 1].set_xlabel('X (m)', fontsize=20)
        axes[0, 1].set_ylabel('Z (m)', fontsize=20)
        axes[0, 1].set_ylim(bottom=0)  # 设置Z轴从0开始
        axes[0, 1].legend(fontsize=13)
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].tick_params(axis='x', labelsize=20, pad=8)
        axes[0, 1].tick_params(axis='y', labelsize=20, pad=8)
        
        axes[1, 0].set_title('YZ Plane Projection', fontsize=22, fontweight='bold')
        axes[1, 0].set_xlabel('Y (m)', fontsize=20)
        axes[1, 0].set_ylabel('Z (m)', fontsize=20)
        axes[1, 0].set_ylim(bottom=0)  # 设置Z轴从0开始
        axes[1, 0].legend(fontsize=13)
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].tick_params(axis='x', labelsize=20, pad=8)
        axes[1, 0].tick_params(axis='y', labelsize=20, pad=8)
        
        axes[1, 1].set_title('Altitude vs Time', fontsize=22, fontweight='bold')
        axes[1, 1].set_xlabel('Time (s)', fontsize=20)
        axes[1, 1].set_ylabel('Z (m)', fontsize=20)
        axes[1, 1].set_ylim(bottom=0)  # 设置Z轴从0开始
        axes[1, 1].legend(fontsize=13)
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].tick_params(axis='x', labelsize=20, pad=8)
        axes[1, 1].tick_params(axis='y', labelsize=20, pad=8)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"2D projection plots saved to: {save_path}")
        
        if show_plot:
            plt.show()
    
    def plot_velocity_analysis(self, save_path=None, show_plot=True):
        """绘制速度分析图"""
        if not self.trajectories:
            print("No trajectory data to plot")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Drone Velocity Analysis', fontsize=18, fontweight='bold')
        
        colors = plt.cm.tab10(np.linspace(0, 1, len(self.trajectories)))
        
        for i, (drone_name, data) in enumerate(self.trajectories.items()):
            if len(data['x']) > 1:
                color = colors[i]
                
                # 计算速度
                x_vel = np.diff(data['x']) / np.diff(data['relative_time'])
                y_vel = np.diff(data['y']) / np.diff(data['relative_time'])
                z_vel = np.diff(data['z']) / np.diff(data['relative_time'])
                total_vel = np.sqrt(x_vel**2 + y_vel**2 + z_vel**2)
                
                time_vel = data['relative_time'][1:]  # 速度对应的时间点
                
                # X方向速度
                axes[0, 0].plot(time_vel, x_vel, color=color, linewidth=2, label=drone_name)
                
                # Y方向速度
                axes[0, 1].plot(time_vel, y_vel, color=color, linewidth=2, label=drone_name)
                
                # Z方向速度
                axes[1, 0].plot(time_vel, z_vel, color=color, linewidth=2, label=drone_name)
                
                # 总速度
                axes[1, 1].plot(time_vel, total_vel, color=color, linewidth=2, label=drone_name)
        
        axes[0, 0].set_title('X Velocity', fontsize=14, fontweight='bold')
        axes[0, 0].set_xlabel('Time (s)', fontsize=12)
        axes[0, 0].set_ylabel('Velocity (m/s)', fontsize=12)
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        axes[0, 1].set_title('Y Velocity', fontsize=14, fontweight='bold')
        axes[0, 1].set_xlabel('Time (s)', fontsize=12)
        axes[0, 1].set_ylabel('Velocity (m/s)', fontsize=12)
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        axes[1, 0].set_title('Z Velocity', fontsize=14, fontweight='bold')
        axes[1, 0].set_xlabel('Time (s)', fontsize=12)
        axes[1, 0].set_ylabel('Velocity (m/s)', fontsize=12)
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        axes[1, 1].set_title('Total Velocity', fontsize=14, fontweight='bold')
        axes[1, 1].set_xlabel('Time (s)', fontsize=12)
        axes[1, 1].set_ylabel('Velocity (m/s)', fontsize=12)
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Velocity analysis plot saved to: {save_path}")
        
        if show_plot:
            plt.show()
    
    def plot_distance_analysis(self, save_path=None, show_plot=True):
        """绘制距离分析图"""
        if not self.trajectories:
            print("No trajectory data to plot")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Drone Distance Analysis', fontsize=18, fontweight='bold')
        
        colors = plt.cm.tab10(np.linspace(0, 1, len(self.trajectories)))
        
        for i, (drone_name, data) in enumerate(self.trajectories.items()):
            if len(data['x']) > 0:
                color = colors[i]
                
                # 计算累计距离
                if len(data['x']) > 1:
                    dx = np.diff(data['x'])
                    dy = np.diff(data['y'])
                    dz = np.diff(data['z'])
                    segment_distances = np.sqrt(dx**2 + dy**2 + dz**2)
                    cumulative_distance = np.cumsum(segment_distances)
                    time_dist = data['relative_time'][1:]
                else:
                    cumulative_distance = [0]
                    time_dist = [0]
                
                # 累计距离
                axes[0, 0].plot(time_dist, cumulative_distance, color=color, linewidth=2, label=drone_name)
                
                # 高度分布直方图
                axes[0, 1].hist(data['z'], bins=20, alpha=0.7, color=color, label=drone_name)
                
                # 轨迹长度分布
                if len(data['x']) > 1:
                    axes[1, 0].hist(segment_distances, bins=20, alpha=0.7, color=color, label=drone_name)
                
                # 3D散点图（颜色表示高度）
                scatter = axes[1, 1].scatter(data['x'], data['y'], c=data['z'], 
                                           cmap='viridis', s=20, alpha=0.7, label=drone_name)
        
        axes[0, 0].set_title('Cumulative Flight Distance', fontsize=14, fontweight='bold')
        axes[0, 0].set_xlabel('Time (s)', fontsize=12)
        axes[0, 0].set_ylabel('Distance (m)', fontsize=12)
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        axes[0, 1].set_title('Altitude Distribution', fontsize=14, fontweight='bold')
        axes[0, 1].set_xlabel('Altitude (m)', fontsize=12)
        axes[0, 1].set_ylabel('Frequency', fontsize=12)
        axes[0, 1].set_xlim(left=0)  # 设置高度轴从0开始
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        axes[1, 0].set_title('Trajectory Segment Length Distribution', fontsize=14, fontweight='bold')
        axes[1, 0].set_xlabel('Segment Length (m)', fontsize=12)
        axes[1, 0].set_ylabel('Frequency', fontsize=12)
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        axes[1, 1].set_title('XY Plane Trajectory (Color = Altitude)', fontsize=14, fontweight='bold')
        axes[1, 1].set_xlabel('X (m)', fontsize=12)
        axes[1, 1].set_ylabel('Y (m)', fontsize=12)
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Distance analysis plot saved to: {save_path}")
        
        if show_plot:
            plt.show()
    
    def generate_summary_report(self):
        """生成轨迹数据摘要报告"""
        if not self.trajectories:
            print("No trajectory data to analyze")
            return
        
        print("\n" + "="*60)
        print("Drone Trajectory Data Summary Report")
        print("="*60)
        
        for drone_name, data in self.trajectories.items():
            if len(data['x']) > 0:
                print(f"\n{drone_name}:")
                print(f"  Number of data points: {len(data['x'])}")
                print(f"  Flight time: {data['relative_time'][-1]:.2f} seconds")
                
                # 位置范围
                x_range = (min(data['x']), max(data['x']))
                y_range = (min(data['y']), max(data['y']))
                z_range = (min(data['z']), max(data['z']))
                print(f"  X range: {x_range[0]:.3f} ~ {x_range[1]:.3f} m")
                print(f"  Y range: {y_range[0]:.3f} ~ {y_range[1]:.3f} m")
                print(f"  Z range: {z_range[0]:.3f} ~ {z_range[1]:.3f} m")
                
                # 计算累计距离
                if len(data['x']) > 1:
                    dx = np.diff(data['x'])
                    dy = np.diff(data['y'])
                    dz = np.diff(data['z'])
                    segment_distances = np.sqrt(dx**2 + dy**2 + dz**2)
                    total_distance = np.sum(segment_distances)
                    avg_speed = total_distance / data['relative_time'][-1]
                    print(f"  Total flight distance: {total_distance:.3f} m")
                    print(f"  Average speed: {avg_speed:.3f} m/s")
                
                # 起点和终点
                print(f"  Start point: ({data['x'][0]:.3f}, {data['y'][0]:.3f}, {data['z'][0]:.3f})")
                print(f"  End point: ({data['x'][-1]:.3f}, {data['y'][-1]:.3f}, {data['z'][-1]:.3f})")
        
        print("\n" + "="*60)

def main():
    parser = argparse.ArgumentParser(description='无人机轨迹可视化工具')
    parser.add_argument('--json', type=str, help='JSON轨迹数据文件路径')
    parser.add_argument('--csv', type=str, help='CSV轨迹数据文件或目录路径')
    parser.add_argument('--output_dir', type=str, default='trajectory_plots', help='输出图片目录')
    parser.add_argument('--no_show', action='store_true', help='不显示图片，只保存')
    
    args = parser.parse_args()
    
    # 创建输出目录
    os.makedirs(args.output_dir, exist_ok=True)
    
    # 创建可视化器
    visualizer = TrajectoryVisualizer()
    
    # 加载数据
    if args.json:
        if not visualizer.load_json_data(args.json):
            return
    elif args.csv:
        if not visualizer.load_csv_data(args.csv):
            return
    else:
        print("Please specify data file:")
        print("  --json <file_path>    Load trajectory data in JSON format")
        print("  --csv <file_or_dir>   Load trajectory data in CSV format")
        print("\nExamples:")
        print("  python3 visualize_trajectories.py --json drone_trajectories.json")
        print("  python3 visualize_trajectories.py --csv cf231_trajectory.csv")
        print("  python3 visualize_trajectories.py --csv drone_trajectory_data/")
        return
    
    # 生成摘要报告
    visualizer.generate_summary_report()
    
    # 生成各种可视化图表
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    # 3D轨迹图
    visualizer.plot_3d_trajectories(
        save_path=os.path.join(args.output_dir, f'3d_trajectories_{timestamp}.png'),
        show_plot=not args.no_show
    )
    
    # 2D投影图
    visualizer.plot_2d_projections(
        save_path=os.path.join(args.output_dir, f'2d_projections_{timestamp}.png'),
        show_plot=not args.no_show
    )
    
    # 速度分析图
    visualizer.plot_velocity_analysis(
        save_path=os.path.join(args.output_dir, f'velocity_analysis_{timestamp}.png'),
        show_plot=not args.no_show
    )
    
    # 距离分析图
    visualizer.plot_distance_analysis(
        save_path=os.path.join(args.output_dir, f'distance_analysis_{timestamp}.png'),
        show_plot=not args.no_show
    )
    
    print(f"\nAll plots saved to: {args.output_dir}")

if __name__ == '__main__':
    main()
