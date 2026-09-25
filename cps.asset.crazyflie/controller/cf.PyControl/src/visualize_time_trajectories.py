#!/usr/bin/env python3
"""
无人机轨迹可视化脚本
用于加载已保存的轨迹数据并生成各种可视化图表
"""

import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import os
import argparse
from datetime import datetime
import seaborn as sns

class TrajectoryVisualizer:
    def __init__(self, data_file=None):
        self.data_file = data_file
        self.trajectories = {}
        self.timestamps = {}
        
        # 设置Seaborn样式
        sns.set_style("whitegrid")
        sns.set_palette("husl")
        
        # 在 Seaborn 设置之后，再强制指定整体字体为论文常用的衬线体
        plt.rcParams['font.family'] = 'serif'
        plt.rcParams['font.serif'] = ['Times New Roman', 'DejaVu Serif']
        plt.rcParams['axes.unicode_minus'] = False
    
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
            
            data = {
                'x': df['X'].tolist(),
                'y': df['Y'].tolist(),
                'z': df['Z'].tolist(),
                'timestamp': df['Timestamp'].tolist(),
                'relative_time': (df['Timestamp'] - df['Timestamp'].iloc[0]).tolist()
            }
            # 可选列：BatteryVoltage, RSSI
            if 'BatteryVoltage' in df.columns:
                data['battery'] = df['BatteryVoltage'].tolist()
            if 'RSSI' in df.columns:
                data['rssi'] = df['RSSI'].tolist()
            self.trajectories[drone_name] = data
            
            print(f"  Successfully loaded {drone_name}: {len(df)} data points")
            
        except Exception as e:
            print(f"Failed to load file {file_path}: {e}")
    
    def plot_3d_trajectories(self, save_path=None, show_plot=True):
        """绘制3D轨迹图：
        - **轨迹线用 viridis 色系表示时间**（论文风格，时间过渡清晰可读）
        - **起点和终点用圆点**，起点标注 UAV1、UAV2 等以区分无人机
        - 在整个空间范围内绘制 1x1x0.35 的 3D 栅格
        """
        if not self.trajectories:
            print("No trajectory data to plot")
            return

        # 统计全局时间范围（用于时间着色）和空间范围（用于栅格）
        all_times = []
        all_x, all_y, all_z = [], [], []
        for data in self.trajectories.values():
            if 'relative_time' in data and len(data['relative_time']) > 0:
                all_times.extend(data['relative_time'])
            if len(data.get('x', [])) > 0:
                all_x.extend(data['x'])
                all_y.extend(data['y'])
                all_z.extend(data['z'])

        use_time_color = len(all_times) > 0
        if use_time_color:
            t_min = min(all_times)
            t_max = max(all_times)
            # 避免除零
            if t_max == t_min:
                t_max = t_min + 1e-6
        else:
            t_min, t_max = 0.0, 1.0

        # 计算空间范围，用于设置坐标轴和栅格范围
        if len(all_x) > 0:
            x_min, x_max = min(all_x), max(all_x)
            y_min, y_max = min(all_y), max(all_y)
            z_min, z_max = min(all_z), max(all_z)
        else:
            x_min = y_min = z_min = 0.0
            x_max = y_max = z_max = 1.0
        
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 轨迹线用 viridis 色系表示时间：深紫→靛蓝→青绿→黄绿，符合高水平论文审美
        # 感知均匀、色盲友好，Nature/Science 常用，时间过渡清晰且不刺眼
        time_cmap = plt.cm.viridis

        # 起点/终点颜色：符合高水平论文的配色（ColorBrewer/色盲友好）
        marker_colors = [
            '#0173b2',  # 蓝
            '#de8f05',  # 琥珀
            '#029e73',  # 青绿
            '#cc78bc',  # 粉紫
            '#ca9161',  # 棕褐
        ]

        for i, (drone_name, data) in enumerate(self.trajectories.items()):
            if len(data['x']) > 0:
                xs = np.asarray(data['x'], dtype=float)
                ys = np.asarray(data['y'], dtype=float)
                zs = np.asarray(data['z'], dtype=float)
                valid_mask = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
                if not np.any(valid_mask):
                    continue

                first_idx = int(np.argmax(valid_mask))
                last_idx = int(len(valid_mask) - 1 - np.argmax(valid_mask[::-1]))

                # 1）轨迹线：用 Line3DCollection 按时间着色（viridis 色系，论文风格）
                segments = []
                seg_colors = []
                ts = np.asarray(data.get('relative_time', []), dtype=float) if use_time_color else None
                for j in range(first_idx, last_idx):
                    seg = [[xs[j], ys[j], zs[j]], [xs[j + 1], ys[j + 1], zs[j + 1]]]
                    segments.append(seg)
                    if use_time_color and ts is not None and len(ts) == len(xs) and np.isfinite(ts[j]):
                        t_norm = (ts[j] - t_min) / (t_max - t_min)
                        seg_colors.append(time_cmap(t_norm))
                    else:
                        seg_colors.append((0.5, 0.5, 0.5, 0.9))

                if segments:
                    # alpha=0.75 略带透明度，避免轨迹过于厚重
                    lc = Line3DCollection(segments, colors=seg_colors, linewidths=2.0, alpha=0.75)
                    ax.add_collection3d(lc)

                # 2）起点和终点：都用圆点，每架无人机用不同颜色区分
                mc = marker_colors[i % len(marker_colors)]
                ax.scatter(
                    xs[first_idx], ys[first_idx], zs[first_idx],
                    marker='o',
                    s=150,
                    facecolors='none',
                    edgecolors=mc,
                    linewidths=2.0,
                    zorder=8,
                    depthshade=False,
                )
                ax.scatter(
                    xs[last_idx], ys[last_idx], zs[last_idx],
                    marker='o',
                    s=150,
                    color=mc,
                    edgecolors=mc,
                    linewidths=2.0,
                    zorder=9,
                    depthshade=False,
                )

                # 3）起点标注无人机名称（直接使用数据中的 drone_name，如 cf231）
                ax.text(
                    xs[first_idx], ys[first_idx], zs[first_idx],
                    f'  {drone_name}',
                    fontsize=14,
                    fontweight='bold',
                    zorder=10,
                )

        
        ax.set_xlabel('X [m]', fontsize=26, labelpad=18)
        ax.set_ylabel('Y [m]', fontsize=26, labelpad=18)
        ax.set_zlabel('Z [m]', fontsize=26, labelpad=18)
        # 关闭 Matplotlib 自带的坐标轴网格，只保留我们自定义的 3D 栅格
        ax.grid(False)
        

        # 设置坐标轴刻度字体大小
        ax.tick_params(axis='x', labelsize=20, pad=8)
        ax.tick_params(axis='y', labelsize=20, pad=8)
        ax.tick_params(axis='z', labelsize=20, pad=8)
        
        # 设置坐标轴范围，使其与数据和栅格对齐
        # X/Y 用 1m 网格，**边界对齐到 -0.5, 0.5, 1.5, … 这种半格点**，
        # 也就是每个 1×1 的方格以整数为中心、以 n±0.5 为边界
        import math
        grid_step_xy = 0.5  ########修改网格大小

        def _half_shifted_bounds(v_min, v_max, step):
            half = step / 2
            start = math.floor((v_min - half) / step) * step + half
            end = math.ceil((v_max + half) / step) * step - half
            return start, end

        x_min_grid, x_max_grid = _half_shifted_bounds(x_min, x_max, grid_step_xy)
        y_min_grid, y_max_grid = _half_shifted_bounds(y_min, y_max, grid_step_xy)

        z_step = 0.35
        z_min_grid = max(0.0, math.floor(z_min / z_step) * z_step)  # Z 轴从 0 开始，不出现负数
        z_max_grid = math.ceil(z_max / z_step) * z_step

        ax.set_xlim(x_min_grid, x_max_grid)
        ax.set_ylim(y_min_grid, y_max_grid)
        ax.set_zlim(z_min_grid, z_max_grid)
        
        # 设置坐标轴等比例
        ax.set_box_aspect([1, 1, 1])

        # 绘制 1x1x0.35 的 3D 栅格线（wireframe）
        # 在 X/Y 平面上画纵向和横向格线，在 Z 方向按 0.35m 分层，
        # 并在每个 (x, y) 网格交点沿 Z 方向连成立方体框架
        # 栅格线使用极细线，降低透明度，不干扰轨迹阅读
        grid_color = (0.0, 0.6, 0.0, 0.25)  # RGBA: 淡绿，低透明度
        grid_linewidth = 0.3  # 最细，仅作背景参考

        # X/Y 方向的网格线位置：半格点（…, -0.5, 0.5, 1.5, …）
        x_lines = np.arange(x_min_grid, x_max_grid + 1e-6, grid_step_xy)
        y_lines = np.arange(y_min_grid, y_max_grid + 1e-6, grid_step_xy)
        z_layers = np.arange(z_min_grid, z_max_grid + 1e-6, z_step)

        # 朝外的面用实线，内部的线用虚线；底面内部的线也用虚线
        def _is_outer_line(v, v_min, v_max, tol=1e-9):
            return abs(v - v_min) < tol or abs(v - v_max) < tol

        # X 方向平行线（随 Y/Z 变化）：仅当在左右面边界(x边界)时为实线，底面/顶面内部为虚线
        for x in x_lines:
            for z in z_layers:
                outer = _is_outer_line(x, x_min_grid, x_max_grid)
                ax.plot(
                    [x, x],
                    [y_min_grid, y_max_grid],
                    [z, z],
                    color=grid_color,
                    linewidth=grid_linewidth,
                    linestyle='-' if outer else '--',
                )

        # Y 方向平行线（随 X/Z 变化）：仅当在前后面边界(y边界)时为实线，底面/顶面内部为虚线
        for y in y_lines:
            for z in z_layers:
                outer = _is_outer_line(y, y_min_grid, y_max_grid)
                ax.plot(
                    [x_min_grid, x_max_grid],
                    [y, y],
                    [z, z],
                    color=grid_color,
                    linewidth=grid_linewidth,
                    linestyle='-' if outer else '--',
                )

        # Z 方向平行线（随 X/Y 变化）：在底面/顶面周界(x或y边界)时为实线，内部为虚线
        for x in x_lines:
            for y in y_lines:
                outer = _is_outer_line(x, x_min_grid, x_max_grid) or _is_outer_line(y, y_min_grid, y_max_grid)
                ax.plot(
                    [x, x],
                    [y, y],
                    [z_min_grid, z_max_grid],
                    color=grid_color,
                    linewidth=grid_linewidth,
                    linestyle='-' if outer else '--',
                )

        # 如果有时间着色，添加颜色条（蓝→白→红 表示时间从早到晚）
        if use_time_color:
            from matplotlib.cm import ScalarMappable
            from matplotlib.colors import Normalize
            sm = ScalarMappable(cmap=time_cmap, norm=Normalize(vmin=t_min, vmax=t_max))
            sm.set_array([])
            cbar = plt.colorbar(sm, ax=ax, pad=0.1, shrink=0.8)
            cbar.set_label('Time (s, relative)', fontsize=20)
            cbar.ax.tick_params(labelsize=18)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"3D trajectory plot saved to: {save_path}")
        
        if show_plot:
            plt.show()
    

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
    # visualizer.generate_summary_report()
    
    # 生成各种可视化图表
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    # 3D轨迹图
    visualizer.plot_3d_trajectories(
        save_path=os.path.join(args.output_dir, f'3d_trajectories_{timestamp}.png'),
        show_plot=not args.no_show
    )
    
    
    print(f"\nAll plots saved to: {args.output_dir}")

if __name__ == '__main__':
    main()
