#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多层地图论文评价指标分析工具

为多层SLAM地图提供全面的评价指标，包括：
1. 覆盖率分析 (Coverage Analysis)
2. 层间一致性分析 (Inter-layer Consistency)
3. 重叠区域分析 (Overlap Analysis)
4. 地图质量指标 (Map Quality Metrics)
5. 空间分布分析 (Spatial Distribution Analysis)
6. 时间序列分析 (Temporal Analysis)
"""

import os
import sys
import numpy as np
import yaml
from pathlib import Path
import argparse
from typing import Dict, List, Tuple, Optional
import pandas as pd
from scipy import ndimage
from scipy.spatial.distance import cdist
from sklearn.metrics import adjusted_rand_score
from datetime import datetime

LEGACY_ANALYSIS_FIGURES = [
    'coverage_analysis.png',
    'consistency_analysis.png',
    'spatial_analysis.png',
    'temporal_analysis.png',
    'radar_chart.png',
]

class MultiLayerMapAnalyzer:
    """多层地图分析器"""
    
    def __init__(self, experiment_dir: Path):
        self.experiment_dir = experiment_dir
        self.export_dir = experiment_dir / 'export'
        self.layers_data = []
        self.metrics = {}
        
        # 实际场地尺寸 (米)
        self.actual_field_width = 2.15  # 实际场地宽度
        self.actual_field_height = 2.0   # 实际场地高度
        
    def load_experiment_data(self):
        """加载实验数据"""
        print("正在加载实验数据...")
        
        # 加载map_info
        map_info_path = self.experiment_dir / 'map_info.yaml'
        with open(map_info_path, 'r') as f:
            self.map_info = yaml.safe_load(f)
        
        # 加载各层数据
        for layer_id in self.map_info['layers']:
            try:
                # 加载NPY数据
                npy_path = self.export_dir / f'layer_{layer_id}.npy'
                grid = np.load(npy_path)
                
                # 加载元数据
                yaml_path = self.export_dir / f'layer_{layer_id}.yaml'
                with open(yaml_path, 'r') as f:
                    metadata = yaml.safe_load(f)
                
                # 加载原始元数据
                orig_meta_path = self.experiment_dir / f'layer_{layer_id}_metadata.yaml'
                with open(orig_meta_path, 'r') as f:
                    orig_metadata = yaml.safe_load(f)
                
                self.layers_data.append({
                    'layer_id': layer_id,
                    'grid': grid,
                    'metadata': metadata,
                    'orig_metadata': orig_metadata,
                    'timestamp': orig_metadata.get('timestamp', '')
                })
                
                print(f"已加载层 {layer_id}: {grid.shape}, 时间戳: {orig_metadata.get('timestamp', 'N/A')}")
                
            except Exception as e:
                print(f"加载层 {layer_id} 时出错: {e}")
                continue
        
        print(f"成功加载 {len(self.layers_data)} 层数据")
        return len(self.layers_data) > 0
    
    def get_actual_field_mask(self, grid: np.ndarray, resolution: float) -> np.ndarray:
        """获取实际场地区域的掩码"""
        h, w = grid.shape
        
        # 计算实际场地区域在像素坐标中的范围
        # 假设地图中心对应场地中心
        field_width_pixels = int(self.actual_field_width / resolution)
        field_height_pixels = int(self.actual_field_height / resolution)
        
        # 计算场地区域在网格中的起始和结束位置
        start_x = max(0, (w - field_width_pixels) // 2)
        end_x = min(w, start_x + field_width_pixels)
        start_y = max(0, (h - field_height_pixels) // 2)
        end_y = min(h, start_y + field_height_pixels)
        
        # 创建掩码
        mask = np.zeros((h, w), dtype=bool)
        mask[start_y:end_y, start_x:end_x] = True
        
        return mask
    
    def calculate_coverage_metrics(self):
        """计算覆盖率指标"""
        print("计算覆盖率指标...")
        
        coverage_metrics = {}
        
        for layer_data in self.layers_data:
            layer_id = layer_data['layer_id']
            grid = layer_data['grid']
            resolution = layer_data['metadata'].get('resolution', 0.05)
            
            # 获取实际场地区域掩码
            field_mask = self.get_actual_field_mask(grid, resolution)
            
            # 只计算实际场地区域的统计
            field_grid = grid[field_mask]
            
            # 基本统计 (仅限实际场地区域)
            total_field_cells = np.sum(field_mask)
            occupied_cells = np.sum(field_grid == 100)
            free_cells = np.sum(field_grid == 0)
            unknown_cells = np.sum(field_grid == -1)
            
            # 覆盖率计算 (基于实际场地区域)
            explored_cells = occupied_cells + free_cells
            coverage_ratio = explored_cells / total_field_cells if total_field_cells > 0 else 0
            occupied_ratio = occupied_cells / total_field_cells if total_field_cells > 0 else 0
            free_ratio = free_cells / total_field_cells if total_field_cells > 0 else 0
            unknown_ratio = unknown_cells / total_field_cells if total_field_cells > 0 else 0
            
            # 地图质量指标
            if explored_cells > 0:
                occupied_in_explored = occupied_cells / explored_cells
            else:
                occupied_in_explored = 0
            
            # 计算实际场地面积 (平方米)
            actual_field_area = self.actual_field_width * self.actual_field_height
            
            coverage_metrics[layer_id] = {
                'total_cells': total_field_cells,  # 实际场地区域的总细胞数
                'occupied_cells': occupied_cells,
                'free_cells': free_cells,
                'unknown_cells': unknown_cells,
                'explored_cells': explored_cells,
                'coverage_ratio': coverage_ratio,
                'occupied_ratio': occupied_ratio,
                'free_ratio': free_ratio,
                'unknown_ratio': unknown_ratio,
                'occupied_in_explored': occupied_in_explored,
                'actual_field_area_m2': actual_field_area,
                'actual_field_width_m': self.actual_field_width,
                'actual_field_height_m': self.actual_field_height
            }
        
        self.metrics['coverage'] = coverage_metrics
        return coverage_metrics
    
    def calculate_inter_layer_consistency(self):
        """计算层间一致性指标"""
        print("计算层间一致性指标...")
        
        if len(self.layers_data) < 2:
            print("需要至少2层数据来计算一致性")
            return {}
        
        consistency_metrics = {}
        
        # 获取所有层的网格
        grids = [layer_data['grid'] for layer_data in self.layers_data]
        layer_ids = [layer_data['layer_id'] for layer_data in self.layers_data]
        
        # 确保所有层具有相同的形状
        base_shape = grids[0].shape
        for i, grid in enumerate(grids):
            if grid.shape != base_shape:
                print(f"警告: 层 {layer_ids[i]} 形状不匹配: {grid.shape} vs {base_shape}")
                return {}
        
        # 计算层间重叠 (仅限实际场地区域)
        for i in range(len(grids)):
            for j in range(i + 1, len(grids)):
                layer_i, layer_j = layer_ids[i], layer_ids[j]
                
                # 获取实际场地区域掩码
                resolution_i = self.layers_data[i]['metadata'].get('resolution', 0.05)
                resolution_j = self.layers_data[j]['metadata'].get('resolution', 0.05)
                field_mask_i = self.get_actual_field_mask(grids[i], resolution_i)
                field_mask_j = self.get_actual_field_mask(grids[j], resolution_j)
                
                # 使用实际场地区域的重叠分析
                field_overlap_mask = field_mask_i & field_mask_j
                field_grid_i = grids[i][field_overlap_mask]
                field_grid_j = grids[j][field_overlap_mask]
                
                # 重叠区域分析 (仅限实际场地区域)
                both_explored = (field_grid_i != -1) & (field_grid_j != -1)
                both_occupied = (field_grid_i == 100) & (field_grid_j == 100)
                both_free = (field_grid_i == 0) & (field_grid_j == 0)
                
                # 一致性指标
                total_overlap = np.sum(both_explored)
                if total_overlap > 0:
                    consistency_occupied = np.sum(both_occupied) / total_overlap
                    consistency_free = np.sum(both_free) / total_overlap
                    overall_consistency = (np.sum(both_occupied) + np.sum(both_free)) / total_overlap
                else:
                    consistency_occupied = consistency_free = overall_consistency = 0
                
                # 冲突分析 (仅限实际场地区域)
                conflicts = ((field_grid_i == 100) & (field_grid_j == 0)) | ((field_grid_i == 0) & (field_grid_j == 100))
                conflict_ratio = np.sum(conflicts) / total_overlap if total_overlap > 0 else 0
                
                consistency_metrics[f'layer_{layer_i}_vs_{layer_j}'] = {
                    'total_overlap': total_overlap,
                    'consistency_occupied': consistency_occupied,
                    'consistency_free': consistency_free,
                    'overall_consistency': overall_consistency,
                    'conflict_ratio': conflict_ratio,
                    'conflict_cells': np.sum(conflicts)
                }
        
        self.metrics['consistency'] = consistency_metrics
        return consistency_metrics
    
    def calculate_spatial_distribution(self):
        """计算空间分布指标"""
        print("计算空间分布指标...")
        
        spatial_metrics = {}
        
        for layer_data in self.layers_data:
            layer_id = layer_data['layer_id']
            grid = layer_data['grid']
            resolution = layer_data['metadata'].get('resolution', 0.05)
            
            # 获取实际场地区域掩码
            field_mask = self.get_actual_field_mask(grid, resolution)
            
            # 计算占用区域的连通性 (仅限实际场地区域)
            occupied_mask = (grid == 100) & field_mask
            if np.any(occupied_mask):
                # 连通组件分析
                labeled_array, num_features = ndimage.label(occupied_mask)
                
                # 连通组件统计
                component_sizes = []
                for i in range(1, num_features + 1):
                    component_size = np.sum(labeled_array == i)
                    component_sizes.append(component_size)
                
                if component_sizes:
                    largest_component = max(component_sizes)
                    avg_component_size = np.mean(component_sizes)
                    num_components = num_features
                else:
                    largest_component = avg_component_size = num_components = 0
            else:
                largest_component = avg_component_size = num_components = 0
            
            # 计算占用区域的分布特征 (仅限实际场地区域)
            occupied_coords = np.where(occupied_mask)
            if len(occupied_coords[0]) > 0:
                # 重心计算
                center_y = np.mean(occupied_coords[0])
                center_x = np.mean(occupied_coords[1])
                
                # 分布范围
                y_range = np.max(occupied_coords[0]) - np.min(occupied_coords[0])
                x_range = np.max(occupied_coords[1]) - np.min(occupied_coords[1])
                
                # 分布密度 (基于实际场地区域)
                total_field_area = np.sum(field_mask)
                occupied_area = len(occupied_coords[0])
                density = occupied_area / total_field_area if total_field_area > 0 else 0
            else:
                center_y = center_x = y_range = x_range = density = 0
            
            spatial_metrics[layer_id] = {
                'num_components': num_components,
                'largest_component_size': largest_component,
                'avg_component_size': avg_component_size,
                'center_y': center_y,
                'center_x': center_x,
                'y_range': y_range,
                'x_range': x_range,
                'density': density
            }
        
        self.metrics['spatial'] = spatial_metrics
        return spatial_metrics
    
    def calculate_temporal_metrics(self):
        """计算时间序列指标"""
        print("计算时间序列指标...")
        
        if len(self.layers_data) < 2:
            print("需要至少2层数据来计算时间指标")
            return {}
        
        # 按时间戳排序
        sorted_layers = sorted(self.layers_data, key=lambda x: x['timestamp'])
        
        temporal_metrics = {}
        
        # 计算时间间隔
        timestamps = []
        for layer_data in sorted_layers:
            try:
                timestamp = datetime.fromisoformat(layer_data['timestamp'].replace('Z', '+00:00'))
                timestamps.append(timestamp)
            except:
                timestamps.append(None)
        
        # 计算时间间隔
        time_intervals = []
        for i in range(1, len(timestamps)):
            if timestamps[i] and timestamps[i-1]:
                interval = (timestamps[i] - timestamps[i-1]).total_seconds()
                time_intervals.append(interval)
        
        # 计算地图演化指标
        evolution_metrics = {}
        for i in range(1, len(sorted_layers)):
            prev_grid = sorted_layers[i-1]['grid']
            curr_grid = sorted_layers[i]['grid']
            
            # 计算变化
            changes = (prev_grid != curr_grid) & (prev_grid != -1) & (curr_grid != -1)
            change_ratio = np.sum(changes) / np.sum((prev_grid != -1) | (curr_grid != -1))
            
            # 新增探索区域
            new_explored = (prev_grid == -1) & (curr_grid != -1)
            new_explored_ratio = np.sum(new_explored) / np.sum(prev_grid == -1) if np.sum(prev_grid == -1) > 0 else 0
            
            evolution_metrics[f'layer_{i-1}_to_{i}'] = {
                'change_ratio': change_ratio,
                'new_explored_ratio': new_explored_ratio,
                'change_cells': np.sum(changes),
                'new_explored_cells': np.sum(new_explored)
            }
        
        temporal_metrics = {
            'time_intervals': time_intervals,
            'evolution': evolution_metrics,
            'total_time': sum(time_intervals) if time_intervals else 0
        }
        
        self.metrics['temporal'] = temporal_metrics
        return temporal_metrics
    
    def generate_comprehensive_report(self):
        """生成综合报告"""
        print("生成综合分析报告...")
        
        # 计算所有指标
        self.calculate_coverage_metrics()
        self.calculate_inter_layer_consistency()
        self.calculate_spatial_distribution()
        self.calculate_temporal_metrics()
        
        # 创建报告
        report = {
            'experiment_info': {
                'map_name': self.map_info.get('map_name', ''),
                'timestamp': self.map_info.get('timestamp', ''),
                'total_layers': len(self.layers_data),
                'layer_ids': [layer['layer_id'] for layer in self.layers_data]
            },
            'metrics': self.metrics
        }
        
        return report
    
    @staticmethod
    def cleanup_legacy_analysis_figures(output_dir: Path):
        """Remove deprecated analysis PNGs superseded by paper_metrics figures."""
        for filename in LEGACY_ANALYSIS_FIGURES:
            figure_path = output_dir / filename
            if figure_path.exists():
                figure_path.unlink()
                print(f"已删除旧分析图: {figure_path}")
    
    
    def save_metrics_to_csv(self, output_dir: Path):
        """保存指标到CSV文件"""
        print("保存指标到CSV文件...")
        
        # 覆盖率指标
        coverage_df = pd.DataFrame(self.metrics['coverage']).T
        coverage_df.to_csv(output_dir / 'coverage_metrics.csv')
        
        # 一致性指标
        if self.metrics['consistency']:
            consistency_df = pd.DataFrame(self.metrics['consistency']).T
            consistency_df.to_csv(output_dir / 'consistency_metrics.csv')
        
        # 空间分布指标
        spatial_df = pd.DataFrame(self.metrics['spatial']).T
        spatial_df.to_csv(output_dir / 'spatial_metrics.csv')
        
        # 时间指标
        if self.metrics['temporal']:
            temporal_df = pd.DataFrame(self.metrics['temporal']['evolution']).T
            temporal_df.to_csv(output_dir / 'temporal_metrics.csv')
    
def main():
    """主函数"""
    # 设置实验目录
    script_dir = Path(__file__).parent
    experiment_dir = script_dir / 'experiments' / 'used_multi_layer_map_20251023_093826'
    output_dir = experiment_dir / 'analysis'
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"实验目录: {experiment_dir}")
    print(f"分析输出目录: {output_dir}")
    
    # 创建分析器
    analyzer = MultiLayerMapAnalyzer(experiment_dir)
    
    # 加载数据
    if not analyzer.load_experiment_data():
        print("数据加载失败!")
        return
    
    # 生成综合报告
    report = analyzer.generate_comprehensive_report()
    
    # 生成可视化图表
    analyzer.cleanup_legacy_analysis_figures(output_dir)
    
    # 保存指标到CSV
    analyzer.save_metrics_to_csv(output_dir)
    
    
    # 保存完整报告
    import json
    
    # 转换numpy类型为Python原生类型
    def convert_numpy_types(obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {key: convert_numpy_types(value) for key, value in obj.items()}
        elif isinstance(obj, list):
            return [convert_numpy_types(item) for item in obj]
        else:
            return obj
    
    report_serializable = convert_numpy_types(report)
    with open(output_dir / 'complete_analysis_report.json', 'w', encoding='utf-8') as f:
        json.dump(report_serializable, f, indent=2, ensure_ascii=False)
    
    print(f"\n分析完成! 结果保存在: {output_dir}")
    print("生成的文件:")
    for file in output_dir.glob('*'):
        print(f"  - {file.name}")

if __name__ == '__main__':
    main()
