#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate high-quality figures for academic papers from multi-layer maps.

This script reads exported NPY files and YAML metadata to create publication-ready figures:
- Individual layer maps with proper styling
- Overlay comparison of all layers
- Grid layout showing all layers
- Statistical analysis plots
"""

import os
import sys
import numpy as np
import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from pathlib import Path
import argparse
from typing import Dict, List, Tuple, Optional

# Set matplotlib for high-quality output
plt.rcParams['figure.dpi'] = 300
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 12
plt.rcParams['axes.linewidth'] = 1.5
plt.rcParams['xtick.major.width'] = 1.5
plt.rcParams['ytick.major.width'] = 1.5

def load_layer_data(export_dir: Path, layer_id: int) -> Tuple[np.ndarray, Dict]:
    """Load layer data from NPY and YAML files."""
    npy_path = export_dir / f"layer_{layer_id}.npy"
    yaml_path = export_dir / f"layer_{layer_id}.yaml"
    
    if not npy_path.exists():
        raise FileNotFoundError(f"Layer {layer_id} NPY file not found: {npy_path}")
    
    # Load occupancy grid
    grid = np.load(npy_path)
    
    # Load metadata
    with open(yaml_path, 'r') as f:
        metadata = yaml.safe_load(f)
    
    return grid, metadata

def create_occupancy_colormap():
    """Create a custom colormap for occupancy grids."""
    from matplotlib.colors import ListedColormap
    
    # Define colors: unknown=gray, free=white, occupied=black
    colors = ['#808080', '#FFFFFF', '#000000']  # gray, white, black
    n_bins = 3
    cmap = ListedColormap(colors[:n_bins])
    return cmap

def plot_single_layer(grid: np.ndarray, metadata: Dict, layer_id: int, 
                     output_path: Path, title: str = None):
    """Plot a single layer with high quality like RViz."""
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    
    # Create RGB image like RViz
    h, w = grid.shape
    rgb_image = np.zeros((h, w, 3), dtype=np.float32)
    
    # Map occupancy values to colors (like RViz)
    rgb_image[grid == -1] = [0.5, 0.5, 0.5]  # unknown = gray
    rgb_image[grid == 0] = [1.0, 1.0, 1.0]   # free = white  
    rgb_image[grid == 100] = [0.0, 0.0, 0.0] # occupied = black
    
    # Plot the RGB image
    ax.imshow(rgb_image, origin='lower', extent=[0, w, 0, h])
    
    # Add grid lines for better visualization
    ax.grid(True, alpha=0.3, linewidth=0.5)
    ax.set_xticks(np.arange(0, grid.shape[1], max(1, grid.shape[1]//10)))
    ax.set_yticks(np.arange(0, grid.shape[0], max(1, grid.shape[0]//10)))
    
    # Set labels - 使用实际场地尺寸
    resolution = metadata.get('resolution', 0.05)
    actual_width_m = 2.15  # 实际场地宽度
    actual_height_m = 2.0  # 实际场地高度
    
    ax.set_xlabel(f'X (m) - Actual Field: {actual_width_m}m x {actual_height_m}m')
    ax.set_ylabel(f'Y (m) - Resolution: {resolution:.3f}m/pixel')
    
    # Set title
    if title is None:
        title = f'Layer {layer_id} - Multistorey Map'
    ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
    
    # Add scale bar - 使用实际场地尺寸的比例
    scale_length = 0.5  # 0.5 meters (适合2.15m x 2m场地)
    scale_pixels = scale_length / resolution
    scale_bar = patches.Rectangle((10, 10), scale_pixels, 5, 
                                linewidth=2, edgecolor='red', facecolor='red')
    ax.add_patch(scale_bar)
    ax.text(10 + scale_pixels/2, 20, f'{scale_length}m', 
            ha='center', va='bottom', fontsize=10, color='red', fontweight='bold')
    
    # Add statistics text
    occupied_cells = np.sum(grid == 100)
    free_cells = np.sum(grid == 0)
    unknown_cells = np.sum(grid == -1)
    total_cells = grid.size
    
    stats_text = f'Occupied: {occupied_cells} ({occupied_cells/total_cells*100:.1f}%)\n'
    stats_text += f'Free: {free_cells} ({free_cells/total_cells*100:.1f}%)\n'
    stats_text += f'Unknown: {unknown_cells} ({unknown_cells/total_cells*100:.1f}%)'
    
    # ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
    #         verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
    #         fontsize=10)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()

def plot_layers_overlay(layers_data: List[Tuple[np.ndarray, Dict, int]], 
                       output_path: Path):
    """Plot all layers overlaid with different colors like RViz."""
    fig, ax = plt.subplots(1, 1, figsize=(12, 12))
    
    # Define colors for each layer (with alpha for transparency)
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
    
    # Create a composite map showing all layers
    if layers_data:
        # Use the first layer as base
        base_grid = layers_data[0][0].copy()
        h, w = base_grid.shape
        
        # Create RGB image for overlay
        overlay_image = np.zeros((h, w, 3), dtype=np.float32)
        
        # Process each layer
        for i, (grid, metadata, layer_id) in enumerate(layers_data):
            if grid.shape != base_grid.shape:
                print(f"Warning: Layer {layer_id} has different shape {grid.shape} vs base {base_grid.shape}")
                continue
                
            color = colors[i % len(colors)]
            # Convert color name to RGB
            color_rgb = plt.cm.colors.to_rgb(color)
            
            # Create layer mask (occupied cells)
            occupied_mask = (grid == 100)
            
            # Add this layer's occupied cells to overlay
            for c in range(3):
                overlay_image[:, :, c] += occupied_mask.astype(np.float32) * color_rgb[c] * 0.6
        
        # Normalize overlay image
        overlay_image = np.clip(overlay_image, 0, 1)
        
        # Plot the base map first (unknown=gray, free=white, occupied=black)
        base_display = np.zeros((h, w, 3), dtype=np.float32)
        base_display[base_grid == -1] = [0.5, 0.5, 0.5]  # unknown = gray
        base_display[base_grid == 0] = [1.0, 1.0, 1.0]   # free = white
        base_display[base_grid == 100] = [0.0, 0.0, 0.0] # occupied = black
        
        # Show base map
        ax.imshow(base_display, origin='lower', extent=[0, w, 0, h])
        
        # Show overlay with transparency
        ax.imshow(overlay_image, origin='lower', extent=[0, w, 0, h], alpha=0.7)
        
        # Add legend
        legend_elements = []
        for i, (_, _, layer_id) in enumerate(layers_data):
            color = colors[i % len(colors)]
            legend_elements.append(plt.Rectangle((0,0),1,1, facecolor=color, alpha=0.7, 
                                               label=f'Layer {layer_id}'))
        ax.legend(handles=legend_elements, bbox_to_anchor=(1.05, 1), loc='upper left')
    
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_title('Multistorey Map Overlay - All Layers', 
                fontsize=16, fontweight='bold', pad=20)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()

def plot_layers_overlay_regions(layers_data: List[Tuple[np.ndarray, Dict, int]], 
                                output_path: Path, use_explored: bool = True):
    """Overlay all layers by coloring their regions (RViz-like continuous areas).

    - If use_explored=True: a layer's region = (grid != -1), i.e., explored (free or occupied)
    - If False: region = (grid == 100), i.e., occupied only
    """
    fig, ax = plt.subplots(1, 1, figsize=(12, 12))
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']

    if not layers_data:
        plt.close()
        return

    base_grid = layers_data[0][0]
    h, w = base_grid.shape

    # Base background
    background = np.zeros((h, w, 3), dtype=np.float32)
    background[base_grid == -1] = [0.5, 0.5, 0.5]
    background[base_grid == 0] = [1.0, 1.0, 1.0]
    background[base_grid == 100] = [0.0, 0.0, 0.0]
    ax.imshow(background, origin='lower', extent=[0, w, 0, h])

    # Region overlays per layer
    for i, (grid, metadata, layer_id) in enumerate(layers_data):
        if grid.shape != base_grid.shape:
            print(f"[warn] Layer {layer_id} shape mismatch: {grid.shape} vs {base_grid.shape}; skipping in overlay_regions.")
            continue
        mask = (grid != -1) if use_explored else (grid == 100)
        if not np.any(mask):
            continue
        color_rgb = plt.cm.colors.to_rgb(colors[i % len(colors)])
        overlay = np.zeros((h, w, 4), dtype=np.float32)
        overlay[:, :, :3] = color_rgb
        overlay[:, :, 3] = mask.astype(np.float32) * (0.35 if use_explored else 0.6)
        ax.imshow(overlay, origin='lower', extent=[0, w, 0, h])

        # Outline for clarity
        try:
            from skimage import measure
            contours = measure.find_contours(mask.astype(float), 0.5)
            for contour in contours:
                ax.plot(contour[:, 1], contour[:, 0], color=colors[i % len(colors)], linewidth=1.0, alpha=0.9)
        except Exception:
            pass

    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_title('Multistorey Overlay (Region-colored)', fontsize=16, fontweight='bold', pad=20)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()

def plot_layers_3d(layers_data: List[Tuple[np.ndarray, Dict, int]], 
                   output_path: Path, layer_gap_m: float = 0.3):
    """Render a 3D view where each 2D layer is a colored sheet at specific heights.

    - First layer (index 0): z = 0.3m
    - Second layer (index 1): z = 0.6m  
    - Additional layers: z = 0.3 + index * layer_gap_m
    - X/Y axes are in meters using the first layer's resolution/origin.
    - Occupied regions (grid == 100): alpha = 0.8 (more opaque)
    - Free regions (grid == 0): alpha = 0.3 (more transparent)
    - Unknown regions (grid == -1): not displayed
    """
    if not layers_data:
        return

    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    # Use first layer meta for common XY scaling
    base_grid, base_meta, _ = layers_data[0]
    h, w = base_grid.shape
    res = float(base_meta.get('resolution', 0.1))
    origin = base_meta.get('origin', [0.0, 0.0, 0.0])
    if isinstance(origin, dict):
        origin_x = float(origin.get('x', 0.0))
        origin_y = float(origin.get('y', 0.0))
    else:
        origin_x = float(origin[0]) if len(origin) > 0 else 0.0
        origin_y = float(origin[1]) if len(origin) > 1 else 0.0

    x = origin_x + np.arange(w) * res
    y = origin_y + np.arange(h) * res
    X, Y = np.meshgrid(x, y)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']

    for i, (grid, metadata, layer_id) in enumerate(layers_data):
        if grid.shape != (h, w):
            print(f"[warn] Layer {layer_id} shape mismatch in 3D: {grid.shape} vs {(h, w)}; skipping.")
            continue
        # 设置固定高度：第一层0.3m，第二层0.6m
        if i == 0:
            z_height = 0.3
        elif i == 1:
            z_height = 0.6
        else:
            z_height = 0.15 + i * layer_gap_m  # 对于更多层，使用原来的逻辑
        z = np.full_like(X, fill_value=z_height, dtype=float)
        explored = (grid != -1)
        occupied = (grid == 100)
        free = (grid == 0)
        
        if not np.any(explored):
            continue
            
        color_rgb = np.array(plt.cm.colors.to_rgb(colors[i % len(colors)]))
        
        # 设置不同的透明度：occupied区域更明显，free区域稍微透明
        alpha = np.zeros_like(grid, dtype=float)
        alpha[occupied] = 0.8  # occupied区域更不透明
        alpha[free] = 0.3      # free区域更透明
        alpha[grid == -1] = 0.0  # unknown区域不显示
        
        facecolors = np.zeros((h, w, 4), dtype=float)
        facecolors[..., :3] = color_rgb
        facecolors[..., 3] = alpha
        ax.plot_surface(X, Y, z, rstride=1, cstride=1, facecolors=facecolors, linewidth=0, antialiased=False, shade=False)

    # 坐标轴标题（X/Y/Z 标签）字号
    ax.set_xlabel('X (m)', fontsize=30, labelpad=12)
    ax.set_ylabel('Y (m)', fontsize=30, labelpad=12)
    ax.set_zlabel('Z (m)', fontsize=30, labelpad=12)

    # 坐标轴刻度数字（tick）字号 - 放大以便在论文中清晰可见
    ax.tick_params(axis='x', which='both', labelsize=28)
    ax.tick_params(axis='y', which='both', labelsize=28)
    ax.tick_params(axis='z', which='both', labelsize=28)
    
    # 设置Z轴范围从0到0.8
    ax.set_zlim(0, 0.8)
    ax.set_zticks([0.0, 0.2, 0.4, 0.6, 0.8])
    
    # ax.set_title('3D Layered Map', fontsize=14, fontweight='bold')
    ax.view_init(elev=35, azim=-60)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()

def plot_layers_grid(layers_data: List[Tuple[np.ndarray, Dict, int]], 
                    output_path: Path):
    """Plot all layers in a grid layout."""
    n_layers = len(layers_data)
    cols = min(3, n_layers)
    rows = (n_layers + cols - 1) // cols
    
    fig, axes = plt.subplots(rows, cols, figsize=(5*cols, 5*rows))
    
    # Handle different cases for axes
    if n_layers == 1:
        axes = [axes]
    elif rows == 1 and cols > 1:
        axes = axes.reshape(1, -1)
    elif rows > 1 and cols == 1:
        axes = axes.reshape(-1, 1)
    
    # 实际场地尺寸 (米)
    actual_field_width = 2.15  # 实际场地宽度
    actual_field_height = 2.0   # 实际场地高度
    
    def get_actual_field_mask(grid: np.ndarray, resolution: float) -> np.ndarray:
        """获取实际场地区域的掩码"""
        h, w = grid.shape
        
        # 计算实际场地区域在像素坐标中的范围
        field_width_pixels = int(actual_field_width / resolution)
        field_height_pixels = int(actual_field_height / resolution)
        
        # 计算场地区域在网格中的起始和结束位置
        start_x = max(0, (w - field_width_pixels) // 2)
        end_x = min(w, start_x + field_width_pixels)
        start_y = max(0, (h - field_height_pixels) // 2)
        end_y = min(h, start_y + field_height_pixels)
        
        # 创建掩码
        mask = np.zeros((h, w), dtype=bool)
        mask[start_y:end_y, start_x:end_x] = True
        
        return mask
    
    cmap = create_occupancy_colormap()
    
    for i, (grid, metadata, layer_id) in enumerate(layers_data):
        row = i // cols
        col = i % cols
        
        # Get the correct axis
        if n_layers == 1:
            ax = axes[0]
        elif rows == 1:
            ax = axes[0, col] if cols > 1 else axes[col]
        else:
            ax = axes[row, col] if cols > 1 else axes[row, 0]
        
        # Create RGB image like RViz
        h, w = grid.shape
        rgb_image = np.zeros((h, w, 3), dtype=np.float32)
        rgb_image[grid == -1] = [0.5, 0.5, 0.5]  # unknown = gray
        rgb_image[grid == 0] = [1.0, 1.0, 1.0]   # free = white  
        rgb_image[grid == 100] = [0.0, 0.0, 0.0] # occupied = black
        ax.imshow(rgb_image, origin='lower', extent=[0, w, 0, h])
        
        # 添加比例尺
        resolution = metadata.get('resolution', 0.05)
        scale_length = 0.5  # 0.5 meters (适合2.15m x 2m场地)
        scale_pixels = scale_length / resolution
        scale_bar = patches.Rectangle((10, 10), scale_pixels, 5, 
                                    linewidth=2, edgecolor='red', facecolor='red')
        ax.add_patch(scale_bar)
        ax.text(10 + scale_pixels/2, 20, f'{scale_length}m', 
                ha='center', va='bottom', fontsize=10, color='red', fontweight='bold')
        
        # 计算覆盖率信息 (基于实际场地区域)
        field_mask = get_actual_field_mask(grid, resolution)
        field_grid = grid[field_mask]
        
        total_field_cells = np.sum(field_mask)
        occupied_cells = np.sum(field_grid == 100)
        free_cells = np.sum(field_grid == 0)
        explored_cells = occupied_cells + free_cells
        coverage_ratio = explored_cells / total_field_cells if total_field_cells > 0 else 0
        
        # 坐标轴标题和刻度样式（适合论文中阅读）
        ax.set_xlabel('X (pixels)', fontsize=20, labelpad=6)
        ax.set_ylabel('Y (pixels)', fontsize=20, labelpad=6)
        ax.tick_params(axis='both', which='both', labelsize=18)
        ax.grid(True, alpha=0.3)
    
    # Hide empty subplots
    for i in range(n_layers, rows * cols):
        row = i // cols
        col = i % cols
        if n_layers == 1:
            pass  # No empty subplots
        elif rows == 1:
            if cols > 1:
                axes[0, col].set_visible(False)
            else:
                axes[col].set_visible(False)
        else:
            if cols > 1:
                axes[row, col].set_visible(False)
            else:
                axes[row, 0].set_visible(False)
    
    # plt.suptitle('Multistorey Maps - Grid View', fontsize=18, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()

def plot_statistics(layers_data: List[Tuple[np.ndarray, Dict, int]], 
                   output_path: Path):
    """Plot statistical analysis of all layers."""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    # 实际场地尺寸 (米)
    actual_field_width = 2.15  # 实际场地宽度
    actual_field_height = 2.0   # 实际场地高度
    
    def get_actual_field_mask(grid: np.ndarray, resolution: float) -> np.ndarray:
        """获取实际场地区域的掩码"""
        h, w = grid.shape
        
        # 计算实际场地区域在像素坐标中的范围
        field_width_pixels = int(actual_field_width / resolution)
        field_height_pixels = int(actual_field_height / resolution)
        
        # 计算场地区域在网格中的起始和结束位置
        start_x = max(0, (w - field_width_pixels) // 2)
        end_x = min(w, start_x + field_width_pixels)
        start_y = max(0, (h - field_height_pixels) // 2)
        end_y = min(h, start_y + field_height_pixels)
        
        # 创建掩码
        mask = np.zeros((h, w), dtype=bool)
        mask[start_y:end_y, start_x:end_x] = True
        
        return mask
    
    layers = [layer_id for _, _, layer_id in layers_data]
    occupied_counts = []
    free_counts = []
    unknown_counts = []
    resolutions = []
    
    for grid, metadata, layer_id in layers_data:
        resolution = metadata.get('resolution', 0.05)
        
        # 获取实际场地区域掩码
        field_mask = get_actual_field_mask(grid, resolution)
        
        # 只计算实际场地区域的统计
        field_grid = grid[field_mask]
        
        occupied_counts.append(np.sum(field_grid == 100))
        free_counts.append(np.sum(field_grid == 0))
        unknown_counts.append(np.sum(field_grid == -1))
        resolutions.append(resolution)
    
    # Occupancy statistics
    ax1.bar(layers, occupied_counts, color='red', alpha=0.7, label='Occupied')
    ax1.bar(layers, free_counts, bottom=occupied_counts, color='green', alpha=0.7, label='Free')
    ax1.bar(layers, unknown_counts, bottom=np.array(occupied_counts) + np.array(free_counts), 
           color='gray', alpha=0.7, label='Unknown')
    ax1.set_xlabel('Layer ID')
    ax1.set_ylabel('Number of Cells')
    ax1.set_title('Cell Distribution by Layer')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Resolution comparison
    ax2.bar(layers, resolutions, color='blue', alpha=0.7)
    ax2.set_xlabel('Layer ID')
    ax2.set_ylabel('Resolution (m/pixel)')
    ax2.set_title('Map Resolution by Layer')
    ax2.grid(True, alpha=0.3)
    
    # Coverage percentage
    total_cells = [occupied + free + unknown for occupied, free, unknown in 
                  zip(occupied_counts, free_counts, unknown_counts)]
    coverage_pct = [(occupied + free) / total * 100 for occupied, free, total in 
                   zip(occupied_counts, free_counts, total_cells)]
    
    ax3.bar(layers, coverage_pct, color='orange', alpha=0.7)
    ax3.set_xlabel('Layer ID')
    ax3.set_ylabel('Coverage (%)')
    ax3.set_title('Map Coverage by Layer')
    ax3.set_ylim(0, 100)
    ax3.grid(True, alpha=0.3)
    
    # Actual field area comparison (基于实际场地区域)
    actual_field_areas = []
    for grid, metadata, layer_id in layers_data:
        resolution = metadata.get('resolution', 0.05)
        field_mask = get_actual_field_mask(grid, resolution)
        actual_field_areas.append(np.sum(field_mask))
    
    ax4.bar(layers, actual_field_areas, color='purple', alpha=0.7)
    ax4.set_xlabel('Layer ID')
    ax4.set_ylabel('Actual Field Cells')
    ax4.set_title(f'Actual Field Size by Layer\n({actual_field_width}m × {actual_field_height}m)')
    ax4.grid(True, alpha=0.3)
    
    plt.suptitle('Multistorey Map Statistical Analysis', fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()

def main():
    # 硬编码路径 - 直接使用当前目录下的export文件夹
    script_dir = Path(__file__).parent
    input_dir = script_dir / 'experiments' / 'used_multi_layer_map_20251023_091856' / 'export'
    output_dir = input_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Input directory: {input_dir}")
    print(f"Output directory: {output_dir}")
    
    # Find available layers
    npy_files = list(input_dir.glob('layer_*.npy'))
    available_layers = sorted([int(f.stem.split('_')[1]) for f in npy_files])
    
    if not available_layers:
        print("No layer files found!")
        return
    
    print(f"Processing layers: {available_layers}")
    
    # Load all layer data
    layers_data = []
    for layer_id in available_layers:
        try:
            grid, metadata = load_layer_data(input_dir, layer_id)
            layers_data.append((grid, metadata, layer_id))
            print(f"Loaded layer {layer_id}: {grid.shape}")
        except Exception as e:
            print(f"Error loading layer {layer_id}: {e}")
            continue
    
    if not layers_data:
        print("No valid layers loaded!")
        return
    
    # Generate individual layer plots
    print("Generating individual layer plots...")
    for grid, metadata, layer_id in layers_data:
        output_path = output_dir / f'layer_{layer_id}_single.png'
        plot_single_layer(grid, metadata, layer_id, output_path)
        print(f"Saved: {output_path}")
    
    # Generate overlay plot
    if len(layers_data) > 1:
        print("Generating overlay plot...")
        overlay_path = output_dir / 'all_layers_overlay.png'
        plot_layers_overlay(layers_data, overlay_path)
        print(f"Saved: {overlay_path}")
        # Region-colored overlay (explored regions)
        print("Generating overlay (region-colored, explored)...")
        overlay_regions_path = output_dir / 'all_layers_overlay_regions.png'
        plot_layers_overlay_regions(layers_data, overlay_regions_path, use_explored=True)
        print(f"Saved: {overlay_regions_path}")
        # Region-colored overlay (occupied only)
        print("Generating overlay (region-colored, occupied only)...")
        overlay_occ_path = output_dir / 'all_layers_overlay_occupied.png'
        plot_layers_overlay_regions(layers_data, overlay_occ_path, use_explored=False)
        print(f"Saved: {overlay_occ_path}")
    
    # Generate grid plot
    print("Generating grid plot...")
    grid_path = output_dir / 'all_layers_grid.png'
    plot_layers_grid(layers_data, grid_path)
    print(f"Saved: {grid_path}")
    
    # Generate statistics plot
    print("Generating statistics plot...")
    stats_path = output_dir / 'statistics.png'
    plot_statistics(layers_data, stats_path)
    print(f"Saved: {stats_path}")

    # Generate 3D layered view
    print("Generating 3D layered view...")
    fig3d_path = output_dir / 'layers_3d.png'
    plot_layers_3d(layers_data, fig3d_path, layer_gap_m=0.3)
    print(f"Saved: {fig3d_path}")
    
    print(f"\nAll figures saved to: {output_dir}")
    print("Generated files:")
    for file in output_dir.glob('*.png'):
        print(f"  - {file.name}")

if __name__ == '__main__':
    main()
