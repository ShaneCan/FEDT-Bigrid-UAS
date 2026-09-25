#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
论文指标评价总结工具

基于实验数据生成论文中需要的具体指标和数据分析
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import json
from typing import Dict, List

# 设置matplotlib
plt.rcParams['figure.dpi'] = 300
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 12
plt.rcParams['font.family'] = 'serif'

class PaperMetricsSummary:
    """论文指标总结生成器"""
    
    def __init__(self, analysis_dir: Path):
        self.analysis_dir = analysis_dir
        self.load_data()
    
    def load_data(self):
        """加载分析数据"""
        # 加载CSV数据
        self.coverage_df = pd.read_csv(self.analysis_dir / 'coverage_metrics.csv', index_col=0)
        self.spatial_df = pd.read_csv(self.analysis_dir / 'spatial_metrics.csv', index_col=0)
        
        if (self.analysis_dir / 'consistency_metrics.csv').exists():
            self.consistency_df = pd.read_csv(self.analysis_dir / 'consistency_metrics.csv', index_col=0)
        else:
            self.consistency_df = None
        
        # 加载JSON报告
        with open(self.analysis_dir / 'complete_analysis_report.json', 'r', encoding='utf-8') as f:
            self.report = json.load(f)
    
    def generate_quantitative_results(self):
        """生成定量结果"""
        # 动态获取实际存在的层数
        available_layers = self.coverage_df.index.tolist()
        
        # 生成每层的覆盖指标
        coverage_metrics = {}
        for layer_idx in available_layers:
            coverage_metrics[f"layer_{layer_idx}"] = {
                "coverage_ratio": self.coverage_df.loc[layer_idx, 'coverage_ratio'],
                "occupied_cells": int(self.coverage_df.loc[layer_idx, 'occupied_cells']),
                "free_cells": int(self.coverage_df.loc[layer_idx, 'free_cells']),
                "explored_cells": int(self.coverage_df.loc[layer_idx, 'explored_cells']),
                "total_cells": int(self.coverage_df.loc[layer_idx, 'total_cells'])
            }
        
        results = {
            "coverage_metrics": coverage_metrics,
            "summary_statistics": {
                "average_coverage": self.coverage_df['coverage_ratio'].mean(),
                "coverage_std": self.coverage_df['coverage_ratio'].std(),
                "coverage_improvement": self.coverage_df['coverage_ratio'].iloc[-1] - self.coverage_df['coverage_ratio'].iloc[0],
                "total_explored": int(self.coverage_df['explored_cells'].sum()),
                "total_occupied": int(self.coverage_df['occupied_cells'].sum()),
                "total_free": int(self.coverage_df['free_cells'].sum()),
                "total_layers": len(available_layers)
            }
        }
        return results
    
    def generate_consistency_analysis(self):
        """生成一致性分析"""
        if self.consistency_df is None:
            return {"message": "Consistency analysis not available"}
        
        consistency_analysis = {
            "overall_consistency": {
                "mean": self.consistency_df['overall_consistency'].mean(),
                "std": self.consistency_df['overall_consistency'].std(),
                "min": self.consistency_df['overall_consistency'].min(),
                "max": self.consistency_df['overall_consistency'].max()
            },
            "conflict_analysis": {
                "total_conflicts": int(self.consistency_df['conflict_cells'].sum()),
                "average_conflict_ratio": self.consistency_df['conflict_ratio'].mean(),
                "max_conflict_ratio": self.consistency_df['conflict_ratio'].max()
            },
            "layer_pairs": {
                pair: {
                    "consistency": self.consistency_df.loc[pair, 'overall_consistency'],
                    "conflicts": int(self.consistency_df.loc[pair, 'conflict_cells']),
                    "conflict_ratio": self.consistency_df.loc[pair, 'conflict_ratio']
                }
                for pair in self.consistency_df.index
            }
        }
        return consistency_analysis
    
    def generate_spatial_analysis(self):
        """生成空间分析"""
        spatial_analysis = {
            "connectivity": {
                "average_components": self.spatial_df['num_components'].mean(),
                "components_std": self.spatial_df['num_components'].std(),
                "largest_component_size": int(self.spatial_df['largest_component_size'].max()),
                "connectivity_score": 1 / (1 + self.spatial_df['num_components'].mean())
            },
            "distribution": {
                "average_density": self.spatial_df['density'].mean(),
                "density_std": self.spatial_df['density'].std(),
                "spatial_coverage": self.spatial_df['x_range'].mean() * self.spatial_df['y_range'].mean()
            },
            "per_layer": {
                f"layer_{i}": {
                    "components": int(self.spatial_df.loc[i, 'num_components']),
                    "largest_component": int(self.spatial_df.loc[i, 'largest_component_size']),
                    "density": self.spatial_df.loc[i, 'density'],
                    "x_range": self.spatial_df.loc[i, 'x_range'],
                    "y_range": self.spatial_df.loc[i, 'y_range']
                }
                for i in self.spatial_df.index
            }
        }
        return spatial_analysis
    
    def generate_performance_comparison(self):
        """生成性能对比"""
        # 模拟单层基线数据（基于第一层）
        single_layer_baseline = {
            "coverage_ratio": self.coverage_df.loc[0, 'coverage_ratio'],
            "explored_cells": int(self.coverage_df.loc[0, 'explored_cells']),
            "occupied_cells": int(self.coverage_df.loc[0, 'occupied_cells'])
        }
        
        # 多层方法数据
        multi_layer_method = {
            "total_coverage": self.coverage_df['coverage_ratio'].sum(),
            "total_explored": int(self.coverage_df['explored_cells'].sum()),
            "total_occupied": int(self.coverage_df['occupied_cells'].sum()),
            "average_coverage": self.coverage_df['coverage_ratio'].mean()
        }
        
        comparison = {
            "baseline_single_layer": single_layer_baseline,
            "proposed_multi_layer": multi_layer_method,
            "improvements": {
                "coverage_improvement": multi_layer_method['total_coverage'] - single_layer_baseline['coverage_ratio'],
                "explored_cells_improvement": multi_layer_method['total_explored'] - single_layer_baseline['explored_cells'],
                "occupied_cells_improvement": multi_layer_method['total_occupied'] - single_layer_baseline['occupied_cells'],
                "improvement_ratio": (multi_layer_method['total_coverage'] - single_layer_baseline['coverage_ratio']) / single_layer_baseline['coverage_ratio'] * 100
            }
        }
        return comparison
    
    def create_publication_figures(self, output_dir: Path):
        """创建论文级别的图表。

        Only keep high-value publication figures. The main figure is a compact
        Nature-style performance panel combining effectiveness and reliability.
        """
        output_dir.mkdir(parents=True, exist_ok=True)
        legacy_figures = [
            output_dir / 'coverage_comparison_publication.png',
            output_dir / 'spatial_distribution_publication.png',
            output_dir / 'comprehensive_radar_publication.png',
            output_dir / 'interlayer_consistency_composition_publication.png',
            output_dir / 'performance_improvement_publication.pdf',
        ]
        for figure_path in legacy_figures:
            if figure_path.exists():
                figure_path.unlink()

        # Main paper-ready performance figure
        self._create_performance_improvement_figure(output_dir)
    
    def _create_performance_improvement_figure(self, output_dir: Path):
        """创建性能提升图（Nature-style effectiveness + reliability panel）"""
        single_layer = float(self.coverage_df.loc[0, 'coverage_ratio'])
        multi_layer = float(self.coverage_df['coverage_ratio'].sum())
        improvement = ((multi_layer - single_layer) / single_layer) * 100.0

        single_explored = int(self.coverage_df.loc[0, 'explored_cells'])
        multi_explored = int(self.coverage_df['explored_cells'].sum())
        single_occupied = int(self.coverage_df.loc[0, 'occupied_cells'])
        multi_occupied = int(self.coverage_df['occupied_cells'].sum())

        explored_ratio = (multi_explored / single_explored) if single_explored > 0 else 1.0
        occupied_ratio = (multi_occupied / single_occupied) if single_occupied > 0 else 1.0

        consistency = None
        conflict_ratio = None
        conflict_cells = None
        if self.consistency_df is not None and not self.consistency_df.empty:
            row = self.consistency_df.iloc[0]
            consistency = float(row['overall_consistency']) * 100.0
            conflict_ratio = float(row['conflict_ratio']) * 100.0
            conflict_cells = int(row['conflict_cells'])

        # Restrained NMI/Nature-style palette
        palette = {
            'baseline': '#A8A8A8',
            'baseline_dark': '#606060',
            'method': '#0F4D92',
            'method_soft': '#DCE8F5',
            'gain': '#2E9E44',
            'gain_soft': '#EAF4EC',
            'conflict': '#B64342',
            'conflict_soft': '#F6CFCB',
            'grid': '#E8E8E8',
            'frame': '#272727'
        }

        nature_rc = {
            'font.family': 'sans-serif',
            'font.sans-serif': ['Arial', 'Helvetica', 'DejaVu Sans', 'sans-serif'],
            'svg.fonttype': 'none',
            'pdf.fonttype': 42,
            'font.size': 8,
            'axes.linewidth': 0.9,
            'xtick.major.width': 0.8,
            'ytick.major.width': 0.8,
            'xtick.major.size': 3.5,
            'ytick.major.size': 3.5,
        }

        def style_axis(ax, letter):
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.spines['left'].set_color(palette['frame'])
            ax.spines['bottom'].set_color(palette['frame'])
            ax.grid(False)
            ax.tick_params(axis='both', labelsize=9, pad=2)
            ax.text(
                -0.12, 1.06, letter, transform=ax.transAxes,
                ha='left', va='bottom', fontsize=12, fontweight='bold',
                color=palette['frame']
            )

        with plt.rc_context(nature_rc):
            fig = plt.figure(figsize=(7.2, 2.55))
            gs = fig.add_gridspec(1, 3, width_ratios=[1.15, 1.0, 0.95], wspace=0.58)
            ax_cov = fig.add_subplot(gs[0, 0])
            ax_gain = fig.add_subplot(gs[0, 1])
            ax_rel = fig.add_subplot(gs[0, 2])

            # Panel a: coverage gain as the hero evidence
            style_axis(ax_cov, 'a')
            x_base, x_method = 0.0, 1.0
            y_base, y_method = single_layer, multi_layer

            ax_cov.axhspan(y_base, y_method, color=palette['method_soft'], alpha=0.75, zorder=0)
            ax_cov.plot(
                [x_base, x_method], [y_base, y_method],
                color=palette['method'], linewidth=2.2, solid_capstyle='round', zorder=2
            )
            ax_cov.scatter(
                [x_base, x_method], [y_base, y_method],
                s=72, c=[palette['baseline_dark'], palette['method']],
                edgecolors='white', linewidths=1.2, zorder=4
            )

            y_span = max(y_method - y_base, 1e-6)
            ax_cov.set_xlim(-0.42, 1.42)
            ax_cov.set_ylim(y_base - 0.06 * y_span, y_method + 0.30 * y_span + 0.02)
            ax_cov.set_xticks([x_base, x_method])
            ax_cov.set_xticklabels(['Single\nlayer', 'Two\nlayers'])
            ax_cov.set_ylabel('Total coverage ratio', fontsize=10)
            ax_cov.grid(True, axis='y', color=palette['grid'], linewidth=0.7, alpha=0.9)

            ax_cov.annotate(
                f'{y_base:.3f}',
                xy=(x_base, y_base), xytext=(0, 18),
                textcoords='offset points', ha='center', va='bottom',
                fontsize=8.5, fontweight='semibold', color=palette['baseline_dark'], clip_on=False
            )
            ax_cov.annotate(
                f'{y_method:.3f}',
                xy=(x_method, y_method), xytext=(4, 10),
                textcoords='offset points', ha='center', va='bottom',
                fontsize=8.5, fontweight='semibold', color=palette['method'], clip_on=False
            )
            ax_cov.annotate(
                f'+{improvement:.1f}%',
                xy=(0.52, y_base + 0.55 * y_span),
                xytext=(0, 0), textcoords='offset points',
                ha='center', va='center', fontsize=8.5, fontweight='bold', color=palette['gain'],
                bbox=dict(boxstyle='round,pad=0.25', facecolor=palette['gain_soft'],
                          edgecolor=palette['gain'], linewidth=0.7),
                clip_on=False,
            )

            # Panel b: relative map-evidence gain
            style_axis(ax_gain, 'b')
            metrics = ['Explored', 'Occupied']
            values = [explored_ratio, occupied_ratio]
            y_pos = np.arange(len(metrics))[::-1]
            ax_gain.axvline(1.0, color=palette['baseline'], linewidth=1.0, linestyle='--', zorder=1)
            for y, value, label in zip(y_pos, values, [f'{multi_explored:,}', f'{multi_occupied:,}']):
                ax_gain.plot([1.0, value], [y, y], color=palette['method'], linewidth=1.8, zorder=2)
                ax_gain.scatter([1.0], [y], s=48, color=palette['baseline_dark'],
                                edgecolors='white', linewidths=1.0, zorder=3)
                ax_gain.scatter([value], [y], s=64, color=palette['method'],
                                edgecolors='white', linewidths=1.0, zorder=4)
                ax_gain.text(value + 0.04, y, f'{value:.2f}x\n{label}',
                             ha='left', va='center', fontsize=8.2, color=palette['method'],
                             fontweight='semibold', linespacing=1.0)

            ax_gain.set_xlim(0.82, max(values) + 0.42)
            ax_gain.set_ylim(-0.5, len(metrics) - 0.5)
            ax_gain.set_yticks(y_pos)
            ax_gain.set_yticklabels(metrics)
            ax_gain.set_xlabel('Relative gain', fontsize=10)
            ax_gain.grid(True, axis='x', color=palette['grid'], linewidth=0.7)

            # Panel c: reliability/risk from layer consistency
            style_axis(ax_rel, 'c')
            if consistency is None:
                ax_rel.text(0.5, 0.5, 'Consistency\nnot available',
                            transform=ax_rel.transAxes, ha='center', va='center', fontsize=9)
                ax_rel.set_axis_off()
            else:
                bars = ax_rel.barh(
                    [1, 0], [consistency, conflict_ratio],
                    color=[palette['method'], palette['conflict']],
                    height=0.42, alpha=0.96
                )
                ax_rel.set_xlim(0, 100)
                ax_rel.set_yticks([1, 0])
                ax_rel.set_yticklabels(['Consistency', 'Conflict'])
                ax_rel.set_xlabel('Overlap region (%)', fontsize=10)
                ax_rel.grid(True, axis='x', color=palette['grid'], linewidth=0.7)
                labels = [f'{consistency:.1f}%', f'{conflict_ratio:.1f}%\n{conflict_cells} cells']
                for bar, text, color in zip(bars, labels, [palette['method'], palette['conflict']]):
                    width = bar.get_width()
                    ax_rel.text(
                        min(width + 2.2, 95), bar.get_y() + bar.get_height() / 2,
                        text, ha='left', va='center', fontsize=8.2,
                        color=color, fontweight='semibold', linespacing=1.0
                    )

            fig.subplots_adjust(left=0.08, right=0.98, top=0.92, bottom=0.20, wspace=0.58)
            out_png = output_dir / 'performance_improvement_publication.png'
            fig.savefig(out_png, dpi=600, bbox_inches='tight', pad_inches=0.04)
            fig.savefig(out_png.with_suffix('.pdf'), bbox_inches='tight', pad_inches=0.04)
            fig.savefig(out_png.with_suffix('.svg'), bbox_inches='tight', pad_inches=0.04)
            plt.close(fig)

    @staticmethod
    def create_cross_experiment_validation_figure(experiment_dirs: List[Path], output_dir: Path):
        """Create a high-value cross-experiment robustness summary."""
        rows = []
        for exp_dir in experiment_dirs:
            analysis_dir = exp_dir / 'analysis'
            coverage = pd.read_csv(analysis_dir / 'coverage_metrics.csv', index_col=0)
            consistency_path = analysis_dir / 'consistency_metrics.csv'
            consistency = pd.read_csv(consistency_path, index_col=0) if consistency_path.exists() else None

            single = float(coverage.loc[0, 'coverage_ratio'])
            total = float(coverage['coverage_ratio'].sum())
            rows.append({
                'experiment': exp_dir.name.replace('used_multi_layer_map_', ''),
                'coverage_gain': (total - single) / single * 100.0,
                'explored_gain': int(coverage['explored_cells'].sum()) / int(coverage.loc[0, 'explored_cells']),
                'occupied_gain': int(coverage['occupied_cells'].sum()) / int(coverage.loc[0, 'occupied_cells']),
                'consistency': float(consistency.iloc[0]['overall_consistency']) * 100.0 if consistency is not None else np.nan,
                'conflict': float(consistency.iloc[0]['conflict_ratio']) * 100.0 if consistency is not None else np.nan,
            })

        df = pd.DataFrame(rows)
        output_dir.mkdir(parents=True, exist_ok=True)

        palette = {
            'coverage': '#0F4D92',
            'explored': '#3775BA',
            'occupied': '#7884B4',
            'consistency': '#2E9E44',
            'conflict': '#B64342',
            'grid': '#E8E8E8',
            'frame': '#272727'
        }
        rc = {
            'font.family': 'sans-serif',
            'font.sans-serif': ['Arial', 'Helvetica', 'DejaVu Sans', 'sans-serif'],
            'svg.fonttype': 'none',
            'pdf.fonttype': 42,
            'font.size': 8,
            'axes.linewidth': 0.9,
        }

        with plt.rc_context(rc):
            fig = plt.figure(figsize=(7.2, 3.0))
            gs = fig.add_gridspec(1, 2, width_ratios=[1.25, 1.0], wspace=0.45)
            ax_gain = fig.add_subplot(gs[0, 0])
            ax_rel = fig.add_subplot(gs[0, 1])

            x = np.arange(len(df))
            width = 0.22
            ax_gain.bar(x - width, df['coverage_gain'], width, color=palette['coverage'], label='Coverage gain')
            ax_gain.bar(x, (df['explored_gain'] - 1.0) * 100.0, width, color=palette['explored'], label='Explored gain')
            ax_gain.bar(x + width, (df['occupied_gain'] - 1.0) * 100.0, width, color=palette['occupied'], label='Occupied gain')
            ax_gain.set_ylabel('Improvement over single layer (%)')
            ax_gain.set_xticks(x)
            ax_gain.set_xticklabels(['Run 1', 'Run 2'])
            ax_gain.grid(True, axis='y', color=palette['grid'], linewidth=0.7)
            ax_gain.legend(frameon=False, fontsize=7, loc='upper left')
            ax_gain.text(-0.16, 1.06, 'a', transform=ax_gain.transAxes, fontweight='bold', fontsize=12)

            ax_rel.plot(
                x, df['consistency'].to_numpy(), '-o',
                color=palette['consistency'], linewidth=1.8, label='Consistency'
            )
            ax_rel.plot(
                x, df['conflict'].to_numpy(), '-o',
                color=palette['conflict'], linewidth=1.8, label='Conflict'
            )
            ax_rel.set_ylim(0, 105)
            ax_rel.set_ylabel('Overlap region (%)')
            ax_rel.set_xticks(x)
            ax_rel.set_xticklabels(['Run 1', 'Run 2'])
            ax_rel.grid(True, axis='y', color=palette['grid'], linewidth=0.7)
            ax_rel.legend(frameon=False, fontsize=7, loc='center right')
            ax_rel.text(-0.16, 1.06, 'b', transform=ax_rel.transAxes, fontweight='bold', fontsize=12)

            for ax in (ax_gain, ax_rel):
                ax.spines['top'].set_visible(False)
                ax.spines['right'].set_visible(False)

            out_png = output_dir / 'cross_experiment_validation_publication.png'
            fig.savefig(out_png, dpi=600, bbox_inches='tight', pad_inches=0.04)
            fig.savefig(out_png.with_suffix('.pdf'), bbox_inches='tight', pad_inches=0.04)
            fig.savefig(out_png.with_suffix('.svg'), bbox_inches='tight', pad_inches=0.04)
            plt.close(fig)

    def generate_metrics_table(self, output_dir: Path):
        """生成指标表格"""
        # 动态获取层数
        available_layers = self.coverage_df.index.tolist()
        
        # 创建综合指标表格
        coverage_metrics = {}
        spatial_metrics = {}
        
        for layer_idx in available_layers:
            coverage_metrics[f"Layer {layer_idx}"] = {
                "Coverage Ratio": f"{self.coverage_df.loc[layer_idx, 'coverage_ratio']:.6f}",
                "Explored Cells": f"{int(self.coverage_df.loc[layer_idx, 'explored_cells'])}",
                "Occupied Cells": f"{int(self.coverage_df.loc[layer_idx, 'occupied_cells'])}",
                "Free Cells": f"{int(self.coverage_df.loc[layer_idx, 'free_cells'])}"
            }
            
            spatial_metrics[f"Layer {layer_idx}"] = {
                "Components": f"{int(self.spatial_df.loc[layer_idx, 'num_components'])}",
                "Largest Component": f"{int(self.spatial_df.loc[layer_idx, 'largest_component_size'])}",
                "Density": f"{self.spatial_df.loc[layer_idx, 'density']:.6f}",
                "X Range": f"{self.spatial_df.loc[layer_idx, 'x_range']:.1f}",
                "Y Range": f"{self.spatial_df.loc[layer_idx, 'y_range']:.1f}"
            }
        
        metrics_table = {
            "Coverage Metrics": coverage_metrics,
            "Spatial Metrics": spatial_metrics
        }
        
        # 保存为JSON
        with open(output_dir / 'metrics_table.json', 'w', encoding='utf-8') as f:
            json.dump(metrics_table, f, indent=2, ensure_ascii=False)
        
        # 生成LaTeX表格
        self._generate_latex_table(output_dir, metrics_table)
    
    def _generate_latex_table(self, output_dir: Path, metrics_table: Dict):
        """生成LaTeX表格"""
        latex_content = """\\begin{table}[h]
\\centering
\\caption{Multi-Layer SLAM Performance Metrics}
\\label{tab:performance_metrics}
\\begin{tabular}{|c|c|c|c|c|}
\\hline
\\textbf{Layer} & \\textbf{Coverage Ratio} & \\textbf{Explored Cells} & \\textbf{Occupied Cells} & \\textbf{Components} \\\\
\\hline
"""
        
        # 动态获取层数
        available_layers = self.coverage_df.index.tolist()
        
        for layer_idx in available_layers:
            layer_name = f"Layer {layer_idx}"
            layer_num = str(layer_idx)
            coverage = metrics_table["Coverage Metrics"][layer_name]["Coverage Ratio"]
            explored = metrics_table["Coverage Metrics"][layer_name]["Explored Cells"]
            occupied = metrics_table["Coverage Metrics"][layer_name]["Occupied Cells"]
            components = metrics_table["Spatial Metrics"][layer_name]["Components"]
            
            latex_content += f"{layer_num} & {coverage} & {explored} & {occupied} & {components} \\\\\n"
        
        latex_content += """\\hline
\\end{tabular}
\\end{table}
"""
        
        with open(output_dir / 'metrics_table.tex', 'w', encoding='utf-8') as f:
            f.write(latex_content)
    
    def save_comprehensive_summary(self, output_dir: Path):
        """保存综合总结"""
        # 生成所有分析结果
        quantitative_results = self.generate_quantitative_results()
        consistency_analysis = self.generate_consistency_analysis()
        spatial_analysis = self.generate_spatial_analysis()
        performance_comparison = self.generate_performance_comparison()
        
        # 创建综合总结
        available_layers = self.coverage_df.index.tolist()
        comprehensive_summary = {
            "experiment_info": {
                "total_layers": len(available_layers),
                "map_size": "100x100 pixels (5m x 5m)",
                "actual_field_size": "2.15m x 2.0m",
                "resolution": "0.05m/pixel",
                "timestamp": "20251023_093826"
            },
            "quantitative_results": quantitative_results,
            "consistency_analysis": consistency_analysis,
            "spatial_analysis": spatial_analysis,
            "performance_comparison": performance_comparison,
            "key_findings": [
                f"Average coverage ratio: {quantitative_results['summary_statistics']['average_coverage']:.6f}",
                f"Total explored cells: {quantitative_results['summary_statistics']['total_explored']}",
                f"Coverage improvement: {quantitative_results['summary_statistics']['coverage_improvement']:.6f}",
                f"Average components per layer: {spatial_analysis['connectivity']['average_components']:.1f}",
                f"Connectivity score: {spatial_analysis['connectivity']['connectivity_score']:.3f}"
            ]
        }
        
        # 保存综合总结
        with open(output_dir / 'comprehensive_summary.json', 'w', encoding='utf-8') as f:
            json.dump(comprehensive_summary, f, indent=2, ensure_ascii=False)
        
        # 生成论文级别的图表
        self.create_publication_figures(output_dir)
        
        # 生成指标表格
        self.generate_metrics_table(output_dir)

def main():
    """主函数"""
    script_dir = Path(__file__).parent
    experiments_dir = script_dir / 'experiments'
    experiment_names = [
        'used_multi_layer_map_20251023_091856',
        'used_multi_layer_map_20251023_093826',
    ]

    experiment_dirs = []
    print("Generating Nature-style publication figures...")
    for experiment_name in experiment_names:
        analysis_dir = experiments_dir / experiment_name / 'analysis'
        output_dir = analysis_dir / 'paper_metrics'
        output_dir.mkdir(parents=True, exist_ok=True)
        summary = PaperMetricsSummary(analysis_dir)
        summary.create_publication_figures(output_dir)
        experiment_dirs.append(experiments_dir / experiment_name)
        print(f"  - {output_dir / 'performance_improvement_publication.png'}")

    cross_output_dir = experiments_dir / 'paper_metrics'
    PaperMetricsSummary.create_cross_experiment_validation_figure(experiment_dirs, cross_output_dir)
    print(f"  - {cross_output_dir / 'cross_experiment_validation_publication.png'}")

if __name__ == '__main__':
    main()
