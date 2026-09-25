#!/usr/bin/env python3
"""集中管理墙面跟随配置参数。"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict


@dataclass
class WallFollowingConfig:
    """墙面跟随相关的统一配置。"""

    robot_prefix: str
    delay: float = 5.0
    max_turn_rate: float = 0.5
    max_forward_speed: float = 0.2
    direction: str = "right" ########################
    hover_height: float = 0.5
    use_sim_time: bool = False
    auto_land: bool = False

    # 算法模式：'simple'（简化版）或 'full'（完整版）
    algorithm_mode: str = "simple"
    
    # 简化算法参数
    target_wall_distance: float = 0.3      # 与墙保持距离(m)
    front_wall_threshold: float = 0.35     # 前墙触发转弯距离(m)

    # 路径配置 - 使用相对路径或环境变量
    image_output_base_dir: str = field(default="")
    
    additional_parameters: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        """初始化后处理，设置默认路径"""
        if not self.image_output_base_dir:
            # 使用环境变量或默认相对路径
            base_dir = os.environ.get(
                'CRAZYFLIE_WALL_FOLLOWING_IMG_DIR',
                os.path.join(os.path.dirname(__file__), '..', '..', 'webview', 'wall_following', 'img')
            )
            self.image_output_base_dir = os.path.abspath(base_dir)

    @classmethod
    def from_request(
        cls,
        robot_prefix: str,
        request_data: Dict[str, Any] | None,
        defaults: "WallFollowingConfig" | None = None,
    ) -> "WallFollowingConfig":
        """根据REST请求构建配置，允许覆盖默认值。"""

        data = request_data or {}
        base = defaults or cls(robot_prefix=robot_prefix)

        return cls(
            robot_prefix=robot_prefix,
            delay=float(data.get("delay", base.delay)),
            max_turn_rate=float(data.get("max_turn_rate", base.max_turn_rate)),
            max_forward_speed=float(data.get("max_forward_speed", base.max_forward_speed)),
            direction=str(data.get("direction", base.direction)),
            hover_height=float(data.get("hover_height", base.hover_height)),
            use_sim_time=bool(data.get("use_sim_time", base.use_sim_time)),
            auto_land=bool(data.get("auto_land", base.auto_land)),
            algorithm_mode=str(data.get("algorithm_mode", base.algorithm_mode)),
            target_wall_distance=float(data.get("target_wall_distance", base.target_wall_distance)),
            front_wall_threshold=float(data.get("front_wall_threshold", base.front_wall_threshold)),
            image_output_base_dir=str(data.get("image_output_base_dir", base.image_output_base_dir)),
            additional_parameters={k: v for k, v in data.items() if k not in {
                "delay",
                "max_turn_rate",
                "max_forward_speed",
                "direction",
                "hover_height",
                "use_sim_time",
                "auto_land",
                "algorithm_mode",
                "target_wall_distance",
                "front_wall_threshold",
                "image_output_base_dir",
            }},
        )

    def to_wall_following_kwargs(self) -> Dict[str, Any]:
        """转换为WallFollowingMultiranger构造所需的关键字参数。"""

        return {
            "robot_prefix": self.robot_prefix,
            "delay": self.delay,
            "max_turn_rate": self.max_turn_rate,
            "max_forward_speed": self.max_forward_speed,
            "wall_following_direction": self.direction,
            "hover_height": self.hover_height,
            "auto_land": self.auto_land,
            "use_sim_time": self.use_sim_time,
            "algorithm_mode": self.algorithm_mode,
            "target_wall_distance": self.target_wall_distance,
            "front_wall_threshold": self.front_wall_threshold,
            "image_output_base_dir": self.image_output_base_dir,
            **self.additional_parameters,
        }

    def get_image_dir(self, drone_id: str | None = None) -> str:
        """获取指定无人机的图片输出目录
        
        Args:
            drone_id: 无人机ID，如果为None则使用robot_prefix提取
            
        Returns:
            图片输出目录的绝对路径
        """
        if drone_id is None:
            drone_id = self.robot_prefix.replace('/', '')
        
        img_dir = os.path.join(self.image_output_base_dir, drone_id)
        os.makedirs(img_dir, exist_ok=True)
        return img_dir

