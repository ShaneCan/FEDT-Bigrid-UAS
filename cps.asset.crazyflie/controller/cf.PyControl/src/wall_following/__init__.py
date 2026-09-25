#!/usr/bin/env python3
"""
墙面跟随模块

提供Crazyflie墙面跟随功能的完整实现，包括：
- 核心墙面跟随算法
- 状态机可视化
- ROS2节点实现
- 配置管理
- 生命周期管理
"""

__version__ = "1.0.0"

# 核心算法层
from .wall_following import WallFollowing
from .wall_following_simple import SimpleWallFollowing

# 状态机层
from .wall_following_state_machine import WallFollowingStateMachine

# 配置管理层
from .wall_following_config import WallFollowingConfig

# ROS2节点层
from .wall_following_multiranger import WallFollowingMultiranger

# 生命周期管理层
from .wall_following_runner import (
    ConfigurableWallFollowingMultiranger,
    WallFollowingOrchestrator,
)

__all__ = [
    # 核心类
    "WallFollowing",
    "SimpleWallFollowing",
    "WallFollowingStateMachine",
    "WallFollowingConfig",
    "WallFollowingMultiranger",
    "ConfigurableWallFollowingMultiranger",
    "WallFollowingOrchestrator",
]

