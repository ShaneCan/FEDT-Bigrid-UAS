#!/usr/bin/env python3
"""封装墙面跟随相关ROS2节点的创建与生命周期管理。"""

from __future__ import annotations

import time
from typing import Optional

from geometry_msgs.msg import Twist  # noqa: F401  # 供动态创建publisher时引用
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from .wall_following_config import WallFollowingConfig

try:
    # 首先尝试导入本地版本
    from .wall_following_multiranger import WallFollowingMultiranger as BaseWallFollowingMultiranger
except ImportError:
    try:
        from wall_following.wall_following_multiranger import WallFollowingMultiranger as BaseWallFollowingMultiranger
    except ImportError:
        raise RuntimeError(
            "无法导入wall_following_multiranger，请确保文件在同目录或ROS2工作空间已构建。"
        )


class ConfigurableWallFollowingMultiranger(BaseWallFollowingMultiranger):
    """官方墙面跟随节点的参数化封装。"""

    def __init__(self, config: WallFollowingConfig):
        self._param_overrides = {
            "robot_prefix": config.robot_prefix,
            "delay": float(config.delay),
            "max_turn_rate": float(config.max_turn_rate),
            "max_forward_speed": float(config.max_forward_speed),
            "wall_following_direction": str(config.direction),
            "hover_height": float(config.hover_height),
            "image_output_base_dir": str(config.image_output_base_dir),
            # 简化算法相关参数（以及后续扩展参数）
            "algorithm_mode": str(getattr(config, "algorithm_mode", "simple")),
            "target_wall_distance": float(getattr(config, "target_wall_distance", 0.3)),
            "front_wall_threshold": float(getattr(config, "front_wall_threshold", 0.35)),
        }
        self._use_sim_time = bool(config.use_sim_time)
        
        # 传递kwargs给父类，包括hover_height和image_output_base_dir
        super().__init__(**self._param_overrides)
        
        # 添加调试信息
        self.get_logger().info(f"ConfigurableWallFollowingMultiranger initialized with params: {self._param_overrides}")
        
        # 确保墙面跟随状态机目录存在
        drone_id = config.robot_prefix.replace('/', '')
        img_dir = config.get_image_dir(drone_id)
        self.get_logger().info(f"Image output directory: {img_dir}")

    def declare_parameter(self, name, default_value=None):  # type: ignore[override]
        if hasattr(self, "_param_overrides") and name in self._param_overrides:
            default_value = self._param_overrides[name]
        return super().declare_parameter(name, default_value)


class WallFollowingOrchestrator:
    """负责管理墙面跟随所需节点的启动与停止。"""

    def __init__(self, executor):
        self._executor = executor
        self._wall_node: Optional[ConfigurableWallFollowingMultiranger] = None
        self._config: Optional[WallFollowingConfig] = None

    @property
    def is_active(self) -> bool:
        return self._wall_node is not None

    def start(self, config: WallFollowingConfig, logger=None) -> bool:
        if self.is_active:
            if logger:
                logger.warning("Wall following already started, ignoring duplicate request.")
            return False

        try:
            wall_node = ConfigurableWallFollowingMultiranger(config)

            if config.use_sim_time:
                sim_param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
                wall_node.set_parameters([sim_param])

            self._executor.add_node(wall_node)

            self._wall_node = wall_node
            self._config = config

            if logger:
                logger.info(
                    "Wall following node started: prefix=%s, direction=%s, speed=%.2f"
                    % (config.robot_prefix, config.direction, config.max_forward_speed)
                )
            return True
        except Exception as exc:  # pragma: no cover - 运行时日志
            if logger:
                logger.error(f"Failed to start wall following node: {exc}")
            self._cleanup()
            return False

    def stop(self, logger=None) -> bool:
        if not self.is_active:
            if logger:
                logger.warning("Wall following not started, no need to stop.")
            return True

        try:
            assert self._wall_node is not None
            # 使用节点内部服务逻辑安全降落
            try:
                req = Trigger.Request()
                res = Trigger.Response()
                self._wall_node.stop_wall_following_cb(req, res)
            except Exception as exc:
                if logger:
                    logger.warning(f"Failed to call wall following stop callback: {exc}")

            time.sleep(0.5)

            assert self._wall_node is not None
            self._executor.remove_node(self._wall_node)

            self._wall_node.destroy_node()

            if logger:
                logger.info("Wall following node stopped and cleaned up.")
            self._cleanup()
            return True
        except Exception as exc:  # pragma: no cover
            if logger:
                logger.error(f"Stop wall following node exception: {exc}")
            self._cleanup()
            return False

    def _cleanup(self) -> None:
        self._wall_node = None
        self._config = None

