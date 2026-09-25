#!/usr/bin/env python3
"""使用REST接口自动执行Crazyflie墙面跟随流程。"""

import argparse
import json
import logging
import time
import threading
from typing import Any, Dict, Optional, List

import requests

from auto_state_machine import AutoStateMachine


logger = logging.getLogger(__name__)


class WallFollowAutoRunner:
    """封装墙面跟随自动流程。"""

    def __init__(
        self,
        base_url: str,
        wall_follow_config: Optional[Dict[str, Any]] = None,
        hover_after_takeoff: float = 2.0,
        wall_follow_duration: float = 20.0,
    ) -> None:
        self.base_url = base_url.rstrip('/')
        self.wall_follow_config = wall_follow_config or {}
        self.hover_after_takeoff = hover_after_takeoff
        self.wall_follow_duration = wall_follow_duration
        self.sm = AutoStateMachine(base_url=self.base_url)

    def _post(self, endpoint: str, payload: Optional[Dict[str, Any]] = None) -> Optional[Dict[str, Any]]:
        url = f"{self.base_url}{endpoint}"
        try:
            response = requests.post(url, json=payload, timeout=5.0)
            if response.status_code != 200:
                logger.error("POST %s failed: %s", url, response.text)
                return None
            return response.json()
        except Exception as exc:
            logger.error("POST %s exception: %s", url, exc)
            return None

    def activate_idle(self) -> bool:
        logger.info("Activating idle")
        return self.sm.activate_idle() and self.sm.wait_for_state("idle", timeout=5.0)

    def takeoff(self) -> bool:
        logger.info("Starting takeoff")
        if not self.sm.begin_takeoff():
            return False
        if not self.sm.wait_for_state("hovering", timeout=10.0):
            logger.error("Takeoff did not reach hovering state")
            return False
        if self.hover_after_takeoff > 0:
            logger.info("Hovering for %.1f seconds", self.hover_after_takeoff)
            time.sleep(self.hover_after_takeoff)
        return True

    def start_wall_follow(self) -> bool:
        logger.info("Starting wall following with config: %s", json.dumps(self.wall_follow_config))
        # 如果配置中包含hover_height，记录高度信息
        if 'hover_height' in self.wall_follow_config:
            logger.info("Wall following at height: %.2f meters", self.wall_follow_config['hover_height'])
        result = self._post("/wall_follow/start", self.wall_follow_config)
        if not result:
            return False
        return self.sm.wait_for_state("flying", timeout=10.0)

    def run_wall_follow(self) -> None:
        if self.wall_follow_duration > 0:
            logger.info("Wall following for %.1f seconds", self.wall_follow_duration)
            time.sleep(self.wall_follow_duration)

    def stop_wall_follow(self) -> bool:
        logger.info("Stopping wall following")
        result = self._post("/wall_follow/stop")
        if not result:
            return False
        return self.sm.wait_for_state("hovering", timeout=10.0)

    def land(self) -> bool:
        logger.info("Starting landing")
        
        # 先停止持续控制（hover/move命令），确保降落指令能够正常执行
        try:
            logger.info("Stopping controller streaming before landing")
            response = requests.post(f"{self.base_url}/stop_controller", timeout=2.0)
            if response.status_code == 200:
                logger.info("Controller streaming stopped successfully")
            else:
                logger.warning(f"Failed to stop controller streaming: {response.status_code}")
        except Exception as e:
            logger.warning(f"Error stopping controller streaming: {str(e)}")
        
        # 然后执行降落指令
        if not self.sm.begin_landing():
            return False
        return self.sm.wait_for_state("landing", timeout=10.0)

    def execute(self) -> bool:
        if not self.activate_idle():
            logger.error("Failed to enter idle state")
            return False
        if not self.takeoff():
            logger.error("Failed to take off")
            return False
        if not self.start_wall_follow():
            logger.error("Failed to start wall following")
            return False
        self.run_wall_follow()
        if not self.stop_wall_follow():
            logger.error("Failed to stop wall following")
            return False
        if not self.land():
            logger.error("Failed to land")
            return False
        logger.info("Wall follow mission completed successfully")
        return True


class MultiDroneWallFollowManager:
    """多机墙面跟随管理器。"""

    def __init__(
        self,
        wall_follow_config: Optional[Dict[str, Any]] = None,
        hover_after_takeoff: float = 2.0,
        wall_follow_duration: float = 20.0,
    ) -> None:
        """初始化多机管理器
        
        Args:
            wall_follow_config: 墙面跟随配置
            hover_after_takeoff: 起飞后悬停时间
            wall_follow_duration: 墙面跟随持续时间
        """
        # 3机配置 - 可以通过注释来控制哪些无人机参与
        # 要禁用某架无人机，只需注释掉对应的行
        # 每架无人机可以设置不同的高度和墙面跟随参数
        self.drone_configs = {
            'cf231': {
                'port': 5000, 
                'wall_follow_config': {
                    **(wall_follow_config or {}),
                    'hover_height': 0.4,
                    'direction': 'right',  # cf231 贴右侧飞行
                    'algorithm_mode': 'simple',  # full为完整 simple为简化 使用简化算法
                    'target_wall_distance': 0.35,  # 距离墙面35cm（增大以适应小场地）
                    'front_wall_threshold': 0.40,  # 前墙40cm时转弯（提前转弯）
                    'max_forward_speed': 0.12,  # 降低速度到0.12
                    'max_turn_rate': 0.35  # 降低转速避免过冲
                }
            },
            'cf232': {
                'port': 5001, 
                'wall_follow_config': {
                    **(wall_follow_config or {}),
                    'hover_height': 0.65,
                    'direction': 'right',  # cf232 贴左侧飞行
                    'algorithm_mode': 'simple',  # full为完整 simple为简化
                    'target_wall_distance': 0.35,  # 距离墙面35cm（增大以适应小场地）
                    'front_wall_threshold': 0.40,  # 前墙40cm时转弯（提前转弯）
                    'max_forward_speed': 0.12,  # 降低速度到0.12
                    'max_turn_rate': 0.35  # 降低转速避免过冲
                }
            },
            # 'cf233': {
            #     'port': 5002, 
            #     'wall_follow_config': {
            #         **(wall_follow_config or {}),
            #         'hover_height': 0.65  
            #     }
            # }
            # 'cf234': {'port': 5003, 'wall_follow_config': {**(wall_follow_config or {}), 'hover_height': 1.4}},  # 示例：添加第4架无人机，高度1.4米
        }
        
        self.hover_after_takeoff = hover_after_takeoff
        self.wall_follow_duration = wall_follow_duration
        
        # 创建每架无人机的控制器
        self.drone_runners = {}
        for drone_name, config in self.drone_configs.items():
            base_url = f"http://127.0.0.1:{config['port']}"
            wall_follow_config = config.get('wall_follow_config', {})
            
            self.drone_runners[drone_name] = WallFollowAutoRunner(
                base_url=base_url,
                wall_follow_config=wall_follow_config,
                hover_after_takeoff=self.hover_after_takeoff,
                wall_follow_duration=self.wall_follow_duration
            )
            
        logger.info(f"Initialized multi-drone wall follow manager with {len(self.drone_runners)} drones")
        
        # 显示每架无人机的配置信息
        for drone_name, config in self.drone_configs.items():
            height = config['wall_follow_config'].get('hover_height', 'default')
            port = config['port']
            logger.info(f"  {drone_name}: port={port}, height={height}m")

    def execute_single_drone(self, drone_name: str) -> bool:
        """执行单架无人机的墙面跟随任务"""
        if drone_name not in self.drone_runners:
            logger.error(f"Drone {drone_name} not found in configuration")
            return False
            
        logger.info(f"Starting wall follow mission for {drone_name}")
        runner = self.drone_runners[drone_name]
        
        try:
            success = runner.execute()
            if success:
                logger.info(f"Wall follow mission completed successfully for {drone_name}")
            else:
                logger.error(f"Wall follow mission failed for {drone_name}")
            return success
        except Exception as e:
            logger.error(f"Exception during wall follow mission for {drone_name}: {str(e)}")
            return False

    def execute_parallel(self) -> Dict[str, bool]:
        """并行执行所有无人机的墙面跟随任务"""
        logger.info("Starting parallel wall follow missions for all drones")
        
        results = {}
        threads = []
        
        def run_drone_task(drone_name: str):
            results[drone_name] = self.execute_single_drone(drone_name)
        
        # 创建并启动线程
        for drone_name in self.drone_runners.keys():
            thread = threading.Thread(target=run_drone_task, args=(drone_name,))
            threads.append(thread)
            thread.start()
        
        # 等待所有线程完成
        for thread in threads:
            thread.join()
        
        # 统计结果
        successful = sum(1 for success in results.values() if success)
        total = len(results)
        logger.info(f"Parallel execution completed: {successful}/{total} drones successful")
        
        return results

    def execute_sequential(self) -> Dict[str, bool]:
        """顺序执行所有无人机的墙面跟随任务"""
        logger.info("Starting sequential wall follow missions for all drones")
        
        results = {}
        for drone_name in self.drone_runners.keys():
            results[drone_name] = self.execute_single_drone(drone_name)
            # 在每架无人机之间稍作停顿
            time.sleep(2.0)
        
        # 统计结果
        successful = sum(1 for success in results.values() if success)
        total = len(results)
        logger.info(f"Sequential execution completed: {successful}/{total} drones successful")
        
        return results

    def execute_staggered(self, stagger_delay: float = 10.0) -> Dict[str, bool]:
        """错开执行所有无人机的墙面跟随任务（每架无人机间隔指定时间启动）"""
        logger.info(f"Starting staggered wall follow missions with {stagger_delay}s delay")
        
        results = {}
        threads = []
        
        def run_drone_task_with_delay(drone_name: str, delay: float):
            time.sleep(delay)
            results[drone_name] = self.execute_single_drone(drone_name)
        
        # 创建并启动线程，每架无人机延迟不同时间
        for i, drone_name in enumerate(self.drone_runners.keys()):
            delay = i * stagger_delay
            thread = threading.Thread(target=run_drone_task_with_delay, args=(drone_name, delay))
            threads.append(thread)
            thread.start()
        
        # 等待所有线程完成
        for thread in threads:
            thread.join()
        
        # 统计结果
        successful = sum(1 for success in results.values() if success)
        total = len(results)
        logger.info(f"Staggered execution completed: {successful}/{total} drones successful")
        
        return results


def create_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Multi-drone Crazyflie wall follow runner")
    parser.add_argument("--wall_follow_duration", type=float, default=30.0, help="墙面跟随持续时间（秒）")
    parser.add_argument("--hover_after_takeoff", type=float, default=2.0, help="起飞完成后悬停时间（秒）")
    parser.add_argument("--config", type=str, default="", help="墙面跟随配置JSON字符串")
    parser.add_argument("--execution_mode", type=str, choices=["parallel", "sequential", "staggered"], 
                       default="parallel", help="多机执行模式：parallel（并行）、sequential（顺序）、staggered（错开）")
    parser.add_argument("--stagger_delay", type=float, default=10.0, help="错开执行时的延迟时间（秒）")
    parser.add_argument("--debug", action="store_true", help="输出调试信息")
    return parser


def main() -> None:
    parser = create_arg_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    # 解析配置
    config: Dict[str, Any] = {}
    if args.config:
        try:
            config = json.loads(args.config)
        except json.JSONDecodeError as exc:
            logger.error("配置解析失败: %s", exc)
            return

    # 创建多机管理器
    logger.info("Running multi-drone wall follow missions")
    manager = MultiDroneWallFollowManager(
        wall_follow_config=config,
        hover_after_takeoff=args.hover_after_takeoff,
        wall_follow_duration=args.wall_follow_duration,
    )
    
    # 根据执行模式运行
    if args.execution_mode == "parallel":
        results = manager.execute_parallel()
    elif args.execution_mode == "sequential":
        results = manager.execute_sequential()
    elif args.execution_mode == "staggered":
        results = manager.execute_staggered(args.stagger_delay)
    else:
        logger.error(f"Unknown execution mode: {args.execution_mode}")
        return
    
    # 输出结果
    logger.info("Execution results:")
    for drone_name, success in results.items():
        status = "SUCCESS" if success else "FAILED"
        logger.info(f"  {drone_name}: {status}")
    
    # 检查是否有失败的无人机
    failed_drones = [name for name, success in results.items() if not success]
    if failed_drones:
        logger.warning(f"Failed drones: {failed_drones}")
    else:
        logger.info("All drones completed successfully!")


def demo_multi_drone_wall_follow():
    """演示多机墙面跟随功能"""
    logger.info("Starting multi-drone wall follow demo with different heights")
    
    # 创建多机管理器 - 每架无人机将使用不同的高度
    manager = MultiDroneWallFollowManager(
        wall_follow_config={'speed': 0.2, 'distance': 0.5},
        hover_after_takeoff=2.0,
        wall_follow_duration=20.0  # 演示用较短时间
    )
    
    # 并行执行
    logger.info("Executing parallel wall follow missions at different heights...")
    results = manager.execute_parallel()
    
    # 输出结果
    logger.info("Demo results:")
    for drone_name, success in results.items():
        status = "SUCCESS" if success else "FAILED"
        height = manager.drone_configs[drone_name]['wall_follow_config']['hover_height']
        logger.info(f"  {drone_name}: {status} (height: {height}m)")


if __name__ == "__main__":
    main()


