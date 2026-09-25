import logging
import sys
import time
from typing import Optional
from threading import Event
import threading
import os

from flask import Flask, jsonify, request
from werkzeug.routing import BaseConverter, ValidationError
from statemachine import StateMachine, State, exceptions
from statemachine.contrib.diagram import DotGraphMachine


import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

from cf_positioning import Point3D, PositionEstimationStrategy, KalmanEstimatePositionStrategy, StateEstimatePositionStrategy
from cf_drone_ops import CFOperationStrategy, HlCommanderCFOperationImpl, DebugLoggingCFOperationImpl
from wall_following import WallFollowingConfig

loggingPeriod_in_ms = 100 #ms

class StateMachineDrone(StateMachine):

    uavOpStrategyImpl = None     # 策略实例 通过cf-ctrl-service主函数中设置，可选ros2或scf控制器
    _image_numbers = {}  # 使用字典存储每个无人机的图片编号
    _last_state = None  # 添加变量来跟踪上一次的状态

    goalReached = False
    isFlying = False
    targetPointsQueue = [] #通过REST API接口来添加目标点的 API接口在routes.py
    _IS_DEBUG = False
    _graph_lock = threading.Lock()  # 添加锁机制
    _last_graph_update = 0  # 记录上次更新时间
    _graph_update_interval = 0.1  # 更新间隔（秒）

    def __init__(self, drone_id: str, debug: bool = False): 
        # 先设置所有必要的属性
        self._IS_DEBUG = debug
        self._uav_name = drone_id  # 直接使用传入的drone_id
        if not self._uav_name:
            raise ValueError("drone_id must be set and cannot be None or empty")
        self.wall_following_config: Optional[WallFollowingConfig] = None
        
        # 为每个无人机初始化图片编号
        if self._uav_name not in self._image_numbers:
            # 检查latest.txt文件是否存在
            img_dir = f"/home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/{self._uav_name}"
            latest_file = f"{img_dir}/latest.txt"
            if os.path.exists(latest_file):
                try:
                    with open(latest_file, 'r') as f:
                        latest_number = int(f.read().strip())
                    self._image_numbers[self._uav_name] = (latest_number % 100) + 1
                except Exception as e:
                    print(f"Error reading latest.txt: {e}")
                    self._image_numbers[self._uav_name] = 1
            else:
                self._image_numbers[self._uav_name] = 1
            
        # 确保图片目录和latest.txt文件存在
        img_dir = f"/home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/{self._uav_name}"
        os.makedirs(img_dir, exist_ok=True)
        latest_file = f"{img_dir}/latest.txt"
        if not os.path.exists(latest_file):
            with open(latest_file, 'w') as f:
                f.write(str(self._image_numbers[self._uav_name]))
            
        self.current_transition = None
        
        # 最后调用父类的初始化
        super().__init__()

    @property
    def uav_name(self):
        return self._uav_name

    def set_uav_name(self, drone_id):
        if not drone_id:
            raise ValueError("drone_id must be set and cannot be None or empty")
        self._uav_name = drone_id

    def set_uavOpsImpl(self, uavOpsImpl):# 通过cf-ctrl-service的main函数中的StateMachineDrone.set_uavOpsImpl设置uavOpsImpl
        print(f"\nSetting uavOpStrategyImpl...")
        print(f"Current uavOpStrategyImpl: {self.uavOpStrategyImpl}")
        print(f"New uavOpsImpl: {uavOpsImpl}")
        if not uavOpsImpl:
            raise ValueError(f"uavOpsImpl must be set and cannot be None or empty. uavOpsImpl must be of type {CFOperationStrategy.__qualname__}")
        self.uavOpStrategyImpl = uavOpsImpl
        print(f"uavOpStrategyImpl after setting: {self.uavOpStrategyImpl}\n")

    def printDebug(self, msg: str):
        if self._IS_DEBUG:
            print(msg)

    # Define the OSGi lifecycle states
    installed = State('INSTALLED', initial=True)
    resolved = State('RESOLVED')
    starting = State('STARTING')
    active = State('ACTIVE')
    stopping = State('STOPPING')
    uninstalled = State('UNINSTALLED', final=True)

    # Define the UAV operation states
    idle = State('IDLE')
    hovering = State('HOVERING') #takeoff completed
    flying = State('FLYING')
    landed = State('LANDING')
    shutdown = State('SHUTDOWN')
    mapping = State('MAPPING')

    # Define the transitions for "OSGi" lifecycle (similar pattern here)
    install = installed.to(resolved, cond='dependencies_resolved')
    start = resolved.to(starting)
    initialize = starting.to(active)
    stop = active.to(stopping)
    stopped = stopping.to(resolved)
    uninstall = resolved.to(uninstalled)

    # Define the transitions for UAV operation
    activate_idle = active.to(idle) #activate_idle是event，active和idle是state
    begin_takeoff = idle.to(hovering) #cond="reached_Height"
    begin_landing = hovering.to(landed)
    begin_nav_goal_sequence = hovering.to(flying)
    next_nav_goal = flying.to(flying, cond='goal_reached')
    keep_hovering = flying.to(hovering, cond='goal_reached')
    abort_to_hover = flying.to(hovering)  # 新增：强制从飞行回到悬停（无条件）
    begin_wall_following = hovering.to(mapping)
    stop_wall_following = mapping.to(hovering)

    landing_completed = landed.to(idle, cond="is_not_flying")
    shutdown_command = idle.to(shutdown)
    begin_stopping = shutdown.to(stopping)

    # Conditional methods for transitions (if needed)
    def dependencies_resolved(self):
        # Implement logic to check if dependencies are resolved
        return True
    def goal_reached(self):
        """检查是否到达目标位置"""
        print("\n" + "="*50)
        print("GOAL REACHED CHECK")
        print("="*50)
        print(f"Current state: {self.current_state}")
        print(f"Current transition: {self.current_transition}")
        print(f"uavOpStrategyImpl exists: {self.uavOpStrategyImpl is not None}")
        
        # 首先检查uavOpStrategyImpl是否存在
        if not self.uavOpStrategyImpl:
            print("ERROR: No uavOpStrategyImpl found!")
            print("This should not happen as it should be set during initialization")
            self.goalReached = True
            return True
            
        # 检查是使用scf还是controller
        has_scf = hasattr(self.uavOpStrategyImpl, 'scf') and self.uavOpStrategyImpl.scf is not None
        has_controller = hasattr(self.uavOpStrategyImpl, 'controller') and self.uavOpStrategyImpl.controller is not None
        
        print(f"Using scf: {has_scf}")
        print(f"Using controller: {has_controller}")
        
        if has_scf:
            print("Using scf mode, returning True")
            self.goalReached = True
            return True
            
        if has_controller:
            # 获取当前位置
            current_pos = self.uavOpStrategyImpl.controller.current_position
            if not current_pos:
                print("No current position available, returning True")
                self.goalReached = True
                return True
            
            # 获取目标位置
            if len(self.targetPointsQueue) > 0:
                target = self.targetPointsQueue[0]
                print(f"Target point: ({target.x}, {target.y}, {target.z})")
            else:
                print("No target points in queue, returning True")
                self.goalReached = True
                return True
            
            # 计算距离
            distance = ((current_pos.x - target.x) ** 2 + 
                        (current_pos.y - target.y) ** 2 + 
                        (current_pos.z - target.z) ** 2) ** 0.5
                    
            # 如果距离小于阈值，认为到达目标
            threshold = 1  # 单位 m (原来为1m)
            self.goalReached = distance < threshold
            print(f"Distance to target: {distance:.2f}m, threshold: {threshold}m")
            print(f"Goal reached: {self.goalReached}")
            print("="*50 + "\n")
            return self.goalReached
            
        print("Neither scf nor controller found, returning True")
        self.goalReached = True
        return True
    def can_install(self):
        return True
    def is_not_flying(self):
        self.printDebug(f"<Condition> Checking for [{self.current_state}]: is_Flying={self.isFlying}")
        while(self.isFlying == True):
            time.sleep(loggingPeriod_in_ms/2/1000)
        self.printDebug(f"</Condition> reached for [{self.current_state}]: is_Flying={self.isFlying}")
        return True


    # Before/after_transition is for firing transitions only.
    # 转换前的处理
    def before_transition(self, event, state):
        # self.printDebug(f"BT: Before '{event}', on the '{state.id}' state.")
        self.current_transition = event
        self.writeSMGraph()
        
        # self.writeSMGraph()
        return "before_transition_return"


    # This is for executing long-running UAV-actions
    # 转换时的处理
    def on_transition(self, event, state):
        """状态转换时的处理"""
        self.printDebug(f"OT: On '{event}', on the '{state.id}' state.")
        
        try:
            # if event == "initialize":
            #     if self.uavOpStrategyImpl is None or not self.uavOpStrategyImpl.isSetSCF():
            #         print("Not CF Operation Strategy selected or controller not initialized!")
            #         raise Exception("Not CF Operation Strategy selected!")

            # 调用Curl命令会触发下列事件， 见routes.py
            if event == "activate_idle":
                if self.uavOpStrategyImpl:
                    success = self.uavOpStrategyImpl.activate_idle_simple()
                    if not success:
                        print("Failed to activate idle")
                
            elif event == "begin_takeoff":
                if self.uavOpStrategyImpl:
                    success = self.uavOpStrategyImpl.take_off_simple()
                    if not success:
                        print("Failed to take off")
                
            elif event == "begin_landing":
                if self.uavOpStrategyImpl:
                    success = self.uavOpStrategyImpl.landing_simple()
                    if not success:
                        print("Failed to land")
                
            elif event == "begin_nav_goal_sequence" or event == "next_nav_goal":
                # 注意第一个导航点导航完毕后会进入after_transition中继续处理
                if self.uavOpStrategyImpl and len(self.targetPointsQueue) > 0:
                    targetPoint = self.targetPointsQueue.pop(0)
                    success = self.uavOpStrategyImpl.navigate_to_simple(targetPoint) #navigate_to_simple是cf_drone_ops.py中的方法,ROS2调用GOTO方法
                    if not success:
                        print(f"Failed to navigate to {targetPoint}")
            elif event == "begin_wall_following":
                if self.uavOpStrategyImpl:
                    success = False
                    if self.wall_following_config:
                        success = self.uavOpStrategyImpl.start_wall_following(self.wall_following_config)
                    else:
                        print("Wall following configuration not set")
                    if not success:
                        print("Failed to start wall following")
            elif event == "stop_wall_following":
                if self.uavOpStrategyImpl:
                    success = self.uavOpStrategyImpl.stop_wall_following()
                    if not success:
                        print("Failed to stop wall following")
            elif event == "abort_to_hover":
                # 立即停止当前轨迹/推送，强制回到hover
                if self.uavOpStrategyImpl:
                    try:
                        # 1. 先停止当前setpoints
                        if hasattr(self.uavOpStrategyImpl, 'controller') and self.uavOpStrategyImpl.controller is not None:
                            self.uavOpStrategyImpl.controller.stop()
                        # 2. 强制标记目标已到达，允许状态转换
                        self.goalReached = True
                        print(f"Force goal_reached=True for {self._uav_name} to allow abort_to_hover transition")
                    except Exception as e:
                        print(f"Failed to stop controller setpoints: {e}")
                
            elif event == "shutdown":
                if self.uavOpStrategyImpl:
                    success = self.uavOpStrategyImpl.shutdown()
                    if not success:
                        print("Failed to shutdown")
                global ISRUNNING
                ISRUNNING = False
            
            # 生成状态机图
            self.writeSMGraph()
            
        except Exception as e:
            print(f"Error in on_transition: {str(e)}")
            import traceback
            traceback.print_exc()
            raise
        
        return "on_transition_return"


    def on_exit_state(self, event, state):
        self.printDebug(f"OnExState: Exiting '{state.id}' state from '{event}' event.")
        self.writeSMGraph()
        
        self.writeSMGraph()
        return "on_exit_state"


    def on_enter_state(self, event, state):
        self.printDebug(f"OnEnState: Entering '{state.id}' state from '{event}' event.")
        self.writeSMGraph()
        if state == self.hovering:
            self.printDebug("\tHovering now")
            time.sleep(0.5)

        self.writeSMGraph()
        return "on_enter_state"

    # Before/after_transition is for firing transitions only.
    # Initiate Long-running action here via transitionFirings instead of on_transition (direct execution of UAV operation)
    ### After 'begin_nav_goal_sequence', on the 'flying' state: only when next_nav_goal is executed the 
    # condition goal_reached(self) on this transition is fired
    # 转换后的处理
    def after_transition(self, event, state):
        """转换后的处理"""
        self.printDebug(f"AT: After '{event}', on the '{state.id}' state.")
        
        try:
            # 当无人机在flying状态，且收到next_nav_goal或begin_nav_goal_sequence事件时
            if (state == self.flying and (event == "next_nav_goal" or event == "begin_nav_goal_sequence")):
                if len(self.targetPointsQueue) > 0:
                    self.printDebug(f"\tNext_Nav_Goal: There are still targets left: {len(self.targetPointsQueue)}")
                    # 等待一段时间让无人机开始移动
                    time.sleep(0.5)
                    self.next_nav_goal() # 如果还有导航点则回到on transition继续处理下一个导航点
                else:
                    self.printDebug("\tLet drone hover now, navgoals are empty")
                    self.keep_hovering() # 如果导航点为空则进入keep_hovering状态
            
            # 当无人机从hovering进入flying状态（可能是实时移动触发的）
            if (state == self.flying and event == "begin_nav_goal_sequence"):
                # 检查是否是通过实时移动触发的
                # 如果是实时移动，我们需要等待移动完成
                self.printDebug("\tReal-time move initiated, waiting for completion...")
                # 实时移动会立即执行，所以这里不需要额外处理
            
            # 当无人机在hovering状态，且是从flying状态转来的
            if (state == self.hovering and event == "keep_hovering"):
                # 我们不会在这里自动触发降落
                # 降落决策由auto_state_machine.py控制
                self.printDebug("\tDrone is now hovering, waiting for next command")
                
            # 当无人机在landed状态时，发出落地完成事件
            if (state == self.landed): 
                self.printDebug("\tstate landed reached")
                self.landing_completed()
                
            # 更新状态机图
            self.writeSMGraph()
        except Exception as e:
            print(f"Error in after_transition: {str(e)}")
            import traceback
            traceback.print_exc()
            
        return "after_transition"

    def writeSMGraph(self):
        # 检查_uav_name是否存在
        if not hasattr(self, '_uav_name'):
            print("Warning: _uav_name not set, skipping graph generation")
            return
            
        current_state = self.current_state.id
        # 只在状态发生变化时生成新图片
        if current_state != self._last_state:
            # Generate the state diagram
            smGraph = DotGraphMachine(self)
            # 使用当前无人机的编号生成文件名
            image_number = self._image_numbers[self._uav_name]

            # 使用无人机ID作为文件夹名
            img_dir = f"/home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/{self._uav_name}"
            os.makedirs(img_dir, exist_ok=True)
            smGraphPath = f"{img_dir}/uav{image_number}.png"
            
            dot = smGraph()
            dot.write_png(smGraphPath)
            
            # 更新当前无人机的编号
            self._image_numbers[self._uav_name] = (image_number % 100) + 1
            self._last_state = current_state
            
            # 更新latest.txt文件
            latest_file = f"{img_dir}/latest.txt"
            try:
                # 读取当前latest.txt中的值
                current_latest = 1
                if os.path.exists(latest_file):
                    with open(latest_file, 'r') as f:
                        current_latest = int(f.read().strip())
                
                # 确保使用较大的编号
                new_latest = max(current_latest, image_number)
                with open(latest_file, 'w') as f:
                    f.write(str(new_latest))
                print(f"Updated latest.txt with image number: {new_latest}")
            except Exception as e:
                print(f"Error updating latest.txt: {e}")

    def get_current_state(self):
        return self.current_state.id
    
    def get_current_transition(self):
        return self.current_transition

# Track the flying state
def checkIfFlying(drone: StateMachineDrone):
    return drone.isFlying

# This method implements a "larger" action sequence in the state machine.
# It  implements navigation for drones based on a set of coordinates.
# It wraps calls to the atomic action "navigate_to_simple" of a drone
# 这是一个状态机中的导航序列实现
# 基于坐标集合进行导航
# 是对无人机原子动作navigate_to_simple的封装
def navigate_to_simple(drone: StateMachineDrone):
    """执行导航命令"""
    if not drone.uavOpStrategyImpl:
        drone.printDebug("\tNo UAV operation strategy available")
        return False
        
    if len(drone.targetPointsQueue) == 0:
        drone.printDebug("\tNo navigation targets in queue")
        drone.keep_hovering()
        return False
        
    # 获取目标位置
    target = drone.targetPointsQueue[0]
    drone.printDebug(f"\tNavigating to: ({target.x:.2f}, {target.y:.2f}, {target.z:.2f})")
    
    # 执行导航命令
    try:
        success = drone.uavOpStrategyImpl.navigate_to_simple(target)
        if success:
            drone.printDebug("\tNavigation command executed successfully")
        else:
            drone.printDebug("\tNavigation command failed")
        return success
    except Exception as e:
        drone.printDebug(f"\tError during navigation: {str(e)}")
        return False