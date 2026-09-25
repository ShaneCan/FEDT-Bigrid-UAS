import logging
import sys
import time
from threading import Event
import threading
from abc import ABC, abstractmethod
import asyncio

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

from cf_positioning import Point3D
from wall_following import WallFollowingConfig

DEFAULT_HEIGHT = 0.5
DEFAULT_VELOCITY = 0.3
BOX_LIMIT = 0.4

def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')

class CFOperationStrategy(ABC): # 是一个抽象类，需要被继承，被下方的HlCommanderCFOperationImpl继承
    _IS_DEBUG = False

    def __init__(self, scf=None, debug: bool = False):
        super().__init__()
        self._IS_DEBUG = debug
        self.mc = None
        self._scf = scf

    def isSetSCF(self):
        return self._scf != None
    
    def printDebug(self, msg: str):
        if self._IS_DEBUG:
            print(msg)

    @abstractmethod
    def activate_idle_simple(self):
        pass

    @abstractmethod    
    def shutdown(self):
        pass

    @abstractmethod
    def take_off_simple(self):
        pass

    @abstractmethod 
    def landing_simple(self):
        pass

    @abstractmethod
    def navigate_to_simple(self, targetPoint: Point3D):
        pass


class HlCommanderCFOperationImpl(CFOperationStrategy): # 继承了CFOperationStrategy，实现了CFOperationStrategy的抽象方法
    """这是策略模式的实现，在状态机中使用，封装了ROS2CrazyflieController的功能，
    提供了统一的接口，可以方便地切换scf和ROS2控制器，其实例是cf_sm.StateMachineDrone的uavOpStrategyImpl"""

    def __init__(self, scf=None, controller=None, debug: bool = False):   # 需要传入scf或controller
     # 创建HlCommanderCFOperationImpl实例的时候传入scf表示使用SyncCrazyflie官方库通过USB与Crazyflie无人机通信，传入controller是使用ROS2控制器  
        super().__init__(scf=scf, debug=debug)
        self.controller = controller

    def isSetSCF(self):
        """检查是否已设置Crazyflie控制器"""
        return self.controller is not None or self._scf is not None
        #return self.controller is not None or self.scf is not None


    def activate_idle_simple(self):
        """激活空闲状态"""
        print(f"HlCommanderCFOperationImpl: activate_idle_simple")
        return True

    def take_off_simple(self):
        try:
            if self.controller:
                # ROS2控制方式
                self.controller.take_off()
            elif self._scf:
                # 直接控制方式
                self.mc = PositionHlCommander(self._scf, 
                                      default_height=DEFAULT_HEIGHT, 
                                      default_velocity=DEFAULT_VELOCITY)
                self.mc.take_off(velocity=DEFAULT_VELOCITY+0.15) #take_off(self, height=None, velocity=0.2)
            return True
        except Exception as e:
            print(f"take off failed: {str(e)}")
            return False
            
    def landing_simple(self):
        try:
            if self.controller:
                # ROS2控制方式
                self.controller.land()
            elif self._scf:
                # 直接控制方式
                self.mc.land()
            return True
        except Exception as e:
            print(f"landing failed: {str(e)}")
            return False

    def navigate_to_simple(self, targetPoint):
        try:
            if self.controller:
                # ROS2控制方式
                self.controller.go_to(targetPoint.x, targetPoint.y, targetPoint.z)
            elif self._scf:
                # 直接控制方式
                self.mc.go_to(targetPoint.x, targetPoint.y, targetPoint.z)
            return True
        except Exception as e:
            print(f"navigation failed: {str(e)}")
            return False

    def hover_at_position_realtime(self, x=None, y=None, z=None, yaw=0.0, emergency_stop=True):
        """立即停止在当前位置并悬停 - 立即生效，可中断当前轨迹"""
        try:
            print(f"hover_at_position_realtime called: controller={self.controller is not None}, scf={self._scf is not None}, emergency_stop={emergency_stop}")
            
            if self.controller:
                # ROS2控制方式 - 停止在当前位置
                print("Using ROS2 controller for hover")
                return self.controller.hover_at_position(x, y, z, yaw, emergency_stop)
            elif self._scf:
                # 直接控制方式 - 停止在当前位置
                print(f"Using SCF controller for hover, mc={self.mc is not None}")
                if not self.mc:
                    # 如果 mc 未初始化，先初始化
                    print("Initializing PositionHlCommander...")
                    self.mc = PositionHlCommander(self._scf, 
                                          default_height=DEFAULT_HEIGHT, 
                                          default_velocity=DEFAULT_VELOCITY)
                    print("PositionHlCommander initialized successfully")
                
                print("Calling mc.stop()...")
                self.mc.stop()  # 停止当前运动，保持在当前位置
                if emergency_stop:
                    time.sleep(0.1)  # 紧急模式，减少等待时间
                else:
                    time.sleep(0.2)  # 正常模式
                print("mc.stop() completed successfully")
                return True
            else:
                print("Neither controller nor scf is available!")
                return False
        except Exception as e:
            print(f"hover_at_position_realtime failed: {str(e)}")
            import traceback
            traceback.print_exc()
            return False

    def start_wall_following(self, config: WallFollowingConfig) -> bool:
        if self.controller:
            return self.controller.start_wall_following(config)
        return False

    def stop_wall_following(self) -> bool:
        if self.controller:
            return self.controller.stop_wall_following()
        return False

    def move_to_position_realtime(self, x, y, z, velocity=0.1, timeout=10.0, skip_stop=False):
        """实时移动到指定位置 - 使用速度控制，可中断"""
        try:
            if self.controller:
                # ROS2控制方式 - 使用实时控制接口
                return self.controller.move_to_position_realtime(x, y, z, velocity, timeout, skip_stop=skip_stop)
            elif self._scf:
                # 直接控制方式
                if not self.mc:
                    # 如果 mc 未初始化，先初始化
                    self.mc = PositionHlCommander(self._scf, 
                                          default_height=DEFAULT_HEIGHT, 
                                          default_velocity=DEFAULT_VELOCITY)
                
                if not skip_stop:
                    self.mc.stop()  # Bug:
                    #AttributeError: 'PositionHlCommander' object has no attribute 'stop'
                    #PositionHlCommander中没有stop，MotionCommand有但是直接调用会提示没有起飞，需要从起飞开始就要用MotionCommand，没必要
                    time.sleep(0.2)  # 等待停止生效
                self.mc.go_to(x, y, z, velocity=velocity)
                return True
            return False
        except Exception as e:
            print(f"move_to_position_realtime failed: {str(e)}")
            return False

    def shutdown(self):
        """关闭无人机"""
        print(f"HlCommanderCFOperationImpl: shutdown")
        try:
            self.controller.stop()
            return True
        except Exception as e:
            print(f"shutdown failed: {str(e)}")
            import traceback
            traceback.print_exc()
            return False

class DebugLoggingCFOperationImpl(CFOperationStrategy):
    timeSleep = 0.1
    
    def __init__(self, scf=None, debug: bool = False):
        super().__init__(scf=scf, debug=debug)
        
    def activate_idle_simple(self):
        self.printDebug("\tactivate_idle_simple")
        if self._scf and hasattr(self._scf, 'cf'):
            # x=0.0, y=0.0, z=0.0 # initial position
            #controller=CONTROLLER_MELLINGER #=2
            self.mc = PositionHlCommander(self._scf.cf, default_height=DEFAULT_HEIGHT, default_velocity=DEFAULT_VELOCITY)
            activate_mellinger_controller(cf=self._scf.cf)
        time.sleep(self.timeSleep)
        self.printDebug("\tI am idling now!!")
        return True

    def shutdown(self):
        self.printDebug("shutdown")
        if self.mc:
            self.mc.stop()
        return True

    def take_off_simple(self):
        self.printDebug(f"\ttake off simple")
        time.sleep(self.timeSleep)
        self.printDebug("Take off finished : I am hovering now")
        return True

    def landing_simple(self):
        self.printDebug("I am landing now!")
        time.sleep(self.timeSleep)
        self.printDebug("Technically, I am on the ground!")
        return True

    def navigate_to_simple(self, targetPoint: Point3D):
        self.printDebug(f"\tNavigate to: {targetPoint.x}, {targetPoint.y}, {targetPoint.z}")
        time.sleep(self.timeSleep)
        self.printDebug("\tNavigation finished")
        return True
        
