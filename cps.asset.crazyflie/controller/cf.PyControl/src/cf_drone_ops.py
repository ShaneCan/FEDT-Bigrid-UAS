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

class CFOperationStrategy(ABC): # Abstract class, implemented below by HlCommanderCFOperationImpl
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


class HlCommanderCFOperationImpl(CFOperationStrategy): # Extends CFOperationStrategy and implements its abstract methods
    """Strategy-pattern implementation used by the state machine. It wraps ROS2CrazyflieController
    behind a uniform interface so the scf and ROS2 controllers can be swapped easily. The instance is
    cf_sm.StateMachineDrone.uavOpStrategyImpl."""

    def __init__(self, scf=None, controller=None, debug: bool = False):   # either scf or controller must be supplied
     # Passing scf uses the official SyncCrazyflie library over USB; passing controller uses the ROS2 controller  
        super().__init__(scf=scf, debug=debug)
        self.controller = controller

    def isSetSCF(self):
        """Checks whether a Crazyflie controller has been set."""
        return self.controller is not None or self._scf is not None
        #return self.controller is not None or self.scf is not None


    def activate_idle_simple(self):
        """Activates the idle state."""
        print(f"HlCommanderCFOperationImpl: activate_idle_simple")
        return True

    def take_off_simple(self):
        try:
            if self.controller:
                # ROS2 control path
                self.controller.take_off()
            elif self._scf:
                # Direct control path
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
                # ROS2 control path
                self.controller.land()
            elif self._scf:
                # Direct control path
                self.mc.land()
            return True
        except Exception as e:
            print(f"landing failed: {str(e)}")
            return False

    def navigate_to_simple(self, targetPoint):
        try:
            if self.controller:
                # ROS2 control path
                self.controller.go_to(targetPoint.x, targetPoint.y, targetPoint.z)
            elif self._scf:
                # Direct control path
                self.mc.go_to(targetPoint.x, targetPoint.y, targetPoint.z)
            return True
        except Exception as e:
            print(f"navigation failed: {str(e)}")
            return False

    def hover_at_position_realtime(self, x=None, y=None, z=None, yaw=0.0, emergency_stop=True):
        """Stops at the current position and hovers, immediately, interrupting the current trajectory."""
        try:
            print(f"hover_at_position_realtime called: controller={self.controller is not None}, scf={self._scf is not None}, emergency_stop={emergency_stop}")
            
            if self.controller:
                # ROS2 control path - stop at the current position
                print("Using ROS2 controller for hover")
                return self.controller.hover_at_position(x, y, z, yaw, emergency_stop)
            elif self._scf:
                # Direct control path - stop at the current position
                print(f"Using SCF controller for hover, mc={self.mc is not None}")
                if not self.mc:
                    # Initialise mc if it has not been initialised yet
                    print("Initializing PositionHlCommander...")
                    self.mc = PositionHlCommander(self._scf, 
                                          default_height=DEFAULT_HEIGHT, 
                                          default_velocity=DEFAULT_VELOCITY)
                    print("PositionHlCommander initialized successfully")
                
                print("Calling mc.stop()...")
                self.mc.stop()  # stop the current motion and stay at the current position
                if emergency_stop:
                    time.sleep(0.1)  # emergency mode: shorter wait
                else:
                    time.sleep(0.2)  # normal mode
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
        """Moves to the given position in real time, using velocity control; interruptible."""
        try:
            if self.controller:
                # ROS2 control path - use the real-time control interface
                return self.controller.move_to_position_realtime(x, y, z, velocity, timeout, skip_stop=skip_stop)
            elif self._scf:
                # Direct control path
                if not self.mc:
                    # Initialise mc if it has not been initialised yet
                    self.mc = PositionHlCommander(self._scf, 
                                          default_height=DEFAULT_HEIGHT, 
                                          default_velocity=DEFAULT_VELOCITY)
                
                if not skip_stop:
                    self.mc.stop()  # Bug:
                    #AttributeError: 'PositionHlCommander' object has no attribute 'stop'
                    # PositionHlCommander has no stop; MotionCommander does, but calling it directly reports that the drone has not taken off - it would have to be used from take-off onwards, which is not worth it
                    time.sleep(0.2)  # wait for the stop to take effect
                self.mc.go_to(x, y, z, velocity=velocity)
                return True
            return False
        except Exception as e:
            print(f"move_to_position_realtime failed: {str(e)}")
            return False

    def shutdown(self):
        """Shuts the drone down."""
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
        
