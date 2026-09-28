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

    uavOpStrategyImpl = None     # Strategy instance, set in the cf-ctrl-service main function; either the ros2 or the scf controller
    _image_numbers = {}  # Per-drone image counter, stored in a dict
    _last_state = None  # Tracks the previous state

    goalReached = False
    isFlying = False
    targetPointsQueue = [] # Target points added through the REST API; the endpoints live in routes.py
    _IS_DEBUG = False
    _graph_lock = threading.Lock()  # Lock
    _last_graph_update = 0  # Timestamp of the last update
    _graph_update_interval = 0.1  # Update interval in seconds

    def __init__(self, drone_id: str, debug: bool = False): 
        # Set every required attribute first
        self._IS_DEBUG = debug
        self._uav_name = drone_id  # use the drone_id that was passed in
        if not self._uav_name:
            raise ValueError("drone_id must be set and cannot be None or empty")
        self.wall_following_config: Optional[WallFollowingConfig] = None
        
        # Initialise the image counter for each drone
        if self._uav_name not in self._image_numbers:
            # Check whether latest.txt exists
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
            
        # Make sure the image directory and latest.txt exist
        img_dir = f"/home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/{self._uav_name}"
        os.makedirs(img_dir, exist_ok=True)
        latest_file = f"{img_dir}/latest.txt"
        if not os.path.exists(latest_file):
            with open(latest_file, 'w') as f:
                f.write(str(self._image_numbers[self._uav_name]))
            
        self.current_transition = None
        
        # Finally, call the superclass initialiser
        super().__init__()

    @property
    def uav_name(self):
        return self._uav_name

    def set_uav_name(self, drone_id):
        if not drone_id:
            raise ValueError("drone_id must be set and cannot be None or empty")
        self._uav_name = drone_id

    def set_uavOpsImpl(self, uavOpsImpl):# uavOpsImpl is set via StateMachineDrone.set_uavOpsImpl in the cf-ctrl-service main function
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
    activate_idle = active.to(idle) # activate_idle is an event; active and idle are states
    begin_takeoff = idle.to(hovering) #cond="reached_Height"
    begin_landing = hovering.to(landed)
    begin_nav_goal_sequence = hovering.to(flying)
    next_nav_goal = flying.to(flying, cond='goal_reached')
    keep_hovering = flying.to(hovering, cond='goal_reached')
    abort_to_hover = flying.to(hovering)  # Forces a return from flying to hovering, unconditionally
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
        """Checks whether the target position has been reached."""
        print("\n" + "="*50)
        print("GOAL REACHED CHECK")
        print("="*50)
        print(f"Current state: {self.current_state}")
        print(f"Current transition: {self.current_transition}")
        print(f"uavOpStrategyImpl exists: {self.uavOpStrategyImpl is not None}")
        
        # First check that uavOpStrategyImpl exists
        if not self.uavOpStrategyImpl:
            print("ERROR: No uavOpStrategyImpl found!")
            print("This should not happen as it should be set during initialization")
            self.goalReached = True
            return True
            
        # Check whether scf or controller is in use
        has_scf = hasattr(self.uavOpStrategyImpl, 'scf') and self.uavOpStrategyImpl.scf is not None
        has_controller = hasattr(self.uavOpStrategyImpl, 'controller') and self.uavOpStrategyImpl.controller is not None
        
        print(f"Using scf: {has_scf}")
        print(f"Using controller: {has_controller}")
        
        if has_scf:
            print("Using scf mode, returning True")
            self.goalReached = True
            return True
            
        if has_controller:
            # Get the current position
            current_pos = self.uavOpStrategyImpl.controller.current_position
            if not current_pos:
                print("No current position available, returning True")
                self.goalReached = True
                return True
            
            # Get the target position
            if len(self.targetPointsQueue) > 0:
                target = self.targetPointsQueue[0]
                print(f"Target point: ({target.x}, {target.y}, {target.z})")
            else:
                print("No target points in queue, returning True")
                self.goalReached = True
                return True
            
            # Compute the distance
            distance = ((current_pos.x - target.x) ** 2 + 
                        (current_pos.y - target.y) ** 2 + 
                        (current_pos.z - target.z) ** 2) ** 0.5
                    
            # The target counts as reached once the distance is below the threshold
            threshold = 1  # in metres (previously 1 m)
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
    # Pre-transition handling
    def before_transition(self, event, state):
        # self.printDebug(f"BT: Before '{event}', on the '{state.id}' state.")
        self.current_transition = event
        self.writeSMGraph()
        
        # self.writeSMGraph()
        return "before_transition_return"


    # This is for executing long-running UAV-actions
    # On-transition handling
    def on_transition(self, event, state):
        """Handles a state transition."""
        self.printDebug(f"OT: On '{event}', on the '{state.id}' state.")
        
        try:
            # if event == "initialize":
            #     if self.uavOpStrategyImpl is None or not self.uavOpStrategyImpl.isSetSCF():
            #         print("Not CF Operation Strategy selected or controller not initialized!")
            #         raise Exception("Not CF Operation Strategy selected!")

            # A curl call triggers the events below; see routes.py
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
                # Note: once the first waypoint is reached, processing continues in after_transition
                if self.uavOpStrategyImpl and len(self.targetPointsQueue) > 0:
                    targetPoint = self.targetPointsQueue.pop(0)
                    success = self.uavOpStrategyImpl.navigate_to_simple(targetPoint) # navigate_to_simple lives in cf_drone_ops.py; under ROS2 it calls the GoTo service
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
                # Stop the current trajectory/stream immediately and force a return to hover
                if self.uavOpStrategyImpl:
                    try:
                        # 1. Stop the current setpoints first
                        if hasattr(self.uavOpStrategyImpl, 'controller') and self.uavOpStrategyImpl.controller is not None:
                            self.uavOpStrategyImpl.controller.stop()
                        # 2. Force the target to count as reached, so the transition is allowed
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
            
            # Render the state-machine diagram
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
    # Post-transition handling
    def after_transition(self, event, state):
        """Handles the state after a transition."""
        self.printDebug(f"AT: After '{event}', on the '{state.id}' state.")
        
        try:
            # When the drone is flying and a next_nav_goal or begin_nav_goal_sequence event arrives
            if (state == self.flying and (event == "next_nav_goal" or event == "begin_nav_goal_sequence")):
                if len(self.targetPointsQueue) > 0:
                    self.printDebug(f"\tNext_Nav_Goal: There are still targets left: {len(self.targetPointsQueue)}")
                    # Wait a moment for the drone to start moving
                    time.sleep(0.5)
                    self.next_nav_goal() # if waypoints remain, go back to on_transition and handle the next one
                else:
                    self.printDebug("\tLet drone hover now, navgoals are empty")
                    self.keep_hovering() # if there are no waypoints left, switch to keep_hovering
            
            # When the drone goes from hovering to flying (possibly triggered by a real-time move)
            if (state == self.flying and event == "begin_nav_goal_sequence"):
                # Check whether this was triggered by a real-time move
                # For a real-time move we would have to wait for it to finish,
                self.printDebug("\tReal-time move initiated, waiting for completion...")
                # but a real-time move executes immediately, so nothing extra is needed here
            
            # When the drone is hovering after coming from the flying state
            if (state == self.hovering and event == "keep_hovering"):
                # landing is not triggered automatically here;
                # the landing decision is made by auto_state_machine.py
                self.printDebug("\tDrone is now hovering, waiting for next command")
                
            # When the drone is landed, emit the landing-complete event
            if (state == self.landed): 
                self.printDebug("\tstate landed reached")
                self.landing_completed()
                
            # Update the state-machine diagram
            self.writeSMGraph()
        except Exception as e:
            print(f"Error in after_transition: {str(e)}")
            import traceback
            traceback.print_exc()
            
        return "after_transition"

    def writeSMGraph(self):
        # Check that _uav_name exists
        if not hasattr(self, '_uav_name'):
            print("Warning: _uav_name not set, skipping graph generation")
            return
            
        current_state = self.current_state.id
        # Only render a new image when the state has actually changed
        if current_state != self._last_state:
            # Generate the state diagram
            smGraph = DotGraphMachine(self)
            # Build the file name from this drone's counter
            image_number = self._image_numbers[self._uav_name]

            # Use the drone ID as the folder name
            img_dir = f"/home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/{self._uav_name}"
            os.makedirs(img_dir, exist_ok=True)
            smGraphPath = f"{img_dir}/uav{image_number}.png"
            
            dot = smGraph()
            dot.write_png(smGraphPath)
            
            # Update this drone's counter
            self._image_numbers[self._uav_name] = (image_number % 100) + 1
            self._last_state = current_state
            
            # Update latest.txt
            latest_file = f"{img_dir}/latest.txt"
            try:
                # Read the current value from latest.txt
                current_latest = 1
                if os.path.exists(latest_file):
                    with open(latest_file, 'r') as f:
                        current_latest = int(f.read().strip())
                
                # Keep the larger of the two counters
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
# Navigation-sequence implementation used by the state machine.
# It navigates through a list of coordinates
# and wraps the atomic drone action navigate_to_simple.
def navigate_to_simple(drone: StateMachineDrone):
    """Executes a navigation command."""
    if not drone.uavOpStrategyImpl:
        drone.printDebug("\tNo UAV operation strategy available")
        return False
        
    if len(drone.targetPointsQueue) == 0:
        drone.printDebug("\tNo navigation targets in queue")
        drone.keep_hovering()
        return False
        
    # Get the target position
    target = drone.targetPointsQueue[0]
    drone.printDebug(f"\tNavigating to: ({target.x:.2f}, {target.y:.2f}, {target.z:.2f})")
    
    # Execute the navigation command
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