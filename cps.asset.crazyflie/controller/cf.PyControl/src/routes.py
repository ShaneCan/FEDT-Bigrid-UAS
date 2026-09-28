import importlib
import logging
import time
from flask import Blueprint, jsonify, request, current_app
from statemachine import exceptions
from cf_positioning import Point3D
from wall_following import WallFollowingConfig
from werkzeug.routing import FloatConverter, BaseConverter

# Configure the logger
logger = logging.getLogger(__name__)

# Float converter that also accepts negative values
class SignedFloatConverter(FloatConverter):
    regex = r'-?\d+(\.\d+)?'  # matches a signed floating-point number

# Define a blueprint
drone_blueprint = Blueprint('drone', __name__) # Using a Blueprint rather than app keeps all drone-related routes together
# Register the custom converter
drone_blueprint.record(lambda state: state.app.url_map.converters.update(
    signed_float=SignedFloatConverter
))

DEBUG = False

# Generic exception-handling decorator
def handle_exceptions(func):
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except exceptions.TransitionNotAllowed as e:
            logger.error(f"state transition error: {str(e)}", exc_info=True)
            return jsonify({"error": str(e)}), 400
        except Exception as e:
            logger.error(f"Error processing request: {str(e)}", exc_info=True)
            return jsonify({"error": str(e)}), 500
    wrapper.__name__ = func.__name__
    return wrapper

# Print available URLs when the Flask server starts
@drone_blueprint.route('/routes', methods=['GET'])
def routes():
    urls = []
    for rule in current_app.url_map.iter_rules():
        if DEBUG:
            print(f"http://{request.host}{rule}")
        urls.append(f"http://{request.host}{rule}")
    return jsonify({"routes": urls})

# Get the drone instance
def get_drone():
    return current_app.config['DRONE']

@drone_blueprint.route('/install', methods=['POST'])
@handle_exceptions
def install():
    """Installs the drone."""
    drone = get_drone()
    drone.install() # Methods such as .install are state-machine events defined in cf_sm.py
    return jsonify({"message": "Installed", "state": drone.get_current_state()})

@drone_blueprint.route('/start', methods=['POST'])
@handle_exceptions
def start():
    """Starts the drone."""
    drone = get_drone()
    drone.start()
    return jsonify({"message": "Started", "state": drone.get_current_state()})

@drone_blueprint.route('/initialize', methods=['POST'])
@handle_exceptions
def initialize():
    """Initialises the drone."""
    drone = get_drone()
    drone.initialize()
    return jsonify({"message": "Initialized", "state": drone.get_current_state()})

@drone_blueprint.route('/stop', methods=['POST'])
@handle_exceptions
def stop():
    """Stops the drone."""
    drone = get_drone()
    drone.stop()
    return jsonify({"message": "Stopped", "state": drone.get_current_state()})

@drone_blueprint.route('/stop_controller', methods=['POST'])
@handle_exceptions
def stop_controller():
    """Stops the controller setpoints without triggering a state-machine transition."""
    drone = get_drone()
    # Only call the controller's stop method; do not trigger a state transition
    if drone.uavOpStrategyImpl and hasattr(drone.uavOpStrategyImpl, 'controller') and drone.uavOpStrategyImpl.controller is not None:
        success = drone.uavOpStrategyImpl.controller.stop()
        if success:
            return jsonify({"message": "Controller setpoints stopped", "state": drone.get_current_state()})
        else:
            return jsonify({"error": "Failed to stop controller setpoints"}), 500
    else:
        return jsonify({"error": "No controller available"}), 500

@drone_blueprint.route('/uninstall', methods=['POST'])
def uninstall():
    drone = current_app.config['DRONE']
    drone.uninstall()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/activate_idle', methods=['POST'])
@handle_exceptions
def activate_idle():
    """Activates the idle state."""
    drone = get_drone()
    logger.info("Activate idle request processing")
    drone.activate_idle()
    logger.info(f"Activate idle completed, current state: {drone.get_current_state()}")
    return jsonify({"message": "Activated idle", "state": drone.get_current_state()})

@drone_blueprint.route('/begin_takeoff', methods=['POST'])
@handle_exceptions
def begin_takeoff():
    """Begins take-off."""
    drone = get_drone()
    logger.info("Begin takeoff request processing")
    drone.begin_takeoff()
    logger.info(f"Takeoff completed, current state: {drone.get_current_state()}")
    return jsonify({"message": "Takeoff initiated", "state": drone.get_current_state()})

@drone_blueprint.route('/begin_landing', methods=['POST'])
@handle_exceptions
def begin_landing():
    """Begins landing."""
    drone = get_drone()
    logger.info("Begin landing request processing")
    drone.begin_landing()
    logger.info(f"Landing completed, current state: {drone.get_current_state()}")
    return jsonify({"message": "Landing initiated", "state": drone.get_current_state()})

@drone_blueprint.route('/navigate', methods=['POST'])
@handle_exceptions
def navigate_json():
    """Navigates to the position given in the JSON request body."""
    drone = get_drone()
    data = request.json
    
    if not data or not all(k in data for k in ['x', 'y', 'z']):
        return jsonify({"error": "Missing coordinates"}), 400
    
    x, y, z = data['x'], data['y'], data['z']
    point = Point3D(x, y, z)
    drone.targetPointsQueue.append(point)
    
    if drone.current_state == drone.hovering:
        drone.begin_nav_goal_sequence()
    
    return jsonify({
        "message": f"Navigation point added ({x}, {y}, {z})",
        "queue_length": len(drone.targetPointsQueue), 
        "state": drone.get_current_state()
    })

@drone_blueprint.route('/navigate/append/<signed_float:x>/<signed_float:y>/<signed_float:z>', methods=['POST'])
def append_navigation_goal(x, y, z):
    drone = current_app.config['DRONE']
    print(f"REST API: Point3D(x,y,z): Point3D({x},{y},{z})")
    drone.targetPointsQueue.append(Point3D(x,y,z))
    return jsonify({"message": f'Appended Target for Navigation to ({x}, {y}, {z})', "state": drone.get_current_state()})

@drone_blueprint.route('/navigate/append', methods=['POST'])
@handle_exceptions
def append_navigation_goal_json():
    """Adds a waypoint from the JSON request body."""
    drone = get_drone()
    data = request.json
    
    if not data or not all(k in data for k in ['x', 'y', 'z']):
        return jsonify({"error": "Missing coordinates"}), 400
    
    x, y, z = data['x'], data['y'], data['z']
    print(f"REST API: Point3D(x,y,z): Point3D({x},{y},{z})")
    drone.targetPointsQueue.append(Point3D(x, y, z))
    
    return jsonify({
        "message": f'Appended Target for Navigation to ({x}, {y}, {z})', 
        "state": drone.get_current_state()
    })

# Real-time position control endpoint: takes effect immediately and can interrupt the current trajectory
@drone_blueprint.route('/hover/realtime', methods=['POST'])
@handle_exceptions
def hover_realtime():
    """Stops at the current position and hovers, immediately, interrupting the current trajectory."""
    drone = get_drone()
    data = request.json or {}
    
    # The coordinates are optional; without them the drone stops at its current position
    x = data.get('x')
    y = data.get('y')
    z = data.get('z')
    yaw = data.get('yaw', 0.0)
    emergency_stop = data.get('emergency_stop', True)  # emergency stop is enabled by default

    try:
        # 1. If flying and this is not an emergency, force a return to hovering first
        if drone.current_state == drone.flying and not emergency_stop:
            drone.abort_to_hover()
            # Wait for the transition to complete
            time.sleep(0.5)
        
        # 2. Call the real-time hover method. drone.uavOpStrategyImpl abstracts the two control paths; execution continues in cf_drone_ops.py
        success = drone.uavOpStrategyImpl.hover_at_position_realtime(x, y, z, yaw, emergency_stop)
        if success:
            if x is not None and y is not None and z is not None:
                message = f'Real-time hover at ({x}, {y}, {z})'
            else:
                message = 'Real-time hover at current position'
            return jsonify({
                "message": message,
                "state": drone.get_current_state()
            })
        else:
            return jsonify({"error": "Failed to execute real-time hover"}), 500
    except Exception as e:
        logger.error(f"Real-time hover failed: {e}")
        return jsonify({"error": f"Real-time hover failed: {e}"}), 500

# Real-time movement endpoint: velocity controlled and interruptible
@drone_blueprint.route('/move/realtime', methods=['POST'])
@handle_exceptions
def move_realtime():
    """Moves to the given position in real time, using velocity control; interruptible."""
    drone = get_drone()
    data = request.json
    if not data or not all(k in data for k in ['x', 'y', 'z']):
        return jsonify({"error": "Missing coordinates"}), 400

    x, y, z = data['x'], data['y'], data['z']
    velocity = data.get('velocity', 0.3)
    timeout = data.get('timeout', 10.0)
    skip_stop = data.get('skip_stop', False)  # additional parameter

    try:
        # 1. If flying, force a return to hovering first
        if drone.current_state == drone.flying:
            drone.abort_to_hover()
            # Wait for the transition to complete
            time.sleep(0.5)
        
        # 2. Call the real-time movement method
        success = drone.uavOpStrategyImpl.move_to_position_realtime(x, y, z, velocity, timeout, skip_stop)
        if success:
            # 3. Once the real-time move completes, enter the flying state
            # Note: a real-time move executes immediately, so the drone should be flying afterwards
            try:
                # A successful real-time move means the drone is moving, so it should enter the flying state
                if drone.current_state == drone.hovering:
                    # Trigger the transition to flying manually
                    drone.begin_nav_goal_sequence()
                    # Wait for the transition
                    time.sleep(0.2)
                    logger.info(f"Drone {drone._uav_name} entered flying state after real-time move")
            except Exception as state_error:
                logger.warning(f"State transition warning: {state_error}")
            
            return jsonify({
                "message": f'Real-time move to ({x}, {y}, {z})',
                "state": drone.get_current_state()
            })
        else:
            return jsonify({"error": "Failed to execute real-time move"}), 500
    except Exception as e:
        logger.error(f"Real-time move failed: {e}")
        return jsonify({"error": f"Real-time move failed: {e}"}), 500

@drone_blueprint.route('/shutdown_command', methods=['POST'])
def shutdown_command():
    drone = current_app.config['DRONE']
    drone.shutdown_command()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/begin_stopping', methods=['POST'])
def begin_stopping():
    drone = current_app.config['DRONE']
    drone.begin_stopping()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/state', methods=['GET'])
@handle_exceptions
def get_state():
    """Returns the drone status."""
    drone = get_drone()
    return jsonify({
        "state": drone.get_current_state(),
        "transition": drone.get_current_transition(),
        "queue_length": len(drone.targetPointsQueue)
    })

@drone_blueprint.route('/transition', methods=['GET'])
def transition():
    drone = current_app.config['DRONE']
    return jsonify({"transition": drone.get_current_transition()})

@drone_blueprint.route('/status', methods=['GET'])
def status():
    drone = current_app.config['DRONE']
    return jsonify({"state": drone.get_current_state(), "transition": drone.get_current_transition()})


@drone_blueprint.route('/wall_follow/start', methods=['POST'])
@handle_exceptions
def start_wall_follow():
    drone = get_drone()
    data = request.json or {}
    config = WallFollowingConfig.from_request(
        robot_prefix=f"/{drone.uav_name}",
        request_data=data,
        defaults=drone.wall_following_config,
    )
    drone.wall_following_config = config
    drone.begin_wall_following()
    return jsonify({
        "message": "Wall following started",
        "state": drone.get_current_state(),
        "config": config.__dict__
    })


@drone_blueprint.route('/wall_follow/stop', methods=['POST'])
@handle_exceptions
def stop_wall_follow():
    drone = get_drone()
    drone.stop_wall_following()
    return jsonify({
        "message": "Wall following stop requested",
        "state": drone.get_current_state()
    })

@drone_blueprint.route('/navigate/<signed_float:x>/<signed_float:y>/<signed_float:z>', methods=['POST'])
@handle_exceptions
def navigate_direct(x, y, z):
    """Navigates directly to the given position. This route is convenient for manual testing because the command is simpler."""
    drone = get_drone()
    point = Point3D(x, y, z)
    drone.targetPointsQueue.append(point) # /navigate/<float:x>/<float:y>/<float:z> appends the point to the navigation sequence
    
    if drone.current_state == drone.hovering:
        drone.begin_nav_goal_sequence() # begin_nav_goal_sequence is a state-machine event defined in cf_sm.py
    
    return jsonify({
        "message": "Navigation point added",
        "queue_length": len(drone.targetPointsQueue), 
        "state": drone.get_current_state()
    })