import importlib
import logging
import time
from flask import Blueprint, jsonify, request, current_app
from statemachine import exceptions
from cf_positioning import Point3D
from wall_following import WallFollowingConfig
from werkzeug.routing import FloatConverter, BaseConverter

# 设置日志记录器
logger = logging.getLogger(__name__)

# 定义支持负数的浮点数转换器
class SignedFloatConverter(FloatConverter):
    regex = r'-?\d+(\.\d+)?'  # 匹配带符号的浮点数

# Define a blueprint
drone_blueprint = Blueprint('drone', __name__) #使用Blueprint而不是app可以方便将同一类如无人机类的路由组织在一起
# 注册自定义转换器
drone_blueprint.record(lambda state: state.app.url_map.converters.update(
    signed_float=SignedFloatConverter
))

DEBUG = False

# 定义通用异常处理装饰器
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

# 获取无人机实例
def get_drone():
    return current_app.config['DRONE']

@drone_blueprint.route('/install', methods=['POST'])
@handle_exceptions
def install():
    """安装无人机"""
    drone = get_drone()
    drone.install() #这些黑的无法点的方法 如.install, 都是状态机的event，在cf_sm.py中定义
    return jsonify({"message": "Installed", "state": drone.get_current_state()})

@drone_blueprint.route('/start', methods=['POST'])
@handle_exceptions
def start():
    """启动无人机"""
    drone = get_drone()
    drone.start()
    return jsonify({"message": "Started", "state": drone.get_current_state()})

@drone_blueprint.route('/initialize', methods=['POST'])
@handle_exceptions
def initialize():
    """初始化无人机"""
    drone = get_drone()
    drone.initialize()
    return jsonify({"message": "Initialized", "state": drone.get_current_state()})

@drone_blueprint.route('/stop', methods=['POST'])
@handle_exceptions
def stop():
    """停止无人机"""
    drone = get_drone()
    drone.stop()
    return jsonify({"message": "Stopped", "state": drone.get_current_state()})

@drone_blueprint.route('/stop_controller', methods=['POST'])
@handle_exceptions
def stop_controller():
    """停止控制器setpoint（不触发状态机转换）"""
    drone = get_drone()
    # 只调用控制器的stop方法，不触发状态机转换
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
    """激活空闲状态"""
    drone = get_drone()
    logger.info("Activate idle request processing")
    drone.activate_idle()
    logger.info(f"Activate idle completed, current state: {drone.get_current_state()}")
    return jsonify({"message": "Activated idle", "state": drone.get_current_state()})

@drone_blueprint.route('/begin_takeoff', methods=['POST'])
@handle_exceptions
def begin_takeoff():
    """开始起飞"""
    drone = get_drone()
    logger.info("Begin takeoff request processing")
    drone.begin_takeoff()
    logger.info(f"Takeoff completed, current state: {drone.get_current_state()}")
    return jsonify({"message": "Takeoff initiated", "state": drone.get_current_state()})

@drone_blueprint.route('/begin_landing', methods=['POST'])
@handle_exceptions
def begin_landing():
    """开始降落"""
    drone = get_drone()
    logger.info("Begin landing request processing")
    drone.begin_landing()
    logger.info(f"Landing completed, current state: {drone.get_current_state()}")
    return jsonify({"message": "Landing initiated", "state": drone.get_current_state()})

@drone_blueprint.route('/navigate', methods=['POST'])
@handle_exceptions
def navigate_json():
    """通过JSON请求体导航到指定位置"""
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
    """通过JSON请求体添加导航点"""
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

# 新增：实时位置控制接口 - 立即生效，可中断当前轨迹
@drone_blueprint.route('/hover/realtime', methods=['POST'])
@handle_exceptions
def hover_realtime():
    """立即停止在当前位置并悬停 - 立即生效，可中断当前轨迹"""
    drone = get_drone()
    data = request.json or {}
    
    # 坐标是可选的，如果不提供则停止在当前位置
    x = data.get('x')
    y = data.get('y')
    z = data.get('z')
    yaw = data.get('yaw', 0.0)
    emergency_stop = data.get('emergency_stop', True)  # 默认启用紧急停止

    try:
        # 1. 如果当前在flying状态且非紧急模式，先强制回到hovering状态
        if drone.current_state == drone.flying and not emergency_stop:
            drone.abort_to_hover()
            # 等待状态转换完成
            time.sleep(0.5)
        
        # 2. 调用实时悬停方法  drone.uavOpStrategyImpl代表支持两种control方式，下一步转入cf_drone_ops.py中的方法
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

# 新增：实时移动控制接口 - 使用速度控制，可中断
@drone_blueprint.route('/move/realtime', methods=['POST'])
@handle_exceptions
def move_realtime():
    """实时移动到指定位置 - 使用速度控制，可中断"""
    drone = get_drone()
    data = request.json
    if not data or not all(k in data for k in ['x', 'y', 'z']):
        return jsonify({"error": "Missing coordinates"}), 400

    x, y, z = data['x'], data['y'], data['z']
    velocity = data.get('velocity', 0.3)
    timeout = data.get('timeout', 10.0)
    skip_stop = data.get('skip_stop', False)  # 新增参数

    try:
        # 1. 如果当前在flying状态，先强制回到hovering状态
        if drone.current_state == drone.flying:
            drone.abort_to_hover()
            # 等待状态转换完成
            time.sleep(0.5)
        
        # 2. 调用实时移动方法
        success = drone.uavOpStrategyImpl.move_to_position_realtime(x, y, z, velocity, timeout, skip_stop)
        if success:
            # 3. 实时移动完成后，进入flying状态
            # 注意：实时移动是立即执行的，完成后无人机应该处于flying状态
            try:
                # 如果实时移动成功，说明无人机正在移动，应该进入flying状态
                if drone.current_state == drone.hovering:
                    # 手动触发状态转换到flying
                    drone.begin_nav_goal_sequence()
                    # 等待状态转换
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
    """获取无人机状态"""
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
    """直接导航到指定位置，该路由更适合手动测试，因为命令更简单直观"""
    drone = get_drone()
    point = Point3D(x, y, z)
    drone.targetPointsQueue.append(point) # /navigate/<float:x>/<float:y>/<float:z> 会把点直接加入到导航序列中
    
    if drone.current_state == drone.hovering:
        drone.begin_nav_goal_sequence() # begin_nav_goal_sequence 是一个状态机event，在cf_sm.py中定义
    
    return jsonify({
        "message": "Navigation point added",
        "queue_length": len(drone.targetPointsQueue), 
        "state": drone.get_current_state()
    })