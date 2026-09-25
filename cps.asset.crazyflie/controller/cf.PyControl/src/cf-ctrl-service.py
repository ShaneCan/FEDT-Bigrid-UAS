#!venv/bin/python
import logging
import sys
import time
from typing import Optional
from threading import Event
import threading
import argparse
import asyncio
import websockets
import json

from flask import Flask, jsonify, request
from routes import drone_blueprint  # Import the blueprint
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

import cf_positioning
from cf_positioning import Point3D, PositionEstimationStrategy, KalmanEstimatePositionStrategy, StateEstimatePositionStrategy
from cf_drone_ops import CFOperationStrategy, HlCommanderCFOperationImpl, DebugLoggingCFOperationImpl
import cf_sm
from cf_logger import FileLogger

# CF1 = "radio://0/80/2M/E7E7E7E7E1" # cf uri (local radio)
# CF1 = "usb://0" # cf uri (local usb)
# CF1 = "radio://0/80/2M/E7E7E7E701" #cf uri (simulation)
# URI = uri_helper.uri_from_env(default=CF1)

#################################################################################

def create_arg_parser():
    # Create the parser.
    parser = argparse.ArgumentParser(description='Crazyflie 2.x RESTful Drone Controller Service')
    
    # Add the arguments.
    parser.add_argument('--host', type=str, default='0.0.0.0', help='The host of the web server.')
    parser.add_argument('--port', type=int, default=5000, help='Port of the web server.')
    parser.add_argument('--uri', type=str, help='The URI of the crazyflie 2.x drone.')
    parser.add_argument('--sim', action='store_true', help='Specify whether you want to run this controller with sim_cf2.')
    parser.add_argument('--debug', action='store_true', help='Outputs many additional debug messages to the console.')
    parser.add_argument('--logging', action='store_true', help='Add a logger that writes CF states to a file (e.g., position estimates).')
    ws_group = parser.add_argument_group('WebSocket settings')
    ws_group.add_argument('--wsendpoint', action='store_true', help='Add a websocket that publishes CF state (e.g., position/accel/... estimates).')
    ws_group.add_argument('--wshost', type=str, default='0.0.0.0', help='The host of the websocket server.')
    ws_group.add_argument('--wsport', type=int, default=8765, help='Port of the websocket server.')

    return parser

#################################################################################

global drone
SIM_MODE = True
DEBUG = False
LOGGING = False
global ISRUNNING # MainAppLoop
ISRUNNING = True
lock = threading.Lock()

logger = None # FileLogger(f'./log_position-estimate-{POSITION_ESTIMATE_FILTER}.txt')
loggingPeriod_in_ms = 100 #ms
logging.basicConfig(level=logging.ERROR)

global positionEstimator
positionEstimator = None # KalmanEstimatePositionStrategy(), StateEstimatePositionStrategy()
POSITIONING_SYSTEM = "LPS" #"bcFlow2" # "LPS" # "bcFlow2"
POSITION_ESTIMATE_FILTER = "kalman" # "state" # "kalman"

websocketserver_started = threading.Event() # webserver signal
flask_started = threading.Event() # webserver signal
deck_attached_event = threading.Event()

def LOG(msg: str):
    global logger, LOGGING
    if(LOGGING and logger != None):
        logger.log(msg)

#################################################################################

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


def logCallback_isFlying(timestamp, data, logconf):
    global drone
    global positionEstimator

    isMovedByHand = (abs(data['acc.x']) > 0.04 or abs(data['acc.y']) > 0.04)
    

    # print(drone.get_current_state())
    # A simple heuristic to determine if the drone is moving based on acceleration
    if abs(data['acc.x']) > 0.1 or abs(data['acc.y']) > 0.1: # or abs(data['acc.z']) > 0.1:
        drone.isFlying = True
    else:
        drone.isFlying = False

    LOG(f"[{drone.get_current_state()}]: {positionEstimator.position_estimate}")
    if drone.get_current_state() == "flying" or (isMovedByHand and (drone.get_current_state() == "idle" or drone.get_current_state() == "active")):
        print(f"\t--[S({drone.get_current_state()})]: Position Estimate={positionEstimator.position_estimate}")

#################################################################################

## Flask Webserver
def start_flask_app(app, host, port):
    flask_started.set()
    # DEV
    # app.run(host=host, port=port)
    # PROD
    from waitress import serve
    serve(app, host=host, port=port)

def before_request_callback():
    global drone
    drone.writeSMGraph()

# After each fully completed request: this is the final one: Here we send ACK that everything was OK. UAV OPERATION COMPLETED
def after_request_callback(response):
    # print("After request callback executed.")
    global drone
    drone.writeSMGraph()
    return response

# Custom converter to handle negative float values
class FloatConverter(BaseConverter):
    regex = r'-?\d+(\.\d+)?'

    def to_python(self, value):
        try:
            return float(value)
        except ValueError:
            raise ValidationError()

    def to_url(self, value):
        return str(value)

#################################################################################

async def send_pos_data(websocket, path):
    global positionEstimator
    while True:
        data = {
            "message": "Hello from server",
            "value": ("%s" % positionEstimator.position_estimate),  # Simulate random data
        }
        await websocket.send(json.dumps(data))
        await asyncio.sleep(1)  # Send data every second

def start_websocket_server(host, port):
    async def server():
        websocketserver_started.set()
        async with websockets.serve(send_pos_data, host, port):
            print(f"WebSocket server started on ws://{host}:{port}")
            await asyncio.Future()  # Keeps running the server
        
    # Launching the asyncio event loop inside the thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(server())

#################################################################################

def cleanup():
    # Stop MotionCommander
    global drone;
    try:
        if (drone.uavOpStrategyImpl.mc != None):
            drone.uavOpStrategyImpl.mc.stop()
    except:
        print(f"--[{drone.get_current_state()}] Quitting program: exception when stopping motor")

###################################主函数运行顺序###################################
# 1. 基础设置
# - 解析参数
# - 设置日志
# - 创建状态机

# 2. 驱动初始化
# - 设置重试参数
# - 初始化驱动

# 3. URI设置
# - 获取无人机地址

# 4. 状态机转换
# - 安装软件
# - 启动状态机

# 5. Flask服务器
# - 创建应用
# - 启动服务

# 6. 无人机连接
# - 建立连接
# - 配置参数
# - 设置策略

if __name__ == '__main__':
    parser = create_arg_parser()
    args = parser.parse_args()
    SIM_MODE = args.sim
    DEBUG = args.debug
    LOGGING = args.logging
    STARTWSSERVER = args.wsendpoint
    #print("Arguments supplied: %s" % args)

    if (LOGGING):
        logFile = f'./log_position-estimate-{POSITION_ESTIMATE_FILTER}.txt'
        logger = FileLogger(logFile)
        logger.close()
        print(f"Position estimate log file is {logFile}")

    # Instantiate the state machine
    uav_name = "uav1"
    drone = cf_sm.StateMachineDrone(drone_id=uav_name, debug=True)  # 强制设置debug=True
    # 生成当前状态下的状态机可视化图表
    drone.writeSMGraph()
    print(f"--[{drone.get_current_state()}] cflib.crtp.init_drivers() ...")
    print(f"--[{drone.get_current_state()}] SIM_MODE = {SIM_MODE}")
    # Initialize the low-level drivers
    # 设置重试次数
    cflib.crtp.radiodriver.set_retries_before_disconnect(1500)
    cflib.crtp.radiodriver.set_retries(3)
    if(SIM_MODE):
        cflib.crtp.init_drivers(enable_sim_driver=True)
    else:
        cflib.crtp.init_drivers()
    
    # 设置无人机URI
    URI = None
    if args.uri:
        # 如果指定了URI，使用指定的URI
        cfURI = args.uri
        URI = uri_helper.uri_from_env(default=cfURI)
    else:
        # 如果未指定URI，扫描所有接口
        print("No CF URI specified. Scanning interfaces now ...")
        found = False
        for a in range(7):
            available = cflib.crtp.scan_interfaces(0xe7e7e7e700 + a)
            if(len(available) > 0):
                print('Crazyflies found %s ... taking first one:' % len(available))
                for i in available:
                    # 将接口地址转换为URI格式
                    URI = uri_helper.uri_from_env(default=i[0])
                    print(i[0])
                    found = True
                    break # 找到第一个就退出内层循环
            if(found): break
        if(found == False): 
            print("No interfaces found ...")
            exit(0)
    print(f"--[{drone.get_current_state()}] URI of drone = {URI}")

    # Installation
    print(f"--[{drone.get_current_state()}] Installing software packages now ...")
    drone.install()
    drone.writeSMGraph() 

    #TODO wrap in lambda function: tryRepeat(lambda: Function(Void) -> {}, maxFailCnt) # 尝试启动，最多重试3次
    failCnt = 0;
    failCntMax = 3;
    while(drone.current_state != drone.starting):
        try:
            drone.start()   # !!!I changed squence of these two lines
            drone.writeSMGraph()
        except:
            print(f"--[{drone.get_current_state()}] Try restart ...")
            drone.writeSMGraph()
            time.sleep(1)
            if(failCnt > failCntMax):
                print(f"--[{drone.get_current_state()}] FailCounter Max reached")
                sys.exit()
    drone.writeSMGraph()
    print(f"--[{drone.get_current_state()}] Dependencies resolved.")

    # Start Flask app in a separate thread
    print(f"--[{drone.get_current_state()}] Starting Flask Command WebServer now ...")
    app = Flask(__name__)
    app.register_blueprint(drone_blueprint)  # Register the blueprint with the app
    app.config['DRONE'] = drone
    # 配置静态文件目录
    app.static_folder = '../webview'
    app.static_url_path = ''
    # Register the request callback. 当调用服务时，Flask会自动执行所有注册的回调
    app.after_request(after_request_callback)
    app.before_request(before_request_callback) # app.before_request(lambda: before_request_callback(callback_argument))
    # Register the custom converter
    app.url_map.converters['float'] = FloatConverter
    # 启动Flask app
    flask_thread = threading.Thread(target=start_flask_app, args=(app, args.host, args.port))
    flask_thread.daemon = True # Allows the thread to exit when the main program does
    flask_thread.start()
    # Wait until Flask has fully started
    flask_started.wait()
    print(f"--[{drone.get_current_state()}] Flask WebServer started [OK]")

    #TODO tryRepeat(lambda: ) 我放到了上面来，放到下面有bug无法转换进入，什么原因？
    drone.initialize()
    drone.writeSMGraph()
    print(f"--[{drone.get_current_state()}] Hardware checks completed.")
    

    # "Main Loop"
    print(f"--[{drone.get_current_state()}] Connecting to drone now ...")
    
    # 创建策略实例
    print("\n=== Creating Drone Operation Implementation ===")
    print(f"DEBUG mode: {DEBUG}")
    droneOpsImpl = HlCommanderCFOperationImpl(scf=None, controller=None, debug=True)  # 先创建实例，scf设为None
    print(f"Created droneOpsImpl: {droneOpsImpl}")
    
    # 设置到状态机
    print("\n=== Setting uavOpStrategyImpl to State Machine ===")
    print(f"Current drone state: {drone.get_current_state()}")
    drone.set_uavOpsImpl(droneOpsImpl)
    print(f"State machine uavOpStrategyImpl after setting: {drone.uavOpStrategyImpl}")
    print(f"Current drone state: {drone.get_current_state()}")
    print("=== End of Implementation Setup ===\n")
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # 更新策略实例的scf
        print("\n=== Updating Drone Operation Implementation with scf ===")
        droneOpsImpl._scf = scf
        #droneOpsImpl.scf = scf  如果scf有问题记得改回来，和cf_drone_ops中的def isSetSCF(self)对应
        print(f"Updated droneOpsImpl with scf: {droneOpsImpl}")
        print(f"scf: {scf}")
        print("=== End of scf Update ===\n")

        if SIM_MODE == False:
            if(POSITIONING_SYSTEM == "bcFlow2" or POSITIONING_SYSTEM == "LPS|bcFlow2"):
                scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                            cb=param_deck_flow)
                if not deck_attached_event.wait(timeout=5):
                    print(f'--[{drone.get_current_state()}] No flow deck detected!')
                    sys.exit(1)
            if POSITIONING_SYSTEM == "LPS" or POSITIONING_SYSTEM == "LPS|bcFlow2":
                print(f"--[{drone.get_current_state()}] cf_positioning.reset_estimator()")
                cf_positioning.reset_estimator(scf)

        if SIM_MODE == True:
            print(f"--[{drone.get_current_state()}] cf_positioning.reset_estimator()")
            cf_positioning.reset_estimator(scf)
            
        # Create a logging configuration
        print(f"--[{drone.get_current_state()}] Starting State Logger Monitor")
        print(f"--[{drone.get_current_state()}] POSITIONING_SYSTEM: {POSITIONING_SYSTEM}")
        print(f"--[{drone.get_current_state()}] POSITION ESTIMATE FILTER: {POSITION_ESTIMATE_FILTER}")
        # Position
        logConfig_Pos = None
        positionEstimator = KalmanEstimatePositionStrategy()
        if POSITION_ESTIMATE_FILTER == "state":
            positionEstimator = StateEstimatePositionStrategy()
        logConfig_Pos = LogConfig(name='Position', period_in_ms=loggingPeriod_in_ms)
        positionEstimator.add_variables(logConfig_Pos)
        logConfig_Pos.data_received_cb.add_callback(positionEstimator.estimatePositionLogCallback)
        print(f"--[{drone.get_current_state()}] Start logging interface to get state estimates (position/acceleration)")
        scf.cf.log.add_config(logConfig_Pos)
        logConfig_Pos.start() # Start logging
        # Wait until the first position estimate is received
        if not positionEstimator.position_estimate_event.wait(timeout=5):
            print(f'--[{drone.get_current_state()}] No position estimate received!')
            sys.exit(1)
        print(f"--[{drone.get_current_state()}] Got Position State Estimate: {positionEstimator.position_estimate}")
        # Acceleration
        logConfig_Acc = LogConfig(name='Stabilizer', period_in_ms=loggingPeriod_in_ms)
        logConfig_Acc.add_variable('acc.x', 'float')
        logConfig_Acc.add_variable('acc.y', 'float')
        logConfig_Acc.add_variable('acc.z', 'float')
        logConfig_Acc.data_received_cb.add_callback(logCallback_isFlying)
        scf.cf.log.add_config(logConfig_Acc)
        logConfig_Acc.start() # Start logging

        # 等待状态机转换到ACTIVE状态
        max_wait_time = 5  # 最大等待时间5秒
        start_time = time.time()
        while drone.current_state != drone.active:
            if time.time() - start_time > max_wait_time:
                print(f"--[{drone.get_current_state()}] Timeout waiting for state machine to become ACTIVE")
                sys.exit(1)
            time.sleep(0.1)
        print(f"--[{drone.get_current_state()}] State machine successfully initialized to ACTIVE state")
        
        if STARTWSSERVER == True:
            server_thread = threading.Thread(target=start_websocket_server, args=(args.wshost, args.wsport))
            server_thread.daemon = True  # Allows the thread to exit when the main program does
            server_thread.start()
            # Wait until websocket server has fully started
            websocketserver_started.wait()
            print(f"--[{drone.get_current_state()}] WebSocket started")

        # Main Loop
        print(f"--[{drone.get_current_state()}] The drone is ready to take commands.")
        print(f"--[{drone.get_current_state()}] Check available commands here:")
        print(f"--[{drone.get_current_state()}] http://{args.host}:{args.port}/routes")
        
        try:
            # Keep the main thread alive to keep the Crazyflie connection open 让主线程卡在while循环不退出
            # 其他线程可以在这个时候执行：
            # 处理HTTP请求
            # 处理WebSocket数据
            while ISRUNNING:
                time.sleep(0.1)
        except KeyboardInterrupt:
            cleanup()
        
        # Shutdown
        # Stop logging
        logConfig_Acc.stop()
        logConfig_Pos.stop()
        
        # Finish
        cleanup()
