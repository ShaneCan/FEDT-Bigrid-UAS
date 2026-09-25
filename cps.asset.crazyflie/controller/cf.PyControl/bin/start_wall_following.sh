#!/bin/bash

# 墙面跟随专用启动脚本
# 基于start.sh，添加墙面跟随状态机Webview服务器

# 清除可能存在的旧环境变量
unset ROS_DISTRO
unset ROS_PYTHON_VERSION
unset PYTHONPATH
unset LD_LIBRARY_PATH

# 设置ROS2环境变量
export ROS_DISTRO=humble
export ROS_PYTHON_VERSION=3.10

# 设置Python路径
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
export PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH
export PYTHONPATH=/home/crazy/crazyflie_mapping_demo/ros2_ws/install/crazyflie_interfaces/local/lib/python3.10/dist-packages:$PYTHONPATH
export PYTHONPATH=/home/crazy/crazyflie_mapping_demo/ros2_ws/install/crazyflie_py/lib/python3.10/site-packages:$PYTHONPATH
export PYTHONPATH=/home/crazy/crazyflie_mapping_demo/ros2_ws/install/crazyflie_examples/local/lib/python3.10/dist-packages:$PYTHONPATH
export PYTHONPATH=/home/crazy/crazyflie_mapping_demo/ros2_ws/install/crazyflie_sim/local/lib/python3.10/dist-packages:$PYTHONPATH

# 设置库路径
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/local/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/crazy/crazyflie_mapping_demo/ros2_ws/install/crazyflie_interfaces/lib:$LD_LIBRARY_PATH

# 确保ROS2环境变量已设置
source /opt/ros/humble/setup.bash
source /home/crazy/crazyflie_mapping_demo/ros2_ws/install/setup.bash

# 进入项目目录
cd "$(dirname "$0")/.."

# 激活Python虚拟环境（如果存在）
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# 清理旧的图片文件
echo "Cleaning up old image files..."
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf231/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf232/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf233/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf234/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf235/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf236/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf237/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf238/*.png

# 清理墙面跟随状态机图片文件
echo "Cleaning up wall following state machine image files..."
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf231/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf232/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf233/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf234/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf235/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf236/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf237/*.png
rm -rf /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf238/*.png

# 创建图片目录
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf231
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf232
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf233
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf234
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf235
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf236
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf237
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf238

# 创建墙面跟随状态机图片目录
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf231
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf232
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf233
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf234
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf235
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf236
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf237
mkdir -p /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf238

# 初始化latest.txt文件
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf231/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf232/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf233/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf234/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf235/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf236/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf237/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/cf238/latest.txt

# 初始化墙面跟随状态机latest.txt文件
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf231/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf232/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf233/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf234/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf235/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf236/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf237/latest.txt
echo "1" > /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/wall_following/img/cf238/latest.txt

# ------端口配置------
# 检查并关闭占用端口的进程
echo "Checking ports..."
for port in 8080 8081 8082 8083 8084 8085 8086 8087 5000 5001 5002 5003 5004 5005 5006 5007 9000 9001 9002 9003 9004 9005 9006 9007; do
    if lsof -i :$port > /dev/null; then
        echo "Port $port was found to be occupied and is being shut down..."
        lsof -ti :$port | xargs kill -9 2>/dev/null
        sleep 1
    fi
done

# 启动通用状态机HTTP服务器（重定向输出到http_server.log）
cd /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview
python3 -m http.server 8080 > http_server.log 2>&1 &
HTTP_SERVER_PID=$!

# 启动第二个HTTP服务器用于第二个无人机
python3 -m http.server 8081 > http_server_2.log 2>&1 &
HTTP_SERVER_PID_2=$!

# 启动第三个HTTP服务器用于第三个无人机
python3 -m http.server 8082 > http_server_3.log 2>&1 &
HTTP_SERVER_PID_3=$!

# 启动第四个HTTP服务器用于第四个无人机
python3 -m http.server 8083 > http_server_4.log 2>&1 &
HTTP_SERVER_PID_4=$!

# 启动第五个HTTP服务器用于第五个无人机
python3 -m http.server 8084 > http_server_5.log 2>&1 &
HTTP_SERVER_PID_5=$!

# 启动第六个HTTP服务器用于第六个无人机
python3 -m http.server 8085 > http_server_6.log 2>&1 &
HTTP_SERVER_PID_6=$!

# 启动第七个HTTP服务器用于第七个无人机
python3 -m http.server 8086 > http_server_7.log 2>&1 &
HTTP_SERVER_PID_7=$!

# 启动第八个HTTP服务器用于第八个无人机
python3 -m http.server 8087 > http_server_8.log 2>&1 &
HTTP_SERVER_PID_8=$!

# 启动墙面跟随状态机Webview服务器
echo "Starting wall following state machine webview servers..."
cd /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/src

# 启动墙面跟随Webview服务器 - cf231
python3 wall_following_webview_server.py --host 127.0.0.1 --port 9000 > ../webview/wall_following_webview_1.log 2>&1 &
WALL_FOLLOWING_WEBVIEW_PID_1=$!

# 启动墙面跟随Webview服务器 - cf232
python3 wall_following_webview_server.py --host 127.0.0.1 --port 9001 > ../webview/wall_following_webview_2.log 2>&1 &
WALL_FOLLOWING_WEBVIEW_PID_2=$!

# 启动墙面跟随Webview服务器 - cf233
python3 wall_following_webview_server.py --host 127.0.0.1 --port 9002 > ../webview/wall_following_webview_3.log 2>&1 &
WALL_FOLLOWING_WEBVIEW_PID_3=$!

# 启动墙面跟随Webview服务器 - cf234
python3 wall_following_webview_server.py --host 127.0.0.1 --port 9003 > ../webview/wall_following_webview_4.log 2>&1 &
WALL_FOLLOWING_WEBVIEW_PID_4=$!

# 启动墙面跟随Webview服务器 - cf235
python3 wall_following_webview_server.py --host 127.0.0.1 --port 9004 > ../webview/wall_following_webview_5.log 2>&1 &
WALL_FOLLOWING_WEBVIEW_PID_5=$!

# 启动墙面跟随Webview服务器 - cf236
python3 wall_following_webview_server.py --host 127.0.0.1 --port 9005 > ../webview/wall_following_webview_6.log 2>&1 &
WALL_FOLLOWING_WEBVIEW_PID_6=$!

# 启动墙面跟随Webview服务器 - cf237
python3 wall_following_webview_server.py --host 127.0.0.1 --port 9006 > ../webview/wall_following_webview_7.log 2>&1 &
WALL_FOLLOWING_WEBVIEW_PID_7=$!

# 启动墙面跟随Webview服务器 - cf238
python3 wall_following_webview_server.py --host 127.0.0.1 --port 9007 > ../webview/wall_following_webview_8.log 2>&1 &
WALL_FOLLOWING_WEBVIEW_PID_8=$!

# 设置清理函数
cleanup() {
    echo "Shutting down servers..."
    # 关闭通用状态机HTTP服务器
    kill $HTTP_SERVER_PID 2>/dev/null
    kill $HTTP_SERVER_PID_2 2>/dev/null
    kill $HTTP_SERVER_PID_3 2>/dev/null
    kill $HTTP_SERVER_PID_4 2>/dev/null
    kill $HTTP_SERVER_PID_5 2>/dev/null
    kill $HTTP_SERVER_PID_6 2>/dev/null
    kill $HTTP_SERVER_PID_7 2>/dev/null
    kill $HTTP_SERVER_PID_8 2>/dev/null
    
    # 关闭墙面跟随Webview服务器
    kill $WALL_FOLLOWING_WEBVIEW_PID_1 2>/dev/null
    kill $WALL_FOLLOWING_WEBVIEW_PID_2 2>/dev/null
    kill $WALL_FOLLOWING_WEBVIEW_PID_3 2>/dev/null
    kill $WALL_FOLLOWING_WEBVIEW_PID_4 2>/dev/null
    kill $WALL_FOLLOWING_WEBVIEW_PID_5 2>/dev/null
    kill $WALL_FOLLOWING_WEBVIEW_PID_6 2>/dev/null
    kill $WALL_FOLLOWING_WEBVIEW_PID_7 2>/dev/null
    kill $WALL_FOLLOWING_WEBVIEW_PID_8 2>/dev/null
    
    # 确保所有Python进程都被清理
    pkill -f "cf-ctrl-service-ros2.py"
    pkill -f "wall_following_webview_server.py"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 启动ROS2版本的控制服务
echo "Starting ROS2 control services..."
cd /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/src

# 启动ROS2版本的控制服务
python3 cf-ctrl-service-ros2.py --drone_id cf231 --port 5000 --debug &
CONTROLLER_PID_1=$!

python3 cf-ctrl-service-ros2.py --drone_id cf232 --port 5001 --debug &
CONTROLLER_PID_2=$!

python3 cf-ctrl-service-ros2.py --drone_id cf233 --port 5002 --debug &
CONTROLLER_PID_3=$!

python3 cf-ctrl-service-ros2.py --drone_id cf234 --port 5003 --debug &
CONTROLLER_PID_4=$!

python3 cf-ctrl-service-ros2.py --drone_id cf235 --port 5004 --debug &
CONTROLLER_PID_5=$!

python3 cf-ctrl-service-ros2.py --drone_id cf236 --port 5005 --debug &
CONTROLLER_PID_6=$!

python3 cf-ctrl-service-ros2.py --drone_id cf237 --port 5006 --debug &
CONTROLLER_PID_7=$!

python3 cf-ctrl-service-ros2.py --drone_id cf238 --port 5007 --debug &
CONTROLLER_PID_8=$!

# 等待所有进程
echo "All services started successfully!"
echo ""
echo "=== Addresses of services ==="
echo "general SM Webview:"
echo "  cf231: http://127.0.0.1:8080"
echo "  cf232: http://127.0.0.1:8081"
echo "  cf233: http://127.0.0.1:8082"
echo "  cf234: http://127.0.0.1:8083"
echo "  cf235: http://127.0.0.1:8084"
echo "  cf236: http://127.0.0.1:8085"
echo "  cf237: http://127.0.0.1:8086"
echo "  cf238: http://127.0.0.1:8087"
echo ""
echo "wall following Webview:"
echo "  cf231: http://127.0.0.1:9000/webview.html"
echo "  cf232: http://127.0.0.1:9001/webview.html"
echo "  cf233: http://127.0.0.1:9002/webview.html"
echo "  cf234: http://127.0.0.1:9003/webview.html"
echo "  cf235: http://127.0.0.1:9004/webview.html"
echo "  cf236: http://127.0.0.1:9005/webview.html"
echo "  cf237: http://127.0.0.1:9006/webview.html"
echo "  cf238: http://127.0.0.1:9007/webview.html"
echo ""
echo "REST API services:"
echo "  cf231: http://127.0.0.1:5000"
echo "  cf232: http://127.0.0.1:5001"
echo "  cf233: http://127.0.0.1:5002"
echo "  cf234: http://127.0.0.1:5003"
echo "  cf235: http://127.0.0.1:5004"
echo "  cf236: http://127.0.0.1:5005"
echo "  cf237: http://127.0.0.1:5006"
echo "  cf238: http://127.0.0.1:5007"
echo ""
echo "Press Ctrl+C to stop all services"

wait $CONTROLLER_PID_1 $CONTROLLER_PID_2 $CONTROLLER_PID_3 $CONTROLLER_PID_4 $CONTROLLER_PID_5 $CONTROLLER_PID_6 $CONTROLLER_PID_7 $CONTROLLER_PID_8
