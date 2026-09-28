#!/bin/bash

# Clear any stale environment variables
unset PYTHONPATH
unset LD_LIBRARY_PATH

# Set the Python path
export PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python3.10/site-packages:$PYTHONPATH
export PYTHONPATH=/opt/ros/$ROS_DISTRO/local/lib/python3.10/dist-packages:$PYTHONPATH
export PYTHONPATH=/home/crazy/crazyflie_mapping_demo/ros2_ws/install/crazyflie_interfaces/local/lib/python3.10/dist-packages:$PYTHONPATH
export PYTHONPATH=/home/crazy/crazyflie_mapping_demo/ros2_ws/install/crazyflie_py/lib/python3.10/site-packages:$PYTHONPATH
export PYTHONPATH=/home/crazy/crazyflie_mapping_demo/ros2_ws/install/crazyflie_examples/local/lib/python3.10/dist-packages:$PYTHONPATH
export PYTHONPATH=/home/crazy/crazyflie_mapping_demo/ros2_ws/install/crazyflie_sim/local/lib/python3.10/dist-packages:$PYTHONPATH

# Set the library path
export LD_LIBRARY_PATH=/opt/ros/$(ROS_DISTRO)/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/ros/$(ROS_DISTRO)/local/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/crazy/crazyflie_mapping_demo/ros2_ws/install/crazyflie_interfaces/lib:$LD_LIBRARY_PATH

CRAZYFLIE_CPS_PATH=$(pwd)/../../..

# Make sure the ROS2 environment variables are set
source /opt/ros/$(ROS_DISTRO)/setup.bash
source /home/crazy/crazyflie_mapping_demo/ros2_ws/install/setup.bash

# Change into the project directory
cd "$(dirname "$0")/.."

# Activate the Python virtual environment, if present
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Remove the old image files
echo "Cleaning up old image files..."
rm -rf $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf231/*.png
rm -rf $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf232/*.png
rm -rf $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf233/*.png
rm -rf $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf234/*.png
rm -rf $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf235/*.png
rm -rf $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf236/*.png
rm -rf $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf237/*.png
rm -rf $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf238/*.png
mkdir -p $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf231
mkdir -p $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf232
mkdir -p $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf233
mkdir -p $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf234
mkdir -p $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf235
mkdir -p $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf236
mkdir -p $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf237
mkdir -p $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf238

# Initialise the latest.txt files
echo "1" > $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf231/latest.txt
echo "1" > $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf232/latest.txt
echo "1" > $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf233/latest.txt
echo "1" > $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf234/latest.txt
echo "1" > $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf235/latest.txt
echo "1" > $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf236/latest.txt
echo "1" > $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf237/latest.txt
echo "1" > $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview/img/cf238/latest.txt

# ------ Port-forwarding configuration ------
# Find and kill any process occupying the ports
echo "Checking ports..."
for port in 8080 8081 8082 8083 8084 8085 8086 8087 5000 5001 5002 5003 5004 5005 5006 5007; do
    if lsof -i :$port > /dev/null; then
        echo "Port $port was found to be occupied and is being shut down..."
        lsof -ti :$port | xargs kill -9 2>/dev/null
        sleep 1
    fi
done

# Start the HTTP server (output redirected to http_server.log)
cd $CRAZYFLIE_CPS_PATH/controller/cf.PyControl/webview
python3 -m http.server 8080 > http_server.log 2>&1 &
HTTP_SERVER_PID=$!

# Start the HTTP server for the second drone
python3 -m http.server 8081 > http_server_2.log 2>&1 &
HTTP_SERVER_PID_2=$!

# Start the HTTP server for the third drone
python3 -m http.server 8082 > http_server_3.log 2>&1 &
HTTP_SERVER_PID_3=$!

# Start the HTTP server for the fourth drone
python3 -m http.server 8083 > http_server_4.log 2>&1 &
HTTP_SERVER_PID_4=$!

# Start the HTTP server for the fifth drone
python3 -m http.server 8084 > http_server_5.log 2>&1 &
HTTP_SERVER_PID_5=$!

# Start the HTTP server for the sixth drone
python3 -m http.server 8085 > http_server_6.log 2>&1 &
HTTP_SERVER_PID_6=$!

# Start the HTTP server for the seventh drone
python3 -m http.server 8086 > http_server_7.log 2>&1 &
HTTP_SERVER_PID_7=$!

# Start the HTTP server for the eighth drone
python3 -m http.server 8087 > http_server_8.log 2>&1 &
HTTP_SERVER_PID_8=$!    



# Define the cleanup function
cleanup() {
    echo "Shutting down servers..."
    kill $HTTP_SERVER_PID 2>/dev/null
    kill $HTTP_SERVER_PID_2 2>/dev/null
    kill $HTTP_SERVER_PID_3 2>/dev/null
    kill $HTTP_SERVER_PID_4 2>/dev/null
    kill $HTTP_SERVER_PID_5 2>/dev/null
    kill $HTTP_SERVER_PID_6 2>/dev/null
    kill $HTTP_SERVER_PID_7 2>/dev/null
    kill $HTTP_SERVER_PID_8 2>/dev/null
    # Make sure every Python process is cleaned up
    pkill -f "cf-ctrl-service-ros2.py"
    exit 0
}

# Install the signal handlers
trap cleanup SIGINT SIGTERM
# ------------------------------------------------------------------------------------------------

cd ../src/
# Start the ROS2 version of the control service
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

# Wait for all processes
wait $CONTROLLER_PID_1 $CONTROLLER_PID_2 $CONTROLLER_PID_3 $CONTROLLER_PID_4 $CONTROLLER_PID_5 $CONTROLLER_PID_6 $CONTROLLER_PID_7 $CONTROLLER_PID_8

# Start the drone logger
# python3 drone_logger.py &