#!/bin/bash
set -euo pipefail

# This script is meant to be run INSIDE the ds-crazyflies container:
# - it uses the ds-crazyflies ROS2 overlay, which provides crazyflie_interfaces/msg Takeoff/Land/GoTo
# - it uses the controller code mounted at /workspace/controller

# The ROS2 setup.bash may read undefined environment variables, so nounset (-u) is disabled while sourcing it
set +u
source /opt/ros/$ROS_DISTRO/setup.bash
source /ds/ds-crazyflies/install/setup.bash
set -u

ROOT="/ds/controller/cf.PyControl"

cd "$ROOT"

# Clean up stale processes left inside the container. Note: with network_mode: host, a port held by
# another process on the host cannot be freed here; start_dssim_docker.sh handles that case on the host.
echo "Cleaning up old controller/http processes in container..."
pkill -f "cf-ctrl-service-ros2.py" 2>/dev/null || true
pkill -f "python3 -m http.server 8080" 2>/dev/null || true
pkill -f "python3 -m http.server 8081" 2>/dev/null || true

# Optional: use a venv (if you keep one inside the repository)
if [ -d "venv" ]; then
  source venv/bin/activate
else
  python3 -m pip install --user -r "$ROOT/requirements_dssim.txt" --break-system-packages
fi

# Prepare the webview directories (same logic as before, but using the in-container mount paths)
echo "Preparing webview folders..."
for id in cf231 cf232 cf233 cf234 cf235 cf236 cf237 cf238; do
  mkdir -p "$ROOT/webview/img/$id"
  rm -f "$ROOT/webview/img/$id/"*.png || true
  echo "1" > "$ROOT/webview/img/$id/latest.txt"
done

# Start the HTTP server used by the webview
cd "$ROOT/webview"
python3 -m http.server 8080 > http_server.log 2>&1 &
HTTP1=$!
python3 -m http.server 8081 > http_server_2.log 2>&1 &
HTTP2=$!

cleanup() {
  echo "Shutting down..."
  kill $HTTP1 2>/dev/null || true
  kill $HTTP2 2>/dev/null || true
  pkill -f "cf-ctrl-service-ros2.py" || true
}
trap cleanup SIGINT SIGTERM

# Start the control service (dssim: topic mode)
cd "$ROOT/src"
python3 cf-ctrl-service-ros2.py --dssim --drone_id cf231 --port 5000 --debug &
P1=$!
python3 cf-ctrl-service-ros2.py --dssim --drone_id cf232 --port 5001 --debug &
P2=$!
python3 cf-ctrl-service-ros2.py --dssim --drone_id cf233 --port 5002 --debug &
P3=$!
python3 cf-ctrl-service-ros2.py --dssim --drone_id cf234 --port 5003 --debug &
P4=$!
python3 cf-ctrl-service-ros2.py --dssim --drone_id cf235 --port 5004 --debug &
P5=$!
python3 cf-ctrl-service-ros2.py --dssim --drone_id cf236 --port 5005 --debug &
P6=$!
python3 cf-ctrl-service-ros2.py --dssim --drone_id cf237 --port 5006 --debug &
P7=$!
python3 cf-ctrl-service-ros2.py --dssim --drone_id cf238 --port 5007 --debug &
P8=$!
wait $P1 $P2 $P3 $P4 $P5 $P6 $P7 $P8

