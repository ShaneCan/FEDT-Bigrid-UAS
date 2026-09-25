#!/bin/bash
set -euo pipefail

# 这个脚本设计为“在 ds-crazyflies 容器内运行”：
# - 使用 ds-crazyflies 的 ROS2 overlay（包含 crazyflie_interfaces/msg 的 Takeoff/Land/GoTo）
# - 使用挂载进来的 /workspace/controller 代码

# ROS2 的 setup.bash 里可能会读取未定义环境变量；因此 source 时临时关闭 nounset(-u)
set +u
source /opt/ros/$ROS_DISTRO/setup.bash
source /ds/ds-crazyflies/install/setup.bash
set -u

ROOT="/ds/controller/cf.PyControl"

cd "$ROOT"

# 清理旧进程（容器内残留）。注意：network_mode: host 下如果端口被“宿主机其它进程”占用，这里杀不到；
# 这种情况由宿主机脚本 start_dssim_docker.sh 负责先清理。
echo "Cleaning up old controller/http processes in container..."
pkill -f "cf-ctrl-service-ros2.py" 2>/dev/null || true
pkill -f "python3 -m http.server 8080" 2>/dev/null || true
pkill -f "python3 -m http.server 8081" 2>/dev/null || true

# 可选：使用 venv（若你愿意把 venv 放在 repo 内）
if [ -d "venv" ]; then
  source venv/bin/activate
else
  python3 -m pip install --user -r "$ROOT/requirements_dssim.txt" --break-system-packages
fi

# 准备 webview 目录（沿用你现有逻辑，但路径换成容器内挂载路径）
echo "Preparing webview folders..."
for id in cf231 cf232 cf233 cf234 cf235 cf236 cf237 cf238; do
  mkdir -p "$ROOT/webview/img/$id"
  rm -f "$ROOT/webview/img/$id/"*.png || true
  echo "1" > "$ROOT/webview/img/$id/latest.txt"
done

# 启动 HTTP server（用于 webview）
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

# 启动控制服务（dssim：topic 模式）
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

