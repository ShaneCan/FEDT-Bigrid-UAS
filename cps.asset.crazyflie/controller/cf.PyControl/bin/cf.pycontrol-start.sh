#!/bin/bash

# Default arguments for the Python application
URI="radio://0/80/2M/E7E7E7E7E1"
PORT="5000"

# 清理旧的图片文件
rm -f /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/*.png

# ------转发5000端口到8080端口，因为wsl的防火墙限制，只能通过8080端口访问，原生linux系统则不需要------
# 检查并关闭占用8080端口的进程
echo "Check port 8080..."
if lsof -i :8080 > /dev/null; then
    echo "Port 8080 was found to be occupied and is being shut down..."
    lsof -ti :8080 | xargs kill -9 2>/dev/null
    sleep 1
fi

# 启动HTTP服务器（重定向输出到http_server.log）
cd /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview
python3 -m http.server 8080 > http_server.log 2>&1 &
HTTP_SERVER_PID=$!

# 设置清理函数
cleanup() {
    echo "shutdown http server..."
    kill $HTTP_SERVER_PID 2>/dev/null
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM
# ------------------------------------------------------------------------------------------------


cd ../src/
# Run the Python application with default arguments and any extra arguments passed to the script
python3 cf-ctrl-service.py --uri "$URI" --port "$PORT" "$@"