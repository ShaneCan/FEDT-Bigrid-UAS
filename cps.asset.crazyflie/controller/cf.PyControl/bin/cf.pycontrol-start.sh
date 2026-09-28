#!/bin/bash

# Default arguments for the Python application
URI="radio://0/80/2M/E7E7E7E7E1"
PORT="5000"

# Remove the old image files
rm -f /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview/img/*.png

# ------ Forward port 5000 to 8080. The WSL firewall only allows access through 8080; this is not needed on native Linux. ------
# Find and kill any process occupying port 8080
echo "Check port 8080..."
if lsof -i :8080 > /dev/null; then
    echo "Port 8080 was found to be occupied and is being shut down..."
    lsof -ti :8080 | xargs kill -9 2>/dev/null
    sleep 1
fi

# Start the HTTP server (output redirected to http_server.log)
cd /home/crazy/cps.asset.crazyflie/controller/cf.PyControl/webview
python3 -m http.server 8080 > http_server.log 2>&1 &
HTTP_SERVER_PID=$!

# Define the cleanup function
cleanup() {
    echo "shutdown http server..."
    kill $HTTP_SERVER_PID 2>/dev/null
    exit 0
}

# Install the signal handlers
trap cleanup SIGINT SIGTERM
# ------------------------------------------------------------------------------------------------


cd ../src/
# Run the Python application with default arguments and any extra arguments passed to the script
python3 cf-ctrl-service.py --uri "$URI" --port "$PORT" "$@"