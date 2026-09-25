#!/bin/bash
set -euo pipefail

# Go up to repo parent directory, where the user should have the other repos stored
ROOT_DIR=$(pwd)/../../../..

# 宿主机一键启动：进入 ds-crazyflies 容器运行控制器（dssim topic 模式）

CONTAINER="ds-crazyflies-dev"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "Container ${CONTAINER} not running. Start it first:"
  echo "  cd $ROOT_DIR/cps.asset.crazyflie/simulation/ds-crazyflies-ext/simulation.ds-crazyflies/docker && sudo docker compose up -d"
  exit 1
fi

# network_mode: host，所以容器内服务会占用宿主机端口；先在宿主机侧清理端口占用
echo "Checking ports on host..."
for port in 8080 8081 8082 8083 8084 8085 8086 8087 5000 5001 5002 5003 5004 5005 5006 5007; do
  if lsof -i :"$port" > /dev/null 2>&1; then
    echo "Port $port is occupied; killing processes..."
    lsof -ti :"$port" | xargs kill -9 2>/dev/null || true
    sleep 1
  fi
done

docker exec -it "${CONTAINER}" bash -lc "bash /ds/controller/cf.PyControl/bin/start_dssim_in_container.sh"

