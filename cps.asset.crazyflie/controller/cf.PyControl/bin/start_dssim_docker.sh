#!/bin/bash
set -euo pipefail

# Go up to repo parent directory, where the user should have the other repos stored
ROOT_DIR=$(pwd)/../../../..

# One-shot host launcher: enters the ds-crazyflies container and runs the controller in dssim topic mode

CONTAINER="ds-crazyflies-dev"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "Container ${CONTAINER} not running. Start it first:"
  echo "  cd $ROOT_DIR/cps.asset.crazyflie/simulation/ds-crazyflies-ext/simulation.ds-crazyflies/docker && sudo docker compose up -d"
  exit 1
fi

# With network_mode: host the container services bind host ports, so free them on the host first
echo "Checking ports on host..."
for port in 8080 8081 8082 8083 8084 8085 8086 8087 5000 5001 5002 5003 5004 5005 5006 5007; do
  if lsof -i :"$port" > /dev/null 2>&1; then
    echo "Port $port is occupied; killing processes..."
    lsof -ti :"$port" | xargs kill -9 2>/dev/null || true
    sleep 1
  fi
done

docker exec -it "${CONTAINER}" bash -lc "bash /ds/controller/cf.PyControl/bin/start_dssim_in_container.sh"

