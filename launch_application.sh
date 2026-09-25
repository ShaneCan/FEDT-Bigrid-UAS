#!/usr/bin/env bash
set -uo pipefail

# ---------------------------------------------------------------------------
# Dependency check, usage
# ---------------------------------------------------------------------------
require_cmd() {
  local cmd="$1" pkg="${2:-$1}"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "ERROR: '$cmd' is required but not installed." >&2
    echo "Install it with: sudo apt install $pkg" >&2
    exit 1
  fi
}
require_cmd tmux

# Instead of explicit default, use already set $ROS_DISTRO
#ROS_DISTRO="jazzy"

usage() {
    cat <<EOF
Usage: $0 [ROS_VERSION]

Launch the script using the specified ROS 2 distribution.

Arguments:
  ROS_VERSION    ROS 2 distribution to use (default: jazzy)

Options:
  -h, --help     Show this help message and exit

Examples:
  $0             # Use ROS 2 Jazzy (default)
  $0 jazzy       # Use ROS 2 Jazzy (explicit)
  $0 humble      # Use ROS 2 Humble
EOF
}

case "${1:-}" in
    -h|--help)
        usage
        exit 0
        ;;
    "")
        ;;
    *)
        ROS_DISTRO="$1"
        ;;
esac

echo "Using ROS 2 distribution: $ROS_DISTRO"


# ---------------------------------------------------------------------------
# Vars
# ---------------------------------------------------------------------------
export ROOT_DIR="$(pwd)"
export ROS_DISTRO
export ROS_DOMAIN_ID=30
export CONTAINER="ds-crazyflies-dev"
export DOCKER_DIR="$ROOT_DIR/cps.asset.crazyflie/simulation/ds-crazyflies-ext/simulation.ds-crazyflies/docker"

# Set SUDO="" if your user is in the 'docker' group (recommended, avoids per-tty
# password prompts in every spawned terminal).
export SUDO=""

# Unique tag stamped into every spawned command line so cleanup can find them.
export TAG="CFSIM_LAUNCHER_$$"

declare -a WAIT_PIDS=()

# ---------------------------------------------------------------------------
# Terminal payloads (exported so a fresh bash in gnome-terminal inherits them)
# ---------------------------------------------------------------------------
term1() {   # docker compose (the container itself)
  xhost +local:"$USER"
  cd "$DOCKER_DIR" || exit 1
  $SUDO docker compose up
  status=$?

  if $SUDO nvidia-smi >/dev/null 2>&1; then
    echo "NVIDIA GPU detected, launching with GPU support..."
    $SUDO docker compose \
      -f docker-compose.yaml \
      -f docker-compose.gpu.yaml \
      up
  else
    echo "No NVIDIA GPU detected, launching CPU/VM configuration..."
    $SUDO docker compose up
  fi
}

term2() {   # webots inside the container
  # If you encounter any graphical glitches, try omitting the scaling vars.
  # They help if you don't have a high DPI monitor and/or non-default scaling
  # settings or 'Large Text' enabled in Ubuntu accessibility settings.
  $SUDO docker exec -it "ds-crazyflies-dev" bash -c \
  "QT_AUTO_SCREEN_SCALE_FACTOR=0 \
   QT_ENABLE_HIGHDPI_SCALING=0 \
   QT_SCALE_FACTOR=1 \
   QT_FONT_DPI=96 \
   GDK_SCALE=1 \
   GDK_DPI_SCALE=1 \
   __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only \
   webots /ds/crazywebotsworld/worlds/crazyflie.wbt --batch"
}

term3() {   # ROS crazyflie backend
  # This uses port 1234. If you have any applications installed that already
  # use it (e.g. LM-Studio), you will get a warning in Webots. Make sure
  # to terminate these applications, otherwise the launch script won't work.
  $SUDO docker exec -it $CONTAINER bash -ic \
  "ros2 launch crazyflies framework.launch.py backend:=webots"
}

term4() {   # add 8 crazyflies (231..238) -- one-shot
  $SUDO docker exec -it "$CONTAINER" bash -ic '
    for id in 231 232 233 234 235 236 237 238; do
      ros2 service call /crazyflie_webots_gateway/add_crazyflie \
        crazyflie_webots_gateway_interfaces/srv/WebotsCrazyflie "id: '"'"'$id'"'"'"
    done
  '
}

term5() {   # python controller
  $SUDO docker exec -it "$CONTAINER" bash -ic \
    "cd /ds/controller/cf.PyControl/src && python3 cf_positions_relay_in_container.py"
}

term6() {   # dssim docker helper
  $SUDO bash "$ROOT_DIR/cps.asset.crazyflie/controller/cf.PyControl/bin/start_dssim_docker.sh" "$ROOT_DIR"
}

term7() {   # rosbridge
  source /opt/ros/"$ROS_DISTRO"/setup.bash
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml
}

term8() {   # bigrid provider service
  java -jar "$ROOT_DIR/bigrid-provider-service/bin/bigrid-provider-service.jar" --server.port=7070
}

term9() {   # CDO standalone server test (long-running server)
  cd "$ROOT_DIR/bigraphs.drone-collision-free" || exit 1
  ./mvnw test -DrunDisabledTests=true -Dtest='CDOStandaloneServerTest#run_server_test_01'
}

term10() {  # spring-boot app
  cd "$ROOT_DIR/bigraphs.drone-collision-free" || exit 1
  ./mvnw spring-boot:run
}

export -f term1 term2 term3 term4 term5 term6 term7 term8 term9 term10

# ---------------------------------------------------------------------------
# tmux session setup
# ---------------------------------------------------------------------------
TMUX_SOCK="drone_launcher_$$"      # dedicated socket -> fresh server, guaranteed
SESSION="drone_launcher"           # to inherit our exported functions above
PANE_COUNT=0

tmx() { tmux -L "$TMUX_SOCK" "$@"; }

# ---------------------------------------------------------------------------
# Terminal launcher: opens/splits a tmux pane, stamps $TAG into the command,
# and titles the pane.
# ---------------------------------------------------------------------------
launch() {
  local title="$1" func="$2" pane_id
  local run_cmd="$func; ec=\$?; echo; echo \"[$title] exited (\$ec).\"; exec bash  # $TAG"

  # Guard: if the session was already killed (user hit Ctrl-q early), stop
  # trying to add more panes instead of spamming tmux errors.
  if [ "$PANE_COUNT" -gt 0 ] && ! tmx has-session -t "$SESSION" 2>/dev/null; then
    echo "[launcher] session gone, skipping: $title"
    return 1
  fi

  if [ "$PANE_COUNT" -eq 0 ]; then
    tmx new-session -d -s "$SESSION" -n "launcher" -x 220 -y 50
    tmx set-option -t "$SESSION" pane-border-status top
    tmx set-option -t "$SESSION" pane-border-format "#[fg=cyan,bold] #{pane_title} #[default]"
    pane_id=$(tmx list-panes -t "$SESSION" -F '#{pane_id}')
  else
    pane_id=$(tmx split-window -t "$SESSION" -P -F '#{pane_id}')
    tmx select-layout -t "$SESSION" tiled >/dev/null
  fi

  tmx select-pane -t "$pane_id" -T "$title"
  tmx send-keys -t "$pane_id" "$run_cmd" C-m

  PANE_COUNT=$((PANE_COUNT + 1))
  echo "[launcher] started: $title"
}

# ---------------------------------------------------------------------------
# Readiness gates (unchanged)
# ---------------------------------------------------------------------------
wait_container() {
  echo "[launcher] waiting for container '$CONTAINER'..."
  until [ "$($SUDO docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]; do
    sleep 1
  done
  echo "[launcher] container up."
}

wait_webots() {
  echo "[launcher] waiting for Webots..."
  until $SUDO docker exec "$CONTAINER" pgrep -f webots >/dev/null 2>&1; do sleep 2; done
  sleep 5
  echo "[launcher] Webots running."
}

wait_backend() {
  echo "[launcher] waiting for gateway service..."
  until $SUDO docker exec "$CONTAINER" bash -lc \
      "source /opt/ros/$ROS_DISTRO/setup.bash; ros2 service list 2>/dev/null | grep -q '/crazyflie_webots_gateway/add_crazyflie'"; do
    sleep 2
  done
  echo "[launcher] ROS backend ready."
}

wait_crazyflies() {
  local expected=8
  echo "[launcher] waiting for $expected crazyflies to register..."
  until [ "$($SUDO docker exec "$CONTAINER" bash -lc \
        "source /opt/ros/$ROS_DISTRO/setup.bash; ros2 node list 2>/dev/null | grep -c 'cf'" 2>/dev/null)" -ge "$expected" ]; do
    sleep 2
  done
  echo "[launcher] crazyflies registered."
}

# ---------------------------------------------------------------------------
# Graceful shutdown
# ---------------------------------------------------------------------------
kill_tree() {
  local pid="$1" sig="${2:-TERM}" child
  for child in $(pgrep -P "$pid" 2>/dev/null); do
    kill_tree "$child" "$sig"
  done
  kill -"$sig" "$pid" 2>/dev/null
}

cleanup() {
  trap '' INT TERM
  echo
  echo "[launcher] shutting down..."

  # Stop the background orchestrator immediately so it doesn't keep trying
  # to launch/wait for stages after we've decided to tear down.
  if [ -n "${ORCH_PID:-}" ]; then
    kill_tree "$ORCH_PID" TERM
  fi

  tmx kill-session -t "$SESSION" 2>/dev/null

  for pid in $(pgrep -f "$TAG" 2>/dev/null); do
    kill_tree "$pid" TERM
  done

  pkill -TERM -f "rosbridge_websocket"          2>/dev/null
  pkill -TERM -f "bigrid-provider-service.jar"  2>/dev/null
  pkill -TERM -f "spring-boot:run"              2>/dev/null
  pkill -TERM -f "CDOStandaloneServerTest"      2>/dev/null
  pkill -TERM -f "start_dssim_docker.sh"        2>/dev/null

  echo "[launcher] stopping docker container (freeing ports)..."
  ( cd "$DOCKER_DIR" && $SUDO docker compose down --remove-orphans )

  tmx kill-server 2>/dev/null

  echo "[launcher] done."
  exit 0
}
trap cleanup INT TERM

# ---------------------------------------------------------------------------
# Start the session with pane 1, bind shutdown key, then attach immediately
# so panes appear live instead of waiting for everything to finish first.
# ---------------------------------------------------------------------------
launch "1: docker compose" term1
tmx bind-key -n C-q kill-session -t "$SESSION"

(
  wait_container
  launch "2: webots"          term2 ; wait_webots
  launch "3: ros backend"     term3 ; wait_backend
  launch "4: add crazyflies"  term4 ; wait_crazyflies
  launch "5: cf controller"   term5
  launch "6: dssim"           term6 ; sleep 2
  launch "7: rosbridge"       term7 ; sleep 2
  launch "8: bigrid provider" term8 ; sleep 2
  launch "9: CDO server"      term9 ; sleep 10
  launch "10: spring-boot"    term10
) &
ORCH_PID=$!

echo
echo "==============================================================="
echo " Attaching now — panes will appear as each stage becomes ready."
echo " Press Ctrl-q ANYWHERE inside the tmux window to shut everything down."
echo "==============================================================="

# Attach so the user actually sees the tiled panes in one window.
sleep 0.3
tmx attach -t "$SESSION"

# Once the session ends (Ctrl-q pressed, or window closed), run full cleanup.
cleanup

