# ds-crazyflies-ext: Setup and Usage

This directory extends the upstream [ds-crazyflies](https://github.com/DynamicSwarms/ds-crazyflies) project with Linux-compatible Docker files and a custom Webots world.

## Prerequisites

| Dependency | Install reference |
|---|---|
| Docker Engine | https://docs.docker.com/engine/install/ |
| Docker Compose plugin | https://docs.docker.com/compose/install/linux/ |
| NVIDIA Container Toolkit | https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html |

Additional reference: https://dynamicswarms.github.io/ds-crazyflies/docker.html

## Setup

All commands below assume you are in the `simulation/ds-crazyflies-ext/` directory:

```shell
cd simulation/ds-crazyflies-ext
```

### Step 1 — Prepare (clone upstream repo and apply Linux overrides)

**Linux:**
```shell
./prepare.sh --linux
```

**Windows (WSL):**
```shell
./prepare.sh
```

The script:
1. Clones `https://github.com/DynamicSwarms/ds-crazyflies.git` into `simulation.ds-crazyflies/` and pins it to a tested commit.
2. When `--linux` is given, copies the custom `docker/` directory into the cloned repo, replacing the upstream Docker files with Linux-compatible versions.
3. Copies the custom `crazyflie.wbt` Webots world file into the cloned repo.

### Step 2 — Build the Docker image

```shell
cd simulation.ds-crazyflies/docker/
sudo docker compose build
```

## Running

### Allow GUI display (run once per session)

```shell
xhost +local:$USER
```

### Terminal 1 — Start the container

```shell
cd simulation/ds-crazyflies-ext/simulation.ds-crazyflies/docker/
sudo docker compose up
```

### Terminal 2 — Launch Webots

```shell
sudo docker exec -it ds-crazyflies-dev bash

webots /ds/crazywebotsworld/worlds/crazyflie.wbt --batch
```

To edit the world file directly:
```shell
nano /ds/crazywebotsworld/worlds/crazyflie.wbt
```

### Terminal 3 — Launch the ROS 2 framework

```shell
sudo docker exec -it ds-crazyflies-dev bash

# Simulation only (Webots backend)
ros2 launch crazyflies framework.launch.py backend:=webots

### Terminal 4 — Control a Crazyflie

```shell
sudo docker exec -it ds-crazyflies-dev bash
```

**Add drone (Webots):**
```shell
ros2 service call /crazyflie_webots_gateway/add_crazyflie \
  crazyflie_webots_gateway_interfaces/srv/WebotsCrazyflie "id: 0"
```

**Add drone (Hardware):**
```shell
ros2 service call /crazyflie_hardware_gateway/add_crazyflie \
  crazyflie_hardware_gateway_interfaces/srv/AddCrazyflie \
  "{id: 0, channel: 80, initial_position: {x: 0.0, y: 0.0, z: 0.0}, type: 'default'}"
```

**GUI (rqt):**
```shell
rqt --force-discover
```

## Useful Commands

**Takeoff:**
```shell
ros2 topic pub --once /cf0/takeoff crazyflie_interfaces/msg/Takeoff \
  "{group_mask: 0, height: 0.5, yaw: 0.0, use_current_yaw: false, duration: {sec: 2, nanosec: 0}}"
```

**Go to position (absolute):**
```shell
ros2 topic pub --once /cf0/go_to crazyflie_interfaces/msg/GoTo \
  "{group_mask: 0, relative: false, linear: false, goal: {x: 2.0, y: 1.0, z: 0.5}, yaw: 0.0, duration: {sec: 2, nanosec: 0}}"
```

**Go to position (relative):**
```shell
ros2 topic pub --once /cf0/go_to crazyflie_interfaces/msg/GoTo \
  "{group_mask: 0, relative: true, linear: false, goal: {x: 0.5, y: 0.5, z: 0.0}, yaw: 0.0, duration: {sec: 2, nanosec: 0}}"
```

**Land:**
```shell
ros2 topic pub --once /cf0/land crazyflie_interfaces/msg/Land \
  "{group_mask: 0, height: 0.0, yaw: 0.0, use_current_yaw: false, duration: {sec: 2, nanosec: 0}}"
```

**Monitor positions:**
```shell
ros2 topic echo /cf_positions_poses --qos-reliability best_effort
```

## Crazyradio USB Check

```shell
lsusb
ls -l /dev/bus/usb/003/010
lsusb -v -d 1915:7777
```

## cf.PyControl Integration

```shell
./cfpyctrl.sh --dscf --cf-prefix /cf0 --wsendpoint
./cfpyctrl.sh --dscf --cf-prefix /cf1 --port 5001 --wsendpoint --wsport 8766
```
