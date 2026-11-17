# Crazyflie Digital twin-based Controller and Webots Simulator

This directory is self-contained.

## Requirements

- Docker Engine with Compose v2, and Python 3 on the host.
- Linux or WSL2. A working X11/WSLg display is needed for the GUI; use `--headless`
  without a display. NVIDIA on native Linux needs its Docker runtime configured.
- Internet access for the first source build. No prebuilt project image, host ROS,
  Java, Maven or installation from the parent repository is required for Webots.

The image builds from pinned ROS 2 Humble / Ubuntu 22.04 and Webots R2025a sources.

## Build and run

From this directory:

```bash
bash controller/cf.PyControl/bin/start_dssim_docker.sh --action build
bash controller/cf.PyControl/bin/start_dssim_docker.sh
```

Build the image before the first start. Without a display:

```bash
bash controller/cf.PyControl/bin/start_dssim_docker.sh --headless
```

Wait for `READY`. Ctrl+C stops the container created by this launcher.
The image is retained for ordinary reuse. No unrelated processes or containers
are stopped. If a port is occupied, stop its owner explicitly before retrying.

| Interface | Default |
|---|---|
| HTTP | `http://127.0.0.1:5000/status`, then one port per aircraft in sorted world-ID order |
| State diagrams | `http://127.0.0.1:8080/webview.html`, then one port per aircraft |
| Per-drone WebSocket | 8765 onwards, one port per aircraft |
| Experiment service | `http://127.0.0.1:5090/experiment/status` |
| ROS domains | Native 31; compatibility 30 (see `config.json`) |
| Webots controller port | 1234 |

Java control uses the per-drone HTTP endpoints backed by ROS telemetry. Native
`/cfXXX/status`, `/odom` and `/scan` use the native ROS domain and interfaces.
Optional Java ROSBridge observation requires a separately running bridge on 9090
providing `/cf_positions_path` (`nav_msgs/Path`, drone IDs in `header.frame_id`).

Basic HTTP commands are `/activate_idle`, `/begin_takeoff`, `/navigate`,
`/navigate/append`, `/stop_controller` and `/begin_landing`. JSON coordinate requests
and signed URL coordinates are supported. HTTP 200 accepts a task; check
`GET /tasks/<task_id>` for actual completion. Stopping cancels the old task and
holds position. The shared core also retains its polynomial APIs, without
bundling trajectory examples or optional wall-following features.

HTTP and WebSocket listeners bind all interfaces for the existing
local/Windows integration. They are control endpoints without authentication;
use a trusted network and restrict access when exposing the host.

## World

The editable world is
`simulation/ds-crazyflies-ext/simulation.ds-crazyflies/crazyflie.wbt`, mounted at
`/ds/crazywebotsworld/worlds/crazyflie.wbt`. Stop and restart after editing it.
The simulator discovers IDs from this file. Profiles do not select a world.

| Bundled world | UAS | Use |
| --- | --- | --- |
| `crazyflie.wbt` | 12 in the supplied file | Ordinary multi-UAS mission |
| `crazyflie2.wbt` | 2 | `drift` and `multievent` experiment profiles |
| `crazyflie8.wbt` | 8 | Eight-UAS layout; configure Java targets/grid for this layout |

To switch manually, stop the simulator first. From this directory, back up the
active world once, then copy the file you chose into the mounted path. For example,
**if you want the two-UAS experiments**:

```bash
cd simulation/ds-crazyflies-ext/simulation.ds-crazyflies
cp -n crazyflie.wbt crazyflie.original.wbt
cp crazyflie2.wbt crazyflie.wbt
```

For eight UAS, copy `crazyflie8.wbt` instead; restore the supplied twelve-UAS world
with `cp crazyflie.original.wbt crazyflie.wbt`. Restart the simulator afterwards.
Set Java's `drone.count`, `drone.first-cf`, `drone.targets`, obstacles and
`bigrid.service.*` geometry to match. Java uses consecutive IDs starting at
`drone.first-cf`; targets must be grid cell centers. No script changes worlds.

`controller/cf.PyControl/src/` contains the control runtime modules.

## Hardware

Install [official Crazyswarm2](https://imrclab.github.io/crazyswarm2/installation.html)
in an external ROS 2 Humble workspace. Use tag **1.0.7**, commit
`fcf51e27fe675bbdb564a40f4ca1ee4336c062ea`, with the **C++ backend**. Install the
upstream prerequisites, ROS dependencies and this controller's requirements:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/crazyswarm2_ws/src
cd ~/crazyswarm2_ws/src
git clone --branch 1.0.7 --depth 1 --recursive https://github.com/IMRCLab/crazyswarm2.git
cd ~/crazyswarm2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-up-to crazyflie --cmake-args -DCMAKE_BUILD_TYPE=Release
```

From this repository's `cps.asset.crazyflie` directory, install
`python3 -m pip install -r controller/cf.PyControl/requirements.txt` in the Python
environment used by the host ROS workspace. Use the same ROS distribution and
middleware for the upstream server and controller; do not source the Webots
interface overlay for hardware.

Edit your external `crazyflies.yaml`: configure the actual radio URIs, enabled
aircraft, initial positions and localization. The controller reserves fixed HTTP
ports 5000–5007 for cf231–cf238 and requires readiness only for enabled aircraft.
The Java `hardware` profile is an example for cf231–cf235; align its count,
targets and grid with your configuration. Enable telemetry at these rates:

```yaml
all:
  firmware_logging:
    enabled: true
    default_topics:
      pose:
        frequency: 10
      odom:
        frequency: 50
      status:
        frequency: 10
```

Merge this block into the upstream configuration, retaining its other settings.
Start the configured radio and localization server in one terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/crazyswarm2_ws/install/setup.bash
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch crazyflie launch.py backend:=cpp teleop:=False \
  crazyflies_yaml_file:="$HOME/crazyswarm2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml"
```

Configure `motion_capture_yaml_file` for your motion-capture system; use
`mocap:=False` only when onboard localization is already configured. See the
[upstream positioning instructions](https://imrclab.github.io/crazyswarm2/usage.html#positioning).
Then, from this directory in another terminal:

```bash
export CF_ROS_SETUP="$HOME/crazyswarm2_ws/install/setup.bash"
export CF_CRAZYFLIES_CONFIG="$HOME/crazyswarm2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml"
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
bash controller/cf.PyControl/bin/start.sh
```

Wait for `READY`, then start the grid provider on 7070 and
`org.example.Application --spring.profiles.active=hardware` as described in the
[root Usage section](../README.md#usage). Service responses acknowledge requests;
task completion still requires fresh telemetry and reaching the commanded state.
The upstream backend does not publish the optional custom `/control_result` topic.

Crazyswarm2 and firmware sources are not bundled here. The C++ hardware setup
above does not require a local `crazyflie-firmware` checkout. For firmware changes
or software-in-the-loop simulation, download it separately using the
[official build instructions](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/).
