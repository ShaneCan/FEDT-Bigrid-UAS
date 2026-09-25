# FEDT-Bigraph-UAS

This project demonstrates how Bigraphs can be used to model and control Crazyflie2.1 drones in simulation or reality. It implements a grid-based, collision-free UAV navigation system using Bigraphical Reactive Systems (BRS), ROS2, and A* path planning. This allows UAVs to fly safely from takeoff to landing in a discretized airspace while maintaining real-time digital twin consistency between the world model, UAV agent model, and live ROS2 data.

 It consists of 3 modules:
- `bigrid-provider-service`: An API for generating and retrieving a *Bigrid* (Bigraph world model with grid topology).
- `cps.asset.crazyflie`: Provides a cyber-physical representation of the **Crazyflie 2.x drone**. This component enables integration, simulation, and control of Crazyflie 2.x drones in both physical and virtual environments.
- `bigraphs.drone-collision-free`: A Digital Twin System for safe UAV Navigation based on A*. It employs a joint Bigraph world and drone model and uses ROS2 as middleware.


## Setup Process (Simulation with Webots)

The setup process was tested on Ubuntu24.04.5 with Kernel 7.0.0-31, using X11 with NVIDIA as well as AMD integrated graphics. It is verified to be working on ROS versions Humble and Jazzy and known to be broken on ROS Lyrical due to some dependencies not being up-to-date for this version yet.

### Prerequisites

1. Install Docker and Docker Compose:

```bash
sudo apt install docker-compose-v2 util-linux-extra
sudo usermod -aG docker $USER
newgrp docker (or re-login)
```

2. Install ROS (Jazzy) according to the [Docs](https://docs.ros.org/en/jazzy/Get-Started/Installation/Ubuntu-Install-Debs.html). Make sure to install `ros-$ROS_DISTRO-desktop` instead of the barebones install. Append `source /opt/ros/$ROS_DISTRO/setup.bash` to `$HOME/.bashrc`.

3. Install further dependencies:

```bash
sudo apt-get install ros-dev-tools ros-$ROS_DISTRO-rosbridge-server tmux
```

### Workspace setup

Setup a workspace folder and clone the following dependencies into it:

1. Clone cps.asset.crazyflie (recursive), then create the docker container for Webots and DS-Crazyflies:

```bash
cd cps.asset.crazyflie/simulation/ds-crazyflies-ext
./prepare.sh --linux
cd simulation.ds-crazyflies/docker
docker compose build
```

2. Clone bigraphs.drone-collision-free (recursive), then:

```bash
cd bigraphs.drone-collision-free
chmod +x mvnw
./mvnw clean package -DskipTests
```

3. Clone bigrid-provider-service (recursive), then:

```bash
cd bigrid-provider-service
chmod +x mvnw
./mvnw clean package -DskipTests
```

## Usage

Launching the system involves several steps, so we recommend launching with [tmux](https://tmux.app/), which will start all processes in a joint terminal window, that can be terminated with Ctrl+Q. To do this, after completing the setup process, run:

```bash
./launch_application.sh [ROS_Version]
```

By default, the script assumes the uses `$ROS_DISTRO` to determine the ROS distribution. If you want to use a different version, specify this as a launch argument. 

To use the system with real Crazyflie drones, refer to the docs in `cps.asset.crazyflie` for setup information.