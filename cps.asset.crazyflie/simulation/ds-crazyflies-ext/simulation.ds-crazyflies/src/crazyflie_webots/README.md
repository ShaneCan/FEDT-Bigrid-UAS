# Crazyflie Webots

A Webots-ROS2 connection for Crazyflie Quadcopters. 

This software allows to control a large number of simulated crazyflies through a ROS2 interface within Webots.
It has support for the HighLevelCommander, the Logging and a limited GenericCommander Interface for the Crazyflie Quadcopter.
The interface implementations follow the ones in the [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) project, and should therefore be compatible.

This package is a submodule for [ds-crazyflie](https://github.com/DynamicSwarms/ds-crazyflies), we recommend to not use this software in standalone mode but in the DynamicSwarms ecosystem.


Compatible with this [crazywebotsworld](https://github.com/DynamicSwarms/crazywebotsworld)


Build and testet with Ubuntu 22.04, Webots 2025a, ROS2 Humble.

# Standalone Installation and Usage

This software can also be used as a standalone simulator.

## Installation

### 1. Install dependencies:

1. Install Webots2025a, by following this guide: 

    https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt

2. If you have not already done so, install ROS2 with the following guide:

    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

    2.1 Install some ROS2 dependencies:

    ```
    sudo apt-get install ros-humble-tf-transformations
    ```


### Build this repository with its dependencies in a dedicated workspace folder

1. Create a workspace folder, with a `src` folder inside

```
mkdir ros2_ws
cd ros2_ws
mkdir src
```

2. Navigate into src and clone this repo as well as crazyflie_interfaces

```
cd src
git clone https://github.com/DynamicSwarms/crazyflie_webots.git
git clone https://github.com/DynamicSwarms/crazyflie_interfaces.git
cd ..
```

3. Build the project from `ros2_ws` folder

```
source /opt/ros/humble/setup.bash
export WEBOTS_HOME=/usr/local/webots
colcon build
```

4. Download the associated webotsworld

```
git clone https://github.com/DynamicSwarms/crazywebotsworld.git
```

## Simple Usage

The following steps guide you towards your first flight within Webots:

1. Open the world in webots 

    The world is placed in `crazywebotsworld/worlds/crazyflie.wbt`


2. The following launch-file starts the gateway and automatically adds the crazyflie in the webots-world 

    ```
    ros2 launch crazyflie_webots_examples crazyflie_webots_example.launch.py
    ```

3. Run the example flight pattern in a seperate terminal

    ```
    ros2 run crazyflie_webots_examples crazyflie 
    ```

## Advanced Usage

To use the full potential of this software the following gives a brief overview of its functionalities:

### Logging

The logging framework is implemented similarly to the one in the Crazyswarm2 implementation.
A short guide can be found [here](https://imrclab.github.io/crazyswarm2/howto.html#enabling-logblocks-at-runtime).

Additionally to adding and removing logblocks the following command allows to list all currently available logging variables:

```
ros2 topic pub /cf0/get_logging_toc_info std_msgs/msg/Empty --once
```

#### The `state` topic

The simulated crazyflie continously publishes onto the `cfID/state` topic.
The values represent `pm.vbat, pm.chargeCurrent, pm.state, sys.canfly, sys.isFlying, sys.isTumbled`, as they are simulated `pm` values never change. 
Only the `sys.isFlying` vale updates.

### Getting crazyflie position in ROS2

In order not to overwhelm the tf-Graph positions are published on a seperate topic called `cf_positions`.
To view them: 

```
ros2 topic echo /cf_positions 
```

Also checkout the [DynamicSwarms-Documentation](https://dynamicswarms.github.io/ds-crazyflies/position_topics.html) regarding this topic.

### Multiple Crazyflies

1. Copy the Crazyflie in Webots (CTRL-C, CTRL-V in the ProtoTree on the left)
2. Change the ID-Field of the Crazyflie in Webots (e.g. 1)
3. In a seperate terminal: 

```
ros2 service call /crazyflie_webots_gateway/add_crazyflie crazyflie_webots_gateway_interfaces/srv/WebotsCrazyflie "{id: 1}"
```

4. Use the crazyflie just like the first one: 

```
ros2 service call cf1/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2}}"
```

### The Gateway

The launch file used in the SimpleUsage section is just a wrapper to start the gateway and immediately after adding the crazyflie with id 0.
Instead you can also: 

```
ros2 run crazyflie_webots_gateway gateway 
```

and in a second terminal: 

```
ros2 service call /crazyflie_webots_gateway/add_crazyflie crazyflie_webots_gateway_interfaces/srv/WebotsCrazyflie "{id: 0}"
```



# The Future

## Jazzy

```
sudo apt-get install ros-jazzy-tf-transformations
```
