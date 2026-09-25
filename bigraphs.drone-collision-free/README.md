# UAV Grid-based Collision-Free Navigation System using Bigraph + ROS2 + CDO

A Bigraph × ROS2 × A* Digital Twin System for Safe UAV Navigation

This project implements a grid-based, collision-free UAV navigation system using Bigraphical Reactive Systems (BRS), ROS2, and A* path planning.

The system enables a UAV to fly from takeoff to landing safely in a discretized airspace while maintaining real-time digital twin consistency between the world model, UAV agent model, and live ROS2 data.

---

## Project Overview

The system integrates:

- Bigraph-generated World Model
- UAV Agent Model with attributes (battery, RSSI, state…)
- Real-time ROS2 data
- A* path planning for grid navigation
- RESTful commands to control the drone

Together, the system enables autonomous, rule-based, collision-free UAV flight across a grid-based airspace.

## Core Functions

### 1. Takeoff & Landing Rules

Bigraph reaction rules define takeoff/landing behavior.
When conditions match (rule-based), REST commands are sent to control the UAV, and agent attributes are updated.

### 2. A\* Path Planning in Grid Airspace

Each UAV receives a target destination.
A\* computes a sequence of next grid cells (grid centers), with considering obstacle/weather, until to reach the goal.
Movement uses 8 (2D) or 10 (3D) directions depending on availability.

### 3. Rule-Based Movement & Collision Avoidance

Before entering a new grid:

- Check if the next move matches a **movement rule** in the compose model.
- Ensure the next grid is not occupied by another UAV.
- If blocked, the UAV waits or replans using A\*.

### 4. Dynamic Bigraph Updates

The system continuously updates the world and attributes such as:

- Drone positions
- Battery levels
- RSSI (signal strength)
- Flight status

All updates feed back into the bigraph model.

### 5.  Emergency Reactions

- Low-battery (evaluate if UAV should return or find nearest charging station, then landing for chargering. After charged, the UAV take off and return original mission)
- Empty-battery (emergency landing)
- Low RSSI rule (poor communication handling, triggered UAV return)
- Low-battery + Low RSSI rule (emergency landing)

---

## Setup information

This component works with the following software stack:

- Java >=17 + Maven >=3.8.7
- Spring 3.3.5
- Bigraph Framework 2.3.1
- BDSL 2.0.1
- CDO for Spring Data 0.7.5
- cf.pyControl-ROS2 version
- Bigrid-provider-service

To view and modify the bigraph in the database, you can optionally install CDO explorer in Eclipse:

1. Download CDO Explorer via the [Eclipse Installer](https://www.eclipse.org/downloads/packages/installer).
     Use Eclipse Version 2022-12 (4.26.0), which supports CDO protocol version **48**.
     Eclipse IDE version 2023-09 supports only CDO protocol version 49.
2. Any Eclipse IDE with CDO support, must support **CDO protocol version 48**


### Demo setup

To run the demo, the bigrid provider service and Crazyflie CPS asset are required. Refer to the outer README or `launch_application.sh` in the project root for more information.

To run via IDE (Eclipse, IntelliJ IDEA, Visual Code, ...) instead, replace the last two launch steps in the script with the following manual steps:

- Run a method of `src/test/java/org.example/CDOStandaloneServerTest.java` in the test folder; this launches CDO
- Run the class `src/main/java/org/example/Application.java` (2D, legacy) 
- Run the class `src/main/java/org/example/Application3D.java` (3D, maintained)

Alternatively, they can be manually launched from terminal. For the CDO server, run:

```shell
$ ./mvnw test -DrunDisabledTests=true -Dtest="CDOStandaloneServerTest#run_server_test_01"
```

and for the main application:

```shell
$ ./mvnw spring-boot:run
```
