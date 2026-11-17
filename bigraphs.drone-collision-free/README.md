# UAV Grid-based Collision-Free Navigation System using Bigraph + ROS2 + CDO

A Bigraph × ROS2 × time-expanded A* + CBS Digital Twin System for Safe UAV Navigation

This project implements a grid-based, collision-free UAV navigation system using Bigraphical Reactive Systems (BRS), ROS2, and time-expanded A* with Conflict-Based Search (CBS).

The system enables a UAV to fly from takeoff to landing safely in a discretized airspace while maintaining real-time digital twin consistency between the world model, UAV agent model, and live ROS2 data.

---

## Project Overview

The system integrates:

- Bigraph-generated World Model
- UAV Agent Model with attributes (battery, RSSI, state…)
- Real-time ROS2 data
- Time-expanded A* + CBS for multi-UAS grid navigation
- RESTful commands to control the drone

Together, the system enables autonomous, rule-based, collision-free UAV flight across a grid-based airspace.

## Core Functions

### 1. Takeoff & Landing Rules

Bigraph reaction rules define takeoff/landing behavior.
When conditions match (rule-based), REST commands are sent to control the UAV, and agent attributes are updated.

### 2. A\* + CBS Path Planning in Grid Airspace

Each UAV receives a target destination.
Time-expanded A\* searches individual paths, and CBS resolves conflicts between UAS. Joint polynomial trajectories are checked for grid conflicts and continuous separation before dispatch; planning fails if no checked joint plan is found within the search limits.
Movement uses 10 (3D) directions.

### 3. Rule-Based Movement & Collision Avoidance

Before entering a new grid:

- Check if the next move matches a **movement rule** in the compose model.
- Ensure the next grid is not occupied by another UAV.
- If blocked, the UAV waits or replans using A\* + CBS.

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

- JDK 21 + Maven wrapper
- Spring 3.3.5
- Bigraph Framework 2.3.2
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

Start the selected backend, then the grid provider when needed, then
`src/main/java/org/example/Application.java`. See the root
[Usage instructions](../README.md#usage) for the exact commands and profile table.

In your IDE, use `org.example.Application` as the main class and this module as
the working directory. For ordinary simulation, use Program arguments
`--bigrid.service.base-url=http://127.0.0.1:7070 --cdo.enabled=false --ros.update.enabled=false`.
For another configuration, select one of `--spring.profiles.active=hardware`,
`--spring.profiles.active=drift` or `--spring.profiles.active=multievent`.
The profiles do not switch the simulator world; do that manually before startup.

The Maven equivalent is:

```bash
./mvnw spring-boot:run -Dspring-boot.run.mainClass=org.example.Application \
  -Dspring-boot.run.arguments="--spring.profiles.active=drift"
```

`ApplicationBaseline` uses the same planner and profiles with Bigraph matching
and local authorization disabled. `Application3D` / `Application3DBaseline` retain
the older flow. CDO is optional: start
`./mvnw test -DrunDisabledTests=true -Dtest=CDOStandaloneServerTest#run_server_test_01`
only when enabling `--cdo.enabled=true`. ROSBridge observation is optional too;
use `--ros.update.enabled=false` when no bridge is running.
