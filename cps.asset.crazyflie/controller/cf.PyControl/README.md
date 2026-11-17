# FEDT control runtime

The HTTP server calls `DroneController` directly. Each drone owns its task queue,
telemetry and `DroneState`; PNG publication runs asynchronously. This bundle
contains the runtime only, without radio clients, wall-following, examples or
analysis scripts.

Build, launch and interface instructions are in [the standalone simulator guide](../../README.md).
No Java application or grid provider is needed for this controller to run.

For hardware, use the [upstream Crazyswarm2 setup](../../README.md#hardware), then
set `CF_ROS_SETUP` and `CF_CRAZYFLIES_CONFIG` before running `bin/start.sh`.
The official C++ backend uses standard ROS responses and telemetry; custom
`/control_result` diagnostics are optional (`--require-command-report` enables
them). Webots retains its required execution diagnostics.
