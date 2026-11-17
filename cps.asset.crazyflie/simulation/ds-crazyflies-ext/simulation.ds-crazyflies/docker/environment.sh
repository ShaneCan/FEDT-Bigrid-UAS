#!/usr/bin/env bash
set -e
overlay=${1:-native}
shift || true
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
if [ "$overlay" = compat ]; then
  source /ds/compat/install/setup.bash
  export ROS_DOMAIN_ID=${CF_COMPAT_DOMAIN:-30}
elif [ "$overlay" = native ]; then
  source /ds/ds-crazyflies/install/setup.bash
  export ROS_DOMAIN_ID=${CF_NATIVE_DOMAIN:-31}
else
  echo "Unknown ROS overlay: $overlay" >&2
  exit 2
fi
export WEBOTS_HOME=/usr/local/webots
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH="/usr/local/webots/lib/controller:${LD_LIBRARY_PATH:-}"
if [ -e /dev/dxg ] && [ -d /usr/lib/wsl/lib ]; then
  export LD_LIBRARY_PATH="/usr/lib/wsl/lib:$LD_LIBRARY_PATH"
fi
export PYTHONPATH="/workspace/controller/cf.PyControl/src:${PYTHONPATH:-}"
exec "$@"
