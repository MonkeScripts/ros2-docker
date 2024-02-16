#!/bin/bash
set -e
# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/install/setup.bash"
export "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" 
exec "$@"
