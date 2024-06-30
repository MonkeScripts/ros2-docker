#!/bin/bash
set -e
USERNAME=bb
# setup ros2 environment
echo "sourcing ROS $ROS_DISTRO"
source "/opt/ros/$ROS_DISTRO/install/setup.bash"
echo "sourcing drone workspace"

source "/home/$USERNAME/dwone/install/setup.bash"
export "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
echo "DDS middleware: $RMW_IMPLEMENTATION"
exec "$@"
