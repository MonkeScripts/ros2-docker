#!/bin/bash

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

# Restart udev daemon
sudo service udev restart

# Add bash aliases
if [ -f /usr/local/bin/scripts/drone.bash_aliases ]; then
    cat /usr/local/bin/scripts/drone.bash_aliases >> ~/.bash_aliases
fi

# Source all workspaces
if [ -d /workspaces/zed ]; then 
    cd /workspaces/zed ; 
    source install/local_setup.bash
fi

if [ -d /workspaces/isaac_ros-dev ]; then 
    cd /workspaces/isaac_ros-dev ; 
    source install/setup.bash
fi

if [ -d /workspaces/drone ]; then 
    cd /workspaces/drone ; 
    source install/setup.bash
fi

$@