#!/bin/bash

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