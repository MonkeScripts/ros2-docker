# Source all ROS workspaces
if [ -d /workspaces/zed ]; then 
    source /workspaces/zed/install/local_setup.bash
fi

if [ -d /workspaces/isaac_ros-dev ]; then 
    source /workspaces/isaac_ros-dev/install/setup.bash
fi

if [ -d /workspaces/drone ]; then 
    source /workspaces/drone/install/setup.bash
fi
