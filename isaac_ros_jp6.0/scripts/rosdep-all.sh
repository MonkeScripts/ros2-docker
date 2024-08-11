#!/bin/bash

if [ -d /workspaces/zed ]; then 
    cd /workspaces/zed ; 
    echo "rosdep updating for zed....." ; 
    rosdep install --from-paths src --ignore-src -r -y ;
fi

if [ -d /workspaces/isaac_ros-dev ]; then 
    cd /workspaces/isaac_ros-dev ; 
    echo "rosdep updating for isaac_ros-dev....." ; 
    rosdep install --from-paths src -y --ignore-src ; 
fi

if [ -d /workspaces/drone ]; then 
    cd /workspaces/drone ; 
    echo "rosdep updating for drone....." ; 
    # TODO: Bug fix intercomm_msgs
    # rosdep install --from-paths src -y --ignore-src ; 
    rosdep install --from-paths src --ignore-src --skip-keys intercomm_msgs -y ;
fi

$@
