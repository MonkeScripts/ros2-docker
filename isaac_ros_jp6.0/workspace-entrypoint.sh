#!/bin/bash
#
# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

# Restart udev daemon
sudo service udev restart

# Add aliases
echo 'alias foxglove-bridge="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"' >> ~/.bash_aliases
echo 'alias start-px4-agent="export FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml && \
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1 && \
    MicroXRCEAgent udp4 -p 8888"' >> ~/.bash_aliases
echo 'alias udp4-dds-config="export FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml"' >> ~/.bash_aliases

if [ -d /workspaces/isaac_ros-dev ]; then 
    cd /workspaces/isaac_ros-dev ; 
    echo "rosdep updating....." ; 
    rosdep install --from-paths src -y --ignore-src ; 
    source install/setup.bash
fi

if [ -d /workspaces/drone ]; then 
    cd /workspaces/drone ; 
    echo "rosdep updating....." ; 
    # TODO: Bug fix intercomm_msgs
    # rosdep install --from-paths src -y --ignore-src ; 
    rosdep install --from-paths src --ignore-src --skip-keys intercomm_msgs -y ;
    source install/setup.bash
fi

$@
