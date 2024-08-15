#!/bin/bash

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

# Restart udev daemon
sudo service udev restart

scripts_dir=/usr/local/bin/scripts

# Add bash aliases
if [ -f $scripts_dir/drone.bash_aliases ]; then
    cat $scripts_dir/drone.bash_aliases >> ~/.bash_aliases
fi

# Add bashrc
if [ -f $scripts_dir/drone.bashrc ]; then
    cat $scripts_dir/drone.bashrc >> ~/.bashrc
fi

# Install workspace dependencies
shopt -s expand_aliases
source ~/.bash_aliases
rosdep-all

$@
