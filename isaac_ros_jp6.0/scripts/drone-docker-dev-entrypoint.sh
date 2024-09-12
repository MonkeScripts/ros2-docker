#!/bin/bash

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

# Restart udev daemon
sudo service udev restart

scripts_dir=/usr/local/bin/scripts
usrname=admin
grpname=flirimaging

# Add bash aliases
if [ -f $scripts_dir/drone.bash_aliases ]; then
    cat $scripts_dir/drone.bash_aliases >> ~/.bash_aliases
fi

# Add bashrc
if [ -f $scripts_dir/drone.bashrc ]; then
    cat $scripts_dir/drone.bashrc >> ~/.bashrc
fi
# sudo rm /etc/passwd.lock
# sudo rm /etc/shadow.lock
# sudo rm /etc/group.lock
# sudo rm /etc/gshadow.lock

# sudo mount -o remount,rw /
# add user
# if (getent passwd $usrname > /dev/null)
#     then
#         echo "Adding user $usrname to group $grpname group."
#         # Create user group (if not exists) and add user to it
#             sudo usermod -a -G $grpname $usrname
#             echo "Added user $usrname"
#     else
#         echo "User "\""$usrname"\"" does not exist"
# fi

# Install workspace dependencies
shopt -s expand_aliases
source ~/.bash_aliases
# rosdep-all

$@
