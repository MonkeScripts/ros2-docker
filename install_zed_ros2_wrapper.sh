#!/bin/bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) \
    --packages-select zed_components zed_ros2 zed_interfaces zed_wrapper
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
source ~/.bashrc