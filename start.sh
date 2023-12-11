#!/bin/bash
xhost +local:docker
docker rm ros2 && docker build -t ros2 .
docker run -it \
  --name ros2 \
  --runtime nvidia \
  --privileged \
  --network host \
  --mount type=bind,source=/home/jellyfish2/ros2_ws/,target=/home/bb/ros2_ws/ \
  --mount type=bind,source=/mnt/ros2-ccache,target=/home/bb/.ccache \
  --mount type=bind,source=/dev/,target=/dev/ \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v "$HOME/.Xauthority:/home/bb/.Xauthority:rw" \
  ros2
