#!/bin/bash
docker rm ros2 && docker build -t ros2 .
docker run -it \
  --name ros2 \
  --mount type=bind,source=/home/jellyfish2/ros2_ws/,target=/home/bb/ros2_ws/ \
  --mount type=bind,source=/mnt/ros2-ccache,target=/home/bb/.ccache \
  --mount type=bind,source=/dev/,target=/dev/ \
  ros2
