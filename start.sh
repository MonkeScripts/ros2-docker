#!/bin/bash

CONTAINER=ros2
IMAGE=ros2

# For X forwarding from container
xhost +local:docker >/dev/null

should_run=1
while getopts "a" args; do
  case "$args" in
    a) # Attach
      echo "Attaching to container $CONTAINER..."
      docker exec -it $CONTAINER bash -c 'source /ros_entrypoint.sh && bash'
      ;;
  esac

  # Processed some flag; do not start container
  should_run=0
done

if [ "$should_run" -eq 1 ]; then

    # Build and start
    echo "Starting container $CONTAINER..."
    docker rm $IMAGE && docker build -t $IMAGE .
    docker run -it \
      --name $IMAGE \
      --runtime nvidia \
      --privileged \
      --network host \
      --mount type=bind,source=/home/jellyfish2/ros2_ws/,target=/home/bb/ros2_ws/ \
      --mount type=bind,source=/mnt/ros2-ccache,target=/home/bb/.ccache \
      -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v "$HOME/.Xauthority:/home/bb/.Xauthority:rw" \
      $CONTAINER
fi
#      --mount type=bind,source=/dev/,target=/dev/ \
