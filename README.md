# Installation on SBC

## Setup docker

Source: https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html

Commands from the above website are pasted below:

### Install Jetpack

```
sudo apt install nvidia-jetpack
```

### Install docker

```
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### Add docker to user group

```
sudo usermod -aG docker $USER
newgrp docker
```

## Setup Isaac ROS

Source: https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html

Commands from the above website are pasted below:

```
sudo systemctl daemon-reload && sudo systemctl restart docker

sudo apt-get install git-lfs
git lfs install --skip-repo

mkdir -p  ~/workspaces/isaac_ros-dev/src
echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
source ~/.bashrc
```

## Build Isaac ROS Docker Image

1. Clone `isaac_ros_common`.

```
cd ${ISAAC_ROS_WS}/src && \
   git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```

2. Edit the Isaac ROS Common config file by setting `CONFIG_DOCKER_SEARCH_DIRS` as 

For `isaac_ros_jp6.0`:

```
(<Path to this directory>/isaac_ros_jp6.0)
```

For `isaac_ros_x64`:

```
(<Path to this directory>/isaac_ros_jp6.0)
```

By default, `<Path to this directory>` is `$HOME/workspaces/ros2-docker`.

Take note to enclose it with `()` and ensure that there are no spaces.

3. Copy the required config files and scripts.

For `isaac_ros_jp6.0`:

```
cp isaac_ros_jp6.0/.isaac_ros_common-config ~/.isaac_ros_common-config 
cp isaac_ros_jp6.0/run_main.sh ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
cp isaac_ros_jp6.0/workspace-entrypoint.sh ${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/workspace-entrypoint.sh
```

For `isaac_ros_x64`:

```
cp isaac_ros_x64/.isaac_ros_common-config ~/.isaac_ros_common-config 
cp isaac_ros_x64/run_main.sh ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
cp isaac_ros_x64/workspace-entrypoint.sh ${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/workspace-entrypoint.sh
```

4. Build the docker images.

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_main.sh
```

# Installation on local computer

This is for decoding images compressed on the Jetson.

Follow https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html
and https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html to set up 
Isaac ROS docker dev environment.

Follow ["Build Isaac ROS Docker Image" instructions for SBC](#build-isaac-ros-docker-image), 
using `isaac_ros_x64` instead of `isaac_ros_jp6.0`.

**NOTE: As of Jul 18 2024, there is a bug with `moveit_task_constructor`. Comment out the following lines in
`~/workspaces/isaac_ros-dev/src/isaac_ros_common/docker/Dockerfile.ros2_humble`:**

```
# Install MoveIt task constructor from source.  The "demo" package depends on moveit_resources_panda_moveit_config,
# installed from source above.

RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ros-planning/moveit_task_constructor.git -b humble \
    && cd moveit_task_constructor && source ${ROS_ROOT}/setup.bash \
    && cd msgs && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd rviz_marker_tools && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd core && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd capabilities && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd visualization && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd demo && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb
```

and 

```
# Install moveit2_tutorials from source (depends on moveit_hybrid_planning).
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ros-planning/moveit2_tutorials.git -b humble \
    && cd moveit2_tutorials && source ${ROS_ROOT}/setup.bash \
    && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb
```