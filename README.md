- [Installation](#installation)
  - [Installation on SBC](#installation-on-sbc)
    - [Setup docker](#setup-docker)
      - [Install Jetpack](#install-jetpack)
      - [Install docker](#install-docker)
      - [Add docker to user group](#add-docker-to-user-group)
    - [Setup Isaac ROS](#setup-isaac-ros)
    - [Jetson Clocks (Optional)](#jetson-clocks-optional)
    - [Add Authorized SSH Keys (Optional)](#add-authorized-ssh-keys-optional)
  - [Installation on local computer](#installation-on-local-computer)
- [Build Isaac ROS Docker Image](#build-isaac-ros-docker-image)
- [Start Docker Container](#start-docker-container)
  - [Development](#development)
  - [Production](#production)
- [Notes](#notes)
  - [TODO](#todo)
  - [Issues](#issues)
    - [1. MoveIt](#1-moveit)
    - [2. Husarnet](#2-husarnet)
    - [3. Sourcing of ROS Workspaces on Entry](#3-sourcing-of-ros-workspaces-on-entry)
    - [4. Jetson Clocks](#4-jetson-clocks)

# Installation

*For ease of installation, save this directory as `~/workspaces/ros2-docker`.*

## Installation on SBC

### Setup docker

Source: https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html

Commands from the above website are pasted below:

#### Install Jetpack

```
sudo apt install nvidia-jetpack
```

#### Install docker

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

#### Add docker to user group

```
sudo usermod -aG docker $USER
newgrp docker
```

Reboot the computer for the changes to take effect.

### Setup Isaac ROS

Source: https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html

Commands from the above website are pasted below:

```
sudo systemctl daemon-reload && sudo systemctl restart docker

sudo apt-get install git-lfs
git lfs install --skip-repo

mkdir -p ~/workspaces/isaac_ros-dev/src
echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
source ~/.bashrc
```

### Jetson Clocks (Optional)

Running `sudo jetson_clocks` maximises Jetson performance. We can make `jetson_clocks` run on start up.

Create the file `/etc/systemd/system/jetsonClocks.service` and add the following lines:

```
[Unit]
Description=Maximize Jetson Performance
After=nvpmodel.service
Requires=nvpmodel.service

[Service]
ExecStart=/usr/bin/jetson_clocks

[Install]
WantedBy=multi-user.target
```

Then run the following commands

```
sudo chmod 644 /etc/systemd/system/jetsonClocks.service
sudo systemctl daemon-reload
sudo systemctl enable jetsonClocks.service
```

Reboot the computer to let the changes take effect.

### Add Authorized SSH Keys (Optional)

To avoid keying in the password each time login in via SSH, add the client computer's public key (e.g. `id_rsa.pub`) into `~/.ssh/authorized_keys`. Note that `~/.ssh/authorized_keys` should be a **file** not a folder.

## Installation on local computer

This is for decoding images compressed on the Jetson.

Follow https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html
and https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html to set up 
Isaac ROS docker dev environment.

# Build Isaac ROS Docker Image

For SBC, use `isaac_ros_jp6.0`. For local computer, use `isaac_ros_x64`.

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
(<Path to this directory>/isaac_ros_x64)
```

By default, `<Path to this directory>` is `$HOME/workspaces/ros2-docker`.

Take note to enclose it with `()` and ensure that there are no spaces.

3. Create copies and symbolic links for the required config files and scripts.

By default, the path to this directory is `$HOME/workspaces/ros2-docker`. Replace it below with the path to this directory if yours is different.

For `isaac_ros_jp6.0`:

```
export SOURCE_DIRECTORY=$HOME/workspaces/ros2-docker/isaac_ros_jp6.0
ln -sf $SOURCE_DIRECTORY/.isaac_ros_common-config   ~/.isaac_ros_common-config 
ln -sf $SOURCE_DIRECTORY/run_dev.sh                 ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/run_dev.sh
ln -sf $SOURCE_DIRECTORY/run_main.sh                ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/run_main.sh
cp $SOURCE_DIRECTORY/workspace-entrypoint.sh        ${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/workspace-entrypoint.sh
unset SOURCE_DIRECTORY
```

For `isaac_ros_x64`:

```
export SOURCE_DIRECTORY=$HOME/workspaces/ros2-docker/isaac_ros_x64
ln -sf $SOURCE_DIRECTORY/.isaac_ros_common-config   ~/.isaac_ros_common-config 
ln -sf $SOURCE_DIRECTORY/run_dev.sh                 ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/run_dev.sh
ln -sf $SOURCE_DIRECTORY/run_main.sh                ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/run_main.sh
unset SOURCE_DIRECTORY
```

**Note that `workspace-entrypoint.sh` is copied and not linked** to support Docker's `COPY`.

4. Build the docker images.

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

# Start Docker Container

`run_dev.sh` is for development, while `run_main.sh` is for production. 
The difference is that `run_dev.sh` attempts a Docker image build each time, while
`run_main.sh` looks up an existing Docker image.

## Development

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

## Production

`run_main.sh` is meant to run a built image, which can be built by `run_dev.sh`.

By default, file changes (except in the mounted workspaces) and installations in a running Docker container
are not persistent. To save the current state of the container's filesystem to an image, do `docker container commit`
(https://docs.docker.com/reference/cli/docker/container/commit/).

`run_main.sh` requires the environment variable `BUILT_DOCKER_CONTAINER_NAME`. 
It reads environment variables from `ENV_FILE`, which by default is `$HOME/workspaces/ros2-docker/environments/.env`. 
If the path of your file is different, change `ENV_FILE` in `run_main.sh`.

# Notes

## TODO

- (Good to have) combine Isaac ROS and Husarnet default Fast RTPS scripts to poll topics outside of container while publishing over the internet
- (Good to have) Consider https://github.com/husarnet/ros-docker/tree/main

## Issues

### 1. MoveIt

**UPDATE: As of 2 Aug 2024, this issue seems to be fixed.**

**NOTE: As of 18 Jul 2024, there is a bug with `moveit_task_constructor`. Comment out the following lines in
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

### 2. Husarnet

- For now, install and join the network outside Docker. Unable to join while building the Docker containers.
- For now, run `husarnet-dds singleshot` inside the running container. No effect when starting in Dockerfiles.

### 3. Sourcing of ROS Workspaces on Entry

- Only the first terminal instance running the Docker container source the ROS workspaces automatically. Subsequent instances do not. 

### 4. Jetson Clocks

- Even after setting Jetson Clocks to run on startup [above](#jetson-clocks-optional), it may randomly fail to start up due to a bug with `nvpmodel` (https://forums.developer.nvidia.com/t/segfault-in-usr-sbin-nvpmodel/295010/16). Simply do:

```
sudo systemctl restart nvpmodel.service
sudo systemctl restart jetsonClocks.service
```

Where the second line can be replaced with `sudo jetson_clocks` if the service is not set up.
