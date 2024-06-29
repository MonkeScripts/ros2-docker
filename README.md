# Installation

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

Add the following to `~/.isaac_ros_common-config`. Replace `<Path to this directory>`
with the path to this directory. Take note to enclose it with `()`.

```
BASE_IMAGE_KEY = ros2_humble.zed_for_isaac_ros
CONFIG_DOCKER_SEARCH_DIRS = (<Path to this directory>)
```

Build the docker images.

```
cd ${ISAAC_ROS_WS}/src && \
   git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

cd isaac_ros_common
./scripts/run_dev.sh
```