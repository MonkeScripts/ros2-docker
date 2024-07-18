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
`(<Path to this directory>/isaac_ros_jp6.0)`. Take note to enclose it with `()` and 
ensure that there are no spaces. Change `isaac_ros_jp6.0` to `isaac_ros_x64` where required.

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

Follow ["Build Isaac ROS Docker Image" instructions for SBC](#build-isaac-ros-docker-image)
