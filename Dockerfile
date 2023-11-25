FROM --platform=linux/arm64 ubuntu:jammy as humble-build

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO humble

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

# FROM humble-build
FROM dustynv/ros:humble-desktop-l4t-r32.7.1

RUN apt-get update && \
  DEBIAN_FRONTEND=noninteractive \
  apt-get install --no-install-recommends -y \
  build-essential \
  atop \
	ca-certificates \
  cmake \
	curl \
  expect \
  gdb \
  git \
	gnupg2 \
  gnutls-bin \
	iputils-ping \
  libbluetooth-dev \
  libccd-dev \
  libcwiid-dev \
	libeigen3-dev \
  libfcl-dev \
  libgflags-dev \
  libgles2-mesa-dev \
  libgoogle-glog-dev \
  libspnav-dev \
  libusb-dev \
	lsb-release \
	net-tools \
	pkg-config \
  protobuf-compiler \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-dbg \
  python3-empy \
  python3-numpy \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  python3-pip \
  python3-venv \
  software-properties-common \
	sudo \
  vim \
	wget \
	xvfb \
  && apt-get clean -qq

RUN useradd bb --create-home --shell /bin/bash -g sudo
RUN echo 'bb:bb' | chpasswd
USER bb
WORKDIR /home/bb

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
