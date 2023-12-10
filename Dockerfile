FROM dustynv/ros:humble-desktop-l4t-r35.3.1

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
RUN usermod -aG video bb
RUN echo 'bb:bb' | chpasswd

USER bb
WORKDIR /home/bb
COPY "./cuda-samples/Samples/deviceQuery/deviceQuery" "/home/bb/deviceQuery"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["./deviceQuery"]
