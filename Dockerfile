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
  nano \
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
  usbutils \
  vim \
  wget \
  xvfb \
  && apt-get clean -qq

# C/C++ build
RUN DEBIAN_FRONTEND=noninteractive \
  apt-get install --no-install-recommends -y \
  g++-10 ccache
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 99
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 99
ENV CC=/usr/lib/ccache/gcc-10
ENV CXX=/usr/lib/ccache/g++-10

# eProsima/Fast-DDS
ENV RMW_IMPLEMENTATION="rmw_fastrtps_cpp"

# create user bb
ARG USERNAME=bb
RUN useradd $USERNAME --create-home --shell /bin/bash -g sudo
RUN usermod -aG video,dialout bb
RUN echo 'bb:bb' | chpasswd
RUN mkdir /home/${USERNAME}/.ccache && chown -R $USERNAME /home/${USERNAME}/.ccache
VOLUME /home/${USERNAME}/.ccache
# COPY bashrc /home/${USERNAME}/.bashrc
RUN mkdir /home/${USERNAME}/dwone/
RUN mkdir /home/${USERNAME}/Micro-XRCE-DDS-Agent
COPY Micro-XRCE-DDS-Agent/ /home/${USERNAME}/Micro-XRCE-DDS-Agent

WORKDIR /home/${USERNAME}/
# RUN git clone https://github.com/PX4/PX4-Autopilot.git
# Update shared library cache
RUN ldconfig /usr/local/lib/
USER ${USERNAME}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
