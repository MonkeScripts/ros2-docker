FROM dustynv/ros:humble-desktop-l4t-r35.4.1
ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=1
ARG ZED_SDK_PATCH=0
ARG JETPACK_MAJOR=5
ARG JETPACK_MINOR=0
ARG L4T_MAJOR=35
ARG L4T_MINOR=4
ARG XACRO_VERSION=2.0.8
ARG DIAGNOSTICS_VERSION=3.0.0
ARG AMENT_LINT_VERSION=0.12.4
ARG GEOGRAPHIC_INFO_VERSION=1.0.4
ARG ROBOT_LOCALIZATION_VERSION=3.4.2

RUN apt-get update && \
  DEBIAN_FRONTEND=noninteractive \
  apt-get install --no-install-recommends -y \
  atop \
  build-essential \
  ca-certificates \
  cmake \
  curl \
  expect \
  gdb \
  git \
  gnupg2 \
  gnutls-bin \
  iputils-ping \
  less \
  libbluetooth-dev \
  libccd-dev \
  libcwiid-dev \
  libeigen3-dev \
  libfcl-dev \
  libgeographic-dev \
  libgflags-dev \
  libgles2-mesa-dev \
  libgoogle-glog-dev \
  libspnav-dev \
  libusb-dev \
  lsb-release \
  nano \
  net-tools \
  pciutils \
  pkg-config \
  protobuf-compiler \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-dbg \
  python3-dev \
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
  udev \
  vim \
  wget \
  xvfb \
  zstd \
  && apt-get clean -qq

RUN python3 -m pip install --upgrade pip && \
  python3 -m pip install --upgrade cython

# eProsima/Fast-DDS
ENV RMW_IMPLEMENTATION="rmw_fastrtps_cpp"

#USER variable
ENV USER=bb
RUN  mkdir -p /etc/udev/rules.d/ 

# C/C++ build
RUN DEBIAN_FRONTEND=noninteractive \
  apt-get install --no-install-recommends -y \
  g++-10 ccache
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 99
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 99
ENV CC=/usr/lib/ccache/gcc-10
ENV CXX=/usr/lib/ccache/g++-10

# #create user bb
ARG USERNAME=bb
RUN useradd $USERNAME --create-home --shell /bin/bash -g sudo
RUN usermod -aG video,dialout bb
RUN echo 'bb:bb' | chpasswd

RUN mkdir /home/${USERNAME}/.ccache && chown -R $USERNAME /home/${USERNAME}/.ccache
RUN mkdir /home/${USERNAME}/dwone/
RUN mkdir /home/${USERNAME}/dwone/src 
VOLUME /home/${USERNAME}/.ccache

# Install the ZED SDK
RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release && \
  apt-get update -y || true && \
  wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run \
  https://download.stereolabs.com/zedsdk/4.1/l4t35.4/jetsons && \
  echo "help me why does this not work" && \
  chmod +x ZED_SDK_Linux_JP.run
RUN echo "Installing.....hehehe" && \
  ./ZED_SDK_Linux_JP.run silent skip_tools && \
  rm -rf /usr/local/zed/resources/* && \
  rm -rf ZED_SDK_Linux_JP.run && \
  rm -rf /var/lib/apt/lists/*

# # Install the ZED ROS2 Wrapper
# WORKDIR /home/${USERNAME}/dwone/src/ 
#  RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
#  RUN wget https://github.com/ros/xacro/archive/refs/tags/${XACRO_VERSION}.tar.gz -O - | tar -xvz && mv xacro-${XACRO_VERSION} xacro && \
#   wget https://github.com/ros/diagnostics/archive/refs/tags/${DIAGNOSTICS_VERSION}.tar.gz -O - | tar -xvz && mv diagnostics-${DIAGNOSTICS_VERSION} diagnostics && \
#   wget https://github.com/ament/ament_lint/archive/refs/tags/${AMENT_LINT_VERSION}.tar.gz -O - | tar -xvz && mv ament_lint-${AMENT_LINT_VERSION} ament-lint && \
#   wget https://github.com/cra-ros-pkg/robot_localization/archive/refs/tags/${ROBOT_LOCALIZATION_VERSION}.tar.gz -O - | tar -xvz && mv robot_localization-${ROBOT_LOCALIZATION_VERSION} robot-localization && \
#   wget https://github.com/ros-geographic-info/geographic_info/archive/refs/tags/${GEOGRAPHIC_INFO_VERSION}.tar.gz -O - | tar -xvz && mv geographic_info-${GEOGRAPHIC_INFO_VERSION} geographic-info && \
#   cp -r geographic-info/geographic_msgs/ . && \
#   rm -rf geographic-info && \
#   git clone https://github.com/ros-drivers/nmea_msgs.git --branch ros2 && \  
#   git clone https://github.com/ros/angles.git --branch humble-devel
# # # Check that all the dependencies are satisfied
# WORKDIR /home/${USERNAME}/dwone/
# RUN apt-get update -y || true && rosdep update && \
#   rosdep install --from-paths src --ignore-src -r -y && \
#   rm -rf /var/lib/apt/lists/*  
# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
#   colcon build --parallel-workers $(nproc) \
#   --event-handlers console_direct+ --base-paths src \
#   --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
#   ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
#   ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
#   ' --no-warn-unused-cli' "
# RUN mkdir /home/${USERNAME}/Micro-XRCE-DDS-Agent
# COPY Micro-XRCE-DDS-Agent/ /home/${USERNAME}/Micro-XRCE-DDS-Agent
# #Create build dir
# # WORKDIR /home/${USERNAME}/Micro-XRCE-DDS-Agent
# # RUN mkdir build
# # WORKDIR /home/bb/Micro-XRCE-DDS-Agent/build
# # RUN cmake ..
# # RUN make
# # RUN make install

WORKDIR /home/${USERNAME}/
# RUN git clone https://github.com/PX4/PX4-Autopilot.git
# Update shared library cache
# RUN ldconfig /usr/local/lib/
# Change ownership and permissions of /usr/local to allow access for the user
RUN chown -R $USERNAME:sudo /usr/local && \
  chmod -R g+rx /usr/local

# Change permissions of /etc to allow access for the user
# RUN chmod -R g+rx /etc

# Switch to the new user

USER $USERNAME

# Set the working directory
WORKDIR /home/$USERNAME

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
