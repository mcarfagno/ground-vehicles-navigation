FROM osrf/ros:noetic-desktop-full 

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Install base utilities
RUN apt-get update && \
    apt-get install -y \
    git \
    gnutls-bin \
    jq \
    less \
    openssh-client \
    python-psutil \
    software-properties-common \
    unzip \
    wget \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages

# Install CasADi optimization framework
# see https://github.com/casadi/casadi/wiki/InstallationLinux
# 1- dependencies
RUN apt-get update && \
    apt-get install -y \
    gcc \
    g++ \
    gfortran \
    git \
    patch \
    wget \
    pkg-config \
    liblapack-dev \
    libmetis-dev \
    coinor-libipopt-dev \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages

# 2- build casadi from source
RUN cd /tmp \
    && git clone https://github.com/casadi/casadi.git -b main casadi \
    && cd casadi \
    && mkdir build \
    && cd build \
    && cmake -DWITH_IPOPT=ON .. \
    && make \
    && sudo make install \
    && cd /tmp \
    && rm -rf casadi

# Install ROS packages
RUN apt-get update && \
    apt-get install -y \
    python3-catkin-tools \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-velodyne-simulator \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages

# *** Install and build Polaris GEM e2 Simulator ***
RUN mkdir -p ~/gem_ws/src
WORKDIR /gem_ws/src
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git

WORKDIR /gem_ws
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make

# *** Install and build MPC assignment packages ***
RUN mkdir ~/mpc_ws
COPY src/mpc_gazebo /mpc_ws/src/mpc_gazebo

WORKDIR /mpc_ws
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make

RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers

COPY docker/env.sh /etc/profile.d/ade_env.sh
COPY docker/entrypoint /ade_entrypoint

ENTRYPOINT ["/ade_entrypoint"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & wait ${!}"]
