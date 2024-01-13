FROM osrf/ros:noetic-desktop-full 

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Install misc. deps

RUN apt-get update && \
    apt-get install -y \
    apt-utils \
    python3-tk \
    python3-matplotlib \
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
    ros-noetic-vision-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-velodyne-simulator \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages

# Build the ROS Workspace
RUN mkdir ~/mpc_ws
COPY src/mpc_gazebo /mpc_ws/src/mpc_gazebo
COPY src/mpc_control /mpc_ws/src/mpc_control

WORKDIR /mpc_ws/src
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git
WORKDIR /mpc_ws
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
