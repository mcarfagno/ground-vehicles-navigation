# Multi-stage Dockerfile for ROS Noetic with Gazebo

# ============================================================================
# Base stage: Common dependencies
# ============================================================================
FROM osrf/ros:noetic-desktop-full as base

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000

SHELL ["/bin/bash", "-c"]

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-catkin-tools \
    python3-rosdep \
    python3-vcstool \
    python3-tk \
    python3-matplotlib \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create user with same UID/GID as host user
RUN groupadd -g ${USER_GID} ros && \
    useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ============================================================================
# Dependencies stage: Heavy dependencies that rarely change
# ============================================================================
FROM base as dependencies

# Install CasADi dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    gfortran \
    patch \
    wget \
    pkg-config \
    liblapack-dev \
    libmetis-dev \
    coinor-libipopt-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install CasADi from source
RUN cd /tmp \
    && git clone https://github.com/casadi/casadi.git -b main casadi \
    && cd casadi \
    && mkdir build \
    && cd build \
    && cmake -DWITH_IPOPT=ON -DCMAKE_BUILD_TYPE=Release .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && cd /tmp \
    && rm -rf casadi

# Install Eigen 3.4+ system-wide (ROS Noetic ships with 3.3.7)
RUN cd /tmp \
    && git clone https://gitlab.com/libeigen/eigen.git --depth 1 --branch 3.4 \
    && cd eigen \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install \
    && cd /tmp \
    && rm -rf eigen

# Install ROS packages
RUN apt-get update && apt-get install -y \
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
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Build stage: Build the workspace
# ============================================================================
FROM dependencies as builder

# Create workspace
RUN mkdir -p /mpc_ws/src
WORKDIR /mpc_ws/src

# Clone external dependencies
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git

# Copy your source code
COPY src/mppi_control ./mppi_control
COPY src/mpc_gazebo ./mpc_gazebo
COPY src/mpc_control ./mpc_control

# Initialize rosdep and install dependencies
WORKDIR /mpc_ws
RUN source /opt/ros/noetic/setup.bash \
    && rosdep update --rosdistro=noetic || true \
    && rosdep install -y --from-paths src --ignore-src --rosdistro noetic || true

# Build with catkin build (better than catkin_make)
RUN source /opt/ros/noetic/setup.bash \
    && catkin config --extend /opt/ros/noetic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build

# ============================================================================
# Runtime stage: Minimal runtime environment
# ============================================================================
FROM dependencies as runtime

ARG USER_UID=1000
ARG USER_GID=1000

# Copy built workspace from builder
COPY --from=builder /mpc_ws/devel /mpc_ws/devel
COPY --from=builder /mpc_ws/build /mpc_ws/build
COPY --from=builder /mpc_ws/src /mpc_ws/src

# Set ownership for ros user
RUN chown -R ${USER_UID}:${USER_GID} /mpc_ws

WORKDIR /mpc_ws

# Set up entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ============================================================================
# Development stage: For active development with mounted volumes
# ============================================================================
FROM dependencies as devel

# Install additional development tools
RUN apt-get update && apt-get install -y \
    vim \
    nano \
    gdb \
    valgrind \
    clang-format \
    clang-tidy \
    && rm -rf /var/lib/apt/lists/*

# Create workspace (source will be mounted)
RUN mkdir -p /mpc_ws/src && chown -R ${USER_UID}:${USER_GID} /mpc_ws
WORKDIR /mpc_ws

# Clone external dependencies outside the mount point
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git /opt/POLARIS_GEM_e2 && \
    chown -R ${USER_UID}:${USER_GID} /opt/POLARIS_GEM_e2

# Set up entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
