# Multi-stage build: base → dependencies → builder → runtime/devel
# Benefits: cached dependencies (faster rebuilds), smaller images, no build tools in production

# ============================================================================
# STAGE 1: base - System packages and user setup
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

# Create user matching host UID/GID for correct file permissions
RUN groupadd -g ${USER_GID} ros && \
    useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ============================================================================
# STAGE 2: dependencies - Heavy libraries (cached for fast rebuilds)
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
# STAGE 3: builder - Compile workspace (temporary, not shipped)
# ============================================================================
FROM dependencies as builder

RUN mkdir -p /mpc_ws/src
WORKDIR /mpc_ws/src

# Clone external dependencies
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git

# Copy source code from host
COPY src/mppi_control ./mppi_control
COPY src/mpc_gazebo ./mpc_gazebo
COPY src/mpc_control ./mpc_control

# Install ROS dependencies and build
WORKDIR /mpc_ws
RUN source /opt/ros/noetic/setup.bash \
    && rosdep update --rosdistro=noetic || true \
    && rosdep install -y --from-paths src --ignore-src --rosdistro noetic || true

RUN source /opt/ros/noetic/setup.bash \
    && catkin config --extend /opt/ros/noetic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build

# ============================================================================
# STAGE 4a: runtime - Production image (pre-built, no source code)
# ============================================================================
FROM dependencies as runtime

ARG USER_UID=1000
ARG USER_GID=1000

# Copy workspace from builder: devel (compiled) + src (launch/config/scripts/urdf)
# Note: src is needed because catkin's devel layout references it for package discovery
COPY --from=builder /mpc_ws/devel /mpc_ws/devel
COPY --from=builder /mpc_ws/src /mpc_ws/src

# Strip C++ source code and git history (keeps runtime assets: launch, config, scripts, urdf, data)
RUN find /mpc_ws/src -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.c' \
      -o -name '*.h' -o -name '*.hpp' -o -name '*.hh' \) -delete \
    && rm -rf /mpc_ws/src/POLARIS_GEM_e2/.git

# Set ownership for ros user
RUN chown -R ${USER_UID}:${USER_GID} /mpc_ws

WORKDIR /mpc_ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ============================================================================
# STAGE 4b: devel - Development image (live code editing)
# ============================================================================
FROM dependencies as devel

# Development tools
RUN apt-get update && apt-get install -y \
    vim nano gdb valgrind clang-format clang-tidy \
    && rm -rf /var/lib/apt/lists/*

# Workspace directory (src/ mounted from host via docker compose)
RUN mkdir -p /mpc_ws/src && chown -R ${USER_UID}:${USER_GID} /mpc_ws
WORKDIR /mpc_ws

# External dependencies in /opt (symlinked by entrypoint if not on host)
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git /opt/POLARIS_GEM_e2 && \
    chown -R ${USER_UID}:${USER_GID} /opt/POLARIS_GEM_e2

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
