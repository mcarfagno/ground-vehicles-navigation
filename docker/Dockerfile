# WHAT IS MULTI-STAGE BUILD?
# Instead of one giant image with everything, we build multiple "stages" that
# can depend on each other. This lets us:
#   - Cache heavy dependencies separately (faster rebuilds)
#   - Keep final images smaller (no build tools in production)
#   - Have different images for different purposes (demo vs development)
#
# BUILD FLOW:
#   base → dependencies → builder → runtime (production image)
#                      └→ devel (development image)
#
# Each "FROM ... as <name>" creates a new stage that you can reference later.

# ============================================================================
# STAGE 1: base - System packages and user setup
# ============================================================================
# This stage installs basic tools and creates a user matching your host user.
# Why? So files created in containers have the right permissions on your host.
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
# STAGE 2: dependencies - Heavy stuff that rarely changes
# ============================================================================
# This stage builds/installs CasADi, Eigen, and ROS packages.
# These take a long time to build but rarely change, so Docker caches this layer.
# Result: When you change your code, you don't rebuild these!
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
# STAGE 3: builder - Compile the ROS workspace
# ============================================================================
# This stage only exists to build the code. We copy the compiled result to
# the runtime stage, but don't ship the compiler or source code to production.
# Think of it as a "build server" - does the work then throws away the tools.
FROM dependencies as builder

# Create workspace and copy source code
RUN mkdir -p /mpc_ws/src
WORKDIR /mpc_ws/src

# Clone external dependencies (POLARIS GEM simulator)
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git

# Copy your source code from the host (the "context" is the parent directory)
COPY src/mppi_control ./mppi_control
COPY src/mpc_gazebo ./mpc_gazebo
COPY src/mpc_control ./mpc_control

# Install any missing ROS package dependencies
WORKDIR /mpc_ws
RUN source /opt/ros/noetic/setup.bash \
    && rosdep update --rosdistro=noetic || true \
    && rosdep install -y --from-paths src --ignore-src --rosdistro noetic || true

# Build everything with catkin
RUN source /opt/ros/noetic/setup.bash \
    && catkin config --extend /opt/ros/noetic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build

# ============================================================================
# STAGE 4a: runtime
# ============================================================================
# This is what you run in production. It starts from "dependencies" (which has
# ROS and libraries) but NOT from "builder" (which has compilers and build tools).
# Instead, we COPY the already-compiled workspace FROM the builder stage.
# Result: Smaller image without build tools!
FROM dependencies as runtime

# ARG values don't carry across FROM statements, so we redeclare them here
# These let docker compose pass your user ID so files have correct permissions
ARG USER_UID=1000
ARG USER_GID=1000

# Copy ONLY the compiled workspace from the builder stage (not the build tools!)
# This is the magic of multi-stage: take artifacts, leave the junk behind
COPY --from=builder /mpc_ws/devel /mpc_ws/devel
COPY --from=builder /mpc_ws/build /mpc_ws/build
COPY --from=builder /mpc_ws/src /mpc_ws/src

# Set ownership so the ros user can access everything
RUN chown -R ${USER_UID}:${USER_GID} /mpc_ws

WORKDIR /mpc_ws

# Set up entrypoint script (sources ROS environment automatically)
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ============================================================================
# STAGE 4b: devel
# ============================================================================
# Unlike runtime, this stage does NOT copy compiled code. Instead, docker compose
# mounts your host's src/ directory into the container. This means:
#   - Edit code on your host → instantly reflected in container
#   - You run "catkin build" inside the container when needed
FROM dependencies as devel

# Install development tools for debugging and editing inside the container
RUN apt-get update && apt-get install -y \
    vim \
    nano \
    gdb \
    valgrind \
    clang-format \
    clang-tidy \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory (your src/ will be mounted here by docker compose)
RUN mkdir -p /mpc_ws/src && chown -R ${USER_UID}:${USER_GID} /mpc_ws
WORKDIR /mpc_ws

# Clone external dependencies to /opt (outside the mount point)
# The entrypoint script symlinks this to src/ if you don't have it on your host
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git /opt/POLARIS_GEM_e2 && \
    chown -R ${USER_UID}:${USER_GID} /opt/POLARIS_GEM_e2

# Set up entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
