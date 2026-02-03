# Docker Setup for ROS Noetic + Gazebo

Multi-stage Docker setup with GUI support, GPU acceleration, and live development mode.

## What is Multi-Stage Build?

Instead of one massive Docker image with all build tools and dependencies, we use **multiple stages**:

```
┌──────┐
│ base │ ← System packages & user setup
└──┬───┘
   │
┌──▼────────────┐
│ dependencies  │ ← Heavy deps (CasADi, Eigen) - CACHED!
└──┬────────────┘
   │
   ├─► ┌─────────┐
   │   │ builder │ ← Compiles workspace (only used for runtime)
   │   └──┬──────┘
   │      │
   │   ┌──▼────────┐
   │   │  runtime  │ ← Production: copies built code from builder
   │   └───────────┘
   │
   └─► ┌───────┐
       │ devel │ ← Development: mounts your source code
       └───────┘
```

Docker caches each step (layer) in the Dockerfile. If nothing changes, it reuses the cached layer.
This is why rebuilding after code changes is fast, but the first build is slow.

**Benefits:**
- **Faster rebuilds**: Dependencies are cached and don't rebuild when you change code
- **Smaller images**: Production image has no compilers or build artifacts
- **Flexibility**: Same Dockerfile produces both production and development images

## Quick Start

```bash
cd docker

# Build images
docker compose build

# Run simulation
docker compose run --rm mpc-demo

# Development mode (bash, mounted source, dev tools)
docker compose run --rm mpc-devel

# Custom command
docker compose run --rm mpc-demo rosrun rviz rviz
```

## Development Workflow

```bash
# Start dev container
docker compose run --rm mpc-devel

# Inside container: build and run
catkin build
source devel/setup.bash
roslaunch mpc_gazebo mpc_demo.launch
```

Code changes on host are immediately reflected in container. Rebuild with `catkin build`.

## GUI / Display

The container runs as your host user (matching UID/GID) which provides:
- **Automatic X11 permissions** - No xauth or authorization setup needed
- **File ownership match** - Files created in mounted volumes have correct permissions
- **Works on Wayland/X11** - XWayland handles the translation automatically

The setup mounts `/tmp/.X11-unix` and passes your `$DISPLAY` variable. GUI apps (Gazebo, RViz, etc.) should work out of the box.


## External Dependencies

The **POLARIS_GEM_e2** simulator is automatically handled:

- **mpc-demo**: Cloned during image build, already available
- **mpc-devel**: Symlinked from `/opt/POLARIS_GEM_e2` on first run (unless you already have it in `src/`)

To use your own fork, clone it to your host:
```bash
git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git src/POLARIS_GEM_e2
```

The container will use your version instead of the symlink.

## Persistent Volumes (Development)

The `mpc-devel` container uses Docker volumes to store build artifacts:
- `mpc-build`: Compiled objects (`.o` files, binaries)
- `mpc-devel`: Installed packages and setup files
- `mpc-logs`: Build and runtime logs

**Why?** So you don't rebuild everything when the container restarts.

**Clean up** if something breaks:
```bash
docker volume rm docker_mpc-build docker_mpc-devel docker_mpc-logs
```
