# Docker Setup for ROS Noetic + Gazebo

Multi-stage Docker setup with GUI support, GPU acceleration, and live development mode.

## Quick Start

```bash
cd docker

# Build images
docker compose build

# Run simulation (production)
docker compose run --rm mpc-demo

# Development mode (mounted source, dev tools)
docker compose run --rm mpc-devel

# Custom command
docker compose run --rm mpc-demo rosrun rviz rviz

# Interactive shell
docker compose run --rm mpc-devel bash
```

## Container Types

- **mpc-demo** (production): Pre-built workspace, optimized for demos/deployment
- **mpc-devel** (development): Mounted source code, dev tools, live editing

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

## GPU Support

**Intel/AMD**: Works automatically via `/dev/dri`

**NVIDIA**: Uncomment in `compose.yaml`:
```yaml
environment:
  - NVIDIA_VISIBLE_DEVICES=all
  - NVIDIA_DRIVER_CAPABILITIES=all
deploy:
  resources:
    reservations:
      devices: [{driver: nvidia, count: all, capabilities: [gpu]}]
```

## GUI / Display

The container runs as your host user (matching UID/GID) which provides:
- **Automatic X11 permissions** - No xauth or authorization setup needed
- **File ownership match** - Files created in mounted volumes have correct permissions
- **Works on Wayland/X11** - XWayland handles the translation automatically

The setup mounts `/tmp/.X11-unix` and passes your `$DISPLAY` variable. GUI apps (Gazebo, RViz, etc.) should work out of the box.

## Build Stages

1. `base` - System dependencies
2. `dependencies` - CasADi, Eigen 3.4+, ROS packages (heavy, cached)
3. `builder` - Workspace build (runtime only)
4. `runtime` / `devel` - Final images

Target specific stages:
```bash
docker compose build --target devel mpc-devel
```

## External Dependencies

**POLARIS_GEM_e2** is automatically provided:
- In production: Baked into the image
- In development: Symlinked from `/opt/POLARIS_GEM_e2` if not in your host's `src/`

To use your own version, clone to `src/`:
```bash
git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git src/POLARIS_GEM_e2
```

## Persistent Build Artifacts

Development uses named volumes for faster rebuilds:
```bash
docker volume ls | grep mpc  # View volumes
docker volume rm docker_mpc-build docker_mpc-devel docker_mpc-logs  # Clean
```

## Troubleshooting

**GUI not working?**
- Check `$DISPLAY` is set: `echo $DISPLAY`
- Verify X11 socket exists: `ls -la /tmp/.X11-unix/`
- Test with simple app: `docker compose run --rm mpc-devel xclock`

**Gazebo black screen or rendering issues?**
```bash
# Try software rendering
docker compose run --rm -e LIBGL_ALWAYS_SOFTWARE=1 mpc-demo
```

**Build from scratch:**
```bash
docker compose build --no-cache
```

**Clean persistent volumes:**
```bash
docker volume rm docker_mpc-build docker_mpc-devel docker_mpc-logs
```

## Customization

**Add ROS packages**: Edit Dockerfile `dependencies` stage
**Change default command**: Modify `compose.yaml` command section
**Mount additional volumes**: Add to `compose.yaml` volumes section
