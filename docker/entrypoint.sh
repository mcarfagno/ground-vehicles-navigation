#!/bin/bash
set -e

# ============================================================================
# ROS Noetic Docker Entrypoint
# ============================================================================

# If POLARIS_GEM_e2 is not in src (because of volume mount), symlink it
if [ ! -e /mpc_ws/src/POLARIS_GEM_e2 ] && [ -d /opt/POLARIS_GEM_e2 ]; then
    echo "⚠ POLARIS_GEM_e2 not found in src, creating symlink..."
    ln -sf /opt/POLARIS_GEM_e2 /mpc_ws/src/POLARIS_GEM_e2
    echo "✓ Linked /opt/POLARIS_GEM_e2 -> /mpc_ws/src/POLARIS_GEM_e2"
fi

# Source ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source the workspace if it exists
if [ -f /mpc_ws/devel/setup.bash ]; then
    source /mpc_ws/devel/setup.bash
    echo "✓ Workspace sourced: /mpc_ws/devel/setup.bash"
else
    echo "⚠ Workspace not built yet. Run 'catkin build' to build."
fi

# Display configuration info
echo ""
echo "=== ROS Environment ==="
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "ROS_MASTER_URI: ${ROS_MASTER_URI:-http://localhost:11311}"
echo "ROS_PACKAGE_PATH: ${ROS_PACKAGE_PATH}"
echo ""
echo "=== Display Configuration ==="
echo "DISPLAY: ${DISPLAY:-not set}"
echo "USER: $(whoami) ($(id -u):$(id -g))"
echo "======================="
echo ""

# Execute the command
exec "$@"
