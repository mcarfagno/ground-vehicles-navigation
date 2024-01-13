#!/bin/bash

# Set necessary env variables

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash
 
# Source the mpc workspace
if [ -f /mpc_ws/devel/setup.bash ]
then
  source /mpc_ws/devel/setup.bash
fi
 
# Execute the command passed into this entrypoint
exec "$@"
