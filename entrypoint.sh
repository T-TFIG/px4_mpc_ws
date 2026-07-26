#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /px4_mpc_ws/install/setup.bash ]; then
    source /px4_mpc_ws/install/setup.bash
fi
exec "$@"
