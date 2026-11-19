#!/bin/bash
set -e

# Source ROS 2 and the workspace overlay built by the image.
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f /root/unitree_ws/install/setup.bash ]; then
    source /root/unitree_ws/install/setup.bash
fi

exec "$@"
