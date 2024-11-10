#!/bin/bash
set -e

# Check if /root/robot is empty
if [ -z "$(ls -A /root/robot)" ]; then
    echo "/root/robot is empty. Copying backup files..."
    cp -a /root/robot_backup/. /root/robot/
else
    echo "/root/robot is not empty. Skipping copy."
fi

# Source the ROS setup files
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$ROBOT_WORKSPACE/install/setup.bash"

# Execute the passed command
exec "$@"