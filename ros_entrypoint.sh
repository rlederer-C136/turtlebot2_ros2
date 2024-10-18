#!/bin/bash
set -e

# Source the ROS setup files
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$ROBOT_WORKSPACE/install/setup.bash"

# Execute the passed command
exec "$@"