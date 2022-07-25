#!/usr/bin/env bash

# Source ROS Noetic, Carthography, and Mini Pupper Workspaces
. /opt/ros/noetic/setup.bash
. /workspace/carto_ws/install_isolated/setup.bash
. /workspace/pupper_ws/devel/setup.bash

# Add your own commands here:
#

exec "$@"