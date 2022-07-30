#!/usr/bin/env bash

# Create a tmux config
touch /.tmux.conf
echo "set-option -g default-command bash" >> /.tmux.conf
echo 'set -g default-terminal "screen-256color"' >> /.tmux.conf

# Source ROS Noetic, Carthography, and Mini Pupper Workspaces
. /opt/ros/noetic/setup.bash
. /workspace/carto_ws/install_isolated/setup.bash
. /workspace/pupper_ws/devel/setup.bash
export ROS_PACKAGE_PATH=/workspace/carto_ws/install_isolated/share:$ROS_PACKAGE_PATH

# Add your own commands here:
#

exec "$@"