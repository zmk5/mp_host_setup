#!/usr/bin/env bash

# Create a tmux config
touch /.tmux.conf
echo "set-option -g default-command bash" >> /.tmux.conf
echo 'set -g default-terminal "screen-256color"' >> /.tmux.conf

# Source ROS Noetic, Carthography, and Mini Pupper Workspaces
. /opt/ros/noetic/setup.bash
. /workspace/carto_ws/install_isolated/setup.bash
. /workspace/pupper_ws/devel/setup.bash

# Add your own commands here:
#

exec "$@"