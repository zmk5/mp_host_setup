#!/bin/bash
source /opt/ros/humble/setup.bash
echo "Sourced ROS 2 ${ROS_DISTRO}"

# Source the pupper workspace, if built
if [ -f /pupper_ws/install/setup.bash ]
then
  source /pupper_ws/install/setup.bash
fi

# Execute the command passed into this entrypoint
exec "$@"