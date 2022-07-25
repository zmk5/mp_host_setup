#!/usr/bin/env bash

# Source ROS Noetic files
. /opt/ros/noetic/setup.bash

# Download and install cartographer_ros tools and functions
cd ~/
mkdir carto_ws
cd carto_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
src/cartographer/scripts/install_abseil.sh
sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
catkin_make_isolated --install --use-ninja  # TODO Try and find a catkin build equivalent
source install_isolated/setup.bash

# Download and install mini-pupper ros
cd ~/
mkdir - p pupper_ws/src
cd pupper_ws/src
git clone --recurse-submodules https://github.com/mangdangroboticsclub/minipupper_ros.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
cd ~/

# Add your own commands here:
#

exec "$@"