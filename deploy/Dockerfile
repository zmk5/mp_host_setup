FROM ros:noetic-ros-base-focal

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        curl \
        git \
        ninja-build \
        python3-pip \
        python3-rosdep \
        python3-wstool \
        ros-noetic-gazebo-ros \
        ros-noetic-rviz \
        ros-noetic-rqt \
        software-properties-common \
        stow \
        sudo \
        tmux \
        wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Stop QT from using MITSHM extension
ENV QT_X11_NO_MITSHM=1

# Download and install cartographer_ros tools and functions
WORKDIR /workspace
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        libcairo2-dev \
        libceres-dev \
        libgflags-dev \
        libgoogle-glog-dev \
        liblua5.2-dev \
        python3-sphinx \
        libpcl-dev \
        ros-noetic-pcl-conversions \
        ros-noetic-robot-state-publisher \
        ros-noetic-tf2-eigen \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir carto_ws \
    && cd carto_ws \
    && wstool init src \
    && wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall \
    && wstool update -t src \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y

RUN cd carto_ws \
    && src/cartographer/scripts/install_abseil.sh \
    && . /opt/ros/noetic/setup.sh \
    && catkin_make_isolated --install --use-ninja \
    && . install_isolated/setup.sh

# RUN mkdir carto_ws \
#     && cd carto_ws \
#     && wstool init src \
#     && wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall \
#     && wstool update -t src \
#     && rosdep init \
#     && rosdep update \
#     && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y \
#     && src/cartographer/scripts/install_abseil.sh \
#     && . apt-get remove ros-${ROS_DISTRO}-abseil-cpp \
#     && catkin_make_isolated --install --use-ninja \
#     && . install_isolated/setup.bash

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        python3-catkin-tools \
        ros-noetic-cv-bridge \
        ros-noetic-gmapping \
        ros-noetic-hector-mapping \
        ros-noetic-octomap \
        ros-noetic-octomap-server \
        ros-noetic-map-server \
        ros-noetic-move-base \
        ros-noetic-global-planner \
        ros-noetic-navfn \
        ros-noetic-dwa-local-planner \
        ros-noetic-base-local-planner \
        ros-noetic-amcl \
        ros-noetic-pointcloud-to-laserscan \
        ros-noetic-pcl-ros \
        ros-noetic-joint-state-publisher-gui \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
        ros-noetic-gazebo-plugins \
        ros-noetic-ros-controllers \
        ros-noetic-joint-state-publisher \
        ros-noetic-xacro \
        ros-noetic-hector-gazebo-plugins \
        ros-noetic-hector-sensors-description \
        ros-noetic-velodyne-gazebo-plugins \
        ros-noetic-rosserial-python \
        ros-noetic-robot-localization \
        ros-noetic-rosserial \
        ros-noetic-ecl-threads \
        ros-noetic-joy \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        libudev-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Download and install mini-pupper ros
RUN mkdir -p pupper_ws/src \
    && cd pupper_ws/src \
    && git clone --recurse-submodules https://github.com/mangdangroboticsclub/minipupper_ros.git \
    && cd .. \
    && rosdep install --from-paths src --ignore-src -r -y \
    && . /opt/ros/noetic/setup.sh \
    && catkin build

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        nano \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_MASTER_URI http://127.0.0.1:11311
ENV ROS_IP 127.0.0.1

COPY ["pupper_entrypoint.sh", "/pupper_entrypoint.sh"]
ENTRYPOINT ["/pupper_entrypoint.sh"]
CMD ["/bin/bash"]