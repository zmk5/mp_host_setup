FROM ros:noetic-ros-base-focal

ARG IP_MASTER=localhost
ARG IP_LOCAL=127.0.0.1

ENV ROS_MASTER_URI http://${IP_MASTER}:11311
ENV ROS_IP ${IP_LOCAL}

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        curl \
        git \
        nano \
        ninja-build \
        python3-catkin-tools \
        python3-pip \
        python3-rosdep \
        python3-sphinx \
        python3-wstool \
        libcairo2-dev \
        libceres-dev \
        libgflags-dev \
        libgoogle-glog-dev \
        liblua5.2-dev \
        libpcl-dev \
        libudev-dev \
        ros-noetic-amcl \
        ros-noetic-base-local-planner \
        ros-noetic-cv-bridge \
        ros-noetic-dwa-local-planner \
        ros-noetic-ecl-threads \
        ros-noetic-gazebo-ros \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
        ros-noetic-gazebo-plugins \
        ros-noetic-global-planner \
        ros-noetic-gmapping \
        ros-noetic-hector-gazebo-plugins \
        ros-noetic-hector-mapping \
        ros-noetic-hector-sensors-description \
        ros-noetic-joint-state-publisher \
        ros-noetic-joint-state-publisher-gui \
        ros-noetic-joy \
        ros-noetic-map-server \
        ros-noetic-move-base \
        ros-noetic-navfn \
        ros-noetic-octomap \
        ros-noetic-octomap-server \
        ros-noetic-pcl-conversions \
        ros-noetic-pcl-ros \
        ros-noetic-pointcloud-to-laserscan \
        ros-noetic-robot-localization \
        ros-noetic-robot-state-publisher \
        ros-noetic-ros-controllers \
        ros-noetic-rosserial \
        ros-noetic-rosserial-python \
        ros-noetic-rqt \
        ros-noetic-rviz \
        ros-noetic-tf2-eigen \
        ros-noetic-velodyne-gazebo-plugins \
        ros-noetic-xacro \
        software-properties-common \
        stow \
        sudo \
        tmux \
        wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Stop QT from using MITSHM extension
ENV QT_X11_NO_MITSHM=1

COPY ["pupper_entrypoint.sh", "/pupper_entrypoint.sh"]
ENTRYPOINT ["/pupper_entrypoint.sh"]
CMD ["/bin/bash"]