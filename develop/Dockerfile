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

COPY ["pupper_entrypoint.sh", "/pupper_entrypoint.sh"]
ENTRYPOINT ["/pupper_entrypoint.sh"]
WORKDIR /workspace
CMD ["/bin/bash"]