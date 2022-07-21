FROM ros:noetic-ros-base-focal

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        curl \
        git \
        python3-pip \
        ros-noetic-gazebo-ros \
        ros-noetic-rviz \
        ros-noetic-rqt \
        software-properties-common \
        sudo \
        tmux \
        wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ARG XDG_RUNTIME_DIR="/tmp/xdg_runtime_dir"
# RUN mkdir -p ${XDG_RUNTIME_DIR} && chmod 777 ${XDG_RUNTIME_DIR}
# ENV XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}
# Stop QT from using MITSHM extension
ENV QT_X11_NO_MITSHM=1

CMD ["/bin/bash"]