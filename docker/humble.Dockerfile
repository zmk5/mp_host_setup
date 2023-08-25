## Create pupper base layer
FROM ros:humble as base

ARG DOMAIN_ID=42
ENV ROS_DOMAIN_ID ${DOMAIN_ID}
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        curl \
        git \
        ninja-build \
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
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-teleop-twist-keyboard \
        software-properties-common \
        stow \
        wget

# Stop QT from using MITSHM extension and RMW implementation
ENV QT_X11_NO_MITSHM=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Download minipupper_ros and dependencies
RUN mkdir -p /pupper_ws/src
WORKDIR /pupper_ws/src
RUN git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2 \
    && vcs import < mini_pupper_ros/.minipupper.repos --recursive

# Build the ROS 2 packages.
WORKDIR /pupper_ws
RUN  source /opt/ros/humble/setup.bash \
    && python3 -m pip install setuptools==58.2.0 \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Copy and set entrypoint
COPY ["docker/entrypoint.sh", "/entrypoint.sh"]
ENTRYPOINT ["/entrypoint.sh"]

## Create pupper simulation layer
FROM base as simulation

# Install extra tools for development
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        ros-humble-rviz2 \
        ros-humble-ros-gz \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Copy and set entrypoint
COPY ["docker/entrypoint.sh", "/entrypoint.sh"]
ENTRYPOINT ["/entrypoint.sh"]

## Create pupper developement layer
FROM base as devel

ARG USERNAME=pupper
ARG UID=1000
ARG GID=1000

# Install extra tools for development
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        gdb \
        gdbserver \
        nano \
        sudo \
        tmux \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /pupper_ws/

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc