# Knightykell Forebrain container
# - Base: ROS 2 Jazzy (ros-base)
# - Adds workspace and includes this repo for in-container dev

ARG ROS_DISTRO=jazzy
ARG ROS_BASE_IMAGE=ros:${ROS_DISTRO}-ros-base
FROM ${ROS_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_WS=/opt/ros_ws

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    git curl ca-certificates tzdata \
    python3-pip python3-colcon-common-extensions python3-vcstool \
    build-essential cmake \
    openssh-server \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-image-tools \
    ros-${ROS_DISTRO}-rosbridge-server \
    # CUDA support (NVIDIA drivers, CUDA toolkit, cuDNN)
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin && \
    mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    wget https://developer.download.nvidia.com/compute/cuda/12.4.1/local_installers/cuda-repo-ubuntu2204-12-4-local_12.4.1-550.54.14-1_amd64.deb && \
    dpkg -i cuda-repo-ubuntu2204-12-4-local_12.4.1-550.54.14-1_amd64.deb && \
    cp /var/cuda-repo-ubuntu2204-12-4-local/cuda-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get install -y --no-install-recommends cuda-toolkit-12-4 libcudnn8 libcudnn8-dev && \
    rm -rf /var/lib/apt/lists/*

# Workspace and dependencies
WORKDIR ${ROS_WS}
COPY docker/ros2_ws.repos ${ROS_WS}/ros2_ws.repos
RUN mkdir -p src && \
    vcs import --input ros2_ws.repos src || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/root/.cache,sharing=locked \
    bash -lc '. /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} || true && \
    colcon build --merge-install || true'

# Source environment at shell startup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "[ -f ${ROS_WS}/install/setup.bash ] && source ${ROS_WS}/install/setup.bash" >> /etc/bash.bashrc

ENV ROS_LOG_DIR=/var/log/ros2 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN mkdir -p /var/log/ros2 /var/run/sshd

# Include the full source repo in the image for convenience
ARG REPO_DIR=/opt/src/knightykell
ARG GIT_USER_NAME=""
ARG GIT_USER_EMAIL=""
RUN mkdir -p ${REPO_DIR}
COPY . ${REPO_DIR}
RUN if [ -n "${GIT_USER_NAME}" ]; then git config --global user.name "${GIT_USER_NAME}"; fi && \
    if [ -n "${GIT_USER_EMAIL}" ]; then git config --global user.email "${GIT_USER_EMAIL}"; fi && \
    git config --global init.defaultBranch main && \
    git config --global --add safe.directory ${REPO_DIR}

# Entrypoint and runtime support
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
COPY docker/bringup.sh /opt/entry/bringup.sh
RUN chmod +x /opt/entry/bringup.sh
COPY docker/cyclonedds.xml /opt/entry/cyclonedds.xml
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/opt/entry/bringup.sh"]

