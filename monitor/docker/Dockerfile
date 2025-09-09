# syntax=docker/dockerfile:1.7
ARG ROS_DISTRO=jazzy
ARG ROS_BASE_IMAGE=ros:${ROS_DISTRO}-ros-base
FROM ${ROS_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_WS=/opt/ros_ws

# Use BuildKit cache mounts to speed apt across rebuilds
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    git curl ca-certificates \
    python3-pip python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-image-tools \
    ros-${ROS_DISTRO}-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Workspace (empty by default; handy for overlays)
RUN mkdir -p ${ROS_WS}/src && mkdir -p /opt/entry
WORKDIR ${ROS_WS}

# Runtime env
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "[ -f ${ROS_WS}/install/setup.bash ] && source ${ROS_WS}/install/setup.bash" >> /etc/bash.bashrc

ENV ROS_LOG_DIR=/var/log/ros2 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN mkdir -p /var/log/ros2

# -----------------------------------------------------------------------------
# Include the full source repo inside the image for convenient in-container dev
# -----------------------------------------------------------------------------
ARG REPO_DIR=/opt/src/knightykell
ARG GIT_USER_NAME=""
ARG GIT_USER_EMAIL=""
RUN mkdir -p ${REPO_DIR}
# Copy the entire build context (the repository root as provided by compose)
COPY . ${REPO_DIR}
# Configure git identity if provided and mark repo as safe
RUN if [ -n "${GIT_USER_NAME}" ]; then git config --global user.name "${GIT_USER_NAME}"; fi && \
    if [ -n "${GIT_USER_EMAIL}" ]; then git config --global user.email "${GIT_USER_EMAIL}"; fi && \
    git config --global init.defaultBranch main && \
    git config --global --add safe.directory ${REPO_DIR}

COPY docker/cyclonedds.xml /opt/entry/cyclonedds.xml
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
# -----------------------------------------------------------------------------
# Include the full source repo inside the image for convenient in-container dev
# -----------------------------------------------------------------------------
ARG REPO_DIR=/opt/src/knightykell
ARG GIT_USER_NAME=""
ARG GIT_USER_EMAIL=""
RUN mkdir -p ${REPO_DIR}
# Copy the entire build context (the repository root as provided by compose)
COPY . ${REPO_DIR}
# Configure git identity if provided and mark repo as safe
RUN if [ -n "${GIT_USER_NAME}" ]; then git config --global user.name "${GIT_USER_NAME}"; fi && \
    if [ -n "${GIT_USER_EMAIL}" ]; then git config --global user.email "${GIT_USER_EMAIL}"; fi && \
    git config --global init.defaultBranch main && \
    git config --global --add safe.directory ${REPO_DIR}
