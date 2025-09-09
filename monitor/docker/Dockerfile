ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_WS=/opt/ros_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl ca-certificates \
    python3-pip python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-image-tools \
    ros-${ROS_DISTRO}-diagnostic-tools \
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

COPY cyclonedds.xml /opt/entry/cyclonedds.xml
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

