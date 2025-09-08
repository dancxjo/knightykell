# Knightykell Cerebellum container
# - Base: ROS 2 Kaiju
# - Adds Nav2, common sensors, tools

FROM ros:kaiju-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=kaiju \
    ROS_WS=/opt/ros_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl ca-certificates tzdata \
    python3-pip python3-colcon-common-extensions python3-vcstool \
    build-essential cmake \
    # ROS 2 packages
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-hls-lfcd-lds-driver \
    ros-${ROS_DISTRO}-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# Workspace
WORKDIR ${ROS_WS}
COPY ros2_ws.repos ${ROS_WS}/ros2_ws.repos
RUN mkdir -p src && \
    vcs import --input ros2_ws.repos src || true

# Resolve deps and build (best-effort)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} || true && \
    colcon build --merge-install || true

# Runtime env
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "[ -f ${ROS_WS}/install/setup.bash ] && source ${ROS_WS}/install/setup.bash" >> /etc/bash.bashrc

ENV ROS_LOG_DIR=/var/log/ros2 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN mkdir -p /var/log/ros2

# Entrypoint: source ROS and keep container alive by default
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
COPY bringup.sh /opt/entry/bringup.sh
RUN chmod +x /opt/entry/bringup.sh
COPY ../host/oled/oled_client.py /opt/entry/oled_client.py
RUN chmod +x /opt/entry/oled_client.py
COPY ros2_oled_bridge.py /opt/entry/ros2_oled_bridge.py
RUN chmod +x /opt/entry/ros2_oled_bridge.py
COPY ros_statusd.py /opt/entry/ros_statusd.py
RUN chmod +x /opt/entry/ros_statusd.py
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/opt/entry/bringup.sh"]
