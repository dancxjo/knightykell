#!/usr/bin/env bash
set -e -o pipefail

LOGDIR="${ROS_LOG_DIR:-/var/log/ros2}"
mkdir -p "$LOGDIR"

echo "[forebrain] sourcing ROS env"
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi
if [ -f "/opt/ros_ws/install/setup.bash" ]; then
  set +u
  source "/opt/ros_ws/install/setup.bash"
  set -u
fi

echo "[forebrain] environment ready; idle (logs will stream)"
exec tail -F "$LOGDIR" || exec bash

