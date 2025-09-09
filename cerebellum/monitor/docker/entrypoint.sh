#!/usr/bin/env bash
set -euo pipefail

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Wayland: ensure runtime dir exists
if [ -n "${WAYLAND_DISPLAY:-}" ] && [ -n "${XDG_RUNTIME_DIR:-}" ]; then
  if [ ! -d "$XDG_RUNTIME_DIR" ]; then
    echo "[entry] WARN: XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR not mounted"
  fi
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "/opt/ros_ws/install/setup.bash" ]; then
  source "/opt/ros_ws/install/setup.bash"
fi

exec "$@"

