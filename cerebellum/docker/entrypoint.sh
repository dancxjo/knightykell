#!/usr/bin/env bash
set -euo pipefail

# Prepare and start SSHD (host network expected; host SSH should run on 2222)
mkdir -p /var/run/sshd
ssh-keygen -A >/dev/null 2>&1 || true
/usr/sbin/sshd -D &

# Source ROS env
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "/opt/ros_ws/install/setup.bash" ]; then
  source "/opt/ros_ws/install/setup.bash"
fi

exec "$@"
