#!/usr/bin/env bash
set -e -o pipefail

mkdir -p /var/run/sshd
ssh-keygen -A >/dev/null 2>&1 || true
/usr/sbin/sshd -D &

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

exec "$@"

