#!/usr/bin/env bash
set -euo pipefail

LOGDIR="${ROS_LOG_DIR:-/var/log/ros2}"
mkdir -p "$LOGDIR"

echo "[bringup] sourcing ROS env"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "/opt/ros_ws/install/setup.bash" ]; then
  source "/opt/ros_ws/install/setup.bash"
fi

oled_msg() { :; }  # placeholder in-container

OLED_SOCK="/run/oled/statusd.sock"
oled() {
  if [ -S "$OLED_SOCK" ] && [ -x /opt/entry/oled_client.py ]; then
    /opt/entry/oled_client.py "$@" || true
  fi
}

oled BRINGUP "Starting bringup"
echo "[bringup] starting background nodes…"

# Create 1 bringup (if available)
if ros2 pkg list | grep -q '^create_bringup$'; then
  echo "[bringup] launching create_bringup"
  oled NODE "create_bringup"
  nohup ros2 launch create_bringup create_1.launch > "$LOGDIR/create_bringup.log" 2>&1 &
else
  echo "[bringup] create_bringup not found; skipping"
fi

# MPU6050 driver (optional; repository-dependent)
if ros2 pkg list | grep -q '^mpu6050driver$'; then
  echo "[bringup] launching mpu6050driver"
  oled NODE "mpu6050driver"
  nohup ros2 launch mpu6050driver mpu6050driver_launch.py > "$LOGDIR/mpu6050driver.log" 2>&1 &
else
  echo "[bringup] mpu6050driver not found; skipping"
fi

# Madgwick filter
if ros2 pkg list | grep -q '^imu_filter_madgwick$'; then
  echo "[bringup] launching imu_filter_madgwick"
  oled NODE "imu_filter_madgwick"
  nohup ros2 run imu_filter_madgwick imu_filter_madgwick_node \
    --ros-args \
    -r /imu/data_raw:=/imu/data_raw \
    -r /imu/data:=/imu/data \
    -p use_mag:=false -p world_frame:=enu -p publish_tf:=false -p stateless:=false -p gain:=0.1 \
    > "$LOGDIR/imu_filter_madgwick.log" 2>&1 &
fi

# Robot localization EKF
if ros2 pkg list | grep -q '^robot_localization$'; then
  echo "[bringup] launching robot_localization EKF (defaults)"
  oled NODE "robot_localization EKF"
  nohup ros2 run robot_localization ekf_node > "$LOGDIR/ekf.log" 2>&1 &
fi

# HLS LDS LIDAR (if device present)
if [ -e /dev/ttyACM0 ] && ros2 pkg list | grep -q '^hls_lfcd_lds_driver$'; then
  echo "[bringup] launching hlds_laser on /dev/ttyACM0"
  oled NODE "hlds_laser"
  nohup ros2 launch hls_lfcd_lds_driver hlds_laser.launch.py port:=/dev/ttyACM0 > "$LOGDIR/hls_lfcd.log" 2>&1 &
fi

# v4l2 camera (if device present)
if [ -e /dev/video0 ] && ros2 pkg list | grep -q '^v4l2_camera$'; then
  echo "[bringup] launching v4l2_camera on /dev/video0"
  oled NODE "v4l2_camera"
  nohup ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:=/dev/video0 \
    -p image_size:='[1280,720]' -p pixel_format:=MJPG -p output_encoding:=bgr8 -p frame_rate:=30 \
    -p use_sensor_data_qos:=true -p camera_name:=usb_cam -p camera_info_url:=file:///opt/entry/camera_info.yaml \
    > "$LOGDIR/cam1.log" 2>&1 &
fi

oled BRINGUP "All launched" "Tailing logs"
echo "[bringup] all available nodes launched; tailing logs"
if command -v python3 >/dev/null 2>&1; then
  nohup python3 /opt/entry/ros_statusd.py > "$LOGDIR/ros_statusd.log" 2>&1 &
fi
exec tail -F "$LOGDIR"/*.log || exec bash
