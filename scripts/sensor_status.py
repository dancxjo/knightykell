#!/usr/bin/env python3
"""Summarize /scan and /imu/data to text topics for OLED display.

Publishes human-friendly status lines so the SSD1306 display (which expects
std_msgs/String) can show useful sensor summaries without decoding binary
messages. Subscribes to:
- sensor_msgs/LaserScan on ``/scan``
- sensor_msgs/Imu on ``/imu/data``

Publishes:
- std_msgs/String on ``/status/scan`` (e.g., "scan: n=360 min=0.12 avg=2.31m")
- std_msgs/String on ``/status/imu``  (e.g., "imu: |acc|=9.82m/s² yaw=12.3°")

Examples:
    $ python3 scripts/sensor_status.py  # doctest: +SKIP
    $ ros2 topic echo /status/scan      # doctest: +SKIP
    $ ros2 topic echo /status/imu       # doctest: +SKIP
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu


def _deg(rad: float) -> float:
    return math.degrees(rad)


class SensorStatus(Node):
    def __init__(self) -> None:
        super().__init__("sensor_status")
        self._pub_scan = self.create_publisher(String, "status/scan", 10)
        self._pub_imu = self.create_publisher(String, "status/imu", 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        # Subscribe to common IMU topic names
        self.create_subscription(Imu, "imu/data", self._on_imu, 10)
        self.create_subscription(Imu, "imu", self._on_imu, 10)

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        n = len(ranges)
        if n:
            rmin = min(ranges)
            rmax = max(ranges)
            ravg = sum(ranges) / n
            text = f"scan: n={n} min={rmin:.2f} avg={ravg:.2f} max={rmax:.2f}m"
        else:
            text = "scan: no returns"
        s = String(); s.data = text
        self._pub_scan.publish(s)

    def _on_imu(self, msg: Imu) -> None:
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        amag = math.sqrt(ax*ax + ay*ay + az*az)
        # Yaw estimate if orientation present
        yaw = None
        q = msg.orientation
        if any(v != 0.0 for v in (q.x, q.y, q.z, q.w)):
            # yaw from quaternion (Z axis)
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
        if yaw is not None:
            text = f"imu: |acc|={amag:.2f}m/s² yaw={_deg(yaw):.1f}°"
        else:
            text = f"imu: |acc|={amag:.2f}m/s²"
        s = String(); s.data = text
        self._pub_imu.publish(s)


def main(argv=None) -> None:
    rclpy.init()
    node = SensorStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
