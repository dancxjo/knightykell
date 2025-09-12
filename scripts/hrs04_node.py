#!/usr/bin/env python3
"""Publish ultrasonic range readings from an HC-SR04-like sensor.

Examples:
    Run with GPIO pins (BCM numbering)::

        $ python3 scripts/hrs04_node.py --trig 17 --echo 27  # doctest: +SKIP
"""
from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    from gpiozero import DistanceSensor  # type: ignore
    HAS_GPIOZERO = True
except Exception:  # pragma: no cover
    HAS_GPIOZERO = False


class Hrs04Node(Node):
    """ROS2 node publishing range (meters) at ~10Hz.

    Args:
        trig_pin: BCM pin for trigger.
        echo_pin: BCM pin for echo.
    """

    def __init__(self, trig_pin: int, echo_pin: int) -> None:
        super().__init__("hrs04")
        self._pub = self.create_publisher(Float32, "sensor/range", 10)
        if not HAS_GPIOZERO:
            raise RuntimeError("gpiozero not available; install python3-gpiozero")
        self._sensor = DistanceSensor(echo=echo_pin, trigger=trig_pin, max_distance=4.0)
        self._timer = self.create_timer(0.1, self._tick)

    def _tick(self) -> None:
        msg = Float32()
        try:
            msg.data = float(self._sensor.distance)
        except Exception as e:  # pragma: no cover
            self.get_logger().warning(f"read failed: {e}")
            return
        self._pub.publish(msg)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trig", type=int, required=True)
    parser.add_argument("--echo", type=int, required=True)
    ns = parser.parse_args(argv)
    rclpy.init()
    node = Hrs04Node(trig_pin=ns.trig, echo_pin=ns.echo)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

