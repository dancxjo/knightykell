#!/usr/bin/env python3
"""Publish system logs to the ``voice`` topic as a ticker.

Examples:
    Run the ticker::

        $ python3 log_ticker.py  # doctest: +SKIP
"""
from __future__ import annotations

import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LogTicker(Node):
    """Send journal entries to the ``voice`` topic."""

    def __init__(self) -> None:
        super().__init__("log_ticker")
        self._pub = self.create_publisher(String, "voice", 10)
        threading.Thread(target=self._tail, daemon=True).start()

    def _tail(self) -> None:
        """Follow ``journalctl`` and publish lines."""
        proc = subprocess.Popen(
            ["journalctl", "-f", "-n0", "-o", "cat"],
            stdout=subprocess.PIPE,
            text=True,
        )
        assert proc.stdout is not None
        for line in proc.stdout:
            msg = String()
            msg.data = line.strip()
            self._pub.publish(msg)


def main() -> None:
    """Start the log ticker node."""
    rclpy.init()
    node = LogTicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
