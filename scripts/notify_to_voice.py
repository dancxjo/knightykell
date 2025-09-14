#!/usr/bin/env python3
"""Bridge desktop notifications (Wayland/X via D-Bus) to ROS 2 voice.

Listens on the session D-Bus for `org.freedesktop.Notifications.Notify` and
publishes a spoken-friendly summary to the `voice` topic. Also echoes the
text to a status topic (default `status/notify`) for displays.

This implementation uses `dbus-monitor --session` for maximum compatibility
without extra Python D-Bus dependencies.

Examples:
    Run the service and trigger a desktop notification::

        $ python3 scripts/notify_to_voice.py  # doctest: +SKIP
        $ notify-send "Build" "Finished successfully"  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import re
import shlex
import subprocess
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


_STR_RE = re.compile(r'^\s*string\s+"(.*)"\s*$')
_TAG_RE = re.compile(r"<[^>]+>")


def _clean(s: str) -> str:
    # Remove Pango/HTML-like tags and condense whitespace
    s = _TAG_RE.sub(" ", s)
    s = re.sub(r"\s+", " ", s).strip()
    return s


class NotifyBridge(Node):
    """Monitor D-Bus notifications and publish to voice and status topics."""

    def __init__(self, status_topic: str = "status/notify") -> None:
        super().__init__("notify_bridge")
        self._pub_voice = self.create_publisher(String, "voice", 10)
        self._pub_status = self.create_publisher(String, status_topic, 10)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        cmd = ["dbus-monitor", "--session", "interface='org.freedesktop.Notifications'"]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
        except FileNotFoundError:
            self.get_logger().error("dbus-monitor not found; install dbus-utils")
            return
        assert proc.stdout is not None
        in_notify = False
        strings: list[str] = []
        for line in proc.stdout:
            if "method call" in line and "member=Notify" in line:
                in_notify = True
                strings = []
                continue
            if in_notify and line.strip().startswith("method return"):
                # End of call block without strings captured
                in_notify = False
                continue
            if in_notify:
                m = _STR_RE.match(line)
                if m:
                    strings.append(m.group(1))
                    # Expected order includes summary (4th string) and body (5th)
                    if len(strings) >= 5:
                        app = _clean(strings[0])
                        summary = _clean(strings[3])
                        body = _clean(strings[4])
                        msg = ". ".join([p for p in (summary, body) if p])
                        if app and app.lower() not in msg.lower():
                            msg = f"{app}: {msg}" if msg else app
                        self._publish(msg)
                        in_notify = False
                        strings = []

    def _publish(self, text: str) -> None:
        s = String(); s.data = text
        self._pub_status.publish(s)
        self._pub_voice.publish(s)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--status-topic", default="status/notify")
    ns = parser.parse_args(argv)
    rclpy.init()
    node = NotifyBridge(status_topic=ns.status_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

