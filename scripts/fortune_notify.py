#!/usr/bin/env python3
"""Publish periodic fortunes as notifications and voice.

Every ``period`` seconds, generates a short "fortune" (via the ``fortune``
utility if available; otherwise uses a built-in list), and publishes it to
the ``status/notify`` topic and to ``voice``. Optionally, also emits a
desktop notification using ``notify-send`` so it appears in Wayland/X.

Examples:
    Run with defaults (300s):

        $ python3 scripts/fortune_notify.py  # doctest: +SKIP

    Faster interval and also post a desktop notification:

        $ python3 scripts/fortune_notify.py --period 60 --notify-send  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import random
import subprocess
import time
from typing import Optional
import shutil

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


FALLBACKS = [
    "Stay curious. Stay kind.",
    "Small steps, steady progress.",
    "Robots have a heart: it’s your code.",
    "When in doubt, log it out.",
    "Sensors don’t lie, but they do drift.",
    "A watched build never completes.",
    "Mind on safety; wheels on ground.",
]


def _run_fortune(use_all: bool = False, offensive: bool = False) -> Optional[str]:
    try:
        # Resolve fortune binary; often installed at /usr/games/fortune
        exe = shutil.which("fortune") or "/usr/games/fortune"
        args = [exe, "-s"]
        if use_all:
            args.insert(1, "-a")
        if offensive:
            args.insert(1, "-o")
        out = subprocess.check_output(args, text=True, timeout=2)
        # Normalize whitespace to one line for displays
        return " ".join(out.strip().split())
    except Exception:
        return None


def _notify_send(summary: str, body: str | None = None) -> None:
    try:
        cmd = ["notify-send", summary]
        if body:
            cmd.append(body)
        subprocess.Popen(cmd)
    except Exception:
        pass


class FortuneNode(Node):
    """Publish fortunes periodically to `status/notify` and `voice`."""

    def __init__(self, period: float = 300.0, do_notify: bool = False,
                 use_all: bool = False, offensive: bool = False) -> None:
        super().__init__("fortune_notify")
        self._pub_voice = self.create_publisher(String, "voice", 10)
        self._pub_status = self.create_publisher(String, "status/notify", 10)
        self._period = float(period)
        self._do_notify = bool(do_notify)
        self._use_all = bool(use_all)
        self._offensive = bool(offensive)
        self._last = 0.0
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        now = time.monotonic()
        if now - self._last < self._period:
            return
        self._last = now
        text = _run_fortune(self._use_all, self._offensive) or random.choice(FALLBACKS)
        msg = String(); msg.data = text
        self._pub_status.publish(msg)
        self._pub_voice.publish(msg)
        if self._do_notify:
            _notify_send("Fortune", text)


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--period", type=float, default=300.0, help="seconds between fortunes")
    p.add_argument("--notify-send", action="store_true", help="also emit a desktop notification")
    p.add_argument("--all", action="store_true", help="use all databases (-a)")
    p.add_argument("--offensive", action="store_true", help="include offensive fortunes (-o)")
    ns = p.parse_args(argv)
    rclpy.init()
    node = FortuneNode(period=ns.period, do_notify=ns.notify_send,
                       use_all=ns.all, offensive=ns.offensive)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
