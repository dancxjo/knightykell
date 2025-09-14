#!/usr/bin/env python3
"""Publish Create connection health to a status topic for the OLED.

Listens to one or more Create driver topics and emits a concise status line
on `status/create` indicating whether messages are flowing recently.

Heuristics:
- Subscribes to `/odom` (nav_msgs/Odometry) and `/diagnostics` (DiagnosticArray)
- If any message is received within `--timeout` seconds, status is OK
- Otherwise, status is Waiting (shows how long since last message)

Examples:
    $ python3 scripts/create_health.py  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
try:
    from create_msgs.msg import Mode as CreateMode
except Exception:  # pragma: no cover
    CreateMode = None  # type: ignore


class CreateHealth(Node):
    def __init__(self, timeout: float = 5.0) -> None:
        super().__init__("create_health")
        self._pub = self.create_publisher(String, "status/create", 10)
        self._timeout = float(timeout)
        self._last = 0.0
        self._mode: str | None = None
        self._device: str | None = None
        self._moving: bool = False
        self.create_subscription(Odometry, "odom", self._on_any, 10)
        self.create_subscription(DiagnosticArray, "diagnostics", self._on_any, 10)
        # Mode (if available)
        if CreateMode is not None:
            try:
                self.create_subscription(CreateMode, "mode", self._on_mode, 10)
            except Exception:
                pass
        self.create_timer(1.0, self._tick)

    def _on_any(self, _msg) -> None:
        now = time.monotonic()
        self._last = now
        # If diagnostics, try to discover device path
        try:
            if isinstance(_msg, DiagnosticArray):
                for st in _msg.status:
                    # Heuristic: look for keys that mention serial device
                    for kv in getattr(st, "values", []):
                        k = getattr(kv, "key", "").lower()
                        v = str(getattr(kv, "value", ""))
                        if any(s in k for s in ("serial", "device", "port")) and v:
                            self._device = v
                            break
            elif isinstance(_msg, Odometry):
                try:
                    vx = float(_msg.twist.twist.linear.x)
                    vy = float(_msg.twist.twist.linear.y)
                    spd = (vx * vx + vy * vy) ** 0.5
                    self._moving = spd > 0.02
                except Exception:
                    self._moving = False
        except Exception:
            pass

    def _on_mode(self, msg) -> None:
        try:
            # create_msgs/Mode has integer value; expose as numeric if name map missing
            if hasattr(msg, "mode"):
                val = int(msg.mode)
                name = {0: "OFF", 1: "PASSIVE", 2: "SAFE", 3: "FULL"}.get(val)
                self._mode = name or str(val)
            else:
                self._mode = str(msg)
        except Exception:
            self._mode = None

    def _tick(self) -> None:
        now = time.monotonic()
        dt = now - self._last
        ok = dt <= self._timeout
        msg = String()
        parts = ["create:", "ok" if ok else f"waiting ({int(dt)}s)"]
        if self._mode is not None:
            parts.append(f"mode={self._mode}")
        if self._device:
            parts.append(f"dev={self._device}")
        parts.append("moving" if self._moving else "stopped")
        msg.data = " ".join(parts)
        self._pub.publish(msg)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=5.0)
    ns = ap.parse_args(argv)
    rclpy.init()
    node = CreateHealth(timeout=ns.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
