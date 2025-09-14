#!/usr/bin/env python3
"""Quick first-boot sanity check for PSYCHE hosts.

Runs a few checks and publishes a concise summary to ``status/notify`` so it
appears on the OLED, also prints to stdout for logs. Exits non‑zero if any
configured service is inactive.

Checks:
- Systemd units for configured services (from hosts.toml)
- Presence of common/expected ROS topics (best‑effort)

Examples:
    $ python3 scripts/first_boot_sanity.py  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import os
import pathlib
import socket
import subprocess
from typing import Dict, List, Set

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _load_hosts() -> dict:
    paths = [
        pathlib.Path("/opt/psyche/hosts.toml"),
        pathlib.Path("/opt/hosts.toml"),
        pathlib.Path(__file__).resolve().parents[1] / "hosts.toml",
    ]
    for p in paths:
        try:
            if p.exists():
                import tomllib
                return tomllib.loads(p.read_text())
        except Exception:
            continue
    return {"hosts": {}}


def _services_for_host(cfg: dict, host: str) -> List[str]:
    try:
        return list(cfg.get("hosts", {}).get(host, {}).get("services", []) or [])
    except Exception:
        return []


def _display_topics(cfg: dict, host: str) -> List[str]:
    try:
        return list(cfg.get("hosts", {}).get(host, {}).get("display", {}).get("topics", []) or [])
    except Exception:
        return []


def _is_active(unit: str) -> bool:
    try:
        res = subprocess.run(["systemctl", "is-active", unit], capture_output=True, text=True, check=False)
        return (res.stdout or "").strip() == "active"
    except Exception:
        return False


class _Ping(Node):
    def __init__(self) -> None:
        super().__init__("sanity")
        self.pub = self.create_publisher(String, "status/notify", 10)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=2.0, help="ROS graph settle time")
    ns = ap.parse_args(argv)
    cfg = _load_hosts()
    host = socket.gethostname()
    services = _services_for_host(cfg, host)
    # 1) Check systemd units
    units = [f"psyche-{s}.service" for s in services]
    inactive = sorted([u for u in units if not _is_active(u)])
    # 2) Inspect ROS graph for expected topics (best‑effort)
    expected: Set[str] = set(_display_topics(cfg, host))
    svc_expect: Dict[str, List[str]] = {
        "imu": ["/imu", "/imu/data", "/status/imu"],
        "lidar": ["/scan", "/status/scan"],
        "vision": ["/vision/faces", "/vision/objects", "/vision/motion", "/vision/face_id"],
        "notify": ["/status/notify"],
        "fortune": ["/status/notify"],
        "create": ["/status/create", "/odom"],
        "sensors": ["/status/scan", "/status/imu"],
        "topics": ["/topics"],
        "logticker": ["/logs"],
    }
    for s in services:
        expected.update(svc_expect.get(s, []))
    # Gather visible topics
    visible: Set[str] = set()
    try:
        rclpy.init()
        node = _Ping()
        # Allow a brief moment for graph discovery
        import time
        time.sleep(max(0.1, float(ns.timeout)))
        for name, types in node.get_topic_names_and_types():
            visible.add(name)
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()  # type: ignore[name-defined]
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    missing_topics = sorted([t for t in expected if t and t not in visible])
    ok = not inactive
    # Compose concise summary
    parts: List[str] = []
    parts.append(f"services ok {len(units)-len(inactive)}/{len(units)}")
    if inactive:
        parts.append("down:" + ",".join(u.replace("psyche-", "").replace(".service", "") for u in inactive))
    if missing_topics:
        # limit to a few to keep line short
        parts.append("miss:" + ",".join(missing_topics[:3]))
        if len(missing_topics) > 3:
            parts[-1] += "+"
    summary = " | ".join(parts) if parts else "sanity ok"
    # Publish to status/notify
    try:
        rclpy.init()
        node2 = _Ping()
        msg = String(); msg.data = f"sanity: {summary}"
        node2.pub.publish(msg)
        # tiny sleep to allow publish before shutdown
        import time
        time.sleep(0.1)
        node2.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    print(f"sanity: {summary}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

