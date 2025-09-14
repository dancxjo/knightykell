#!/usr/bin/env python3
"""Remove PSYCHE services and runtime files from the host.

This script is idempotent and best‑effort. It stops and disables systemd
units with the ``psyche-`` prefix, removes their unit files, clears
environment snippets, and deletes staged runtime files. Optional flags let you
also remove the service user and purge ROS packages.

Examples:
    Basic cleanup (recommended)::

        $ sudo python3 scripts/deprovision.py  # doctest: +SKIP

    Aggressive cleanup (also removes service user and models)::

        $ sudo python3 scripts/deprovision.py --remove-user --assets --purge-ros  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import os
import pathlib
import subprocess
import sys
from typing import Iterable

SERVICE_USER = os.getenv("PSYCHE_USER", "root")
HOME_DIR = pathlib.Path(f"/home/{SERVICE_USER}") if SERVICE_USER != "root" else pathlib.Path("/root")
SYSTEMD_DIR = pathlib.Path("/etc/systemd/system")


def run(cmd: list[str], check: bool = False) -> None:
    subprocess.run(cmd, check=check)


def systemctl(args: Iterable[str]) -> None:
    run(["systemctl", *list(args)], check=False)


def stop_and_disable_units() -> None:
    # Try to stop and disable any psyche-* units
    for suffix in ("service", "timer"):
        try:
            cmd = f"systemctl list-unit-files 'psyche-*.{suffix}' --no-legend | awk '{{print $1}}'"
            out = subprocess.run(
                ["bash", "-lc", cmd],
                text=True,
                capture_output=True,
                check=False,
            ).stdout
        except Exception:
            out = ""
        units = [ln.strip() for ln in out.splitlines() if ln.strip()]
        for u in units:
            systemctl(["stop", u])
            systemctl(["disable", u])
            # Remove unit files if present
            try:
                p = SYSTEMD_DIR / u
                if p.exists():
                    p.unlink(missing_ok=True)  # type: ignore[arg-type]
            except Exception:
                pass
    # Reload systemd daemon
    systemctl(["daemon-reload"])


def remove_env_and_profiles() -> None:
    try:
        (pathlib.Path("/etc/psyche.env")).unlink(missing_ok=True)  # type: ignore[arg-type]
    except Exception:
        pass
    try:
        (pathlib.Path("/etc/profile.d/psyche-ros2.sh")).unlink(missing_ok=True)  # type: ignore[arg-type]
    except Exception:
        pass


def remove_runtime_files(assets: bool = False) -> None:
    # Remove staged runtime tree
    for d in (pathlib.Path("/opt/psyche"),):
        try:
            if d.exists():
                run(["rm", "-rf", str(d)])
        except Exception:
            pass
    # Optional assets cleanup
    if assets:
        for d in (pathlib.Path("/opt/llama/models"), pathlib.Path("/opt/piper/voices"), pathlib.Path("/opt/psyche/cache")):
            try:
                if d.exists():
                    run(["rm", "-rf", str(d)])
            except Exception:
                pass


def remove_user_files(remove_user: bool = False) -> None:
    # Remove venv and workspace under the service user's home
    for d in (HOME_DIR / ".venv", HOME_DIR / "ros2_ws", HOME_DIR / "psyche"):
        try:
            if d.exists():
                run(["rm", "-rf", str(d)])
        except Exception:
            pass
    if remove_user and SERVICE_USER != "root":
        # Try to delete the service user and home directory
        try:
            run(["userdel", "-r", SERVICE_USER], check=False)
        except Exception:
            pass


def purge_ros() -> None:
    # Best-effort purge of ROS2 Jazzy packages
    try:
        run(["apt-get", "purge", "-y", "ros-jazzy-*", "ros-dev-tools"], check=False)
        run(["apt-get", "autoremove", "-y"], check=False)
    except Exception:
        pass


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Deprovision PSYCHE from host")
    parser.add_argument("--remove-user", action="store_true", help="Delete the service user and its home directory")
    parser.add_argument("--assets", action="store_true", help="Also remove model caches under /opt/llama/models and /opt/piper/voices")
    parser.add_argument("--purge-ros", action="store_true", help="Apt purge ROS packages (ros-jazzy-*)")
    ns = parser.parse_args(argv)

    if os.geteuid() != 0:
        print("This script should be run as root (use sudo).", file=sys.stderr)
        sys.exit(1)

    stop_and_disable_units()
    remove_env_and_profiles()
    remove_runtime_files(assets=ns.assets)
    remove_user_files(remove_user=ns.remove_user)
    if ns.purge_ros:
        purge_ros()
    print("[deprovision] complete")


if __name__ == "__main__":
    main()
