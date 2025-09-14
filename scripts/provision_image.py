#!/usr/bin/env python3
"""Provision core packages inside a mounted Ubuntu image (chroot context).

This script runs inside the target rootfs and installs heavy dependencies to
minimize first-boot work. It reuses functions from ``setup_host.py``.

Usage (inside chroot):
    $ python3 /opt/psyche/provision_image.py <host>
"""
from __future__ import annotations

import argparse
import pathlib
import sys

sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))
from scripts.setup_host import (
    load_config,
    get_services,
    provision_base,
)


def main() -> None:
    ap = argparse.ArgumentParser(description="Provision core packages in chroot")
    ap.add_argument("host", help="Host name from hosts.toml")
    args = ap.parse_args()
    cfg = load_config()
    services = get_services(args.host, cfg)
    if not services:
        print(f"[image] No services configured for {args.host}; skipping")
        return
    print(f"[image] provisioning base packages for {args.host}: {services}")
    provision_base(args.host, cfg, services, image=True)


if __name__ == "__main__":
    main()

