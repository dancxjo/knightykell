#!/usr/bin/env python3
"""Disable psyche-* services that are not listed for this host.

Reads hosts.toml via setup_host.load_config(), computes the allowed
services for the current hostname, and calls disable_absent_services()
to stop/disable any previously installed units that are no longer in
the host's service list.

Usage:
    sudo python3 scripts/reconcile_services.py  # doctest: +SKIP
"""
from __future__ import annotations

import subprocess

from scripts.setup_host import load_config, get_services, disable_absent_services  # type: ignore


def main() -> None:
    cfg = load_config()
    import socket

    host = socket.gethostname()
    services = get_services(host, cfg)
    # Best-effort: disable and stop anything not in services
    disable_absent_services(services, run=subprocess.run)
    print(f"[reconcile] enforced services for {host}: {services}")


if __name__ == "__main__":
    main()

