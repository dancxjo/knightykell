"""Utilities for building Raspberry Pi images for configured hosts.

Example:
    >>> from scripts.build_images import get_hosts
    >>> cfg = {'hosts': {'a': {'image': 'rpi'}, 'b': {}}}
    >>> get_hosts(config=cfg)
    ['a']
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path
from typing import Iterable

# Dynamically import load_config from setup_host.py
import importlib.util
import sys
setup_host_path = Path(__file__).parent / "setup_host.py"
spec = importlib.util.spec_from_file_location("setup_host", setup_host_path)
setup_host = importlib.util.module_from_spec(spec)
sys.modules["setup_host"] = setup_host
spec.loader.exec_module(setup_host)
load_config = setup_host.load_config

CONFIG_PATH = Path(__file__).resolve().parent.parent / "hosts.toml"
BUILD_SCRIPT = Path(__file__).resolve().parent / "build_rpi_image.sh"

def get_hosts(selected: Iterable[str] | None = None, config: dict | None = None) -> list[str]:
    """Return Raspberry Pi hosts to build images for.

    Args:
        selected: Optional iterable of host names to filter.
        config: Parsed configuration; if omitted, loads :data:`CONFIG_PATH`.

    Returns:
        List of host names with ``image = "rpi"``.

    Examples:
        >>> cfg = {'hosts': {'x': {'image': 'rpi'}, 'y': {}}}
        >>> get_hosts(config=cfg)
        ['x']
        >>> get_hosts(['y'], cfg)
        []
    """
    cfg = config or load_config(CONFIG_PATH)
    hosts_cfg = cfg.get("hosts", {})
    rpi_hosts = [
        name
        for name, hc in hosts_cfg.items()
        if hc.get("image") == "rpi"
    ]
    if selected:
        rpi_hosts = [h for h in rpi_hosts if h in set(selected)]
    return rpi_hosts

def build_image(host: str, run=subprocess.run) -> None:
    """Build an image for ``host`` using the build script.

    Args:
        host: Host name to build.
        run: Command runner, defaults to :func:`subprocess.run`.

    Examples:
        >>> calls = []
        >>> build_image('brainstem', run=lambda cmd, check: calls.append(cmd))
        >>> calls[0][-1]
        'brainstem'
    """
    run([str(BUILD_SCRIPT), host], check=True)

def main() -> None:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description="Build host images")
    parser.add_argument("hosts", nargs="*", help="Hosts to build (default: all)")
    args = parser.parse_args()
    hosts = get_hosts(args.hosts or None)
    for host in hosts:
        print(f"[build] {host}")
        build_image(host)

if __name__ == "__main__":
    main()
