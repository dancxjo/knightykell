"""Build Ubuntu images for configured hosts.

Reads hosts.toml and builds images for hosts that define a ``model`` and
``image = "ubuntu"``. Allows overriding image URLs per host with
``ubuntu_image_url``. Defaults include Ubuntu 24.04 for Raspberry Pi 5.

Examples:
    >>> from scripts.build_ubuntu_images import select_hosts
    >>> cfg = {'hosts': {'a': {'image': 'ubuntu', 'model': 'pi5'}, 'b': {}}}
    >>> select_hosts(config=cfg)
    ['a']
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path
from typing import Iterable

# Reuse load_config from setup_host.py
import importlib.util
import sys

setup_host_path = Path(__file__).parent / "setup_host.py"
spec = importlib.util.spec_from_file_location("setup_host", setup_host_path)
setup_host = importlib.util.module_from_spec(spec)
sys.modules["setup_host"] = setup_host
spec.loader.exec_module(setup_host)
load_config = setup_host.load_config

CONFIG_PATH = Path(__file__).resolve().parent.parent / "hosts.toml"
BUILD_SCRIPT = Path(__file__).resolve().parent / "build_ubuntu_image.sh"


def select_hosts(selected: Iterable[str] | None = None, config: dict | None = None) -> list[str]:
    """Return Ubuntu hosts to build images for.

    Filters by ``image = "ubuntu"``. If ``selected`` is provided, intersects
    with that set.
    """
    cfg = config or load_config(CONFIG_PATH)
    hosts_cfg = cfg.get("hosts", {})
    ubuntu_hosts = [name for name, hc in hosts_cfg.items() if hc.get("image") == "ubuntu"]
    if selected:
        ubuntu_hosts = [h for h in ubuntu_hosts if h in set(selected)]
    return ubuntu_hosts


def resolve_model(host: str, config: dict) -> str:
    """Return model string for host or raise KeyError."""
    model = config.get("hosts", {}).get(host, {}).get("model")
    if not model:
        raise KeyError(f"host {host} missing 'model' in hosts.toml")
    return str(model)


def build_image(host: str, model: str, run=subprocess.run) -> None:
    """Build an image for ``host`` and ``model`` with the shell script."""
    run([str(BUILD_SCRIPT), host, model], check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Build Ubuntu host images")
    parser.add_argument("hosts", nargs="*", help="Hosts to build (default: all Ubuntu hosts)")
    args = parser.parse_args()
    cfg = load_config(CONFIG_PATH)
    hosts = select_hosts(args.hosts or None, config=cfg)
    for host in hosts:
        model = resolve_model(host, cfg)
        print(f"[build] {host} ({model})")
        build_image(host, model)


if __name__ == "__main__":
    main()

