#!/usr/bin/env python3
"""Host-specific setup utilities for BANKS ROS2 deployments.

Examples:
    >>> cfg = {"hosts": {"brainstem": {"services": ["core"]}}}
    >>> get_services("brainstem", cfg)
    ['core']
"""

from __future__ import annotations

import pathlib
import socket
import subprocess
import tomllib

CONFIG_PATH = pathlib.Path(__file__).resolve().parent.parent / "hosts.toml"
SSH_DIR = pathlib.Path("/etc/banks-ssh")
SERVICE_USER = "pete"
HOME_DIR = pathlib.Path(f"/home/{SERVICE_USER}")
REPO_URL = "https://example.com/banks.git"
REPO_DIR = HOME_DIR / "banks"
WORKSPACE = HOME_DIR / "ros2_ws"
SYSTEMD_DIR = pathlib.Path("/etc/systemd/system")


def load_config(path: pathlib.Path | str = CONFIG_PATH) -> dict:
    """Load TOML configuration.

    Args:
        path: Path to a TOML file.

    Returns:
        Parsed configuration dictionary.

    Examples:
        >>> p = pathlib.Path('sample.toml')
        >>> _ = p.write_text('[hosts]\n[hosts.brainstem]\nservices=["core"]')
        >>> cfg = load_config(p)
        >>> cfg['hosts']['brainstem']['services']
        ['core']
    """
    with open(path, "rb") as fh:
        return tomllib.load(fh)


def get_services(hostname: str, config: dict) -> list[str]:
    """Return services configured for ``hostname``.

    Args:
        hostname: Name of the host.
        config: Configuration dictionary.

    Returns:
        List of services, empty if none.

    Examples:
        >>> config = {'hosts': {'a': {'services': ['core']}}}
        >>> get_services('a', config)
        ['core']
        >>> get_services('b', config)
        []
    """
    return config.get("hosts", {}).get(hostname, {}).get("services", [])


def get_service_config(hostname: str, service: str, config: dict) -> dict:
    """Return configuration for ``service`` on ``hostname``.

    Args:
        hostname: Name of the host.
        service: Service identifier.
        config: Configuration dictionary.

    Returns:
        Dictionary of configuration values, empty if missing.

    Examples:
        >>> cfg = {
        ...     'hosts': {
        ...         'brainstem': {'services': ['hrs04'], 'hrs04': {'trig_pin': 1}}
        ...     }
        ... }
        >>> get_service_config('brainstem', 'hrs04', cfg)['trig_pin']
        1
    """
    return config.get("hosts", {}).get(hostname, {}).get(service, {})


def ensure_ssh_keys() -> None:
    """Generate shared SSH keys if absent."""
    SSH_DIR.mkdir(parents=True, exist_ok=True)
    private_key = SSH_DIR / "id_ed25519"
    if not private_key.exists():
        subprocess.run(
            ["ssh-keygen", "-t", "ed25519", "-N", "", "-f", str(private_key)],
            check=True,
        )


def ensure_service_user(run=subprocess.run) -> None:
    """Create the ``pete`` service user if missing.

    Examples:
        >>> calls = []
        >>> def fake_run(cmd, check=False):
        ...     calls.append(cmd)
        ...     class R: returncode = 1
        ...     return R()
        >>> ensure_service_user(fake_run)
        >>> calls[1][:2]
        ['useradd', '--create-home']
    """
    proc = run(["id", SERVICE_USER], check=False)
    if getattr(proc, "returncode", 1) != 0:
        run(["useradd", "--create-home", SERVICE_USER], check=True)


def clone_repo(run=subprocess.run) -> None:
    """Clone the BANKS repository for the service user."""
    if not REPO_DIR.exists():
        run(
            ["sudo", "-u", SERVICE_USER, "git", "clone", REPO_URL, str(REPO_DIR)],
            check=True,
        )


def setup_workspace(run=subprocess.run) -> None:
    """Create and build the ROS2 workspace."""
    src = WORKSPACE / "src"
    src.mkdir(parents=True, exist_ok=True)
    if not (src / "banks").exists():
        run(
            ["sudo", "-u", SERVICE_USER, "git", "clone", REPO_URL, str(src / "banks")],
            check=True,
        )
    run(
        [
            "sudo",
            "-u",
            SERVICE_USER,
            "bash",
            "-lc",
            f"cd {WORKSPACE} && colcon build",
        ],
        check=True,
    )


def install_service_unit(name: str, cmd: list[str], run=subprocess.run) -> None:
    """Create and enable a systemd unit for a service.

    Args:
        name: Service identifier.
        cmd: Command list executed by the unit.

    Examples:
        >>> install_service_unit('demo', ['echo', 'hi'])  # doctest: +SKIP
    """
    unit_path = SYSTEMD_DIR / f"banks-{name}.service"
    unit_content = f"""[Unit]
Description=BANKS {name} service
After=network.target

[Service]
Type=simple
User={SERVICE_USER}
ExecStart={' '.join(cmd)}

[Install]
WantedBy=multi-user.target
"""
    unit_path.write_text(unit_content)
    run(["systemctl", "enable", "--now", f"banks-{name}.service"], check=True)


def launch_hrs04(cfg: dict, run=subprocess.run) -> None:
    """Install systemd unit for an HRS04 ultrasonic sensor node.

    Args:
        cfg: Service configuration with ``trig_pin`` and ``echo_pin``.

    Examples:
        >>> launch_hrs04({'trig_pin': 1, 'echo_pin': 2})  # doctest: +SKIP
    """
    cmd = [
        "ros2",
        "run",
        "sensors",
        "hrs04_sensor",
        f"--trig={cfg.get('trig_pin')}",
        f"--echo={cfg.get('echo_pin')}",
    ]
    install_service_unit("hrs04", cmd, run)


def launch_display(cfg: dict, run=subprocess.run) -> None:
    """Install systemd unit to show topics on an SSD1306 OLED.

    Args:
        cfg: Service configuration with ``topics`` list.

    Examples:
        >>> launch_display({'topics': ['/foo']})  # doctest: +SKIP
    """
    topics = cfg.get("topics", [])
    cmd = ["ros2", "run", "display", "ssd1306_display", *topics]
    install_service_unit("display", cmd, run)


def install_ros2(run=subprocess.run) -> None:
    """Install ROS2 Jazzy base packages.

    Args:
        run: Callable used to execute shell commands.

    Examples:
        >>> calls = []
        >>> install_ros2(lambda cmd, check: calls.append(cmd))
        >>> calls[0][:2]
        ['apt-get', 'update']
    """
    run(["apt-get", "update"], check=True)
    run(["apt-get", "install", "-y", "ros-jazzy-ros-base"], check=True)


def install_zeno(run=subprocess.run) -> None:
    """Install Zeno communication layer via ``pip``.

    Args:
        run: Callable used to execute shell commands.

    Examples:
        >>> calls = []
        >>> install_zeno(lambda cmd, check: calls.append(cmd))
        >>> calls[0][:2]
        ['pip', 'install']
    """
    run(["pip", "install", "zenoh"], check=True)


def install_voice_packages(run=subprocess.run) -> None:
    """Install speech synthesis packages.

    Args:
        run: Callable used to execute shell commands.

    Examples:
        >>> calls = []
        >>> install_voice_packages(lambda cmd, check: calls.append(cmd))
        >>> calls[0][:3]
        ['apt-get', 'install', '-y']
    """
    run(["apt-get", "install", "-y", "espeak-ng", "mbrola", "mbrola-us1"], check=True)


def launch_voice(run=subprocess.run) -> None:
    """Install systemd unit for queued text-to-speech.

    Examples:
        >>> launch_voice(lambda cmd, check: None)  # doctest: +SKIP
    """
    cmd = ["python3", str(REPO_DIR / "scripts" / "voice_service.py")]
    install_service_unit("voice", cmd, run)


def launch_logticker(run=subprocess.run) -> None:
    """Install systemd unit publishing logs to the voice service.

    Examples:
        >>> launch_logticker(lambda cmd, check: None)  # doctest: +SKIP
    """
    cmd = ["python3", str(REPO_DIR / "scripts" / "log_ticker.py")]
    install_service_unit("logticker", cmd, run)


def main() -> None:
    """Entry point for host setup."""
    cfg = load_config()
    host = socket.gethostname()
    services = get_services(host, cfg)
    if not services:
        print(f"No services configured for {host}")
        return
    ensure_service_user()
    clone_repo()
    setup_workspace()
    ensure_ssh_keys()
    install_ros2()
    install_zeno()
    if "voice" in services or "logticker" in services:
        install_voice_packages()
    for svc in services:
        scfg = get_service_config(host, svc, cfg)
        if svc == "hrs04":
            launch_hrs04(scfg)
        elif svc == "display":
            launch_display(scfg)
        elif svc == "voice":
            launch_voice()
        elif svc == "logticker":
            launch_logticker()
        else:
            print(f"Unknown service {svc}")
    print(f"Installed services: {', '.join(services)}")


if __name__ == "__main__":
    main()
