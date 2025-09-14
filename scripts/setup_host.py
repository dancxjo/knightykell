#!/usr/bin/env python3
"""Host-specific setup utilities for PSYCHE ROS2 deployments.

Examples:
    >>> cfg = {"hosts": {"brainstem": {"services": ["core"]}}}
    >>> get_services("brainstem", cfg)
    ['core']
"""

from __future__ import annotations

import os
import pathlib
import socket
import subprocess
import shutil
import tomllib
import re

CONFIG_PATH = pathlib.Path(__file__).resolve().parent.parent / "hosts.toml"
SSH_DIR = pathlib.Path("/etc/psyche-ssh")
SERVICE_USER = "pete"
HOME_DIR = pathlib.Path(f"/home/{SERVICE_USER}")
REPO_URL = "https://example.com/psyche.git"
REPO_DIR = HOME_DIR / "psyche"
WORKSPACE = HOME_DIR / "ros2_ws"
SYSTEMD_DIR = pathlib.Path("/etc/systemd/system")
VENV_DIR = HOME_DIR / ".venv"
ASSETS_DEFAULT_LABEL = "PSYCHE_DATA"
ASSETS_DEFAULT_MOUNT = pathlib.Path("/mnt/psyche")


def load_config(path: pathlib.Path | str = CONFIG_PATH) -> dict:
    """Load TOML configuration.

    Args:
        path: Path to a TOML file. If not provided, searches common locations.

    Returns:
        Parsed configuration dictionary.

    Examples:
        >>> p = pathlib.Path('sample.toml')
        >>> _ = p.write_text('[hosts]\n[hosts.brainstem]\nservices=["core"]')
        >>> cfg = load_config(p)
        >>> cfg['hosts']['brainstem']['services']
        ['core']
    """
    # Resolve candidate locations if a default path is used
    candidates: list[pathlib.Path]
    if isinstance(path, (str, pathlib.Path)) and pathlib.Path(path) == CONFIG_PATH:
        candidates = [
            CONFIG_PATH,
            pathlib.Path("/opt/psyche/hosts.toml"),
            pathlib.Path("/opt/hosts.toml"),
        ]
    else:
        candidates = [pathlib.Path(path)]

    for cand in candidates:
        try:
            with open(cand, "rb") as fh:
                return tomllib.load(fh)
        except FileNotFoundError:
            continue
    raise FileNotFoundError(f"No hosts.toml found at: {', '.join(str(c) for c in candidates)}")


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


def get_audio_config(hostname: str, config: dict) -> dict:
    """Return optional audio configuration for ``hostname``.

    Looks for ``[hosts.<name>.audio]`` table with optional keys:
    - ``card``: ALSA card index (int)
    - ``device``: ALSA PCM string (e.g., ``"hw:1,0"``)
    """
    return config.get("hosts", {}).get(hostname, {}).get("audio", {})


def get_assets_config(hostname: str, config: dict) -> dict:
    """Return optional assets storage configuration for ``hostname``.

    Looks for ``[hosts.<name>.assets]`` table with optional keys:
    - ``label``: filesystem label to mount (default: ``PSYCHE_DATA``)
    - ``mount``: mount point path (default: ``/mnt/psyche``)
    - ``device``: explicit device path (e.g., ``/dev/nvme0n1p1``)
    - ``format_if_empty``: bool; if true, mkfs when device has no FS
    """
    return config.get("hosts", {}).get(hostname, {}).get("assets", {})


def script_path(name: str) -> str:
    """Return absolute path to a bundled service script.

    Prefers the cloned repo under the service user's home; falls back to
    ``/opt/psyche/scripts`` which is populated in the base image.

    Examples:
        >>> isinstance(script_path('voice_service.py'), str)
        True
    """
    repo_script = REPO_DIR / "scripts" / name
    if repo_script.exists():
        return str(repo_script)
    fallback = pathlib.Path("/opt/psyche/scripts") / name
    return str(fallback)


def stage_runtime_assets() -> None:
    """Copy service scripts and config to /opt/psyche for runtime use.

    Ensures units can run even if the repo is moved or unavailable later.
    """
    opt = pathlib.Path("/opt/psyche")
    scripts_src = REPO_DIR / "scripts"
    # Be tolerant in unprivileged environments (tests), but do full copy on host
    try:
        opt.mkdir(parents=True, exist_ok=True)
        (opt / "scripts").mkdir(parents=True, exist_ok=True)
    except PermissionError:
        return
    for name in (
        "voice_service.py",
        "log_ticker.py",
        "log_summarizer.py",
        "asr_service.py",
        "hrs04_node.py",
        "ssd1306_display_node.py",
        "oled_splash.py",
        "oled_clear.py",
    ):
        src = scripts_src / name
        if src.exists():
            shutil.copy2(src, opt / "scripts" / name)
    # Copy hosts.toml so firstboot/setup can find it under /opt/psyche
    if CONFIG_PATH.exists():
        try:
            shutil.copy2(CONFIG_PATH, opt / "hosts.toml")
        except PermissionError:
            pass


def _read_env_file_var(key: str, default: str | None = None) -> str | None:
    """Return a value for ``key`` from /etc/psyche.env, if present."""
    try:
        env_path = pathlib.Path("/etc/psyche.env")
        if not env_path.exists():
            return default
        for line in env_path.read_text().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                if k == key:
                    return v
    except Exception:
        return default
    return default


def ros2_pkg_exists(name: str, run=subprocess.run) -> bool:
    """Return True if a ROS 2 package ``name`` is discoverable.

    Examples:
        >>> isinstance(ros2_pkg_exists('rclpy'), bool)
        True
    """
    proc = run([
        "bash", "-lc",
        "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && ros2 pkg prefix " + name + " >/dev/null 2>&1"
    ], check=False)
    return getattr(proc, "returncode", 1) == 0


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
    """Populate ``/home/pete/psyche`` for the service user.

    Behavior:
    - If the environment variable ``PSYCHE_SRC`` points to a local checkout,
      copy it into place (preferred for offline provisioning).
    - Otherwise, attempt to ``git clone`` from :data:`REPO_URL`.

    Examples:
        >>> clone_repo(lambda cmd, check: None)  # doctest: +SKIP
    """
    src = os.environ.get("PSYCHE_SRC")
    if src and pathlib.Path(src).exists():
        # Mirror local source into REPO_DIR (update in place)
        REPO_DIR.mkdir(parents=True, exist_ok=True)
        for item in pathlib.Path(src).iterdir():
            dst = REPO_DIR / item.name
            if item.is_dir():
                shutil.copytree(item, dst, dirs_exist_ok=True)
            else:
                shutil.copy2(item, dst)
        run(["chown", "-R", f"{SERVICE_USER}:{SERVICE_USER}", str(REPO_DIR)], check=True)
        return
    if REPO_DIR.exists():
        return
    run(
        ["sudo", "-u", SERVICE_USER, "git", "clone", REPO_URL, str(REPO_DIR)],
        check=True,
    )


def setup_workspace(run=subprocess.run) -> None:
    """Create and build the ROS2 workspace.

    If a local repo exists at :data:`REPO_DIR`, mirror it into ``src/psyche``
    to avoid network fetches; otherwise falls back to ``git clone``.
    """
    # Ensure workspace exists and is writable by the service user
    WORKSPACE.mkdir(parents=True, exist_ok=True)
    run(["chown", "-R", f"{SERVICE_USER}:{SERVICE_USER}", str(WORKSPACE)], check=True)
    src = WORKSPACE / "src"
    src.mkdir(parents=True, exist_ok=True)
    target = src / "psyche"
    if not target.exists():
        if REPO_DIR.exists():
            shutil.copytree(REPO_DIR, target)
            run(["chown", "-R", f"{SERVICE_USER}:{SERVICE_USER}", str(target)], check=True)
        else:
            run(
                ["sudo", "-u", SERVICE_USER, "git", "clone", REPO_URL, str(target)],
                check=True,
            )
    run(
        [
            "sudo",
            "-u",
            SERVICE_USER,
            "bash",
            "-lc",
            f"source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && cd {WORKSPACE} && colcon build",
        ],
        check=True,
    )


def install_service_unit(name: str, cmd: list[str], run=subprocess.run, pre: list[str] | None = None) -> None:
    """Create and enable a systemd unit for a service.

    Args:
        name: Service identifier.
        cmd: Command list executed by the unit.
        pre: Optional list of shell commands to run as ``ExecStartPre``.

    Examples:
        >>> install_service_unit('demo', ['echo', 'hi'])  # doctest: +SKIP
        >>> install_service_unit('with-pre', ['echo','ok'], pre=['/sbin/modprobe i2c-dev'])  # doctest: +SKIP
    """
    unit_path = SYSTEMD_DIR / f"psyche-{name}.service"
    wrapped = (
        f"/bin/bash -lc '"
        f"source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; "
        f"[ -f {HOME_DIR}/ros2_ws/install/setup.bash ] && source {HOME_DIR}/ros2_ws/install/setup.bash >/dev/null 2>&1 || true; "
        f"{' '.join(cmd)}'"
    )
    # Render optional ExecStartPre lines
    pre_lines = "\n".join(
        f"ExecStartPre=/bin/bash -lc '{p}'" for p in (pre or [])
    )
    unit_content = f"""[Unit]
Description=PSYCHE {name} service
After=network.target
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
User={SERVICE_USER}
Environment=ROS_DISTRO=jazzy
EnvironmentFile=-/etc/psyche.env
Restart=on-failure
RestartSec=2
WorkingDirectory={HOME_DIR}
StandardOutput=journal
StandardError=journal
SupplementaryGroups=audio i2c gpio
{pre_lines}
ExecStart={wrapped}

[Install]
WantedBy=multi-user.target
"""
    unit_path.write_text(unit_content)
    run(["systemctl", "enable", "--now", f"psyche-{name}.service"], check=True)


def ensure_shell_env() -> None:
    """Ensure interactive shells source ROS and workspace environments.

    Writes to ``/etc/profile.d/psyche-ros2.sh`` so that new shells have access
    to ``ros2`` and workspace overlays.
    """
    snippet = f"""
# Added by PSYCHE provisioning
if [ -f /opt/ros/jazzy/setup.sh ]; then
  . /opt/ros/jazzy/setup.sh
fi
if [ -f {HOME_DIR}/ros2_ws/install/setup.sh ]; then
  . {HOME_DIR}/ros2_ws/install/setup.sh
fi
""".lstrip()
    p = pathlib.Path("/etc/profile.d/psyche-ros2.sh")
    try:
        p.write_text(snippet)
        try:
            os.chmod(p, 0o644)
        except Exception:
            pass
    except PermissionError:
        # Non-root environment; skip silently
        return


def ensure_service_env() -> None:
    """Write a systemd environment file for PSYCHE services.

    Creates ``/etc/psyche.env`` with a minimal environment ensuring the venv
    and ROS tools are on PATH for all units.
    """
    lines = [
        f"PATH={VENV_DIR}/bin:/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin",
        "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        # Force gpiozero to use modern lgpio backend (works on Pi 5)
        "GPIOZERO_PIN_FACTORY=lgpio",
    ]
    try:
        pathlib.Path("/etc/psyche.env").write_text("\n".join(lines) + "\n")
    except PermissionError:
        # Non-root environment; skip silently
        return


def ensure_voice_env(hostname: str, config: dict) -> None:
    """Set Piper environment variables from host config.

    Reads optional ``[hosts.<name>.voice]`` table with keys:
    - ``model``: Piper model basename (e.g., ``en_US-lessac-high``)
    - ``voices_dir``: Directory for Piper models (default ``/opt/piper/voices``)

    Writes values to ``/etc/psyche.env`` so the systemd unit picks them up.

    Examples:
        >>> cfg = {'hosts': {'h': {'voice': {'model': 'en_US-lessac-high'}}}}
        >>> ensure_voice_env('h', cfg)  # doctest: +SKIP
    """
    env_path = pathlib.Path("/etc/psyche.env")
    env: dict[str, str] = {}
    try:
        if env_path.exists():
            for line in env_path.read_text().splitlines():
                if "=" in line:
                    k, v = line.split("=", 1)
                    env[k] = v
    except Exception:
        # Ignore read errors in restricted environments/tests
        pass
    vcfg = get_service_config(hostname, "voice", config)
    if vcfg:
        if "model" in vcfg:
            env["PIPER_MODEL"] = str(vcfg["model"])
        if "voices_dir" in vcfg:
            env["PIPER_VOICES_DIR"] = str(vcfg["voices_dir"])
    # Write back
    env_lines = [f"{k}={v}" for k, v in env.items()]
    try:
        env_path.write_text("\n".join(env_lines) + "\n")
    except Exception:
        # Ignore write errors in restricted environments/tests
        pass


def _detect_usb_audio_card() -> int | None:
    """Return the ALSA card index for a USB audio device, if present.

    Parses ``/proc/asound/cards`` and returns the first USB card, or ``None``.
    """
    try:
        text = pathlib.Path("/proc/asound/cards").read_text()
    except FileNotFoundError:
        return None
    for line in text.splitlines():
        m = re.match(r"\s*(\d+)\s+\[.*\]:\s+USB", line)
        if m:
            try:
                return int(m.group(1))
            except ValueError:
                continue
    return None


def ensure_audio(cfg: dict | None = None, run=subprocess.run) -> None:
    """Install ALSA utils, add user to audio, and set default device.

    Resolution order:
    1. Use ``cfg['device']`` if provided.
    2. Else use ``cfg['card']`` and default to ``hw:<card>,0``.
    3. Else auto-detect first USB audio card and set as default.
    Writes ``/etc/asound.conf`` and appends env vars into ``/etc/psyche.env``.
    """
    try:
        if os.geteuid() != 0:
            return
    except Exception:
        pass
    try:
        run(["apt-get", "install", "-y", "alsa-utils"], check=True)
        run(["usermod", "-aG", "audio", SERVICE_USER], check=True)
    except Exception:
        return

    card = None
    device = None
    if cfg:
        card = cfg.get("card")
        device = cfg.get("device")
    if device is None and card is None:
        card = _detect_usb_audio_card()
    if device is None and card is not None:
        device = f"hw:{card},0"

    if card is None and device is None:
        return  # nothing to configure

    # Build asound.conf content
    as_lines: list[str] = []
    if card is not None:
        as_lines += [f"defaults.pcm.card {card}", f"defaults.ctl.card {card}"]
    if device is not None:
        as_lines += [
            "pcm.!default {",
            "  type plug",
            f"  slave.pcm \"{device}\"",
            "}",
        ]
    if card is not None:
        as_lines += [
            "ctl.!default {",
            "  type hw",
            f"  card {card}",
            "}",
        ]
    try:
        pathlib.Path("/etc/asound.conf").write_text("\n".join(as_lines) + "\n")
    except PermissionError:
        return

    # Append environment for services
    env_path = pathlib.Path("/etc/psyche.env")
    extra: list[str] = []
    if card is not None:
        extra.append(f"ALSA_CARD={card}")
    if device is not None:
        extra += [f"ALSA_PCM={device}", f"AUDIODEV={device}"]
    extra.append("ESPEAKNG_AUDIO_OUTPUT=alsa")
    try:
        with env_path.open("a") as fh:
            fh.write("\n" + "\n".join(extra) + "\n")
    except (FileNotFoundError, PermissionError):
        try:
            env_path.write_text("\n".join(extra) + "\n")
        except PermissionError:
            pass


def launch_hrs04(cfg: dict, run=subprocess.run) -> None:
    """Install systemd unit for an HRS04 ultrasonic sensor node.

    Args:
        cfg: Service configuration with ``trig_pin`` and ``echo_pin``.

    Examples:
        >>> launch_hrs04({'trig_pin': 1, 'echo_pin': 2})  # doctest: +SKIP
    """
    cmd = [
        str(VENV_DIR / "bin/python"),
        script_path("hrs04_node.py"),
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
    driver = str(cfg.get("driver", "ssd1306"))
    width = str(cfg.get("width", 128))
    height = str(cfg.get("height", 64))
    port = str(cfg.get("port", 1))
    address = str(cfg.get("address", "0x3C"))
    cmd = [
        str(VENV_DIR / "bin/python"),
        script_path("ssd1306_display_node.py"),
        "--driver", driver,
        "--width", width,
        "--height", height,
        "--i2c-port", port,
        "--i2c-address", address,
        *topics,
    ]
    # Ensure I2C character device is available before starting the display
    pre = ["/sbin/modprobe i2c-dev || /usr/sbin/modprobe i2c-dev || true"]
    install_service_unit("display", cmd, run, pre=pre)


def install_ros2(run=subprocess.run) -> None:
    """Install ROS 2 Jazzy on Ubuntu 24.04.

    Ensures the ROS 2 apt repository is configured and installs
    ``ros-jazzy-ros-base`` and ``python3-colcon-common-extensions``. If
    enabling ``universe`` or installing colcon via apt fails, continues and
    attempts a pip-based fallback for colcon.

    Examples:
        >>> calls = []
        >>> install_ros2(lambda cmd, check: calls.append(cmd))
        >>> ["apt-get", "update"] in calls
        True
        >>> ["apt-get", "install", "-y", "ros-jazzy-ros-base"] in calls
        True
    """
    run(["apt-get", "update"], check=True)
    run(
        [
            "apt-get",
            "install",
            "-y",
            "curl",
            "gnupg",
            "lsb-release",
            "software-properties-common",
            "ca-certificates",
            "ripgrep",
        ],
        check=True,
    )
    # Try to enable universe; ignore failures on non-Ubuntu/minimal systems
    try:
        run(["add-apt-repository", "-y", "universe"], check=True)
    except Exception:
        pass
    keyring = pathlib.Path("/usr/share/keyrings/ros.gpg")
    sources = pathlib.Path("/etc/apt/sources.list.d/ros2.list")
    if not keyring.exists():
        run(
            [
                "bash",
                "-lc",
                "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor | tee /usr/share/keyrings/ros.gpg >/dev/null",
            ],
            check=True,
        )
    if not sources.exists():
        run(
            [
                "bash",
                "-lc",
                'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list >/dev/null',
            ],
            check=True,
        )
    run(["apt-get", "update"], check=True)
    # Install ROS base first (separate call to match tests)
    run(["apt-get", "install", "-y", "ros-jazzy-ros-base"], check=True)
    # Optionally install CycloneDDS RMW for better performance
    try:
        run(["apt-get", "install", "-y", "ros-jazzy-rmw-cyclonedds-cpp"], check=True)
    except Exception:
        pass
    # Install colcon from apt; if that fails at runtime, pip fallback happens later in build
    try:
        run(["apt-get", "install", "-y", "python3-colcon-common-extensions"], check=True)
    except Exception:
        # fallback to pip install; ignore if pip not present here
        run(["pip", "install", "colcon-common-extensions"], check=False)


def _venv_pip_install(packages: list[str], run=subprocess.run) -> None:
    """Install Python packages into the service virtualenv.

    Uses the service user's ``uv`` if present for speed and reliability,
    otherwise falls back to the venv's ``pip``.

    Examples:
        >>> calls = []
        >>> _venv_pip_install(['example-pkg'], lambda cmd, check: calls.append(cmd))
        >>> any('example-pkg' in c for cmd in calls for c in cmd)
        True
    """
    uv = HOME_DIR / ".local/bin/uv"
    py = str(VENV_DIR / "bin/python")
    pip = str(VENV_DIR / "bin/pip")
    if uv.exists():
        run([str(uv), "pip", "install", "-p", py, *packages], check=True)
    else:
        run([pip, "install", *packages], check=True)


def install_zeno(run=subprocess.run) -> None:
    """Install zenoh client in the service virtualenv.

    Prefers the ``eclipse-zenoh`` package. If unavailable, falls back to the
    legacy ``zenoh`` name with pre-releases permitted.

    Examples:
        >>> calls = []
        >>> install_zeno(lambda cmd, check: calls.append(cmd))
        >>> any('zenoh' in c for cmd in calls for c in cmd)
        True
    """
    try:
        _venv_pip_install(["eclipse-zenoh"], run)
    except Exception:
        _venv_pip_install(["--pre", "zenoh"], run)


def install_voice_packages(run=subprocess.run) -> None:
    """Install Piper TTS and a default voice model.

    - Installs Piper TTS CLI into the service virtualenv (``piper-tts``)
    - Ensures ``aplay`` is available via ``alsa-utils``
    - Creates ``/opt/piper/voices`` and fetches ``en_US-lessac-high`` model

    Args:
        run: Callable used to execute shell commands.

    Examples:
        >>> calls = []
        >>> install_voice_packages(lambda cmd, check: calls.append(cmd))
        >>> any('piper-tts' in c for cmd in calls for c in cmd)
        True
    """
    voices_dir = pathlib.Path(os.getenv("PIPER_VOICES_DIR") or _read_env_file_var("PIPER_VOICES_DIR", "/opt/piper/voices") or "/opt/piper/voices")
    # Ensure playback utility exists
    run(["apt-get", "install", "-y", "alsa-utils"], check=True)
    # Install Piper TTS into the venv to avoid GTK name conflicts with the mouse tool "piper"
    _venv_pip_install(["piper-tts"], run)
    try:
        voices_dir.mkdir(parents=True, exist_ok=True)
    except PermissionError:
        # In restricted environments (tests), skip creating /opt paths
        pass
    # Download a pleasant default English voice (Lessac, high)
    base = voices_dir
    model = base / "en_US-lessac-high.onnx"
    config = base / "en_US-lessac-high.onnx.json"
    if not model.exists():
        run([
            "curl", "-fsSL",
            "-o", str(model),
            "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/high/en_US-lessac-high.onnx?download=true",
        ], check=True)
    if not config.exists():
        run([
            "curl", "-fsSL",
            "-o", str(config),
            "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/high/en_US-lessac-high.onnx.json?download=true",
        ], check=True)


def install_pi_hw_packages(run=subprocess.run) -> None:
    """Install Raspberry Pi GPIO and OLED dependencies.

    - Apt: python3-gpiozero, python3-lgpio, i2c-tools
    - Venv: luma.oled, Pillow
    """
    try:
        run(["apt-get", "install", "-y", "python3-gpiozero", "python3-lgpio", "i2c-tools"], check=True)
    except Exception:
        return
    # Ensure service user can access GPIO and I2C devices
    run(["usermod", "-aG", "gpio", SERVICE_USER], check=True)
    run(["usermod", "-aG", "i2c", SERVICE_USER], check=True)
    _venv_pip_install(["luma.oled", "Pillow"], run)
    # Best-effort: enable I2C at boot on Raspberry Pi and load module now
    try:
        with open("/proc/device-tree/model", "rb") as fh:
            is_pi = b"Raspberry Pi" in fh.read()
    except Exception:
        is_pi = False
    if is_pi:
        cfg_path = pathlib.Path("/boot/firmware/config.txt")
        if not cfg_path.exists():
            cfg_path = pathlib.Path("/boot/config.txt")
        try:
            content = cfg_path.read_text() if cfg_path.exists() else ""
            if "dtparam=i2c_arm=on" not in content:
                new = (content.rstrip() + "\n" + "dtparam=i2c_arm=on\n") if content else "dtparam=i2c_arm=on\n"
                cfg_path.write_text(new)
        except Exception:
            pass
        # Load i2c-dev immediately so /dev/i2c-* is available without reboot
        run(["/bin/bash", "-lc", "/sbin/modprobe i2c-dev || /usr/sbin/modprobe i2c-dev || true"], check=False)


def install_asr_packages(run=subprocess.run) -> None:
    """Install speech recognition dependencies into the service virtualenv.

    Installs ``openai-whisper`` (importable as ``whisper``), ``webrtcvad``, and
    ``sounddevice`` in the service virtualenv. Also ensures ``ffmpeg`` is
    available via apt as it is used by Whisper for decoding when needed.

    Examples:
        >>> calls = []
        >>> install_asr_packages(lambda cmd, check: calls.append(cmd))
        >>> any('whisper' in c for cmd in calls for c in cmd)
        True
    """
    # Ensure ffmpeg exists for audio decoding
    run(["apt-get", "install", "-y", "ffmpeg"], check=True)
    # Install Python packages into the venv
    _venv_pip_install(["openai-whisper", "webrtcvad", "sounddevice"], run)


def install_llama_cpp(run=subprocess.run) -> None:
    """Install llama-cpp-python into the service virtualenv.

    Installs minimal build tools when available to increase wheel success.

    Examples:
        >>> calls = []
        >>> install_llama_cpp(lambda cmd, check: calls.append(cmd))
        >>> any('llama-cpp' in c for cmd in calls for c in cmd)
        True
    """
    # Best-effort tools (skip errors in restricted tests)
    try:
        run(["apt-get", "install", "-y", "cmake", "build-essential"], check=True)
    except Exception:
        pass
    _venv_pip_install(["llama-cpp-python"], run)


def fetch_llama_model(url: str, dest_dir: str | None = None, run=subprocess.run) -> str | None:
    """Download a GGUF model from ``url`` to ``dest_dir`` and return its path.

    If ``dest_dir`` is omitted, uses ``$LLAMA_MODELS_DIR`` or ``/opt/llama/models``. Creates the
    directory if needed.

    Examples:
        >>> fetch_llama_model('https://example.com/model.gguf', '/tmp/llama', lambda cmd, check: None)  # doctest: +SKIP
        '/tmp/llama/model.gguf'
    """
    base = pathlib.Path(dest_dir or os.getenv("LLAMA_MODELS_DIR") or _read_env_file_var("LLAMA_MODELS_DIR", "/opt/llama/models") or "/opt/llama/models")
    try:
        base.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass
    fname = url.split("/")[-1].split("?")[0]
    if not fname.endswith(".gguf"):
        # If URL lacks filename, pick a default
        fname = "model.gguf"
    out = base / fname
    run(["curl", "-fsSL", "-o", str(out), url], check=True)
    return str(out)


def ensure_assets_storage(hostname: str, config: dict, run=subprocess.run) -> None:
    """Ensure a persistent mount for large assets and write env defaults.

    - Mounts a labeled device (default label ``PSYCHE_DATA``) at
      ``/mnt/psyche`` (configurable per host under ``assets``)
    - Creates standard subdirectories under the mount:
      - ``models/llama`` for GGUFs
      - ``piper/voices`` for Piper models
      - ``cache`` for XDG caches (Whisper, HF, etc.)
    - Writes defaults to ``/etc/psyche.env``:
      - ``LLAMA_MODELS_DIR`` -> ``<mount>/models/llama``
      - ``PIPER_VOICES_DIR`` -> ``<mount>/piper/voices`` (if not already set)
      - ``XDG_CACHE_HOME`` -> ``<mount>/cache``

    If ``assets.format_if_empty`` is true and the device has no filesystem,
    it will be formatted ext4 with the specified label.
    """
    acfg = get_assets_config(hostname, config)
    label = str(acfg.get("label", ASSETS_DEFAULT_LABEL))
    mount = pathlib.Path(str(acfg.get("mount", str(ASSETS_DEFAULT_MOUNT))))
    device = acfg.get("device")
    fmt = bool(acfg.get("format_if_empty", False))

    # Resolve device by label if not provided
    by_label = pathlib.Path("/dev/disk/by-label") / label
    dev_path = pathlib.Path(device) if device else by_label

    # Ensure mountpoint exists
    try:
        mount.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass

    # Try to detect filesystem; if absent and allowed, format
    try:
        proc = run(["blkid", str(dev_path)], check=False)
        has_fs = getattr(proc, "returncode", 1) == 0
        if not has_fs and fmt and device:
            run(["mkfs.ext4", "-F", "-L", label, device], check=True)
            has_fs = True
    except Exception:
        has_fs = False

    # Write /etc/fstab entry using LABEL= for portability
    fstab_line = f"LABEL={label} {mount} ext4 noatime,nofail,x-systemd.automount 0 2\n"
    try:
        fstab = pathlib.Path("/etc/fstab")
        content = fstab.read_text() if fstab.exists() else ""
        if f" {mount} " not in content and f"LABEL={label} " not in content:
            with fstab.open("a") as fh:
                fh.write(fstab_line)
        # Attempt to mount now (best effort)
        run(["systemctl", "daemon-reload"], check=False)
        run(["mount", str(mount)], check=False)
    except Exception:
        pass

    # Ensure standard directories
    llama_dir = mount / "models" / "llama"
    voices_dir = mount / "piper" / "voices"
    cache_dir = mount / "cache"
    for d in (llama_dir, voices_dir, cache_dir):
        try:
            d.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass

    # If a seed exists under /opt/psyche/assets_seed, merge it into the mount
    seed = pathlib.Path("/opt/psyche/assets_seed")
    if seed.exists() and seed.is_dir():
        # Mirror known subtrees; ignore errors in restricted environments
        mapping = {
            seed / "models" / "llama": llama_dir,
            seed / "piper" / "voices": voices_dir,
            seed / "cache": cache_dir,
        }
        for src, dst in mapping.items():
            if src.exists():
                try:
                    for item in src.rglob("*"):
                        rel = item.relative_to(src)
                        target = dst / rel
                        if item.is_dir():
                            target.mkdir(parents=True, exist_ok=True)
                        else:
                            target.parent.mkdir(parents=True, exist_ok=True)
                            shutil.copy2(item, target)
                except Exception:
                    pass

    # Merge env defaults
    env_path = pathlib.Path("/etc/psyche.env")
    env: dict[str, str] = {}
    try:
        if env_path.exists():
            for line in env_path.read_text().splitlines():
                if "=" in line:
                    k, v = line.split("=", 1)
                    env[k] = v
    except Exception:
        pass
    env.setdefault("LLAMA_MODELS_DIR", str(llama_dir))
    env.setdefault("PIPER_VOICES_DIR", str(voices_dir))
    env.setdefault("XDG_CACHE_HOME", str(cache_dir))
    try:
        env_path.write_text("\n".join(f"{k}={v}" for k, v in env.items()) + "\n")
    except Exception:
        pass


def ensure_python_env(run=subprocess.run) -> None:
    """Ensure Python tooling and a venv for the service user.

    - Installs ``python3-venv`` and ``curl`` via apt
    - Creates a virtualenv at :data:`VENV_DIR` with system site packages
    - Installs ``uv`` for the service user (best-effort)

    Examples:
        >>> ensure_python_env(lambda cmd, check: None)  # doctest: +SKIP
    """
    # Skip in non-root environments (e.g., unit tests)
    try:
        if os.geteuid() != 0:
            return
    except Exception:
        pass
    try:
        run(["apt-get", "update"], check=True)
        run(["apt-get", "install", "-y", "python3-venv", "curl", "libportaudio2"], check=True)
    except Exception:
        # If apt is not available or permission denied, skip silently
        return
    try:
        run(["sudo", "-u", SERVICE_USER, "python3", "-m", "venv", "--system-site-packages", str(VENV_DIR)], check=True)
        # Install uv into the service user's ~/.local/bin (best effort)
        run(["sudo", "-u", SERVICE_USER, "bash", "-lc", "curl -LsSf https://astral.sh/uv/install.sh | sh"], check=False)
    except Exception:
        # Non-fatal if venv setup fails here; services may still run using system Python
        pass


def launch_voice(run=subprocess.run) -> None:
    """Install systemd unit for queued text-to-speech.

    Examples:
        >>> launch_voice(lambda cmd, check: None)  # doctest: +SKIP
    """
    cmd = [str(VENV_DIR / "bin/python"), script_path("voice_service.py")]
    install_service_unit("voice", cmd, run)


def launch_logticker(run=subprocess.run) -> None:
    """Install systemd unit publishing logs to the voice service.

    Examples:
        >>> launch_logticker(lambda cmd, check: None)  # doctest: +SKIP
    """
    cmd = [str(VENV_DIR / "bin/python"), script_path("log_ticker.py")]
    install_service_unit("logticker", cmd, run)


def launch_logsummarizer(run=subprocess.run) -> None:
    """Install systemd unit that summarizes logs to ``voice``.

    Examples:
        >>> launch_logsummarizer(lambda cmd, check: None)  # doctest: +SKIP
    """
    cmd = [str(VENV_DIR / "bin/python"), script_path("log_summarizer.py")]
    install_service_unit("logsummarizer", cmd, run)


def ensure_logsummarizer_env(hostname: str, config: dict) -> None:
    """Write environment for the log summarizer from host config.

    Reads optional ``[hosts.<name>.logsummarizer]`` keys and writes
    variables into ``/etc/psyche.env``. Supported keys:
    - ``interval`` -> ``SUMMARY_INTERVAL``
    - ``max_lines`` -> ``SUMMARY_MAX_LINES``
    - ``model_path`` -> ``LLAMA_MODEL_PATH``
    - ``threads`` -> ``LLAMA_THREADS``
    - ``ctx`` -> ``LLAMA_CTX``
    - ``grammar_path`` -> ``LLAMA_GRAMMAR_PATH``
    - ``top_k``/``top_p``/``min_p``/``temp``/``repeat_penalty``/``max_tokens`` -> corresponding ``LLAMA_*`` vars
    - ``gguf_url``: if provided, attempts to download the file and set ``LLAMA_MODEL_PATH``
    - ``ollama_model`` -> ``OLLAMA_MODEL`` (for alternate backend)

    Examples:
        >>> cfg = {'hosts': {'h': {'logsummarizer': {'interval': 10, 'model_path': '/m.gguf'}}}}
        >>> ensure_logsummarizer_env('h', cfg)  # doctest: +SKIP
    """
    env_path = pathlib.Path("/etc/psyche.env")
    env: dict[str, str] = {}
    try:
        if env_path.exists():
            for line in env_path.read_text().splitlines():
                if "=" in line:
                    k, v = line.split("=", 1)
                    env[k] = v
    except Exception:
        pass
    lcfg = get_service_config(hostname, "logsummarizer", config)
    if lcfg:
        # Optional model download
        if "gguf_url" in lcfg:
            try:
                path = fetch_llama_model(str(lcfg["gguf_url"]))
                if path:
                    env["LLAMA_MODEL_PATH"] = path
            except Exception:
                pass
        if "model_path" in lcfg:
            env["LLAMA_MODEL_PATH"] = str(lcfg["model_path"])
        if "interval" in lcfg:
            env["SUMMARY_INTERVAL"] = str(lcfg["interval"])
        if "max_lines" in lcfg:
            env["SUMMARY_MAX_LINES"] = str(lcfg["max_lines"])
        if "threads" in lcfg:
            env["LLAMA_THREADS"] = str(lcfg["threads"])
        if "ctx" in lcfg:
            env["LLAMA_CTX"] = str(lcfg["ctx"])
        if "grammar_path" in lcfg:
            env["LLAMA_GRAMMAR_PATH"] = str(lcfg["grammar_path"])
        if "ollama_model" in lcfg:
            env["OLLAMA_MODEL"] = str(lcfg["ollama_model"])
        # Sampling
        mapping = {
            "top_k": "LLAMA_TOP_K",
            "top_p": "LLAMA_TOP_P",
            "min_p": "LLAMA_MIN_P",
            "temp": "LLAMA_TEMP",
            "repeat_penalty": "LLAMA_REPEAT_PENALTY",
            "max_tokens": "LLAMA_MAX_TOKENS",
        }
        for k, ek in mapping.items():
            if k in lcfg:
                env[ek] = str(lcfg[k])
    try:
        env_path.write_text("\n".join(f"{k}={v}" for k, v in env.items()) + "\n")
    except Exception:
        pass


def launch_asr(cfg: dict | None = None, run=subprocess.run) -> None:
    """Install systemd unit for Whisper-based transcription.

    Examples:
        >>> launch_asr({'model': 'tiny'}, lambda cmd, check: None)  # doctest: +SKIP
    """
    model = (cfg or {}).get("model", "tiny")
    cmd = [str(VENV_DIR / "bin/python"), script_path("asr_service.py"), "--model", model]
    install_service_unit("asr", cmd, run)


def main() -> None:
    """Entry point for host setup."""
    cfg = load_config()
    host = socket.gethostname()
    services = get_services(host, cfg)
    if not services:
        print(f"No services configured for {host}")
        return
    ensure_service_user()
    # Install ROS first so colcon and env are available for builds
    install_ros2()
    clone_repo()
    stage_runtime_assets()
    setup_workspace()
    ensure_ssh_keys()
    ensure_python_env()
    install_zeno()
    ensure_shell_env()
    ensure_service_env()
    # Ensure persistent NVMe assets mount and write defaults
    try:
        ensure_assets_storage(host, cfg)
    except Exception:
        pass
    # Write env for log summarizer from host config, if present
    if "logsummarizer" in services:
        ensure_logsummarizer_env(host, cfg)
    # Write Piper env overrides from host config, if present
    if "voice" in services:
        ensure_voice_env(host, cfg)
    if "voice" in services or "logticker" in services:
        install_voice_packages()
    if "logsummarizer" in services:
        install_llama_cpp()
    if "asr" in services:
        install_asr_packages()
    if "voice" in services:
        ensure_audio(get_audio_config(host, cfg))
    if any(s in services for s in ("hrs04", "display")):
        install_pi_hw_packages()
    installed: list[str] = []
    for svc in services:
        scfg = get_service_config(host, svc, cfg)
        if svc == "hrs04":
            launch_hrs04(scfg)
            installed.append("hrs04")
        elif svc == "display":
            launch_display(scfg)
            installed.append("display")
        elif svc == "voice":
            launch_voice()
            installed.append("voice")
        elif svc == "logticker":
            launch_logticker()
            installed.append("logticker")
        elif svc == "logsummarizer":
            launch_logsummarizer()
            installed.append("logsummarizer")
        elif svc == "asr":
            launch_asr(scfg)
            installed.append("asr")
        else:
            print(f"Unknown service {svc}")
    print(f"Installed services: {', '.join(installed)}")


if __name__ == "__main__":
    main()
