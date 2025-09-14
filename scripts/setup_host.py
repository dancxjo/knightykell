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
# Default to running services as root to avoid sudo prompts/passwords.
# Can be overridden via PSYCHE_USER environment variable at runtime if needed.
SERVICE_USER = os.getenv("PSYCHE_USER", "root")
HOME_DIR = pathlib.Path(f"/home/{SERVICE_USER}") if SERVICE_USER != "root" else pathlib.Path("/root")
# ROS 2 distro selection (default jazzy). Can be overridden by env or hosts.toml.
ROS_DISTRO = os.getenv("ROS_DISTRO", "jazzy").strip()
REPO_URL = "https://example.com/psyche.git"
# Place workspace and repo under /opt for easier admin access
REPO_DIR = pathlib.Path("/opt/psyche")
WORKSPACE = pathlib.Path("/opt/ros2_ws")
SYSTEMD_DIR = pathlib.Path("/etc/systemd/system")
VENV_DIR = REPO_DIR / ".venv"
# In containers (no systemd) allow skipping unit management by setting
# PSYCHE_DISABLE_SYSTEMD=1. Also auto-disable if systemd is not present.
DISABLE_SYSTEMD = (
    os.getenv("PSYCHE_DISABLE_SYSTEMD", "").strip().lower() in {"1", "true", "yes"}
    or not pathlib.Path("/run/systemd/system").exists()
)

# Default branch suggestions for common third-party repos per ROS distro
_REPO_BRANCH_DEFAULTS: dict[str, dict[str, str]] = {
    # Create robot bringup
    "github.com/autonomylab/create_robot.git": {
        "jazzy": "jazzy",
        "rolling": "rolling",
    },
    # libcreate fork commonly used with ROS 2
    "github.com/revyos-ros/libcreate.git": {
        "jazzy": "fix-std-string",
        "rolling": "main",
    },
    # Drivers/utilities that generally track main
    "github.com/clydemcqueen/opencv_cam.git": {
        "default": "main",
    },
    "github.com/hiwad-aziz/ros2_mpu6050_driver.git": {
        "default": "main",
    },
}

def _infer_repo_branch(url: str) -> str | None:
    """Return a sensible default branch for a repo URL based on ROS_DISTRO.

    Falls back to a 'default' mapping or None if unknown.
    """
    u = url.lower()
    for key, mapping in _REPO_BRANCH_DEFAULTS.items():
        if key in u:
            # Prefer exact distro, else default
            return mapping.get(ROS_DISTRO.lower()) or mapping.get("default")
    return None


def _has_nvidia() -> bool:
    """Return True if an NVIDIA GPU/driver appears present.

    Checks for ``nvidia-smi`` or driver files under ``/proc/driver/nvidia``.

    Examples:
        >>> isinstance(_has_nvidia(), bool)
        True
    """
    try:
        res = subprocess.run(["nvidia-smi"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if res.returncode == 0:
            return True
    except Exception:
        pass
    try:
        if pathlib.Path("/proc/driver/nvidia/version").exists():
            return True
    except Exception:
        pass
    try:
        out = subprocess.run(["bash", "-lc", "lspci -nnk | grep -i nvidia"], text=True, capture_output=True)
        return bool(out.stdout.strip())
    except Exception:
        return False


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
    """Deprecated: NVMe assets config removed; returns empty dict."""
    return {}


def script_path(name: str) -> str:
    """Return absolute path to a bundled service script.

    Prefers the cloned repo under the service user's home; falls back to
    ``/opt/psyche/scripts`` which is populated in the base image.

    Examples:
        >>> isinstance(script_path('voice_service.py'), str)
        True
    """
    repo_script = REPO_DIR / "scripts" / name
    try:
        if repo_script.exists():
            return str(repo_script)
    except Exception:
        # Permission issues in test envs: prefer fallback
        pass
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
        "chat_service.py",
        "asr_service.py",
        "asr_utterance_service.py",
        "hrs04_node.py",
        "ssd1306_display_node.py",
        "topic_list_service.py",
        "retry_exec.py",
        "create_singer.py",
        "create_health.py",
        "first_boot_sanity.py",
        "notify_to_voice.py",
        "fortune_notify.py",
        "vision_service.py",
        "sensor_status.py",
        "oled_splash.py",
        "oled_clear.py",
        "fetch_models.py",
        "git_updater.py",
    ):
        src = scripts_src / name
        if src.exists():
            dest = opt / "scripts" / name
            try:
                if dest.exists() and dest.samefile(src):
                    continue
            except Exception:
                pass
            shutil.copy2(src, dest)
    # Make admin-friendly perms
    try:
        subprocess.run(["chgrp", "-R", "sudo", str(opt)], check=False)
        subprocess.run(["chmod", "-R", "g+w", str(opt)], check=False)
        subprocess.run(["chown", "-R", f"{SERVICE_USER}:{SERVICE_USER}", str(opt)], check=False)
    except Exception:
        pass
    # Copy hosts.toml so firstboot/setup can find it under /opt/psyche
    if CONFIG_PATH.exists():
        try:
            dest_cfg = opt / "hosts.toml"
            try:
                if dest_cfg.exists() and dest_cfg.samefile(CONFIG_PATH):
                    pass
                else:
                    shutil.copy2(CONFIG_PATH, dest_cfg)
            except Exception:
                shutil.copy2(CONFIG_PATH, dest_cfg)
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
    ros_setup = f"/opt/ros/{ROS_DISTRO}/setup.bash"
    proc = run([
        "bash", "-lc",
        f"source {ros_setup} >/dev/null 2>&1 && ros2 pkg prefix {name} >/dev/null 2>&1"
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
    """Ensure the configured service user exists when not root.

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
    # If running as root user for services, nothing to create.
    proc = run(["id", SERVICE_USER], check=False)
    if getattr(proc, "returncode", 1) != 0 and SERVICE_USER != "root":
        run(["useradd", "--create-home", SERVICE_USER], check=True)


def clone_repo(run=subprocess.run) -> None:
    """Populate ``/opt/psyche`` with the runtime repo for services.

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
    try:
        if REPO_DIR.exists():
            return
    except Exception:
        # Permission issues in tests
        pass
    run(
        ["sudo", "-u", SERVICE_USER, "git", "clone", REPO_URL, str(REPO_DIR)],
        check=True,
    )


def clone_external_repos(config: dict, run=subprocess.run) -> None:
    """Clone additional Git repos specified under ``[hosts.<name>.repos]``.

    Accepts either a list of URLs or a table of {url, dest, branch} entries.
    Repos are cloned into ``/opt/external/<name>`` by default (admin-writable).

    Examples:
        >>> cfg = {'hosts': {'h': {'repos': ['https://example.com/repo.git']}}}
        >>> clone_external_repos(cfg, lambda cmd, check: None)  # doctest: +SKIP
    """
    host = socket.gethostname()
    host_cfg = config.get("hosts", {}).get(host, {})
    repos = host_cfg.get("repos", [])
    if not repos:
        return
    base = pathlib.Path("/opt/external")
    try:
        base.mkdir(parents=True, exist_ok=True)
    except Exception:
        return
    def _clone(url: str, dest: pathlib.Path | None = None, branch: str | None = None) -> None:
        name = dest or base / pathlib.Path(url).stem.replace(".git", "")
        name = pathlib.Path(name)
        if name.exists():
            return
        cmd = ["sudo", "-u", SERVICE_USER, "git", "clone"]
        if branch:
            cmd += ["-b", branch]
        cmd += [url, str(name)]
        run(cmd, check=False)
        # Admin-friendly perms
        try:
            run(["chgrp", "-R", "sudo", str(name)], check=False)
            run(["chmod", "-R", "g+w", str(name)], check=False)
        except Exception:
            pass
    if isinstance(repos, list):
        for item in repos:
            if isinstance(item, str):
                _clone(item)
            elif isinstance(item, dict) and "url" in item:
                dest = pathlib.Path(item.get("dest")) if item.get("dest") else None
                _clone(str(item["url"]), dest, str(item.get("branch")) if item.get("branch") else None)
    elif isinstance(repos, dict):
        for _, entry in repos.items():
            if isinstance(entry, dict) and "url" in entry:
                dest = pathlib.Path(entry.get("dest")) if entry.get("dest") else None
                _clone(str(entry["url"]), dest, str(entry.get("branch")) if entry.get("branch") else None)


def setup_workspace(run=subprocess.run) -> None:
    """Create and build the ROS2 workspace.

    If a local repo exists at :data:`REPO_DIR`, mirror it into ``src/psyche``
    to avoid network fetches; otherwise falls back to ``git clone``.
    """
    # Ensure workspace exists and is writable by the service user
    WORKSPACE.mkdir(parents=True, exist_ok=True)
    # Admin-friendly perms: group sudo, group-writable
    run(["chgrp", "-R", "sudo", str(WORKSPACE)], check=False)
    run(["chmod", "-R", "g+w", str(WORKSPACE)], check=False)
    run(["chown", "-R", f"{SERVICE_USER}:{SERVICE_USER}", str(WORKSPACE)], check=False)
    src = WORKSPACE / "src"
    src.mkdir(parents=True, exist_ok=True)
    target = src / "psyche"
    if not target.exists():
        re_exists = False
        try:
            re_exists = REPO_DIR.exists()
        except Exception:
            re_exists = False
        if re_exists:
            shutil.copytree(REPO_DIR, target)
            run(["chown", "-R", f"{SERVICE_USER}:{SERVICE_USER}", str(target)], check=True)
        else:
            run(
                ["sudo", "-u", SERVICE_USER, "git", "clone", REPO_URL, str(target)],
                check=True,
            )
    # Patch third-party sources if present (e.g., fix missing <array> include)
    try:
        for p in src.rglob("mpu6050sensor.h"):
            try:
                h = p.read_text()
            except Exception:
                continue
            if "#include <array>" not in h:
                if "#include <unordered_map>" in h:
                    h = h.replace("#include <unordered_map>", "#include <unordered_map>\n#include <array>")
                else:
                    h = "#include <array>\n" + h
                p.write_text(h)
    except Exception:
        pass
    # Ensure common native deps are present (best-effort), then rosdep, then build
    try:
        run(["apt-get", "update"], check=False)
        run(["apt-get", "install", "-y", "libboost-dev", "libboost-system-dev"], check=False)
    except Exception:
        pass
    try:
        # Set sane permissions on rosdep cache/registry (no-op if already set)
        run(["bash", "-lc", "command -v rosdep >/dev/null 2>&1 && rosdep fix-permissions || true"], check=False)
    except Exception:
        pass
    # Run rosdep as root so apt installs do not prompt for a password
    try:
        run(
            [
                "bash",
                "-lc",
                f"source /opt/ros/{ROS_DISTRO}/setup.bash >/dev/null 2>&1 && cd {WORKSPACE} && command -v rosdep >/dev/null 2>&1 && rosdep update && rosdep install --from-paths src --ignore-src -r -y || true",
            ],
            check=False,
        )
    except Exception:
        pass
    # Build workspace (ok as root by default; supports non-root SERVICE_USER if set)
    if SERVICE_USER == "root":
        run(
            [
                "bash",
                "-lc",
                f"source /opt/ros/{ROS_DISTRO}/setup.bash >/dev/null 2>&1 && cd {WORKSPACE} && colcon build",
            ],
            check=True,
        )
    else:
        run(
            [
                "sudo",
                "-u",
                SERVICE_USER,
                "bash",
                "-lc",
                f"source /opt/ros/{ROS_DISTRO}/setup.bash >/dev/null 2>&1 && cd {WORKSPACE} && colcon build",
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
    ros_setup = f"/opt/ros/{ROS_DISTRO}/setup.bash"
    wrapped = (
        f"/bin/bash -lc '"
        f"source {ros_setup} >/dev/null 2>&1; "
        f"[ -f {WORKSPACE}/install/setup.bash ] && source {WORKSPACE}/install/setup.bash >/dev/null 2>&1 || true; "
        f"{' '.join(cmd)}'"
    )
    # Always ensure the virtualenv exists before starting, but don't fail hard.
    ensure_venv_pre = (
        f"if [ ! -x {VENV_DIR}/bin/python ]; then "
        f"python3 -m venv --system-site-packages {VENV_DIR} || true; fi"
    )
    # Render optional ExecStartPre lines
    pre_cmds = [ensure_venv_pre, *(pre or [])]
    pre_lines = "\n".join(
        f"ExecStartPre=/bin/bash -lc '{p}'" for p in pre_cmds if p
    )
    unit_content = f"""[Unit]
Description=PSYCHE {name} service
After=network.target
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
User={SERVICE_USER}
Environment=ROS_DISTRO={ROS_DISTRO}
EnvironmentFile=-/etc/psyche.env
Restart=on-failure
RestartSec=2
WorkingDirectory={REPO_DIR}
StandardOutput=journal
StandardError=journal
SupplementaryGroups=audio i2c gpio
{pre_lines}
ExecStart={wrapped}

[Install]
WantedBy=multi-user.target
"""
    try:
        unit_path.write_text(unit_content)
    except PermissionError:
        # Non-root or restricted environment; skip silently
        return
    if DISABLE_SYSTEMD:
        # In containers, skip systemd management entirely.
        return
    # Reload units so changes take effect on re-provision runs
    run(["systemctl", "daemon-reload"], check=True)
    run(["systemctl", "enable", "--now", f"psyche-{name}.service"], check=True)
    # Always restart to pick up script/env updates (idempotent)
    run(["systemctl", "restart", f"psyche-{name}.service"], check=False)


def stop_old_services(services: list[str], run=subprocess.run) -> None:
    """Stop existing systemd units for the given services.

    Non-fatal if a unit is missing. Helps avoid duplicate processes during
    re-provisioning when units are updated in place.

    Examples:
        >>> stop_old_services(["voice", "asr"], lambda cmd, check: None)  # doctest: +SKIP
    """
    if DISABLE_SYSTEMD:
        return
    for name in services:
        try:
            run(["systemctl", "stop", f"psyche-{name}.service"], check=False)
        except Exception:
            pass


def disable_absent_services(services: list[str], run=subprocess.run) -> None:
    """Disable and stop any previously installed psyche-* services not in list.

    This reconciles the machine's running/enabled units with the current
    hosts.toml so that removed services (e.g., "chat") do not keep running.

    Preserves helper units like ``psyche-update.timer``.
    """
    if DISABLE_SYSTEMD:
        return
    desired = set(services)
    try:
        # List all installed psyche-*.service unit files
        cmd = "systemctl list-unit-files 'psyche-*.service' --no-legend | awk '{print $1}'"
        out = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True, check=False).stdout
    except Exception:
        out = ""
    units = [ln.strip() for ln in out.splitlines() if ln.strip()]
    for unit in units:
        # psyche-<name>.service -> <name>
        base = unit.removeprefix("psyche-").removesuffix(".service")
        if not base or base == "update":
            # Keep updater infrastructure intact
            continue
        if base not in desired:
            try:
                run(["systemctl", "disable", "--now", unit], check=False)
            except Exception:
                pass
    # Best-effort daemon reload
    try:
        run(["systemctl", "daemon-reload"], check=False)
    except Exception:
        pass


def ensure_shell_env() -> None:
    """Ensure interactive shells source ROS and workspace environments.

    Writes to ``/etc/profile.d/psyche-ros2.sh`` so that new shells have access
    to ``ros2`` and workspace overlays.
    """
    ros_setup_sh = f"/opt/ros/{ROS_DISTRO}/setup.sh"
    snippet = f"""
# Added by PSYCHE provisioning
# Load service environment (RMW, DOMAIN_ID, etc.) if present
if [ -f /etc/psyche.env ]; then
  . /etc/psyche.env
fi
if [ -f {ros_setup_sh} ]; then
  . {ros_setup_sh}
fi
if [ -f {WORKSPACE}/install/setup.sh ]; then
  . {WORKSPACE}/install/setup.sh
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
    cuda_bin = "/usr/local/cuda/bin"
    cuda_lib = "/usr/local/cuda/lib64"
    path_val = f"{VENV_DIR}/bin:/opt/ros/{ROS_DISTRO}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/usr/games:/usr/local/games"
    try:
        if pathlib.Path(cuda_bin).exists():
            path_val = path_val + f":{cuda_bin}"
    except Exception:
        pass
    lines = [
        f"PATH={path_val}",
        "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        # Force gpiozero to use modern lgpio backend (works on Pi 5)
        "GPIOZERO_PIN_FACTORY=lgpio",
        # Centralize cache path for headless operation
        "XDG_CACHE_HOME=/opt/psyche/cache",
    ]
    try:
        if pathlib.Path(cuda_lib).exists():
            lines.append(f"LD_LIBRARY_PATH={cuda_lib}:${{LD_LIBRARY_PATH:-}}")
    except Exception:
        pass
    try:
        pathlib.Path("/etc/psyche.env").write_text("\n".join(lines) + "\n")
    except PermissionError:
        # Non-root environment; skip silently
        return


def ensure_db_env() -> None:
    """Append local DB connection env to /etc/psyche.env (best-effort)."""
    env_path = pathlib.Path("/etc/psyche.env")
    extra = [
        "QDRANT_URL=http://127.0.0.1:6333",
        "NEO4J_URI=bolt://127.0.0.1:7687",
        "NEO4J_USER=neo4j",
        # Default password must be at least 8 chars
        "NEO4J_PASSWORD=neo4jneo4j",
        "VISION_MODELS_DIR=/opt/psyche/models/vision",
    ]
    try:
        with env_path.open("a") as fh:
            fh.write("\n" + "\n".join(extra) + "\n")
    except Exception:
        pass


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


def provision_base(hostname: str, config: dict, services: list[str], *, image: bool = False, run=subprocess.run) -> None:
    """Install core packages and Python deps; used in-image and on-host.

    When ``image=True``, installs most heavy packages in chroot to reduce
    first-boot work. On-host, main() retains package installs to satisfy tests.

    Steps (best-effort where reasonable):
    - Ensure service user (default root)
    - Install ROS 2 base and optional CycloneDDS
    - Stage runtime assets and SSH keys
    - Ensure Python venv and install zenoh
    - Write base service env and per-service env (voice/chat)
    - If ``image=True``: install voice/asr/llama-cpp packages now
    
    Examples:
        >>> provision_base('h', {'hosts': {'h': {'services': []}}}, [], image=True, run=lambda cmd, check: None)  # doctest: +SKIP
    """
    ensure_service_user(run)
    install_ros2(run)
    try:
        clone_repo(run)
    except Exception:
        pass
    stage_runtime_assets()
    # Clone any extra repos (e.g., external UI libraries)
    try:
        clone_external_repos(config)
    except Exception:
        pass
    try:
        ensure_ssh_keys()
    except Exception:
        pass
    ensure_python_env(run)
    install_zeno(run)
    ensure_shell_env()
    ensure_service_env()
    # Log summarizer removed; LLM reserved for chat
    if "voice" in services:
        ensure_voice_env(hostname, config)
    # Build ROS2 workspace inside the image to reduce first-boot work
    if image:
        try:
            setup_workspace(run)
        except Exception:
            pass
    if image:
        if "voice" in services or "logticker" in services:
            install_voice_packages(run)
        if "chat" in services:
            install_llama_cpp(run)
        if "asr" in services:
            install_asr_packages(run)
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


def ensure_ros2_extra_repos(hostname: str, config: dict, run=subprocess.run) -> None:
    """Clone additional ROS 2 repos into the workspace and (re)build.

    Reads optional ``[hosts.<name>.ros2]`` table with:
    - ``repos``: list of URLs or dicts {url, branch?, dest?}
    - ``domain_id``: optional ROS_DOMAIN_ID
    - ``rmw``: optional RMW implementation (e.g., ``rmw_cyclonedds_cpp``)

    Clones into ``$WORKSPACE/src/<dest or repo-name>`` if missing, then runs a
    best-effort ``rosdep install`` and ``colcon build``. All steps are
    tolerant to errors in test/non-root environments.
    """
    cfg = config.get("hosts", {}).get(hostname, {}).get("ros2", {})
    if not isinstance(cfg, dict):
        return
    repos = cfg.get("repos", [])
    # If the Create service is enabled, ensure create_robot and libcreate are present
    try:
        host_services = (config.get("hosts", {}).get(hostname, {}).get("services", []) or [])
    except Exception:
        host_services = []
    if "create" in host_services:
        # Provide sensible default branches for common repos depending on ROS_DISTRO
        def _branch_for(url: str) -> str | None:
            return _infer_repo_branch(url)
        default_repos = [
            {"url": "https://github.com/AutonomyLab/create_robot.git", "branch": (_branch_for("https://github.com/AutonomyLab/create_robot.git") or "jazzy")},
            {"url": "https://github.com/revyos-ros/libcreate.git", "branch": (_branch_for("https://github.com/revyos-ros/libcreate.git") or "main")},
        ]
        urls = set()
        for r in (repos or []):
            if isinstance(r, str):
                urls.add(r)
            elif isinstance(r, dict) and "url" in r:
                urls.add(str(r["url"]))
        for r in default_repos:
            if r["url"] not in urls:
                if isinstance(repos, list):
                    repos.append(r)
                else:
                    repos = [r]
    if not repos:
        return
    src = WORKSPACE / "src"
    try:
        src.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass

    def _clone(url: str, dest: pathlib.Path | None = None, branch: str | None = None) -> None:
        name = pathlib.Path(url).stem
        if name.endswith(".git"):
            name = name[:-4]
        target = dest or (src / name)
        if target.exists():
            return
        cmd = ["sudo", "-u", SERVICE_USER, "git", "clone"]
        if branch:
            cmd += ["-b", branch]
        cmd += [url, str(target)]
        run(cmd, check=False)

    if isinstance(repos, list):
        for item in repos:
            try:
                if isinstance(item, str):
                    _clone(item, None, _infer_repo_branch(item))
                elif isinstance(item, dict) and "url" in item:
                    dest = pathlib.Path(item.get("dest")) if item.get("dest") else None
                    branch = str(item.get("branch")) if item.get("branch") else None
                    if not branch:
                        branch = _infer_repo_branch(str(item["url"]))
                    _clone(str(item["url"]), dest, branch)
            except Exception:
                continue
    # Best-effort patch: allow MPU6050 driver to pick I2C bus from env
    try:
        for p in src.rglob("mpu6050driver.cpp"):
            try:
                txt = p.read_text()
            except Exception:
                continue
            if "std::make_unique<MPU6050Sensor>()" in txt:
                txt = txt.replace(
                    "std::make_unique<MPU6050Sensor>()",
                    "std::make_unique<MPU6050Sensor>(std::getenv(\"I2C_BUS\") ? std::atoi(std::getenv(\"I2C_BUS\")) : 1)",
                )
                # Ensure <cstdlib> is included
                if "#include <cstdlib>" not in txt:
                    txt = txt.replace("#include <memory>", "#include <memory>\n#include <cstdlib>")
                p.write_text(txt)
        # Best-effort patch: ensure <array> header included for std::array usage
        for p in src.rglob("mpu6050sensor.h"):
            try:
                h = p.read_text()
            except Exception:
                continue
            if "#include <array>" not in h:
                # Insert after the first include block
                if "#include <unordered_map>" in h:
                    h = h.replace("#include <unordered_map>", "#include <unordered_map>\n#include <array>")
                else:
                    h = "#include <array>\n" + h
                p.write_text(h)
    except Exception:
        pass
    # Attempt rosdep + build (best-effort)
    # Some third-party drivers (e.g., HLDS LDS/libcreate) expect Boost components.
    # Ensure Boost dev headers are available even if rosdep skips.
    try:
        run(["apt-get", "install", "-y", "libboost-dev", "libboost-system-dev", "libboost-thread-dev"], check=False)
    except Exception:
        pass
    try:
        # Set sane permissions on rosdep cache/registry (no-op if already set)
        run(["bash", "-lc", "command -v rosdep >/dev/null 2>&1 && rosdep fix-permissions || true"], check=False)
    except Exception:
        pass
    try:
        run([
            "bash", "-lc",
            f"source /opt/ros/{ROS_DISTRO}/setup.bash >/dev/null 2>&1 && cd {WORKSPACE} && command -v rosdep >/dev/null 2>&1 && rosdep update && rosdep install --from-paths src --ignore-src -r -y || true",
        ], check=False)
    except Exception:
        pass
    try:
        if SERVICE_USER == "root":
            run([
                "bash", "-lc",
                f"source /opt/ros/{ROS_DISTRO}/setup.bash >/dev/null 2>&1 && cd {WORKSPACE} && colcon build",
            ], check=False)
        else:
            run([
                "sudo", "-u", SERVICE_USER, "bash", "-lc",
                f"source /opt/ros/{ROS_DISTRO}/setup.bash >/dev/null 2>&1 && cd {WORKSPACE} && colcon build",
            ], check=False)
    except Exception:
        pass


def ensure_ros_network_env(hostname: str, config: dict) -> None:
    """Optionally set ROS networking env (RMW, ROS_DOMAIN_ID) from config."""
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
    cfg = config.get("hosts", {}).get(hostname, {}).get("ros2", {})
    if isinstance(cfg, dict):
        if "rmw" in cfg:
            env["RMW_IMPLEMENTATION"] = str(cfg["rmw"])
        if "domain_id" in cfg:
            env["ROS_DOMAIN_ID"] = str(cfg["domain_id"])
    try:
        env_path.write_text("\n".join(f"{k}={v}" for k, v in env.items()) + "\n")
    except Exception:
        pass


def resolve_ros_distro(hostname: str, config: dict) -> None:
    """Resolve and set the global ROS distribution.

    Priority: $ROS_DISTRO env > hosts.toml [hosts.<name>.ros2].distro > 'jazzy'.
    """
    global ROS_DISTRO
    try:
        if os.getenv("ROS_DISTRO"):
            ROS_DISTRO = os.getenv("ROS_DISTRO", ROS_DISTRO)
            return
        cfg = config.get("hosts", {}).get(hostname, {}).get("ros2", {})
        if isinstance(cfg, dict) and cfg.get("distro"):
            ROS_DISTRO = str(cfg.get("distro")).strip()
    except Exception:
        pass


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
        if SERVICE_USER != "root":
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
        "python3",
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
    page_seconds = str(cfg.get("page_seconds", 8.0))
    tick_interval = str(cfg.get("tick_interval", 0.10))
    cmd = [
        "python3",
        script_path("ssd1306_display_node.py"),
        "--driver", driver,
        "--width", width,
        "--height", height,
        "--i2c-port", port,
        "--i2c-address", address,
        "--page-seconds", page_seconds,
        "--tick-interval", tick_interval,
    ]
    extra = str(cfg.get("extra", ""))
    if extra:
        cmd += ["--extra", extra]
    cmd += [
        *topics,
    ]
    # Ensure I2C character device is available before starting the display
    pre = ["/sbin/modprobe i2c-dev || /usr/sbin/modprobe i2c-dev || true"]
    install_service_unit("display", cmd, run, pre=pre)


def install_ros2(run=subprocess.run) -> None:
    """Install ROS 2 base on Ubuntu 24.04 (configurable distro).

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
    # If an old/broken ROS apt entry exists (e.g., Pop!_OS 'plucky'), disable it
    # before the first update to avoid 404s breaking the cache refresh.
    keyring = pathlib.Path("/usr/share/keyrings/ros.gpg")
    sources = pathlib.Path("/etc/apt/sources.list.d/ros2.list")
    try:
        if sources.exists():
            disabled = sources.with_name(sources.name + ".disabled")
            sources.rename(disabled)
    except Exception:
        # Non-fatal: proceed and let later steps correct the source
        pass

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
    if not keyring.exists():
        run(
            [
                "bash",
                "-lc",
                "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor | tee /usr/share/keyrings/ros.gpg >/dev/null",
            ],
            check=True,
        )
    # Determine correct Ubuntu codename for the ROS apt repo and ensure file content matches.
    # Pop!_OS 24.04 uses UBUNTU_CODENAME=plucky, but ROS packages are published
    # under Ubuntu LTS codenames (e.g., noble for 24.04). Map by ROS_DISTRO first.
    ros_suite_map = {"jazzy": "noble", "humble": "jammy"}
    ros_suite = ros_suite_map.get(ROS_DISTRO.lower())
    ubuntu_codename = None
    try:
        data = pathlib.Path("/etc/os-release").read_text()
    except Exception:
        data = ""
    for line in data.splitlines():
        if line.startswith("UBUNTU_CODENAME=") or line.startswith("VERSION_CODENAME="):
            ubuntu_codename = line.split("=", 1)[1].strip().strip('"')
            break
    if ubuntu_codename == "plucky":
        ubuntu_codename = "noble"
    apt_suite = ros_suite or ubuntu_codename or "noble"
    desired_line = (
        "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros.gpg] "
        f"http://packages.ros.org/ros2/ubuntu {apt_suite} main"
    )
    need_write = True
    if sources.exists():
        try:
            current = sources.read_text()
            need_write = desired_line not in current
        except Exception:
            need_write = True
    if not sources.exists() or need_write:
        run(
            [
                "bash",
                "-lc",
                f'echo "{desired_line}" | tee /etc/apt/sources.list.d/ros2.list >/dev/null',
            ],
            check=True,
        )
    run(["apt-get", "update"], check=True)
    # Install ROS base first (separate call to match tests). Default jazzy.
    try:
        run(["apt-get", "install", "-y", f"ros-{ROS_DISTRO}-ros-base"], check=True)
    except Exception:
        # On Pop!_OS (plucky), ROS Jazzy (noble) may require Ubuntu 24.04 runtime
        # libraries (e.g., libpython3.12t64, libspdlog1.12-fmt9). Add Ubuntu Noble
        # apt sources and tightly pin only the required packages, then retry.
        try:
            noble_list = "/etc/apt/sources.list.d/ubuntu-noble.list"
            noble_pref = "/etc/apt/preferences.d/noble-ros.pref"
            run(
                [
                    "bash",
                    "-lc",
                    (
                        "cat > "
                        + noble_list
                        + " <<'EOF'\n"
                        + "deb http://archive.ubuntu.com/ubuntu noble main universe\n"
                        + "deb http://archive.ubuntu.com/ubuntu noble-updates main universe\n"
                        + "deb http://security.ubuntu.com/ubuntu noble-security main universe\n"
                        + "EOF\n"
                    ),
                ],
                check=True,
            )
            # Pin only the specific packages we need from noble with high priority
            run(
                [
                    "bash",
                    "-lc",
                    (
                        "cat > "
                        + noble_pref
                        + " <<'EOF'\n"
                        + "Package: libpython3.12t64\n"
                        + "Pin: release n=noble\n"
                        + "Pin-Priority: 1001\n\n"
                        + "Package: libspdlog1.12-fmt9\n"
                        + "Pin: release n=noble\n"
                        + "Pin-Priority: 1001\n\n"
                        + "Package: libfmt9\n"
                        + "Pin: release n=noble\n"
                        + "Pin-Priority: 1001\n"
                        + "EOF\n"
                    ),
                ],
                check=True,
            )
            run(["apt-get", "update"], check=True)
            run(["apt-get", "install", "-y", "libpython3.12t64", "libspdlog1.12-fmt9"], check=True)
            # Retry ROS base install
            run(["apt-get", "install", "-y", f"ros-{ROS_DISTRO}-ros-base"], check=True)
        except Exception:
            # Give a clear failure; caller will see apt errors above.
            raise
    # Optionally install CycloneDDS RMW for better performance
    try:
        run(["apt-get", "install", "-y", f"ros-{ROS_DISTRO}-rmw-cyclonedds-cpp"], check=True)
    except Exception:
        pass
    # Useful core packages for drivers and bringup (best-effort)
    try:
        run(["apt-get", "install", "-y", f"ros-{ROS_DISTRO}-diagnostic-updater", f"ros-{ROS_DISTRO}-xacro"], check=False)
    except Exception:
        pass
    try:
        run(["apt-get", "install", "-y", "ros-dev-tools"], check=False)
    except Exception:
        pass
    # Install colcon and rosdep from apt; if colcon fails, pip fallback happens later in build
    try:
        run(["apt-get", "install", "-y", "python3-colcon-common-extensions", "python3-rosdep"], check=True)
        # Initialize rosdep if not already initialized (best-effort)
        try:
            if not pathlib.Path("/etc/ros/rosdep/sources.list.d/20-default.list").exists():
                run(["rosdep", "init"], check=False)
        except Exception:
            pass
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
    # Ensure the venv exists before attempting to install; if not, try to
    # install python3-venv and create it.
    try:
        if not (VENV_DIR / "bin/python").exists():
            try:
                run(["apt-get", "update"], check=False)
                run(["apt-get", "install", "-y", "python3-venv"], check=False)
            except Exception:
                pass
            run(["python3", "-m", "venv", "--system-site-packages", str(VENV_DIR)], check=True)
    except Exception:
        # If venv creation still fails, fall back to system pip install as a last resort
        try:
            run(["python3", "-m", "pip", "install", *packages], check=False)
        except Exception:
            pass
        return
    uv = HOME_DIR / ".local/bin/uv"
    py = str(VENV_DIR / "bin/python")
    pip = str(VENV_DIR / "bin/pip")
    uv_ok = False
    try:
        uv_ok = uv.exists()
    except Exception:
        uv_ok = False
    if uv_ok:
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


def install_vision_packages(run=subprocess.run) -> None:
    """Install OpenCV and NumPy into the service virtualenv.

    Examples:
        >>> install_vision_packages(lambda cmd, check: None)  # doctest: +SKIP
    """
    try:
        _venv_pip_install([
            "numpy",
            "opencv-contrib-python-headless",
            "qdrant-client",
            "neo4j",
        ], run)
    except Exception:
        pass


def verify_venv_imports(modules: list[str], run=subprocess.run) -> None:
    """Best-effort import check for modules inside the service venv.

    Runs ``python -c 'import m1, m2'`` using the venv interpreter. Non-fatal.

    Examples:
        >>> verify_venv_imports(['sys'])  # doctest: +SKIP
    """
    py = VENV_DIR / "bin/python"
    if not py.exists():
        return
    code = "; ".join(f"import {m}" for m in modules)
    try:
        run([str(py), "-c", code], check=False)
    except Exception:
        pass


def install_vision_models(run=subprocess.run) -> None:
    """Fetch face landmark and embedding models into a shared path.

    Downloads:
    - LBF landmark model (68-point): lbfmodel.yaml
    - SFace face recognition embedding model: face_recognition_sface_2021dec.onnx
    """
    base = pathlib.Path(os.getenv("VISION_MODELS_DIR") or _read_env_file_var("VISION_MODELS_DIR", "/opt/psyche/models/vision") or "/opt/psyche/models/vision")
    try:
        base.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass
    lbf = base / "lbfmodel.yaml"
    sface = base / "face_recognition_sface_2021dec.onnx"
    if not lbf.exists():
        run([
            "curl", "-fsSL", "-o", str(lbf),
            "https://raw.githubusercontent.com/opencv/opencv_contrib/4.x/modules/face/data/lbfmodel.yaml",
        ], check=True)
    if not sface.exists():
        run([
            "curl", "-fsSL", "-o", str(sface),
            "https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/face_recognition_sface/face_recognition_sface_2021dec.onnx",
        ], check=True)


def install_camera_ros_packages(run=subprocess.run) -> None:
    """Install common ROS camera packages (image transport + calibration)."""
    try:
        run([
            "apt-get", "install", "-y",
            f"ros-{ROS_DISTRO}-camera-calibration-parsers",
            f"ros-{ROS_DISTRO}-image-transport",
            f"ros-{ROS_DISTRO}-image-transport-plugins",
            f"ros-{ROS_DISTRO}-image-pipeline",
        ], check=True)
    except Exception:
        pass


def install_fortune_packages(run=subprocess.run) -> None:
    """Install fortune databases for variety (best-effort)."""
    try:
        # Install only packages available on this distro to avoid noisy errors
        candidates = [
            "fortune-mod",
            "fortunes",
            "fortunes-min",
            "fortunes-off",
            "fortunes-bofh-excuses",
            "bofh-excuses",
        ]
        run(["apt-get", "update"], check=False)
        for pkg in candidates:
            run(["bash", "-lc", f"apt-cache show {pkg} >/dev/null 2>&1 && apt-get install -y {pkg} || true"], check=False)
    except Exception:
        pass


def install_qdrant(run=subprocess.run) -> None:
    """Install Qdrant vector DB as a system service.

    Downloads the appropriate static binary for the host architecture and
    sets up a simple config with data under ``/opt/qdrant``.
    """
    import platform
    arch = platform.machine().lower()
    # Official Linux tarball is available for x86_64; ARM64 Linux is not provided.
    if "x86_64" in arch or arch == "amd64":
        target = "x86_64-unknown-linux-gnu"
    else:
        # Skip install on unsupported arch; vision falls back to embedded client.
        return
    ver = os.getenv("QDRANT_VERSION", "v1.7.3")
    url = f"https://github.com/qdrant/qdrant/releases/download/{ver}/qdrant-{target}.tar.gz"
    bin_dir = pathlib.Path("/usr/local/bin")
    etc = pathlib.Path("/etc/qdrant")
    data = pathlib.Path("/opt/qdrant")
    try:
        etc.mkdir(parents=True, exist_ok=True)
        data.mkdir(parents=True, exist_ok=True)
        run(["/bin/bash", "-lc", f"curl -fsSL {url} -o /tmp/qdrant.tgz && tar -xzf /tmp/qdrant.tgz -C /tmp && install -m 0755 /tmp/qdrant/qdrant {bin_dir}/qdrant"], check=True)
    except Exception:
        # Best-effort; skip if download fails
        return
    cfg = f"""
log_level: INFO
service:
  http_port: 6333
storage:
  storage_path: {data}
""".strip()
    try:
        (etc / "config.yaml").write_text(cfg + "\n")
    except Exception:
        pass


def launch_qdrant(run=subprocess.run) -> None:
    """Create a systemd unit for qdrant and start it."""
    if DISABLE_SYSTEMD:
        return
    unit = f"""[Unit]
Description=Qdrant Vector Database
After=network.target

[Service]
Type=simple
User={SERVICE_USER}
EnvironmentFile=-/etc/psyche.env
ExecStart=/usr/local/bin/qdrant --config-path /etc/qdrant/config.yaml
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
"""
    path = SYSTEMD_DIR / "psyche-qdrant.service"
    try:
        # Only launch if binary exists
        if not pathlib.Path("/usr/local/bin/qdrant").exists():
            return
        path.write_text(unit)
        run(["systemctl", "daemon-reload"], check=True)
        run(["systemctl", "enable", "--now", "psyche-qdrant.service"], check=True)
    except Exception:
        pass


def install_neo4j(run=subprocess.run) -> None:
    """Install Neo4j (Community) via apt and set initial password."""
    try:
        run(["bash", "-lc", "curl -fsSL https://debian.neo4j.com/neotechnology.gpg.key | gpg --dearmor | tee /usr/share/keyrings/neo4j.gpg >/dev/null"], check=True)
        run(["bash", "-lc", 'echo "deb [signed-by=/usr/share/keyrings/neo4j.gpg] https://debian.neo4j.com stable 5" | tee /etc/apt/sources.list.d/neo4j.list >/dev/null'], check=True)
        run(["apt-get", "update"], check=True)
        run(["apt-get", "install", "-y", "neo4j"], check=True)
    except Exception:
        return
    # Set initial password (ignore if already set)
    try:
        # Choose password from env or default to a safe length
        pwd = os.getenv("NEO4J_PASSWORD") or _read_env_file_var("NEO4J_PASSWORD", "neo4jneo4j") or "neo4jneo4j"
        if len(pwd) < 8:
            pwd = (pwd * 2) if len(pwd) > 0 else "neo4jneo4j"
        run(["neo4j-admin", "dbms", "set-initial-password", str(pwd)], check=False)
    except Exception:
        pass


def launch_neo4j(run=subprocess.run) -> None:
    """Enable and start the packaged neo4j service."""
    if DISABLE_SYSTEMD:
        return
    try:
        run(["systemctl", "enable", "--now", "neo4j"], check=True)
    except Exception:
        pass


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
    # Best-effort apt packages; do not bail if unavailable on this distro
    try:
        run(["apt-get", "install", "-y", "python3-gpiozero", "python3-lgpio", "i2c-tools", "libi2c-dev"], check=True)
    except Exception:
        pass
    # Ensure service user can access GPIO and I2C devices (best-effort)
    try:
        run(["usermod", "-aG", "gpio", SERVICE_USER], check=True)
    except Exception:
        pass
    try:
        run(["usermod", "-aG", "i2c", SERVICE_USER], check=True)
    except Exception:
        pass
    # Always ensure Python deps are installed regardless of apt success
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


def ensure_i2c_bus_overlay(bus: int = 3, pins: str = "pins_4_5") -> None:
    """Enable an additional I2C bus overlay (e.g., i2c3 on GPIO 4/5).

    Best-effort: writes a dtoverlay line into /boot config if running on Pi.
    """
    try:
        with open("/proc/device-tree/model", "rb") as fh:
            is_pi = b"Raspberry Pi" in fh.read()
    except Exception:
        is_pi = False
    if not is_pi:
        return
    cfg_path = pathlib.Path("/boot/firmware/config.txt")
    if not cfg_path.exists():
        cfg_path = pathlib.Path("/boot/config.txt")
    try:
        content = cfg_path.read_text() if cfg_path.exists() else ""
    except Exception:
        content = ""
    line = f"dtoverlay=i2c{int(bus)},{pins}"
    if line not in content:
        try:
            cfg_path.write_text((content.rstrip() + "\n" + line + "\n") if content else (line + "\n"))
        except Exception:
            pass


def _detect_i2c_bus_for_addr(addrs: list[str] | None = None) -> int | None:
    """Return first i2c bus number that reports any address in ``addrs``.

    Best-effort: requires i2c-tools. Returns None on failure.
    """
    addrs = addrs or ["0x68", "0x69"]
    try:
        import glob, re
        buses = []
        for p in glob.glob("/dev/i2c-*"):
            try:
                buses.append(int(p.rsplit("-", 1)[1]))
            except Exception:
                continue
        for b in sorted(buses):
            try:
                # i2cdetect prints the address without 0x prefix in the grid
                hexds = [a.lower().replace("0x", "").zfill(2) for a in addrs]
                out = subprocess.run(["i2cdetect", "-y", str(b)], capture_output=True, text=True, check=False)
                grid = (out.stdout or "").lower()
                if any(h in grid for h in hexds):
                    return b
            except Exception:
                continue
    except Exception:
        return None
    return None


def ensure_imu_env(hostname: str, config: dict) -> None:
    """Append I2C bus selection for IMU to /etc/psyche.env if configured.

    - Reads ``[hosts.<name>.imu].i2c_bus``; supports an integer bus or the
      string "auto" to probe for the MPU6050's default address (0x68).
    - If bus 3 is selected, enables an overlay for i2c3 on GPIO 4/5.
    """
    val = config.get("hosts", {}).get(hostname, {}).get("imu", {}).get("i2c_bus")
    bus: int | None = None
    if val is None:
        return
    if isinstance(val, str) and val.strip().lower() == "auto":
        bus = _detect_i2c_bus_for_addr(["0x68", "0x69"])  # try both MPU6050 addresses
    else:
        try:
            bus = int(val)
        except Exception:
            bus = None
    if not bus:
        return
    if bus == 3:
        ensure_i2c_bus_overlay(3, "pins_4_5")
    env_path = pathlib.Path("/etc/psyche.env")
    try:
        with env_path.open("a") as fh:
            fh.write(f"\nI2C_BUS={bus}\n")
    except Exception:
        pass


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
    pip = str(VENV_DIR / "bin/pip")
    # Prefer CUDA wheel when NVIDIA is present; fall back to generic
    if _has_nvidia():
        try:
            run([pip, "install", "llama-cpp-python-cu121"], check=True)
            return
        except Exception:
            # Fall back to default package if CUDA-specific wheel is unavailable
            pass
    _venv_pip_install(["llama-cpp-python"], run)


def install_hf_chat(run=subprocess.run) -> None:
    """Install Hugging Face chat dependencies into the service virtualenv.

    Installs ``transformers``, ``torch`` (CPU), ``safetensors``, and ``sentencepiece``.

    Examples:
        >>> calls = []
        >>> install_hf_chat(lambda cmd, check: calls.append(cmd))
        >>> any('transformers' in c for cmd in calls for c in cmd)
        True
    """
    # Always install base libs
    _venv_pip_install(["transformers>=4.41", "safetensors", "sentencepiece"], run)
    # Install torch suitable for the environment. Prefer CUDA wheels when NVIDIA GPU is present.
    pip = str(VENV_DIR / "bin/pip")
    try:
        if _has_nvidia():
            # Use PyTorch CUDA 12.1 wheels (broadly compatible on recent drivers)
            run([pip, "install", "--index-url", "https://download.pytorch.org/whl/cu121", "torch", "torchvision", "torchaudio"], check=True)
        else:
            # CPU wheels
            run([pip, "install", "--index-url", "https://download.pytorch.org/whl/cpu", "torch", "torchvision", "torchaudio"], check=True)
    except Exception:
        # Fall back to generic PyPI if the above indexes are unavailable
        run([pip, "install", "torch"], check=False)


def install_cuda_if_available(run=subprocess.run) -> None:
    """Detect NVIDIA GPU and install drivers/CUDA toolkit when applicable.

    Best-effort on Ubuntu/Pop derivatives:
    - Installs ``ubuntu-drivers-common`` then runs ``ubuntu-drivers autoinstall``
    - Installs ``nvidia-cuda-toolkit`` for compiler/runtime (CPU fallback safe)

    Examples:
        >>> install_cuda_if_available(lambda cmd, check: None)  # doctest: +SKIP
    """
    # Quick detection to avoid touching systems without NVIDIA hardware
    try:
        has_gpu = False
        out = subprocess.run(["bash", "-lc", "lspci -nnk | grep -i nvidia"], text=True, capture_output=True)
        has_gpu = bool(out.stdout.strip())
    except Exception:
        has_gpu = pathlib.Path("/proc/driver/nvidia/version").exists()
    if not has_gpu:
        return
    # Try driver autoinstall first
    try:
        run(["apt-get", "update"], check=True)
        run(["apt-get", "install", "-y", "ubuntu-drivers-common"], check=True)
        run(["ubuntu-drivers", "autoinstall"], check=False)
    except Exception:
        # Fallback to specific driver meta-packages (best-effort)
        for pkg in ("nvidia-driver-550", "nvidia-driver-535"):
            try:
                run(["apt-get", "install", "-y", pkg], check=True)
                break
            except Exception:
                continue
    # CUDA toolkit (provides nvcc and shared libs used by some wheels)
    try:
        run(["apt-get", "install", "-y", "nvidia-cuda-toolkit"], check=False)
    except Exception:
        pass


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


def prefetch_whisper_model(name: str = "tiny", run=subprocess.run) -> None:
    """Warm Whisper model cache by importing and loading ``name`` in the venv.

    Respects ``XDG_CACHE_HOME`` from ``/etc/psyche.env`` so cache goes to the
    configured shared cache path.

    Examples:
        >>> prefetch_whisper_model('tiny', lambda cmd, check: None)  # doctest: +SKIP
    """
    py = VENV_DIR / "bin/python"
    if not py.exists():
        return
    xdg = os.getenv("XDG_CACHE_HOME") or _read_env_file_var("XDG_CACHE_HOME")
    env = os.environ.copy()
    if xdg:
        env["XDG_CACHE_HOME"] = xdg
    code = (
        "import sys;\n"
        "try:\n"
        "    import whisper\n"
        "    whisper.load_model(\"" + str(name) + "\")\n"
        "    print(\"ok\")\n"
        "except Exception:\n"
        "    # Missing package or offline; skip quietly to avoid noisy traces\n"
        "    pass\n"
    )
    try:
        run([str(py), "-c", code], check=False, env=env)
    except Exception:
        # Best-effort; skip on failure
        pass


def install_default_assets(run=subprocess.run) -> None:
    """Install small default models: Phi-3.5 mini Instruct GGUF and Whisper tiny.

    - Downloads Phi-3.5 mini Instruct Q4_K_M GGUF into ``LLAMA_MODELS_DIR`` if
      not already present.
    - Warms Whisper ``tiny`` model cache in ``XDG_CACHE_HOME`` (if whisper is installed).
    """
    # Phi-3.5 mini Instruct default GGUF
    models_dir = pathlib.Path(os.getenv("LLAMA_MODELS_DIR") or _read_env_file_var("LLAMA_MODELS_DIR", "/opt/llama/models") or "/opt/llama/models")
    models_dir.mkdir(parents=True, exist_ok=True)
    gguf_name = "Phi-3.5-mini-instruct-Q4_K_M.gguf"
    target = models_dir / gguf_name
    if not target.exists():
        url = "https://huggingface.co/bartowski/Phi-3.5-mini-instruct-GGUF/resolve/main/Phi-3.5-mini-instruct-Q4_K_M.gguf"
        try:
            fetch_llama_model(url, str(models_dir), run)
        except Exception:
            pass
    # Whisper tiny cache warm
    prefetch_whisper_model("tiny", run)


def ensure_assets_prefetch(hostname: str, config: dict, run=subprocess.run) -> None:
    """Download additional models specified under ``assets.prefetch``.

    Schema:
        [hosts.<name>.assets.prefetch]
        whisper = ["tiny", "base", ...]
        llama = ["<url or known name>", ...]

    Known llama names:
        - phi-3.5-mini-instruct-q4_k_m -> Phi-3.5 mini Instruct Q4_K_M GGUF
        - tinyllama-q4_k_m -> TinyLlama 1.1B Chat v1.0 Q4_K_M GGUF (legacy)
    """
    pre = config.get("hosts", {}).get(hostname, {}).get("assets", {}).get("prefetch", {})
    if not isinstance(pre, dict):
        return
    # Whisper models
    for name in map(str, pre.get("whisper", []) or []):
        try:
            prefetch_whisper_model(name, run)
        except Exception:
            continue
    # Llama models
    known = {
        "phi-3.5-mini-instruct-q4_k_m": "https://huggingface.co/bartowski/Phi-3.5-mini-instruct-GGUF/resolve/main/Phi-3.5-mini-instruct-Q4_K_M.gguf",
        "tinyllama-q4_k_m": "https://huggingface.co/TheBloke/TinyLlama-1.1B-Chat-v1.0-GGUF/resolve/main/tinyllama-1.1b-chat-v1.0.Q4_K_M.gguf?download=true",
    }
    dest = os.getenv("LLAMA_MODELS_DIR") or _read_env_file_var("LLAMA_MODELS_DIR", "/opt/llama/models") or "/opt/llama/models"
    for item in map(str, pre.get("llama", []) or []):
        url = known.get(item.lower(), item)
        try:
            fetch_llama_model(url, dest, run)
        except Exception:
            continue


def stage_assets_seed_locally() -> None:
    """If /opt/psyche/assets_seed exists, copy into fixed system paths.

    - Llama GGUFs -> /opt/llama/models
    - Piper voices -> /opt/piper/voices
    - Caches -> /opt/psyche/cache
    """
    seed = pathlib.Path("/opt/psyche/assets_seed")
    if not (seed.exists() and seed.is_dir()):
        return
    llama_dir = pathlib.Path("/opt/llama/models")
    voices_dir = pathlib.Path("/opt/piper/voices")
    cache_dir = pathlib.Path("/opt/psyche/cache")
    for d in (llama_dir, voices_dir, cache_dir):
        try:
            d.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass
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
    cmd = ["python3", script_path("voice_service.py")]
    install_service_unit("voice", cmd, run)


def launch_updater(run=subprocess.run) -> None:
    """Install a systemd service + timer to keep the repo evergreen.

    - Service: runs ``git_updater.py`` under the service user
    - Timer: runs at boot after a short delay and then every 6 hours

    Skips silently if not running as root (e.g., in tests).
    """
    if DISABLE_SYSTEMD:
        return
    try:
        if os.geteuid() != 0:
            return
    except Exception:
        return
    svc_name = "psyche-update.service"
    tmr_name = "psyche-update.timer"
    unit_path = SYSTEMD_DIR / svc_name
    timer_path = SYSTEMD_DIR / tmr_name
    wrapped = (
        f"/bin/bash -lc '"
        f"source /opt/ros/{ROS_DISTRO}/setup.bash >/dev/null 2>&1; "
        f"[ -f {WORKSPACE}/install/setup.bash ] && source {WORKSPACE}/install/setup.bash >/dev/null 2>&1 || true; "
        f"{VENV_DIR}/bin/python {script_path('git_updater.py')}'"
    )
    svc = f"""[Unit]
Description=PSYCHE Auto Update
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
User={SERVICE_USER}
EnvironmentFile=-/etc/psyche.env
WorkingDirectory={REPO_DIR}
ExecStart={wrapped}
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
"""
    timer = """[Unit]
Description=Run PSYCHE Auto Update periodically

[Timer]
OnBootSec=10min
OnUnitActiveSec=6h
Persistent=true

[Install]
WantedBy=timers.target
"""
    unit_path.write_text(svc)
    timer_path.write_text(timer)
    run(["systemctl", "daemon-reload"], check=False)
    run(["systemctl", "enable", "--now", tmr_name], check=False)


def install_host_tools() -> None:
    """Install small host CLI helpers for easy reprovisioning.

    - ``/usr/local/bin/psyche-provision``: reruns setup with optional PSYCHE_SRC
    - ``/usr/bin/psyche-update``: fetches and runs install.sh from GitHub
    """
    try:
        bin_dir = pathlib.Path("/usr/local/bin")
        bin_dir.mkdir(parents=True, exist_ok=True)
        prov = bin_dir / "psyche-provision"
        script = f"""#!/usr/bin/env bash
set -euo pipefail
SRC="${{PSYCHE_SRC:-{REPO_DIR}}}"
if [ -d "$SRC" ]; then
  echo "[psyche] reprovisioning from $SRC" >&2
  exec sudo -E env PSYCHE_SRC="$SRC" python3 /opt/psyche/setup_host.py
else
  echo "[psyche] reprovisioning from /opt/psyche" >&2
  exec sudo -E python3 /opt/psyche/setup_host.py
fi
"""
        prov.write_text(script)
        try:
            os.chmod(prov, 0o755)
        except Exception:
            pass
        # Global updater entrypoint under /usr/bin per request
        upd = pathlib.Path("/usr/bin/psyche-update")
        owner = os.getenv("PSYCHE_OWNER", "dancxjo")
        repo = os.getenv("PSYCHE_REPO", "knightykell")
        branch = os.getenv("PSYCHE_BRANCH", "main")
        updater = f"""#!/usr/bin/env bash
set -euo pipefail
OWNER="${{PSYCHE_OWNER:-{owner}}}"
REPO="${{PSYCHE_REPO:-{repo}}}"
BRANCH="${{PSYCHE_BRANCH:-{branch}}}"
URL="https://raw.githubusercontent.com/$OWNER/$REPO/refs/heads/$BRANCH/scripts/install.sh"
if [ "$EUID" -ne 0 ]; then
  curl -fsSL "$URL" | sudo bash -s -- -o "$OWNER" -r "$REPO" -b "$BRANCH"
else
  curl -fsSL "$URL" | bash -s -- -o "$OWNER" -r "$REPO" -b "$BRANCH"
fi
"""
        # Ensure parent dir exists and write script
        upd.parent.mkdir(parents=True, exist_ok=True)
        upd.write_text(updater)
        try:
            os.chmod(upd, 0o755)
        except Exception:
            pass
    except PermissionError:
        # Non-root environments/tests: ignore
        pass


def launch_logticker(run=subprocess.run) -> None:
    """Install systemd unit publishing logs to the voice service.

    Examples:
        >>> launch_logticker(lambda cmd, check: None)  # doctest: +SKIP
    """
    cmd = ["python3", script_path("log_ticker.py")]
    install_service_unit("logticker", cmd, run)


def launch_topics(run=subprocess.run) -> None:
    """Install systemd unit for a topic list publisher.

    Publishes a single-line summary of currently visible topics to
    the ``/topics`` topic. Handy for OLED ticker displays.

    Examples:
        >>> launch_topics(lambda cmd, check: None)  # doctest: +SKIP
    """
    cmd = ["python3", script_path("topic_list_service.py")]
    install_service_unit("topics", cmd, run)


def launch_notify(run=subprocess.run) -> None:
    """Install systemd unit bridging desktop notifications to voice.

    Examples:
        >>> launch_notify(lambda cmd, check: None)  # doctest: +SKIP
    """
    cmd = ["python3", script_path("notify_to_voice.py")]
    install_service_unit("notify", cmd, run)


def launch_fortune(cfg: dict | None = None, run=subprocess.run) -> None:
    """Install systemd unit for periodic fortunes.

    Examples:
        >>> launch_fortune({'period': 60}, lambda cmd, check: None)  # doctest: +SKIP
    """
    cfg = cfg or {}
    period = str(cfg.get("period", 300))
    do_notify = cfg.get("notify_send", False)
    use_all = cfg.get("all", True)
    offensive = cfg.get("offensive", False)
    cmd = ["python3", script_path("fortune_notify.py"), "--period", period]
    if do_notify:
        cmd.append("--notify-send")
    if use_all:
        cmd.append("--all")
    if offensive:
        cmd.append("--offensive")
    install_service_unit("fortune", cmd, run)


        


def launch_asr(cfg: dict | None = None, run=subprocess.run) -> None:
    """Install systemd unit for Whisper-based transcription.

    Examples:
        >>> launch_asr({'model': 'tiny'}, lambda cmd, check: None)  # doctest: +SKIP
    """
    model = (cfg or {}).get("model", "tiny")
    cmd = ["python3", script_path("asr_service.py"), "--model", model]
    install_service_unit("asr", cmd, run)


def launch_asr_long(cfg: dict | None = None, run=subprocess.run) -> None:
    """Install systemd unit for utterance-level ASR (higher accuracy).

    Examples:
        >>> launch_asr_long({'model': 'base'}, lambda cmd, check: None)  # doctest: +SKIP
    """
    model = (cfg or {}).get("model", "base")
    cmd = ["python3", script_path("asr_utterance_service.py"), "--model", model]
    install_service_unit("asr_long", cmd, run)


def launch_create(cfg: dict | None = None, run=subprocess.run) -> None:
    """Install systemd unit to bring up the iRobot Create base.

    Config options under ``[hosts.<name>.create]``:
    - ``package``: ROS 2 package name providing a launch file (e.g., ``create_robot``)
    - ``launch_file``: Launch file (e.g., ``bringup.launch.py``)
    - ``params``: Table of key/value pairs turned into ``key:=value`` args
    - ``args``: List of additional CLI args (strings)
    - ``port``: Convenience serial port param (merged into params as ``port``/``serial_port``)

    Falls back to ``ros2 run create_driver create_driver_node`` if no launch is given.
    """
    cfg = cfg or {}
    pkg = cfg.get("package")
    lfile = cfg.get("launch_file")
    params = cfg.get("params") or {}
    extra_args = list(map(str, (cfg.get("args") or [])))
    port = cfg.get("port", "/dev/ttyUSB0")
    if "port" not in params:
        params["port"] = port
    # Some stacks use serial_port rather than port
    params.setdefault("serial_port", port)
    kv = [f"{k}:={v}" for k, v in params.items()]
    if pkg and lfile:
        inner = ["ros2", "launch", str(pkg), str(lfile), *kv, *extra_args]
    else:
        inner = ["ros2", "run", "create_driver", "create_driver_node", *kv, *extra_args]
    # Wrap with retry/backoff to keep trying to connect to the robot
    cmd = [
        str(VENV_DIR / "bin/python"),
        script_path("retry_exec.py"),
        "--min", "1", "--max", "60", "--factor", "1.5", "--jitter", "0.3", "--",
        *inner,
    ]
    # Ensure the serial USB driver is present and wait for device if specified
    pre = []
    dev = params.get("dev") or params.get("port") or "/dev/ttyUSB0"
    if dev:
        pre.append("/sbin/modprobe cdc_acm || /sbin/modprobe ch341 || true")
        pre.append(f"/bin/bash -lc 'for i in $(seq 1 30); do [ -e {dev} ] && break; sleep 2; done'")
    install_service_unit("create", cmd, run, pre=pre)
    # Also install a small health reporter so the OLED can show connection state
    try:
        launch_create_health(run)
    except Exception:
        pass


def launch_lidar(cfg: dict | None = None, run=subprocess.run) -> None:
    """Install systemd unit for HLS LFCD LDS LiDAR.

    Config under ``[hosts.<name>.lidar]``:
    - ``port``: serial device (default ``/dev/ttyUSB0``)
    - ``frame_id``: TF frame for laser (default ``laser``)
    """
    cfg = cfg or {}
    port = str(cfg.get("port", "/dev/ttyUSB0"))
    frame = str(cfg.get("frame_id", "laser"))
    cmd = [
        "ros2", "launch", "hls_lfcd_lds_driver", "hlds_laser.launch.py",
        f"port:={port}", f"frame_id:={frame}",
    ]
    install_service_unit("lidar", cmd, run)


def _write_imu_params(cfg: dict | None = None) -> str | None:
    """Write a minimal params YAML for mpu6050driver and return its path."""
    cfg = cfg or {}
    freq = cfg.get("frequency")
    try:
        base = pathlib.Path("/opt/psyche/config")
        base.mkdir(parents=True, exist_ok=True)
        path = base / "mpu6050.yaml"
        lines = [
            "mpu6050driver_node:",
            "  ros__parameters:",
        ]
        if freq is not None:
            lines.append(f"    frequency: {int(freq)}")
        path.write_text("\n".join(lines) + "\n")
        return str(path)
    except Exception:
        return None


def launch_imu(cfg: dict | None = None, run=subprocess.run) -> None:
    """Install systemd unit for MPU6050 IMU driver (via launch)."""
    params_file = _write_imu_params(cfg)
    cmd = ["ros2", "launch", "mpu6050driver", "mpu6050driver_launch.py"]
    if params_file:
        cmd.append(f"params_file:={params_file}")
    # Ensure i2c-dev is present before starting
    pre = ["/sbin/modprobe i2c-dev || /usr/sbin/modprobe i2c-dev || true"]
    install_service_unit("imu", cmd, run, pre=pre)


def launch_sensors_status(run=subprocess.run) -> None:
    """Install systemd unit for sensor status summarizer."""
    cmd = ["python3", script_path("sensor_status.py")]
    install_service_unit("sensors", cmd, run)


def launch_create_health(run=subprocess.run) -> None:
    """Install systemd unit publishing Create connection health."""
    cmd = ["python3", script_path("create_health.py"), "--timeout", "5"]
    install_service_unit("create_status", cmd, run)


def launch_vision(cfg: dict | None = None, run=subprocess.run) -> None:
    """Install systemd unit for the USB webcam vision service.

    Examples:
        >>> launch_vision({'device': 0, 'width': 640, 'height': 480, 'fps': 5}, lambda cmd, check: None)  # doctest: +SKIP
    """
    cfg = cfg or {}
    dev = str(cfg.get("device", 0))
    width = str(cfg.get("width", 640))
    height = str(cfg.get("height", 480))
    fps = str(cfg.get("fps", 5))
    cmd = [
        str(VENV_DIR / "bin/python"),
        script_path("vision_service.py"),
        "--device", dev,
        "--width", width,
        "--height", height,
        "--fps", fps,
    ]
    install_service_unit("vision", cmd, run)


def launch_chat(run=subprocess.run) -> None:
    """Install systemd unit for the chat service.

    Examples:
        >>> launch_chat(lambda cmd, check: None)  # doctest: +SKIP
    """
    cmd = ["python3", script_path("chat_service.py")]
    install_service_unit("chat", cmd, run)


def launch_singer(cfg: dict | None = None, run=subprocess.run) -> None:
    """Install systemd unit for the Create melody announcer.

    Reads port/baud from ``[hosts.<name>.singer]`` or falls back to
    ``[hosts.<name>.create].port``.

    Examples:
        >>> launch_singer({'port': '/dev/ttyUSB0'}, lambda cmd, check: None)  # doctest: +SKIP
    """
    cfg = cfg or {}
    port = str(cfg.get("port") or cfg.get("device") or "/dev/ttyUSB0")
    baud = str(cfg.get("baud", 57600))
    period = str(cfg.get("period", 60))
    cmd = [
        str(VENV_DIR / "bin/python"),
        script_path("create_singer.py"),
        "--port", port,
        "--baud", baud,
        "--period", period,
    ]
    install_service_unit("singer", cmd, run)


def ensure_chat_env(hostname: str, config: dict) -> None:
    """Write environment for the chat service from host config.

    Supported under ``[hosts.<name>.chat]``:
    - ``prompt`` -> ``CHAT_PROMPT``
    - ``model_path`` -> ``LLAMA_MODEL_PATH``
    - ``gguf_url``: download and set ``LLAMA_MODEL_PATH``
    - ``ollama_model`` -> ``OLLAMA_MODEL``
    - ``timeout`` (seconds) -> ``CHAT_OLLAMA_TIMEOUT`` (Ollama backend)
    - ``backend``: ``CHAT_BACKEND`` ("llama" or "ollama")

    Examples:
        >>> cfg = {'hosts': {'h': {'chat': {'prompt': 'You are helpful'}}}}
        >>> ensure_chat_env('h', cfg)  # doctest: +SKIP
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
    ccfg = get_service_config(hostname, "chat", config)
    if ccfg:
        if "gguf_url" in ccfg:
            try:
                path = fetch_llama_model(str(ccfg["gguf_url"]))
                if path:
                    env["LLAMA_MODEL_PATH"] = path
            except Exception:
                pass
        if "model_path" in ccfg:
            env["LLAMA_MODEL_PATH"] = str(ccfg["model_path"])
        if "prompt" in ccfg:
            env["CHAT_PROMPT"] = str(ccfg["prompt"])
    if "ollama_model" in ccfg:
        env["OLLAMA_MODEL"] = str(ccfg["ollama_model"])
    if "hf_model" in ccfg:
        env["HF_MODEL"] = str(ccfg["hf_model"])
        if "timeout" in ccfg:
            env["CHAT_OLLAMA_TIMEOUT"] = str(ccfg["timeout"])
        if "backend" in ccfg:
            env["CHAT_BACKEND"] = str(ccfg["backend"])
    try:
        env_path.write_text("\n".join(f"{k}={v}" for k, v in env.items()) + "\n")
    except Exception:
        pass


def main() -> None:
    """Entry point for host setup."""
    cfg = load_config()
    host = socket.gethostname()
    # Resolve ROS distro from env or hosts.toml (default jazzy)
    resolve_ros_distro(host, cfg)
    services = get_services(host, cfg)
    if not services:
        print(f"No services configured for {host}")
        return
    print("[setup] starting provisioning for:", host, services)
    # Stop any previously running services first
    stop_old_services(services)
    # Disable any leftover psyche-* units not in this host's services
    disable_absent_services(services)
    ensure_service_user()
    # If an NVIDIA GPU is present, install drivers/CUDA early
    try:
        print("[setup] checking for NVIDIA GPU + installing drivers/CUDA if present…")
        install_cuda_if_available()
    except Exception:
        pass
    # Install ROS first so colcon and env are available for builds
    print("[setup] installing ROS 2 base…")
    install_ros2()
    print("[setup] staging repo and runtime assets…")
    clone_repo()
    stage_runtime_assets()
    # Ensure Python tooling/venv early so any subsequent pip installs land in the venv
    print("[setup] ensuring Python env + zenoh…")
    ensure_python_env()
    install_zeno()
    # Pull extra ROS 2 repos (e.g., AutonomyLab create) and patch before first build
    print("[setup] ensuring extra ROS 2 repos…")
    try:
        ensure_ros2_extra_repos(host, cfg)
    except Exception:
        pass
    # Install Pi hardware deps before the first workspace build if needed
    if any(s in services for s in ("hrs04", "display", "imu")):
        print("[setup] installing Pi hardware packages…")
        try:
            install_pi_hw_packages()
        except Exception:
            pass
        # Verify required Python modules are importable in the venv
        verify_venv_imports(["luma.oled", "PIL"])
    print("[setup] building ROS 2 workspace…")
    setup_workspace()
    print("[setup] ensuring SSH keys…")
    ensure_ssh_keys()
    print("[setup] writing shell + service env…")
    ensure_shell_env()
    ensure_service_env()
    ensure_ros_network_env(host, cfg)
    print("[setup] staging baked assets (if any)…")
    stage_assets_seed_locally()
    # Install updater service + timer (evergreen)
    print("[setup] ensuring auto-update timer…")
    try:
        launch_updater()
    except Exception:
        pass
    # Install host helper CLI for idempotent re-provisioning
    install_host_tools()
    # Log summarizer removed; logs are shown directly via logticker -> display
    # Write env for chat service, if present
    if "chat" in services:
        print("[setup] applying chat env…")
        ensure_chat_env(host, cfg)
    # Write Piper env overrides from host config, if present
    if "voice" in services:
        print("[setup] applying voice env…")
        ensure_voice_env(host, cfg)
    # Install runtime Python packages before model prefetch to avoid noisy errors
    if "voice" in services or "logticker" in services:
        print("[setup] installing voice packages…")
        install_voice_packages()
    if "chat" in services:
        print("[setup] installing llama-cpp-python…")
        install_llama_cpp()
        # If chat backend is Hugging Face transformers, install required packages
        try:
            ccfg = get_service_config(host, "chat", cfg)
        except Exception:
            ccfg = None
        if ccfg and str(ccfg.get("backend", "")).lower() in {"hf", "transformers", "huggingface"}:
            print("[setup] installing NVIDIA drivers/CUDA (if GPU present)…")
            install_cuda_if_available()
            print("[setup] installing Hugging Face transformers (chat)…")
            install_hf_chat()
    prefetch_cfg = (cfg.get("hosts", {}).get(host, {}).get("assets", {}).get("prefetch", {}) or {})
    if "asr" in services or ("whisper" in prefetch_cfg):
        # Install ASR deps when ASR service is enabled or when hosts.toml requests whisper prefetch
        print("[setup] installing ASR packages…")
        install_asr_packages()
    # Install embedded defaults and any requested prefetches
    try:
        print("[setup] fetching default models (Llama 3.2 1B Instruct, Whisper tiny)…")
        install_default_assets()
        print("[setup] prefetching configured assets (hosts.toml)…")
        ensure_assets_prefetch(host, cfg)
    except Exception:
        pass
    if "vision" in services:
        print("[setup] installing vision packages…")
        install_vision_packages()
        print("[setup] fetching vision models…")
        try:
            install_vision_models()
        except Exception:
            pass
        print("[setup] installing ROS camera packages…")
        install_camera_ros_packages()
    if "fortune" in services:
        print("[setup] installing fortune databases…")
        install_fortune_packages()
    if "qdrant" in services:
        print("[setup] installing Qdrant…")
        install_qdrant()
    if "neo4j" in services:
        print("[setup] installing Neo4j…")
        # Ensure desired credentials are in /etc/psyche.env before install sets password
        ensure_db_env()
        install_neo4j()
    if "voice" in services:
        print("[setup] configuring audio…")
        ensure_audio(get_audio_config(host, cfg))
    # Pi hardware packages already installed above if needed
    # Ensure serial access for Create, if present
    if "create" in services and SERVICE_USER != "root":
        try:
            subprocess.run(["usermod", "-aG", "dialout", SERVICE_USER], check=True)
        except Exception:
            pass
    # Apply I2C overlay/env for IMU if configured
    try:
        ensure_imu_env(host, cfg)
    except Exception:
        pass
    installed: list[str] = []
    for svc in services:
        scfg = get_service_config(host, svc, cfg)
        if svc == "hrs04":
            print("[setup] launching HRS04 service…")
            launch_hrs04(scfg)
            installed.append("hrs04")
        elif svc == "display":
            print("[setup] launching OLED display service…")
            launch_display(scfg)
            installed.append("display")
        elif svc == "voice":
            print("[setup] launching voice service…")
            launch_voice()
            installed.append("voice")
        elif svc == "logticker":
            print("[setup] launching logticker service…")
            launch_logticker()
            installed.append("logticker")
        elif svc == "topics":
            print("[setup] launching topics service…")
            launch_topics()
            installed.append("topics")
        elif svc == "notify":
            print("[setup] launching notification bridge…")
            launch_notify()
            installed.append("notify")
        elif svc == "fortune":
            print("[setup] launching fortune notifier…")
            launch_fortune(scfg)
            installed.append("fortune")
        elif svc == "qdrant":
            print("[setup] launching Qdrant service…")
            ensure_db_env()
            launch_qdrant()
            installed.append("qdrant")
        elif svc == "neo4j":
            print("[setup] launching Neo4j service…")
            ensure_db_env()
            launch_neo4j()
            installed.append("neo4j")
        elif svc == "lidar":
            print("[setup] launching LiDAR service…")
            launch_lidar(scfg)
            installed.append("lidar")
        elif svc == "imu":
            print("[setup] launching IMU service…")
            launch_imu(scfg)
            installed.append("imu")
        elif svc == "create_status":
            print("[setup] launching Create status service…")
            launch_create_health()
            installed.append("create_status")
        elif svc == "sensors":
            print("[setup] launching sensor status service…")
            launch_sensors_status()
            installed.append("sensors")
        elif svc == "chat":
            print("[setup] launching chat service…")
            launch_chat()
            installed.append("chat")
        elif svc == "asr":
            print("[setup] launching ASR service…")
            launch_asr(scfg)
            installed.append("asr")
        elif svc == "asr_long":
            print("[setup] launching ASR (utterance) service…")
            launch_asr_long(scfg)
            installed.append("asr_long")
        elif svc == "vision":
            print("[setup] launching vision service…")
            launch_vision(scfg)
            installed.append("vision")
        elif svc == "singer":
            print("[setup] launching Create singer service…")
            sc = dict(scfg or {})
            if not sc.get("port"):
                cc = get_service_config(host, "create", cfg)
                if cc and cc.get("port"):
                    sc["port"] = cc.get("port")
            launch_singer(sc)
            installed.append("singer")
        else:
            print(f"Unknown service {svc}")
    print(f"Installed services: {', '.join(installed)}")


if __name__ == "__main__":
    main()
