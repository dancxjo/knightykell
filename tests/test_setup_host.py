import pathlib
import subprocess
import sys

import pytest

sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

from scripts.setup_host import (
    clone_repo,
    ensure_service_user,
    ensure_ssh_keys,
    get_service_config,
    get_services,
    install_ros2,
    install_zeno,
    install_voice_packages,
    launch_hrs04,
    launch_display,
    launch_logticker,
    launch_voice,
    load_config,
    setup_workspace,
    launch_asr,
    install_asr_packages,
)


def test_get_services_returns_configured_services(tmp_path):
    config_path = tmp_path / "hosts.toml"
    config_path.write_text(
        """
[hosts]
[hosts.brainstem]
services = ["core", "monitor"]
"""
    )
    config = load_config(config_path)
    assert get_services("brainstem", config) == ["core", "monitor"]


def test_get_services_unknown_host_returns_empty(tmp_path):
    config_path = tmp_path / "hosts.toml"
    config_path.write_text("[hosts]\n")
    config = load_config(config_path)
    assert get_services("beta", config) == []


def test_get_service_config_returns_pins(tmp_path):
    p = tmp_path / "hosts.toml"
    p.write_text(
        """
[hosts]
[hosts.brainstem]
services=["hrs04"]
[hosts.brainstem.hrs04]
trig_pin=1
echo_pin=2
"""
    )
    cfg = load_config(p)
    assert get_service_config("brainstem", "hrs04", cfg)["trig_pin"] == 1


def test_ensure_ssh_keys_invokes_ssh_keygen(monkeypatch, tmp_path):
    calls = []

    def fake_run(cmd, check):
        calls.append(cmd)
        (ssh_dir / "id_ed25519").write_text("key")
        (ssh_dir / "id_ed25519.pub").write_text("pub")

    ssh_dir = tmp_path / "ssh"
    monkeypatch.setattr("scripts.setup_host.SSH_DIR", ssh_dir)
    monkeypatch.setattr(subprocess, "run", fake_run)
    ensure_ssh_keys()
    assert (ssh_dir / "id_ed25519").exists()
    assert calls and "ssh-keygen" in calls[0][0]


def test_ensure_ssh_keys_skips_if_exists(monkeypatch, tmp_path):
    calls = []

    def fake_run(cmd, check):
        calls.append(cmd)

    ssh_dir = tmp_path / "ssh"
    ssh_dir.mkdir()
    (ssh_dir / "id_ed25519").write_text("key")
    (ssh_dir / "id_ed25519.pub").write_text("pub")
    monkeypatch.setattr("scripts.setup_host.SSH_DIR", ssh_dir)
    monkeypatch.setattr(subprocess, "run", fake_run)
    ensure_ssh_keys()
    assert not calls


def test_install_ros2_runs_apt():
    calls = []

    def fake_run(cmd, check):
        calls.append(cmd)

    install_ros2(fake_run)
    assert ["apt-get", "update"] in calls
    assert ["apt-get", "install", "-y", "ros-jazzy-ros-base"] in calls


def test_install_zeno_installs_zenoh():
    calls = []

    def fake_run(cmd, check):
        calls.append(cmd)

    install_zeno(fake_run)
    assert any("zenoh" in c for cmd in calls for c in cmd)


def test_install_voice_packages_installs_espeak():
    calls = []

    def fake_run(cmd, check):
        calls.append(cmd)

    install_voice_packages(fake_run)
    assert ["apt-get", "install", "-y", "espeak-ng", "mbrola", "mbrola-en1"] in calls


def test_ensure_service_user_adds_user(monkeypatch):
    calls = []

    def fake_run(cmd, check=False):
        calls.append(cmd)
        class R:
            returncode = 1
        return R()

    ensure_service_user(fake_run)
    assert ["useradd", "--create-home", "pete"] in calls


def test_clone_repo_runs_git_clone(monkeypatch, tmp_path):
    calls = []
    repo_dir = tmp_path / "repo"
    monkeypatch.setattr("scripts.setup_host.REPO_DIR", repo_dir)

    def fake_run(cmd, check):
        calls.append(cmd)

    clone_repo(fake_run)
    assert any(part == "git" for cmd in calls for part in cmd)


def test_setup_workspace_builds(monkeypatch, tmp_path):
    calls = []
    workspace = tmp_path / "ws"
    monkeypatch.setattr("scripts.setup_host.WORKSPACE", workspace)

    def fake_run(cmd, check):
        calls.append(cmd)

    setup_workspace(fake_run)
    assert any("colcon build" in " ".join(cmd) for cmd in calls)


def test_launch_hrs04_creates_systemd_unit(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr("scripts.setup_host.SYSTEMD_DIR", tmp_path)

    def fake_run(cmd, check):
        calls.append(cmd)

    launch_hrs04({"trig_pin": 1, "echo_pin": 2}, fake_run)
    unit = tmp_path / "psyche-hrs04.service"
    assert unit.exists()
    assert "hrs04_node.py" in unit.read_text()
    assert ["systemctl", "enable", "--now", "psyche-hrs04.service"] in calls


def test_launch_display_creates_systemd_unit(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr("scripts.setup_host.SYSTEMD_DIR", tmp_path)

    def fake_run(cmd, check):
        calls.append(cmd)

    launch_display({"topics": ["/foo", "/bar"]}, fake_run)
    unit = tmp_path / "psyche-display.service"
    assert unit.exists()
    content = unit.read_text()
    assert "ssd1306_display_node.py" in content and "/foo" in content and "/bar" in content
    assert ["systemctl", "enable", "--now", "psyche-display.service"] in calls


def test_launch_voice_creates_systemd_unit(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr("scripts.setup_host.SYSTEMD_DIR", tmp_path)

    def fake_run(cmd, check):
        calls.append(cmd)

    launch_voice(fake_run)
    unit = tmp_path / "psyche-voice.service"
    assert unit.exists()
    assert "voice_service.py" in unit.read_text()
    assert ["systemctl", "enable", "--now", "psyche-voice.service"] in calls


def test_launch_logticker_creates_systemd_unit(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr("scripts.setup_host.SYSTEMD_DIR", tmp_path)

    def fake_run(cmd, check):
        calls.append(cmd)

    launch_logticker(fake_run)
    unit = tmp_path / "psyche-logticker.service"
    assert unit.exists()
    assert "log_ticker.py" in unit.read_text()
    assert ["systemctl", "enable", "--now", "psyche-logticker.service"] in calls


def test_launch_asr_creates_systemd_unit(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr("scripts.setup_host.SYSTEMD_DIR", tmp_path)

    def fake_run(cmd, check):
        calls.append(cmd)

    launch_asr({'model': 'base'}, fake_run)
    unit = tmp_path / "psyche-asr.service"
    assert unit.exists()
    content = unit.read_text()
    assert '--model base' in content
    assert ['systemctl', 'enable', '--now', 'psyche-asr.service'] in calls


def test_main_launches_configured_services(monkeypatch):
    config = {
        "hosts": {
            "brainstem": {
                "services": ["hrs04"],
                "hrs04": {"trig_pin": 3, "echo_pin": 4},
            }
        }
    }
    monkeypatch.setattr("scripts.setup_host.load_config", lambda: config)
    monkeypatch.setattr("scripts.setup_host.socket.gethostname", lambda: "brainstem")
    monkeypatch.setattr("scripts.setup_host.ensure_service_user", lambda: None)
    monkeypatch.setattr("scripts.setup_host.clone_repo", lambda: None)
    monkeypatch.setattr("scripts.setup_host.setup_workspace", lambda: None)
    monkeypatch.setattr("scripts.setup_host.ensure_ssh_keys", lambda: None)
    monkeypatch.setattr("scripts.setup_host.install_ros2", lambda: None)
    monkeypatch.setattr("scripts.setup_host.install_zeno", lambda: None)
    launched = {}
    monkeypatch.setattr(
        "scripts.setup_host.launch_hrs04", lambda cfg: launched.update(cfg)
    )
    from scripts.setup_host import main

    main()
    assert launched == {"trig_pin": 3, "echo_pin": 4}


def test_main_launches_display_service(monkeypatch):
    config = {
        "hosts": {
            "brainstem": {
                "services": ["display"],
                "display": {"topics": ["/range"]},
            }
        }
    }
    monkeypatch.setattr("scripts.setup_host.load_config", lambda: config)
    monkeypatch.setattr("scripts.setup_host.socket.gethostname", lambda: "brainstem")
    monkeypatch.setattr("scripts.setup_host.ensure_service_user", lambda: None)
    monkeypatch.setattr("scripts.setup_host.clone_repo", lambda: None)
    monkeypatch.setattr("scripts.setup_host.setup_workspace", lambda: None)
    monkeypatch.setattr("scripts.setup_host.ensure_ssh_keys", lambda: None)
    monkeypatch.setattr("scripts.setup_host.install_ros2", lambda: None)
    monkeypatch.setattr("scripts.setup_host.install_zeno", lambda: None)
    launched = {}
    monkeypatch.setattr(
        "scripts.setup_host.launch_display", lambda cfg: launched.update(cfg)
    )
    from scripts.setup_host import main

    main()
    assert launched == {"topics": ["/range"]}


def test_main_launches_voice_and_logticker(monkeypatch):
    config = {"hosts": {"brainstem": {"services": ["voice", "logticker"]}}}
    monkeypatch.setattr("scripts.setup_host.load_config", lambda: config)
    monkeypatch.setattr("scripts.setup_host.socket.gethostname", lambda: "brainstem")
    monkeypatch.setattr("scripts.setup_host.ensure_service_user", lambda: None)
    monkeypatch.setattr("scripts.setup_host.clone_repo", lambda: None)
    monkeypatch.setattr("scripts.setup_host.setup_workspace", lambda: None)
    monkeypatch.setattr("scripts.setup_host.ensure_ssh_keys", lambda: None)
    monkeypatch.setattr("scripts.setup_host.install_ros2", lambda: None)
    monkeypatch.setattr("scripts.setup_host.install_zeno", lambda: None)
    called = {"pkg": False, "voice": False, "tick": False}
    monkeypatch.setattr(
        "scripts.setup_host.install_voice_packages", lambda: called.__setitem__("pkg", True)
    )
    monkeypatch.setattr(
        "scripts.setup_host.launch_voice", lambda: called.__setitem__("voice", True)
    )
    monkeypatch.setattr(
        "scripts.setup_host.launch_logticker", lambda: called.__setitem__("tick", True)
    )
    from scripts.setup_host import main

    main()
    assert all(called.values())

def test_main_launches_asr(monkeypatch):
    config = {
        'hosts': {'brainstem': {'services': ['asr'], 'asr': {'model': 'small'}}}
    }
    monkeypatch.setattr('scripts.setup_host.load_config', lambda: config)
    monkeypatch.setattr('scripts.setup_host.socket.gethostname', lambda: 'brainstem')
    monkeypatch.setattr('scripts.setup_host.ensure_service_user', lambda: None)
    monkeypatch.setattr('scripts.setup_host.clone_repo', lambda: None)
    monkeypatch.setattr('scripts.setup_host.setup_workspace', lambda: None)
    monkeypatch.setattr('scripts.setup_host.ensure_ssh_keys', lambda: None)
    monkeypatch.setattr('scripts.setup_host.install_ros2', lambda: None)
    monkeypatch.setattr('scripts.setup_host.install_zeno', lambda: None)
    called = {}
    monkeypatch.setattr('scripts.setup_host.install_asr_packages', lambda: called.setdefault('pkg', True))
    monkeypatch.setattr('scripts.setup_host.launch_asr', lambda cfg: called.setdefault('cfg', cfg))
    from scripts.setup_host import main
    main()
    assert called['pkg'] and called['cfg']['model'] == 'small'
