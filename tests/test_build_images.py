import subprocess
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

from scripts.build_images import get_hosts, build_image


def test_get_hosts_returns_all(monkeypatch, tmp_path):
    p = tmp_path / "hosts.toml"
    p.write_text("[hosts]\n[hosts.a]\nimage='rpi'\n[hosts.b]\nimage='rpi'\n")
    monkeypatch.setattr("scripts.build_images.CONFIG_PATH", p)
    assert set(get_hosts()) == {"a", "b"}

def test_get_hosts_filters():
    cfg = {"hosts": {"a": {"image": "rpi"}, "b": {"image": "rpi"}}}
    assert get_hosts(["b"], cfg) == ["b"]


def test_get_hosts_skips_non_rpi():
    cfg = {"hosts": {"a": {"image": "rpi"}, "b": {}}}
    assert get_hosts(config=cfg) == ["a"]

def test_build_image_invokes_script(monkeypatch):
    calls = []
    def fake_run(cmd, check):
        calls.append(cmd)
    monkeypatch.setattr(subprocess, "run", fake_run)
    build_image("brainstem", run=fake_run)
    assert calls[0][-1] == "brainstem"
