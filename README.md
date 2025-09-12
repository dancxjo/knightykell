PSYCHE Provisioning
===================

Universal, repeatable provisioning for Ubuntu 24.04 hosts that sets up ROS 2 Jazzy, a ROS 2 workspace, and systemd services for voice, ASR, sensors, and display — all driven by `hosts.toml`.

Quick Start (one‑liner)
-----------------------

Provision any Ubuntu 24.04 machine (no git credentials needed):

- Easiest (via GitHub Pages):

```
curl -fsSL https://dancxjo.github.io/knightykell/install.sh | sudo bash
```

- Alternate (raw GitHub, pass owner/repo/branch):

```
curl -fsSL https://raw.githubusercontent.com/<owner>/<repo>/<ref>/scripts/install.sh \
  | sudo bash -s -- -o <owner> -r <repo> -b <ref>
```

What it does:

- Downloads the repo as a zip and extracts to a temp dir
- Runs `scripts/setup_host.py` with `PSYCHE_SRC` pointing at the extracted source
- Installs ROS 2 Jazzy and colcon, creates service user `pete`, builds the workspace, installs required packages, and enables services

Verify services:

```
systemctl --no-pager --type=service | rg '^psyche-'
```

Local Development
-----------------

- Provision from your current checkout:

```
make provision
```

- Re‑run safely; operations are idempotent where practical.

Configuration
-------------

- Edit `hosts.toml` and ensure the current host’s `hostname` exists with a `services` list. Example:

```
[hosts]
[hosts.brainstem]
services = ["voice", "logticker", "hrs04", "display", "asr"]
[hosts.brainstem.hrs04]
trig_pin = 17
echo_pin = 27
[hosts.brainstem.display]
topics = ["/sensor/range"]
[hosts.brainstem.asr]
model = "tiny"
```

What provisioning installs
--------------------------

- ROS 2 Jazzy (adds apt repo/key if missing), `python3-colcon-common-extensions`
- Service user `pete`, workspace at `/home/pete/ros2_ws` (built with `colcon`)
- Zenoh via pip
- Voice dependencies: `espeak-ng`, `mbrola`, `mbrola-us1`
- ASR dependencies: `whisper`, `webrtcvad`, `sounddevice`
- Systemd services (prefixed `psyche-`) that source ROS env before launching

Services
--------

- `psyche-voice.service`: Queue text‑to‑speech from the `voice` topic
- `psyche-logticker.service`: Publish journal lines to the `voice` topic
- `psyche-asr.service`: Publish Whisper transcripts on the `asr` topic
- `psyche-hrs04.service`: Ultrasonic sensor node (pins from host config)
- `psyche-display.service`: SSD1306 OLED topic display

Notes
-----

- Designed for Ubuntu 24.04; `install_ros2()` is idempotent and safe to re‑run.
- Units automatically source `/opt/ros/jazzy` and the local workspace.
- For offline provisioning, the installer and `make provision` both copy from a local source checkout (no git required).
