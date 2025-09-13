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
curl -fsSL https://raw.githubusercontent.com/dancxjo/knightykell/refs/heads/main/scripts/install.sh \
  | sudo bash -s -- -o dancxjo -r knightykell -b main
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
- Voice dependencies: `espeak-ng`, `mbrola`, `mbrola-en1`
- ASR dependencies: `openai-whisper`, `webrtcvad`, `sounddevice` (plus `ffmpeg`)
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

Autoinstall (forebrain)
-----------------------

For x86/ARM servers using the official Ubuntu Server ISO (Subiquity), you can fully automate installation using autoinstall. Use the template at `docs/autoinstall/autoinstall.yaml` and `docs/autoinstall/meta-data` with the NoCloud datasource.

- Create a small USB labeled `CIDATA` with the two files above as `user-data` and `meta-data`, or host them over HTTP and boot the installer with: `autoinstall ds=nocloud-net;s=http://<server>/`.
- The provided template runs the PSYCHE installer in `late-commands`:
  - It curls `https://dancxjo.github.io/knightykell/install.sh` and provisions after first boot.
- Customize `identity`, storage, and hostname as needed.

Cloud-init (Raspberry Pi Ubuntu images)
--------------------------------------

For official Ubuntu preinstalled images on Raspberry Pi, drop `docs/cloud-init/user-data` (and a minimal `meta-data`) into the first partition (`system-boot`) before first boot.

- Edit hostname in the template.
- On boot, cloud-init runs the one-liner installer and the PSYCHE services come up after provisioning.
