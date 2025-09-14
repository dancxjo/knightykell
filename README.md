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

Re‑provisioning on the device
-----------------------------

To update an already provisioned host (rebuild workspace, refresh scripts, restart services):

- From a checkout on the device:

```
sudo -E env PSYCHE_SRC=$(pwd) python3 scripts/setup_host.py
```

- Or use the helper installed by provisioning:

```
psyche-provision     # uses $PSYCHE_SRC or /home/pete/psyche if present
```

Both approaches are idempotent — units are reloaded, services restart, the workspace rebuilds, and runtime assets refresh.

Configuration
-------------

- Edit `hosts.toml` and ensure the current host’s `hostname` exists with a `services` list. Example:

```
[hosts]
[hosts.brainstem]
services = ["voice", "logticker", "logsummarizer", "hrs04", "display", "asr"]
[hosts.brainstem.hrs04]
trig_pin = 17
echo_pin = 27
[hosts.brainstem.display]
topics = ["/sensor/range"]
[hosts.brainstem.voice]
# Optional: override Piper model and voices dir
model = "en_US-lessac-high"
voices_dir = "/opt/piper/voices"
[hosts.brainstem.asr]
model = "tiny"
```

What provisioning installs
--------------------------

- ROS 2 Jazzy (adds apt repo/key if missing), `python3-colcon-common-extensions`
- Service user `pete`, workspace at `/home/pete/ros2_ws` (built with `colcon`)
- Zenoh via pip
- Voice dependencies: `piper` plus default voice (Lessac, high). Models are downloaded to `/opt/piper/voices`.
- ASR dependencies: `openai-whisper`, `webrtcvad`, `sounddevice` (plus `ffmpeg`)
- Systemd services (prefixed `psyche-`) that source ROS env before launching

Services
--------

- `psyche-voice.service`: Queue text‑to‑speech from the `voice` topic
- `psyche-logticker.service`: Publish journal lines to the `logs` topic
- `psyche-logsummarizer.service`: Summarize recent `logs` and publish concise updates to `voice`
- `psyche-asr.service`: Publish Whisper transcripts on the `asr` topic
- `psyche-hrs04.service`: Ultrasonic sensor node (pins from host config)
- `psyche-display.service`: SSD1306 OLED topic display

Assets storage
--------------

Large assets (GGUF LLMs, Piper voices, Whisper caches) default to fixed paths:

- Llama GGUFs: `/opt/llama/models` (env: `LLAMA_MODELS_DIR`)
- Piper voices: `/opt/piper/voices` (env: `PIPER_VOICES_DIR`)
- Caches (Whisper, HF): `/opt/psyche/cache` (env: `XDG_CACHE_HOME`)

If you seed assets during image build (via `ASSETS_SEED_DIR`), they are copied into these locations on first boot.

Log summarization
-----------------

The log ticker now publishes raw journal lines on the `logs` topic. The new
log summarizer buffers recent lines and periodically (default every 20s)
publishes a short English summary to the `voice` topic for speaking.

Backends (auto-detected):
- Ollama: set `OLLAMA_MODEL` (default `gpt-oss:20b`) and ensure `ollama` is installed
- llama.cpp: set `LLAMA_MODEL_PATH` to a local `.gguf` model and install `llama-cpp-python` (provisioner helper available)
- Fallback: heuristic summary when no local LLM is available

Environment overrides:
- `SUMMARY_INTERVAL` (seconds, default `20`)
- `SUMMARY_MAX_LINES` (default `200`)
- `LLAMA_MODEL_PATH`, `LLAMA_THREADS`, `LLAMA_CTX`
- `LLAMA_GRAMMAR_PATH` (GBNF file to constrain output)
- `LLAMA_TEMP`, `LLAMA_TOP_K`, `LLAMA_TOP_P`, `LLAMA_MIN_P`, `LLAMA_REPEAT_PENALTY`, `LLAMA_MAX_TOKENS`

Provisioning helpers
--------------------

- llama.cpp runtime: installer adds `llama-cpp-python` to the service venv.
- Model download: set `hosts.<name>.logsummarizer.gguf_url` to fetch a `.gguf` into `/opt/llama/models` and write `LLAMA_MODEL_PATH` automatically.
- Env from config: set options under `[hosts.<name>.logsummarizer]` (e.g., `interval`, `max_lines`, `threads`, `ctx`, `grammar_path`, `top_k`, `temp`, etc.).

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

Image Building (Ubuntu on Raspberry Pi)
--------------------------------------

Build fully staged Ubuntu Server images for Pi models using chroot to pre-seed assets:

- Build all Ubuntu-marked hosts: `make ubuntu-images`
- Filter: `make ubuntu-images HOSTS="brainstem cerebellum"`
- By default, the builder chroots into the image and provisions heavy packages (ROS 2, Python venv, deps). Disable with `CHROOT_PROVISION=0`.
- Optional pre-seed assets into the image (copied to `/opt/psyche/assets_seed` and staged on first boot):

```
ASSETS_SEED_DIR=/path/to/assets \
  make ubuntu-images HOSTS="brainstem"
```

Expected `ASSETS_SEED_DIR` layout (any subset is fine):
- `models/llama/*.gguf`
- `piper/voices/*.onnx` and `*.onnx.json`
- `cache/` (Whisper models live under `XDG_CACHE_HOME`)

Internals:
- The builder mounts the image partitions, stages `/opt/psyche`, and runs `/opt/psyche/provision_image.py <host>` inside chroot to install:
  - ROS 2 base (Jazzy), zenoh, Python venv
  - Voice/ASR deps (Piper, Whisper, sounddevice, webrtcvad)
  - llama-cpp-python for log summarizer
- On first boot, `firstboot` completes host-specific setup (env, units, any remaining installs). Pre-seeded assets are copied into `/opt/llama/models`, `/opt/piper/voices`, and `/opt/psyche/cache`.

Default embedded models and fetching helpers
-------------------------------------------

- Embedded defaults during provisioning:
  - LLM: Llama 3.2 1B Instruct Q4_K_M (`Meta-Llama-3.2-1B-Instruct.Q4_K_M.gguf`) → `$LLAMA_MODELS_DIR`
  - ASR: Whisper `tiny` model warmed into `$XDG_CACHE_HOME`
  - Both happen automatically at provisioning time (best-effort, network required). If assets are baked into the image via `ASSETS_SEED_DIR`, they will be copied into the fixed asset paths on first boot.

- Fetch other models via helper:
  - Script: `scripts/fetch_models.py`
  - Examples:

```
# Fetch defaults into seed layout (useful before image build)
python3 scripts/fetch_models.py --defaults

# Fetch Whisper tiny+base into system cache path, and Llama 3.2 1B Instruct GGUF into models dir
XDG_CACHE_HOME=/opt/psyche/cache \
LLAMA_MODELS_DIR=/opt/llama/models \
python3 scripts/fetch_models.py --whisper tiny base --llama llama32-1b-instruct-q4_k_m
```

- hosts.toml-based prefetch:

```
[hosts.brainstem.assets.prefetch]
whisper = ["tiny", "base"]
llama = ["tinyllama-q4_k_m", "https://example.com/OtherModel.gguf"]
# or
# llama = ["llama32-1b-instruct-q4_k_m", "https://example.com/OtherModel.gguf"]
```

Provisioning reads this and fetches at install time.
