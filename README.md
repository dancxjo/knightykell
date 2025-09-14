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
services = ["voice", "logticker", "logsummarizer", "hrs04", "display", "asr"]
[hosts.brainstem.assets]
# NVMe data volume for large models and caches
label = "PSYCHE_DATA"        # filesystem label to mount (recommended)
mount = "/mnt/psyche"        # mount point (created if missing)
# device = "/dev/nvme0n1p1"  # optional explicit device path
# format_if_empty = true      # only if you want auto-mkfs when blank
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

Assets storage (NVMe)
---------------------

Large assets (GGUF LLMs, Piper voices, Whisper caches) live under a single mount:

- Recommended mount: `/mnt/psyche` with filesystem label `PSYCHE_DATA`.
- Provisioning auto-creates `models/llama`, `piper/voices`, and `cache` under this mount and writes env defaults:
  - `LLAMA_MODELS_DIR=/mnt/psyche/models/llama`
  - `PIPER_VOICES_DIR=/mnt/psyche/piper/voices`
  - `XDG_CACHE_HOME=/mnt/psyche/cache` (Whisper uses this for models)

Headless setup tips (Raspberry Pi 5):
- Pre-format the NVMe partition as ext4 and label it `PSYCHE_DATA`.
- The provisioner will add an `/etc/fstab` entry and mount it automatically.
- Optionally set `[hosts.<name>.assets.device]` and `format_if_empty = true` to allow first-boot formatting when blank.

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
- Optional pre-seed assets into the image (copied to `/opt/psyche/assets_seed` and moved to NVMe at first boot):

```
ASSETS_SEED_DIR=/path/to/assets \
  make ubuntu-images HOSTS="brainstem"
```

Expected `ASSETS_SEED_DIR` layout (any subset is fine):
- `models/llama/*.gguf`
- `piper/voices/*.onnx` and `*.onnx.json`
- `cache/` (Whisper models live under `XDG_CACHE_HOME`)

Advanced: pre-install a few packages in chroot (best-effort, needs `qemu-aarch64-static` on x86 build hosts):

```
CHROOT_APT=1 make ubuntu-images HOSTS="brainstem"
```

The image includes `firstboot` service that runs full provisioning on first boot; pre-seeded assets are moved under the NVMe mount (`/mnt/psyche`) automatically.

Default embedded models and fetching helpers
-------------------------------------------

- Embedded defaults during provisioning:
  - LLM: TinyLlama 1.1B Chat Q4_K_M (`tinyllama-1.1b-chat-v1.0.Q4_K_M.gguf`) → `$LLAMA_MODELS_DIR`
  - ASR: Whisper `tiny` model warmed into `$XDG_CACHE_HOME`
  - Both happen automatically at provisioning time (best-effort, network required). If assets are baked into the image via `ASSETS_SEED_DIR`, they will be moved to NVMe on first boot instead.

- Fetch other models via helper:
  - Script: `scripts/fetch_models.py`
  - Examples:

```
# Fetch defaults into seed layout (useful before image build)
python3 scripts/fetch_models.py --defaults

# Fetch Whisper tiny+base into /mnt/psyche cache, and TinyLlama GGUF into models dir
XDG_CACHE_HOME=/mnt/psyche/cache \
LLAMA_MODELS_DIR=/mnt/psyche/models/llama \
python3 scripts/fetch_models.py --whisper tiny base --llama tinyllama-q4_k_m
```

- hosts.toml-based prefetch:

```
[hosts.brainstem.assets.prefetch]
whisper = ["tiny", "base"]
llama = ["tinyllama-q4_k_m", "https://example.com/OtherModel.gguf"]
```

Provisioning reads this and fetches at install time.
