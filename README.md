Overview
- Host roles are defined in `hosts.toml`. Each host provisions itself on first boot using `scripts/setup_host.py`. Raspberry Pi entries set `image = "rpi"` so an OS image is generated; other hosts, such as `forebrain`, install their OS manually but run the same provisioning.
- Every host starts a `voice` topic backed by `espeak-ng` with an MBROLA voice. Publishing strings to `voice` queues speech, while `logticker` streams system logs to it.
- `brainstem` also runs an HRS04 range sensor, an SSD1306 display, and a Whisper-based ASR service publishing to the `asr` topic (default model `tiny`), configured in `hosts.toml`.

Build
- `make image HOSTS="brainstem cerebellum"` builds images for hosts with `image = "rpi"` (default: all such hosts).
- Burn the produced `.img` files with Raspberry Pi Imager.

Folders
- `scripts/`: provisioning and image build helpers
- `tests/`: unit tests
