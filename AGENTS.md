# AGENTS Instructions

- Run `pytest` before committing any changes.
- Document public functions with docstrings including examples.
- Keep tests isolated; patch external commands.
- Use concise commit messages.
- Avoid `ls -R`; use `rg` or targeted `ls` instead to navigate.
- Run services as `root` (no separate service user) and create a ROS2 workspace during host provisioning.
- Generate systemd units so ROS2 services launch at boot.
- Use `make image HOSTS="<names>"` to build Raspberry Pi images for hosts.
- Mark Raspberry Pi hosts with `image = "rpi"` in `hosts.toml`; hosts without it won't get images but still provision at boot.
- Install Piper TTS and a default voice model for the voice service.
- Install Python packages `openai-whisper`, `webrtcvad`, and `sounddevice` for the ASR service.
- Update docstring examples and tests when host names or services change.
