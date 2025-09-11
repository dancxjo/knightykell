# AGENTS Instructions

- Run `pytest` before committing any changes.
- Document public functions with docstrings including examples.
- Keep tests isolated; patch external commands.
- Use concise commit messages.
- Avoid `ls -R`; use `rg` or targeted `ls` instead to navigate.
- Create service user `pete` and a ROS2 workspace during host provisioning.
- Generate systemd units so ROS2 services launch at boot.
