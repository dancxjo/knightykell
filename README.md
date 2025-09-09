Knightykell System

Overview
- Multi-tier robot architecture with distinct roles:
  - brainstem: DEPRECATED — previously Arduino Pro Micro firmware bridging ROS 2 host ↔ iRobot Create 1 via a DB-25 harness. Current design uses a direct USB serial dongle to the Create’s OI port.
  - cerebellum: ROS 2 + Nav2 stack for mapping, localization, and motion (RPi 4).
  - forebrain: GPU host for LLMs, advanced vision, and goal planning.

Folders
- brainstem/: (deprecated) legacy low-level bridge firmware
- cerebellum/: navigation, sensor fusion, and planning on ROS 2
- forebrain/: high-level intelligence, models, and coordination
- monitor/: ROS 2 desktop tools (rviz2, rqt) for development/monitoring

Build/Run
- Use the top-level `Makefile` for common tasks:
  - `make monitor-build` — build the desktop dev image
  - `make monitor-up` — start the monitor stack
  - `make monitor-rviz` | `make monitor-rqt` — launch GUI tools
  - `make cerebellum-build` — build the cerebellum container image
  - `make cerebellum-img` — build Raspberry Pi OS Lite image (arm64) with first-boot bring-up
  - `make cerebellum-burn` — open Raspberry Pi Imager and show the built image
  - `make cerebellum-flash DEVICE=/dev/sdX` — flash the built image (guarded)
  - `make cerebellum-up` | `make cerebellum-update` — run/update the cerebellum stack via compose

Notes
- DDS: monitor and cerebellum use host networking with Cyclone DDS; set `ROS_DOMAIN_ID=<id>` as needed.
- OLED: on the Pi image, an early-boot OLED daemon renders status on SH1106/SSD1306 (see `cerebellum/host/oled/`).

Next Steps
- Confirm inter-process protocols (topics/services or custom serial schema).
- Decide container vs. bare-metal for cerebellum/forebrain.
- Add initial scaffolding for CI.
