Knightykell System

Overview
- Multi-tier robot architecture with distinct roles:
  - brainstem: DEPRECATED — previously Arduino Pro Micro firmware bridging ROS 2 host ↔ iRobot Create 1 via a DB-25 harness. We now use a direct USB serial dongle to the Create’s OI port. If cerebellum boot time proves too slow, we may revive the brainstem.
  - cerebellum: ROS 2 + Nav2 stack for mapping, localization, and motion.
  - forebrain: GPU host for LLMs, advanced vision, and goal planning.

Folders
- brainstem/: (deprecated) legacy low-level bridge firmware
- cerebellum/: navigation, sensor fusion, and planning on ROS 2
- forebrain/: high-level intelligence, models, and coordination
- monitor/: ROS 2 desktop tools (rviz2, rqt) for development/monitoring

Build/Run
- Use the top-level `Makefile` for project tasks.
  - Monitor image: `make monitor-build`
  - Monitor up (dev, with DDS + GUI mounts): `make monitor-up`
  - Run rviz2: `make monitor-rviz`
  - Run rqt: `make monitor-rqt`
  - Cerebellum image: `make cerebellum-build`
  - Bake RPi OS image: `make rpios-img` (under `cerebellum/`)
  - Enable I2C on host: `make enable-i2c`

Notes
- DDS: both monitor and cerebellum use host networking and CycloneDDS; set `ROS_DOMAIN_ID=<id>` in your environment if needed.
- Display: monitor maps X11/Wayland sockets and `~/.Xauthority` for rviz2/rqt to render on the host.

Next Steps
- Confirm inter-process protocols (topics/services or custom serial schema).
- Decide container vs. bare-metal for cerebellum/forebrain.
- Add initial scaffolding for builds, launches, and CI.
