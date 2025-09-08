Knightykell System

Overview
- Multi-tier robot architecture with distinct roles:
  - brainstem: firmware on an Arduino Pro Micro bridging ROS 2 host ↔ iRobot Create 1.
  - cerebellum: ROS 2 + Nav2 stack for mapping, localization, and motion.
  - forebrain: GPU host for LLMs, advanced vision, and goal planning.

Folders
- brainstem/: low-level control and sensor I/O
- cerebellum/: navigation, sensor fusion, and planning on ROS 2
- forebrain/: high-level intelligence, models, and coordination

Next Steps
- Confirm inter-process protocols (topics/services or custom serial schema).
- Decide container vs. bare-metal for cerebellum/forebrain.
- Add initial scaffolding for builds, launches, and CI.
