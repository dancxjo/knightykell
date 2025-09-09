Monitor (Laptop Dev)

Purpose
- Lightweight ROS 2 Jazzy desktop environment for development and monitoring.
- Joins the same Cyclone DDS domain/network as the robot and provides tools like rviz2 and rqt.
- Runs GUIs on the host via X11/Wayland while the processes execute in the container.

Quick start
- Build image: make monitor-build (auto-detects docker/podman compose)
- Run rviz2: make monitor-rviz
- Run rqt: make monitor-rqt
- Open a shell: make monitor-shell

Notes
- Networking: uses host networking and CycloneDDS config (cyclonedds.xml) similar to the robot.
- Display: scripts detect Wayland vs X11 and mount the right sockets.
- Permissions: you may need to allow local X clients: xhost +local:
