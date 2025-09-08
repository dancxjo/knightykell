Pete KnightyKell — Cerebellum

Purpose
- ROS 2 (Kaiju) + Nav2 stack, running in Docker on a Raspberry Pi 4 (Armbian-based image).
- Bridges to brainstem over USB serial; publishes navigation and perception.

What’s here
- docker/Dockerfile: ROS 2 Kaiju base + tools + Nav2 + common drivers
- docker/compose.yml: host networking, privileged device access
- docker/ros2_ws.repos: vcs sources (AutonomyLab create driver, add your own)
- firstboot/: systemd unit + script that installs Docker and starts compose
- firstboot/oled-statusd.service + host/oled/: early-boot SH1106 OLED daemon and client
- tools/build_rpi4_armbian_image.sh: downloads Armbian, injects files, outputs a burnable image

Quick start
- Build container locally (optional):
  - cd cerebellum/docker && docker build -t knightykell/cerebellum:latest .
  - docker save knightykell/cerebellum:latest -o ../firstboot/image.tar
- Bake OS image:
  - cd cerebellum/tools
  - sudo ./build_rpi4_armbian_image.sh
  - result: out/armbian-rpi4-cerebellum.img (flash with Raspberry Pi Imager)

Runtime
- On first boot, the systemd unit runs `/opt/cerebellum/firstboot.sh` which:
  - Installs Docker + compose plugin
  - Optionally loads `/opt/cerebellum/image.tar` if present
  - `docker compose -f /opt/cerebellum/docker/compose.yml up -d`
 - Starts an early OLED daemon (`oled-statusd.service`) that renders status on an SH1106 display.

OLED status
- Daemon: `/opt/cerebellum/oled/oled_statusd.py` (systemd unit `oled-statusd.service`)
- Socket API: UNIX datagram at `/run/oled/statusd.sock`
- Message format: JSON `{"header": "BRINGUP", "lines": ["text1", "text2", ...]}`
- CLI client: `/opt/cerebellum/oled/oled_client.py HEADER [LINE ...]`
- The container mounts `/run/oled` and uses the client to emit progress.
  - A small ROS 2 bridge inside the container (`ros2_oled_bridge.py`) periodically publishes node/topic counts to the OLED.

Notes
- Compose uses host networking for Cyclone DDS; set `ROS_DOMAIN_ID` as needed.
- `ros2_ws.repos` seeds source-based packages; apt installs cover commonly used drivers.
- The MPU6050 ROS 2 driver varies; update `ros2_ws.repos` to the implementation you use.
