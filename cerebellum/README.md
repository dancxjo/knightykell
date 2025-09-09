Pete Knightykell — Cerebellum

Purpose
- ROS 2 (Jazzy) + Nav2 stack, running in Docker on a Raspberry Pi 4 (Armbian-based image).
- Bridges to brainstem over USB serial; publishes navigation and perception.

What’s here
- docker/Dockerfile: ROS 2 Jazzy base + tools + Nav2 + common drivers
- docker/compose.yml: host networking, privileged device access
- docker/ros2_ws.repos: vcs sources (AutonomyLab create driver, add your own)
- firstboot/: systemd unit + script that installs Docker and starts compose
- firstboot/oled-statusd.service + host/oled/: early-boot SH1106 OLED daemon and client
- tools/build_rpi4_armbian_image.sh: downloads Armbian, injects files, outputs a burnable image
- tools/build_rpi4_rpios_image.sh: downloads Raspberry Pi OS Lite (arm64), injects files, outputs a burnable image
 - docker/ros_statusd.py: ROS 2 status daemon with an HTTP dashboard + JSON API

Quick start
- Build container locally (optional):
  - cd cerebellum/docker && docker build -t Knightykell/cerebellum:latest .
  - docker save Knightykell/cerebellum:latest -o ../firstboot/image.tar
- Bake OS image:
  - cd cerebellum/tools
  - sudo ./build_rpi4_armbian_image.sh
  - result: out/armbian-rpi4-cerebellum.img (flash with Raspberry Pi Imager)
  - or for Raspberry Pi OS Lite (Bookworm arm64):
    - sudo ./build_rpi4_rpios_image.sh
    - result: out/rpios-rpi4-cerebellum.img

Runtime
- On first boot, the systemd unit runs `/opt/cerebellum/firstboot.sh` which:
  - Installs Docker + compose plugin
- Optionally loads `/opt/cerebellum/image.tar` if present
- On first boot, it prefers `docker compose up --no-build` (faster). If the image is missing, it falls back to building.
  - `docker compose -f /opt/cerebellum/docker/compose.yml up -d`
 - Starts an early OLED daemon (`oled-statusd.service`) that renders status on an SH1106 display.

OLED and e‑Paper status
- Daemon: `/opt/cerebellum/oled/oled_statusd.py` (systemd unit `oled-statusd.service`)
- Socket API: UNIX datagram at `/run/oled/statusd.sock`
- Message format: JSON `{"header": "BRINGUP", "lines": ["text1", "text2", ...], "ttl": 6}`
- CLI client: `/opt/cerebellum/oled/oled_client.py [--ttl SEC] HEADER [LINE ...]`
- Convenience tool (installed to PATH on first boot): `oledctl`
  - `oledctl msg --ttl 8 "HEADER" "line1" "line2"`
  - `oledctl sys` | `oledctl net` | `oledctl logs`
- The container mounts `/run/oled` and uses the client to emit progress.
  - A ROS 2 status daemon (`ros_statusd.py`) periodically updates the OLED and serves an HTTP dashboard on port 8080.

Waveshare 4.2" e‑Paper (optional)
- Enable with `EPD_ENABLE=1` in the environment (unit `firstboot/oled-statusd.service` sets env for OLED; extend as desired).
- Orientation: `EPD_ORIENTATION=landscape|portrait` (default landscape)
- Update cadence: `EPD_UPDATE_INTERVAL=60` (seconds)
- Shows slow‑changing info: greeting, IP, Wi‑Fi SSID, systemd state, time.
- First boot enables SPI (`dtparam=spi=on` or Armbian `spi-spidev`) and installs `python3-rpi.gpio`, `python3-spidev`, `waveshare-epd` (via pip).

Install helpers
- `make install-oled`: copies `host/oled/oled_statusd.py` to `/opt/cerebellum/oled/oled_statusd.py` (mode 755).
- `make install-oled-service`: installs the systemd unit to `/etc/systemd/system/oled-statusd.service` and reloads systemd.
- `make enable-oled`: enables and starts the service.
- Variables: `SUDO` (default `sudo`), `PREFIX` (default `/opt/cerebellum`). Example: `PREFIX=/opt/cerebellum make install-oled`.

Web dashboard
- URL: `http://<robot-ip>:8080/` (auto-refreshing HTML) and `http://<robot-ip>:8080/api/status` (JSON)
- Shows node/topic counts, rates for `/odom`, `/imu/data`, `/scan`, `/camera/image_raw`, and lifecycle states for common Nav2 nodes.

Notes
- Compose uses host networking for Cyclone DDS; set `ROS_DOMAIN_ID` as needed.
- `ros2_ws.repos` seeds source-based packages; apt installs cover commonly used drivers.
- The MPU6050 ROS 2 driver varies; update `ros2_ws.repos` to the implementation you use.

Time sync (NTP)
- First-boot now enables systemd-timesyncd and waits briefly for sync before starting Docker pulls.
- The compose service waits for `time-sync.target` and `network-online.target` to reduce TLS/registry failures due to bad clocks.
- Manual fix if needed:
  - `timedatectl status`
  - `sudo timedatectl set-ntp true`
  - If no WAN, set a one-off reasonable time: `sudo date -s "2025-09-08 23:15:00"`; then NTP will keep it.
