Pete Knightykell — Cerebellum

Purpose
- ROS 2 (Jazzy) + Nav2 stack, running in Docker on a Raspberry Pi 4 (Raspberry Pi OS Lite).
- Connects directly to the iRobot Create 1 via a USB serial dongle (DB‑25 harness removed); publishes navigation and perception.

Brainstem deprecation
- The separate Arduino “brainstem” bridge is deprecated. We switched from the DB‑25 harness to a direct USB serial dongle, which has worked well so far.
- If cerebellum boot time ends up being too slow for reliable bring‑up, we may reinstate the brainstem.

What’s here
- docker/Dockerfile: ROS 2 Jazzy base + tools + Nav2 + common drivers
- docker/compose.yml: host networking, privileged device access
- docker/ros2_ws.repos: vcs sources (AutonomyLab create driver, add your own)
  - Includes `AutonomyLab/create_robot`, `revyos-ros/libcreate` (fix-std-string), and `kiwicampus/mpu6050driver`.
- firstboot/: systemd unit + script that installs Docker and starts compose
- firstboot/oled-statusd.service + host/oled/: early-boot SH1106 OLED daemon and client
- tools/build_rpi4_rpios_image.sh: downloads Raspberry Pi OS Lite (arm64), injects files, outputs a burnable image
 - docker/ros_statusd.py: ROS 2 status daemon with an HTTP dashboard + JSON API

Quick start
- Build the Raspberry Pi OS image (preferred):
  - At repo root: `make cerebellum-img`
  - Optional preload: if `Knightykell/cerebellum:latest` exists locally, the builder saves and embeds it at `/opt/cerebellum/image.tar` for faster first boot.
  - Output: `cerebellum/out/rpios-rpi4-cerebellum.img`
- Alternative (manual script):
  - `sudo cerebellum/tools/build_rpi4_rpios_image.sh`
- Build the container image (optional):
  - `make cerebellum-build`
  - Or: `cd cerebellum/docker && docker build -t Knightykell/cerebellum:latest .`
  - To preload: `docker save Knightykell/cerebellum:latest -o cerebellum/firstboot/image.tar`

Runtime
- On first boot, `/opt/cerebellum/firstboot.sh`:
  - Enables I2C and installs OLED dependencies (luma.oled, PIL, Noto fonts)
  - Ensures time sync (systemd-timesyncd) before Docker pulls
  - Installs Docker + compose plugin if missing; enables Docker
  - Loads `/opt/cerebellum/image.tar` if present (preloaded image)
  - Starts the stack via `cerebellum-docker.service` (compose). Falls back to `/opt/cerebellum/docker/compose_up.sh` if needed.
- An early OLED daemon (`oled-statusd.service`) renders status on SH1106/SSD1306 and tails boot logs until the system stabilizes.

OLED status
- Daemon: `/opt/cerebellum/oled/oled_statusd.py` (systemd unit `oled-statusd.service`)
- Socket API: UNIX datagram at `/run/oled/statusd.sock`
- Message format: JSON `{"header": "BRINGUP", "lines": ["text1", "text2", ...], "ttl": 6}`
- CLI client: `/opt/cerebellum/oled/oled_client.py [--ttl SEC] HEADER [LINE ...]`
- Convenience tool (installed to PATH on first boot): `oledctl`
  - `oledctl msg --ttl 8 "HEADER" "line1" "line2"`
  - `oledctl sys` | `oledctl net` | `oledctl logs`
- The container mounts `/run/oled` and uses the client to emit progress.
  - A ROS 2 status daemon (`ros_statusd.py`) periodically updates the OLED and serves an HTTP dashboard on port 8080.

Install helpers
- The image builder installs the OLED daemon and unit automatically. On an existing Pi, copy `host/oled/` and `firstboot/oled-statusd.service` into place and run: `sudo systemctl daemon-reload && sudo systemctl enable --now oled-statusd`.

QEMU emulation
- Raspberry Pi QEMU targets are not maintained in this repo; the Makefile omits them.

Storage growth
- Some upstream images auto-expand the rootfs using `resize2fs_once`. Our build expands the image during creation and also installs a one-shot `cerebellum-growroot.service` that grows the partition/filesystem on first boot to fill the actual SD card.
- Manually install/enable on an existing system if needed:
  - Copy `cerebellum/firstboot/cerebellum-growroot.sh` to `/usr/local/sbin/` and `cerebellum/firstboot/cerebellum-growroot.service` to `/etc/systemd/system/`.
  - `sudo systemctl daemon-reload && sudo systemctl enable --now cerebellum-growroot.service`
  - Or run once by hand: `sudo growpart /dev/mmcblk0 2 && sudo resize2fs /dev/mmcblk0p2` (install `cloud-guest-utils` first).

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
Image download cache
- The RPi OS builder caches the downloaded base image in `cerebellum/out/cache/rpios_latest.archive` and reuses it for up to 7 days (configurable via `CACHE_MAX_AGE_DAYS`).
- Clear the cache: `make cerebellum-cache-clean` or delete files under `cerebellum/out/cache/`.
