Pete Knightykell — Brainstem (Deprecated)

Status
- Deprecated: We have switched away from the DB‑25 harness and now use a direct USB serial dongle from the cerebellum to the iRobot Create 1 OI port.
- This setup works fine so far. If the cerebellum’s boot time proves too long for reliable starts, we may revive this Arduino bridge.

Purpose (legacy)
- Arduino Pro Micro code that bridges the cerebellum (ROS 2 host) to the iRobot Create 1 base.
- Handles low‑level serial, motor/turret/LED control passthrough, encoder/bumper/IR reporting.

Notes
- Target: ATmega32u4 (Arduino Pro Micro/Leonardo class).
- Interfaces: USB CDC serial to cerebellum; serial/TTL or GPIO to Create 1.
- Keep real‑time loops tight; debounce and constrain any blocking I/O.

Layout
- platformio.ini: PlatformIO environment for SparkFun Pro Micro 16 MHz (5V)
- src/: main firmware (USB JSON ↔ Create OI bridge)
- include/: headers and protocol definitions
- tools/: flashing scripts and test utilities (future)
- docs/: wiring, pinout, protocol spec (future)

Getting started (legacy)
- Wire `Serial1` (D0=RX, D1=TX on Pro Micro) to the Create 1 OI port (TTL 0–5V), GND shared.
- USB `Serial` is for cerebellum; send newline‑delimited JSON commands (examples below).
- Minimal commands implemented: start/safe/full, drive_direct, leds, estop, sensors.

Example commands (newline‑terminated JSON)
- {"cmd":"start"}
- {"cmd":"safe"}
- {"cmd":"full"}
- {"cmd":"drive_direct","left":150,"right":150}
- {"cmd":"stop"}
- {"cmd":"leds","advance":1,"play":0,"color":128,"intensity":255}
- {"cmd":"sensors","packet":7}

Telemetry
- Periodically queries packet 7 (bumps/wheel drops) and emits lines like:
  {"evt":"bump","raw":3,"left":1,"right":1,"left_drop":0,"right_drop":0}

Safety
- Watchdog stops the robot if no drive command arrives within 500 ms.

Build and flash (PlatformIO, legacy)
- Install PlatformIO Core (CLI) or VS Code + PlatformIO extension.
- Connect the Pro Micro via USB.
- In this folder:
  - Build: `pio run`
  - Upload: `pio run -t upload`
  - Serial monitor: `pio device monitor -b 115200`
