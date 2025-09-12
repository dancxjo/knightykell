#!/usr/bin/env bash
set -euo pipefail
LOG=/var/log/psyche-firstboot.log
exec >>"$LOG" 2>&1
echo "[firstboot] $(date -Is) starting"

# Enable I2C if not already enabled
NEED_REBOOT=0
if ! grep -q '^dtparam=i2c_arm=on' /boot/config.txt; then
	echo 'dtparam=i2c_arm=on' | sudo tee -a /boot/config.txt
	echo "[firstboot] I2C enabled in /boot/config.txt; reboot required for effect"
	NEED_REBOOT=1
fi
sudo modprobe i2c-dev || true

# If we just enabled I2C, reboot now so it takes effect
if [ "$NEED_REBOOT" -eq 1 ]; then
	echo "[firstboot] rebooting to apply I2C setting"
	sync
	reboot || systemctl reboot || true
	exit 0
fi

# Start OLED log display in background (best-effort)
if [ -x /usr/bin/python3 ] && [ -f /opt/psyche/oled_log.py ]; then
	tail -n 20 -f "$LOG" | python3 /opt/psyche/oled_log.py &
fi

python3 /opt/psyche/setup_host.py
systemctl disable psyche-firstboot.service || true
echo "[firstboot] done at $(date -Is)"
