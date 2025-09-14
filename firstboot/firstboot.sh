#!/usr/bin/env bash
set -euo pipefail
LOG=/var/log/psyche-firstboot.log
exec >>"$LOG" 2>&1
echo "[firstboot] $(date -Is) starting"

# Try to display early logs on OLED (best-effort; may fail before I2C enabled)
if [ -x /usr/bin/python3 ] && [ -f /opt/psyche/oled_log.py ]; then
    ( tail -n 20 -f "$LOG" | python3 /opt/psyche/oled_log.py ) &
fi

# Enable I2C if not already enabled
NEED_REBOOT=0
if grep -qi 'raspberry pi' /proc/device-tree/model 2>/dev/null; then
    CFG="/boot/config.txt"
    if [ -f /boot/firmware/config.txt ]; then CFG="/boot/firmware/config.txt"; fi
    if ! grep -q '^dtparam=i2c_arm=on' "$CFG"; then
        echo 'dtparam=i2c_arm=on' | sudo tee -a "$CFG"
        echo "[firstboot] I2C enabled in $CFG; reboot required for effect"
        NEED_REBOOT=1
    fi
    sudo modprobe i2c-dev || true
else
    echo "[firstboot] Not a Raspberry Pi; skipping I2C enable."
fi

# If we just enabled I2C, reboot now so it takes effect
if [ "$NEED_REBOOT" -eq 1 ]; then
	echo "[firstboot] rebooting to apply I2C setting"
	sync
	reboot || systemctl reboot || true
	exit 0
fi

echo "[firstboot] launching provisioning"

python3 /opt/psyche/setup_host.py || echo "[firstboot] setup_host.py exited with status $?" >&2
systemctl disable psyche-firstboot.service || true
echo "[firstboot] done at $(date -Is)"
