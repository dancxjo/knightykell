#!/usr/bin/env bash
set -euo pipefail
LOG=/var/log/banks-firstboot.log
exec >>"$LOG" 2>&1
echo "[firstboot] $(date -Is) starting"

# Enable I2C if not already enabled
if ! grep -q '^dtparam=i2c_arm=on' /boot/config.txt; then
	echo 'dtparam=i2c_arm=on' | sudo tee -a /boot/config.txt
	echo "[firstboot] I2C enabled in /boot/config.txt; reboot required for effect"
fi
sudo modprobe i2c-dev || true

# Start OLED log display in background
tail -n 20 -f "$LOG" | python3 /home/pete/firstboot/oled_log.py &

python3 /opt/banks/setup_host.py || true
systemctl disable banks-firstboot.service || true
echo "[firstboot] done at $(date -Is)"
