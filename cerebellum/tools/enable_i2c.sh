#!/usr/bin/env bash
set -euo pipefail

echo "[i2c] Enabling I2C on this system (persistently)"

SUDO=${SUDO:-sudo}

${SUDO} mkdir -p /etc/modules-load.d
echo i2c-dev | ${SUDO} tee /etc/modules-load.d/i2c-dev.conf >/dev/null

if [ -f /boot/armbianEnv.txt ]; then
  echo "[i2c] Detected Armbian. Ensuring overlays contain i2c and spi-spidev."
  if ! grep -q '^overlays=' /boot/armbianEnv.txt; then
    echo 'overlays=i2c1 i2c0 spi-spidev' | ${SUDO} tee -a /boot/armbianEnv.txt >/dev/null
  elif ! grep -q 'i2c' /boot/armbianEnv.txt; then
    ${SUDO} sed -i 's/^overlays=.*/& i2c1 i2c0 spi-spidev/' /boot/armbianEnv.txt
  fi
elif [ -f /boot/config.txt ]; then
  echo "[i2c] Detected Raspberry Pi OS. Ensuring dtparam=i2c_arm=on and dtparam=spi=on."
  grep -Eq '^(dtparam=i2c_arm=on|dtoverlay=i2c1)' /boot/config.txt || echo 'dtparam=i2c_arm=on' | ${SUDO} tee -a /boot/config.txt >/dev/null
  grep -q '^dtparam=spi=on' /boot/config.txt || echo 'dtparam=spi=on' | ${SUDO} tee -a /boot/config.txt >/dev/null
else
  echo "[i2c] WARN: neither /boot/armbianEnv.txt nor /boot/config.txt found; please enable I2C manually in your bootloader config."
fi

echo "[i2c] Loading i2c-dev module now (will still require reboot for overlays)"
${SUDO} modprobe i2c-dev || true

if command -v i2cdetect >/dev/null 2>&1; then
  echo "[i2c] Present I2C buses:"
  ls -l /dev/i2c-* 2>/dev/null || true
  echo "[i2c] Scanning bus 1 (ignore if nothing attached):"
  ${SUDO} i2cdetect -y 1 || true
else
  echo "[i2c] i2c-tools not installed; install with: ${SUDO} apt-get install -y i2c-tools"
fi

echo "[i2c] Done. Reboot required for boot config changes to take effect."

