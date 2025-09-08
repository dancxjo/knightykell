#!/usr/bin/env bash
set -euo pipefail

LOG=/var/log/cerebellum-firstboot.log
exec >>"$LOG" 2>&1
echo "[firstboot] $(date -Is) starting" 

# Ensure cgroups (best effort; supports either cmdline.txt or armbianEnv.txt)
if [ -f /boot/cmdline.txt ]; then
  if ! grep -q 'cgroup_enable=memory' /boot/cmdline.txt; then
    sudo sed -i '1 s|$| cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1|' /boot/cmdline.txt || true
    echo "[firstboot] appended cgroup args to cmdline.txt"
  fi
elif [ -f /boot/armbianEnv.txt ]; then
  if ! grep -q '^extraargs=.*cgroup_enable=memory' /boot/armbianEnv.txt; then
    echo 'extraargs=cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1' | sudo tee -a /boot/armbianEnv.txt >/dev/null || true
    echo "[firstboot] added extraargs to armbianEnv.txt"
  fi
fi

echo "[firstboot] enabling I2C and installing deps + docker"
apt-get update
apt-get install -y --no-install-recommends \
  i2c-tools python3 python3-pil python3-luma.oled \
  docker.io docker-compose-plugin
echo i2c-dev > /etc/modules-load.d/i2c-dev.conf
if [ -f /boot/armbianEnv.txt ] && ! grep -q '^overlays=.*i2c' /boot/armbianEnv.txt; then
  echo 'overlays=i2c1 i2c0' >> /boot/armbianEnv.txt || true
fi
systemctl enable --now docker
usermod -aG docker ${SUDO_USER:-${USER:-root}} || true

# Restart OLED daemon if present (to pick up luma libs)
systemctl restart oled-statusd || true

# Pre-load image if present
if [ -f /opt/cerebellum/image.tar ]; then
  echo "[firstboot] loading prebuilt image"
  docker load -i /opt/cerebellum/image.tar || true
fi

echo "[firstboot] bringing up compose"
cd /opt/cerebellum/docker
docker compose -f compose.yml up -d

echo "[firstboot] done at $(date -Is)"
