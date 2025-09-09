#!/usr/bin/env bash
set -euo pipefail

LOG=/var/log/cerebellum-firstboot.log
exec >>"$LOG" 2>&1
echo "[firstboot] $(date -Is) starting" 

# Ensure cgroups (Raspberry Pi OS)
if [ -f /boot/cmdline.txt ]; then
  if ! grep -q 'cgroup_enable=memory' /boot/cmdline.txt; then
    sudo sed -i '1 s|$| cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1|' /boot/cmdline.txt || true
    echo "[firstboot] appended cgroup args to cmdline.txt"
  fi
fi

echo "[firstboot] enabling I2C and installing deps + docker (if needed)"
echo i2c-dev > /etc/modules-load.d/i2c-dev.conf

# Enable I2C for Raspberry Pi OS via config.txt
if [ -f /boot/config.txt ]; then
  if ! grep -Eq '^(dtparam=i2c_arm=on|dtoverlay=i2c1)' /boot/config.txt; then
    echo 'dtparam=i2c_arm=on' >> /boot/config.txt || true
  fi
fi

# Ensure clock is sane: enable NTP and try to sync early
echo "[firstboot] enabling NTP time sync"
apt-get update || true
apt-get install -y --no-install-recommends systemd-timesyncd || true
timedatectl set-ntp true || true
systemctl enable --now systemd-timesyncd.service || true
# Briefly wait for sync (best-effort, max ~20s)
for i in $(seq 1 20); do
  synced=$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo "no")
  [ "$synced" = "yes" ] && break
  sleep 1
done
echo "[firstboot] time status: $(timedatectl status | head -n 5 | tr '\n' ' ' )"

# Install only if docker is not already present (speeds up first boot if pre-baked)
if ! command -v docker >/dev/null 2>&1; then
  apt-get update
apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg \
  i2c-tools screen python3 python3-pil python3-luma.oled \
  fonts-noto-core fonts-noto-extra fonts-noto-mono fonts-noto-color-emoji \
  python3-pip
  # Add Docker CE repo to get compose plugin if not present
  install -m 0755 -d /etc/apt/keyrings
  if [ ! -f /etc/apt/keyrings/docker.gpg ]; then
    curl -fsSL https://download.docker.com/linux/debian/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg || true
    chmod a+r /etc/apt/keyrings/docker.gpg || true
  fi
  if [ ! -f /etc/apt/sources.list.d/docker.list ]; then
    arch=$(dpkg --print-architecture)
    codename=$( . /etc/os-release; echo "$VERSION_CODENAME" )
    echo "deb [arch=${arch} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian ${codename} stable" > /etc/apt/sources.list.d/docker.list || true
  fi
  apt-get update || true
  # Prefer docker-ce + compose plugin; fallback to docker.io or pip
  if ! apt-get install -y --no-install-recommends docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin; then
    apt-get install -y --no-install-recommends docker.io || true
    apt-get install -y --no-install-recommends docker-compose-plugin || pip3 install --no-cache-dir docker-compose || true
  fi
  systemctl enable --now docker || true
else
  # Ensure OLED libs present even if docker is preinstalled
  apt-get update
  apt-get install -y --no-install-recommends \
    i2c-tools screen python3 python3-pil python3-luma.oled \
    fonts-noto-core fonts-noto-extra fonts-noto-mono fonts-noto-color-emoji || true
  # Ensure compose present in some form
  if ! docker compose version >/dev/null 2>&1 && ! command -v docker-compose >/dev/null 2>&1; then
    install -m 0755 -d /etc/apt/keyrings
    if [ ! -f /etc/apt/keyrings/docker.gpg ]; then
      curl -fsSL https://download.docker.com/linux/debian/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg || true
      chmod a+r /etc/apt/keyrings/docker.gpg || true
    fi
    if [ ! -f /etc/apt/sources.list.d/docker.list ]; then
      arch=$(dpkg --print-architecture)
      codename=$( . /etc/os-release; echo "$VERSION_CODENAME" )
      echo "deb [arch=${arch} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian ${codename} stable" > /etc/apt/sources.list.d/docker.list || true
      apt-get update || true
    fi
    apt-get install -y --no-install-recommends docker-compose-plugin || pip3 install --no-cache-dir docker-compose || true
  fi
  systemctl enable --now docker || true
fi

usermod -aG docker ${SUDO_USER:-${USER:-root}} || true

# Restart OLED daemon if present (to pick up luma libs)
systemctl restart oled-statusd || true

# Convenience command in PATH
install -m 0755 /opt/cerebellum/oled/oledctl /usr/local/bin/oledctl 2>/dev/null || true

# Ensure Docker build context contains expected client path
if [ -f /opt/cerebellum/oled/oled_client.py ] && [ ! -f /opt/cerebellum/host/oled/oled_client.py ]; then
  mkdir -p /opt/cerebellum/host/oled
  cp -a /opt/cerebellum/oled/oled_client.py /opt/cerebellum/host/oled/oled_client.py || true
fi



# Pre-load image if present
if [ -f /opt/cerebellum/image.tar ]; then
  echo "[firstboot] loading prebuilt image"
  docker load -i /opt/cerebellum/image.tar || true
fi

echo "[firstboot] enabling and starting cerebellum-docker.service"
# Ensure compose wrapper is executable
chmod +x /opt/cerebellum/docker/compose_*.sh 2>/dev/null || true
systemctl enable cerebellum-docker.service || true
if ! systemctl start cerebellum-docker.service; then
  echo "[firstboot] service start failed; running compose directly as fallback"
  /opt/cerebellum/docker/compose_up.sh || true
fi

echo "[firstboot] done at $(date -Is)"
