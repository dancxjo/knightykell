#!/usr/bin/env bash
# Build a Raspberry Pi OS Lite (64-bit) image with cerebellum first-boot + Docker compose
set -euo pipefail

# Config (overridable via env)
# Use the stable redirect for the latest Bookworm Lite arm64 image
IMG_URL_DEFAULT="https://downloads.raspberrypi.com/raspios_lite_arm64_latest"
IMG_URL="${IMG_URL:-$IMG_URL_DEFAULT}"
OUT_DIR="${OUT_DIR:-$(pwd)/out}"
WORK_DIR="${WORK_DIR:-$(pwd)/work}"
REPO_ROOT="${REPO_ROOT:-$(cd "$(dirname "$0")"/../.. && pwd)}"
CE_DIR="$REPO_ROOT/cerebellum"
WORK_IMG="$OUT_DIR/rpios-rpi4-cerebellum.img"

# Preinstall Docker and OLED libs in the image to shorten first boot
PREINSTALL_DOCKER="${PREINSTALL_DOCKER:-1}"

# Desired final image size. Can be overridden via IMG_SIZE (e.g. "16G").
# Default gives ample headroom for Docker and caches.
IMG_SIZE="${IMG_SIZE:-8G}"

echo "[build] Using image: $IMG_URL"
mkdir -p "$OUT_DIR" "$WORK_DIR"
cd "$WORK_DIR"

IMG_FILE_DL="rpios_latest.img.download"
IMG_ARCHIVE="rpios_latest.archive"

download() {
  echo "[build] fetching latest Raspberry Pi OS Lite arm64…"
  curl -fSL --retry 3 --retry-connrefused -o "$IMG_ARCHIVE.tmp" "$IMG_URL"
  mv -f "$IMG_ARCHIVE.tmp" "$IMG_ARCHIVE"
  file "$IMG_ARCHIVE" || true
}

extract() {
  case "$(file -b "$IMG_ARCHIVE")" in
    *Zip\ archive*|*ZIP*)
      echo "[build] unzip archive"
      unzip -o "$IMG_ARCHIVE" > /dev/null
      # Find the first extracted .img
      IMG_FILE=$(ls -1 *.img | head -n1)
      ;;
    *XZ*|*LZMA*)
      echo "[build] xz decompress (suffix-agnostic)"
      local outimg
      outimg="${IMG_ARCHIVE%.*}.img"
      xz -dc "$IMG_ARCHIVE" > "$outimg"
      IMG_FILE="$outimg"
      ;;
    *DOS/MBR\ boot\ sector*|*GPT*)
      echo "[build] appears to be a raw .img; using directly"
      IMG_FILE="$IMG_ARCHIVE"
      ;;
    *)
      echo "[build] ERROR: unsupported archive format: $(file -b "$IMG_ARCHIVE")" >&2
      exit 1
      ;;
  esac
  if [ -z "${IMG_FILE:-}" ] || [ ! -f "$IMG_FILE" ]; then
    echo "[build] ERROR: could not find extracted .img" >&2
    exit 1
  fi
}

download
extract

echo "[build] copying base image -> $WORK_IMG"
cp -f "$IMG_FILE" "$WORK_IMG"
sync

expand_image() {
  # Expand the raw image file to the desired size before partition/filesystem resize
  local target_size="$IMG_SIZE"
  # Convert human size to bytes if possible; if numfmt not available, just attempt truncate
  echo "[build] ensuring image size >= $target_size"
  if command -v stat >/dev/null 2>&1; then
    local cur_bytes
    cur_bytes=$(stat -c%s "$WORK_IMG")
    # numfmt is convenient but optional; fall back to truncate regardless
    if command -v numfmt >/dev/null 2>&1; then
      local want_bytes
      want_bytes=$(numfmt --from=iec "$target_size" 2>/dev/null || echo 0)
      if [ -n "$want_bytes" ] && [ "$want_bytes" -gt "$cur_bytes" ]; then
        truncate -s "$target_size" "$WORK_IMG"
      fi
    else
      # Best effort: always truncate to target (truncate is safe if same size)
      truncate -s "$target_size" "$WORK_IMG"
    fi
  else
    truncate -s "$target_size" "$WORK_IMG"
  fi
}

expand_image

echo "[build] setting up loop device"
LOOP=$(sudo losetup --show -fP "$WORK_IMG")
trap 'sudo losetup -d "$LOOP" || true' EXIT
sleep 1

BOOT_PART="${LOOP}p1"
ROOT_PART="${LOOP}p2"
BOOT_MNT="$WORK_DIR/mnt_boot"
ROOT_MNT="$WORK_DIR/mnt_root"
mkdir -p "$BOOT_MNT" "$ROOT_MNT"

# Grow the root partition and filesystem to fill the expanded image
echo "[build] expanding root partition and filesystem"
sudo parted -s "$LOOP" ---pretend-input-tty "unit % print" >/dev/null 2>&1 || true
sudo parted -s "$LOOP" resizepart 2 100%
# Refresh partition table and remap loop partitions
sudo partprobe "$LOOP" || true
sudo sleep 1
# Run fsck before resizing the filesystem
sudo e2fsck -f -y "$ROOT_PART" >/dev/null 2>&1 || true
sudo resize2fs "$ROOT_PART"

echo "[build] mounting partitions"
sudo mount "$BOOT_PART" "$BOOT_MNT"
sudo mount "$ROOT_PART" "$ROOT_MNT"
trap 'sudo umount -R "$BOOT_MNT" 2>/dev/null || true; sudo umount -R "$ROOT_MNT" 2>/dev/null || true; sudo losetup -d "$LOOP" || true' EXIT

echo "[build] injecting cerebellum payload"
sudo mkdir -p "$ROOT_MNT/opt/cerebellum"
sudo rsync -a --delete "$CE_DIR/docker/" "$ROOT_MNT/opt/cerebellum/docker/"
sudo install -m 0755 "$CE_DIR/firstboot/firstboot.sh" "$ROOT_MNT/opt/cerebellum/firstboot.sh"
sudo mkdir -p "$ROOT_MNT/opt/cerebellum/firstboot"
sudo rsync -a --delete "$CE_DIR/firstboot/" "$ROOT_MNT/opt/cerebellum/firstboot/"
sudo install -m 0644 "$CE_DIR/firstboot/cerebellum-firstboot.service" "$ROOT_MNT/etc/systemd/system/cerebellum-firstboot.service"
sudo install -m 0644 "$CE_DIR/firstboot/cerebellum-docker.service" "$ROOT_MNT/etc/systemd/system/cerebellum-docker.service"
sudo mkdir -p "$ROOT_MNT/opt/cerebellum/oled"
sudo rsync -a --delete "$CE_DIR/host/oled/" "$ROOT_MNT/opt/cerebellum/oled/"
sudo install -m 0644 "$CE_DIR/firstboot/oled-statusd.service" "$ROOT_MNT/etc/systemd/system/oled-statusd.service"
sudo chown -R root:root "$ROOT_MNT/opt/cerebellum" "$ROOT_MNT/etc/systemd/system/cerebellum-firstboot.service"

echo "[build] enabling firstboot and oled services"
sudo mkdir -p "$ROOT_MNT/etc/systemd/system/multi-user.target.wants"
sudo ln -sf ../cerebellum-firstboot.service \
  "$ROOT_MNT/etc/systemd/system/multi-user.target.wants/cerebellum-firstboot.service"
sudo ln -sf ../cerebellum-docker.service \
  "$ROOT_MNT/etc/systemd/system/multi-user.target.wants/cerebellum-docker.service"
sudo mkdir -p "$ROOT_MNT/etc/systemd/system/basic.target.wants"
sudo ln -sf ../oled-statusd.service \
  "$ROOT_MNT/etc/systemd/system/basic.target.wants/oled-statusd.service"

# Install growroot one-shot to ensure full card utilization on first boot
sudo install -m 0755 "$CE_DIR/firstboot/cerebellum-growroot.sh" "$ROOT_MNT/usr/local/sbin/cerebellum-growroot.sh"
sudo install -m 0644 "$CE_DIR/firstboot/cerebellum-growroot.service" "$ROOT_MNT/etc/systemd/system/cerebellum-growroot.service"
sudo ln -sf ../cerebellum-growroot.service \
  "$ROOT_MNT/etc/systemd/system/multi-user.target.wants/cerebellum-growroot.service"

# Optimize boot: keep WiFi (wpa_supplicant + dhcpcd) but mask some slow services if present
mask_unit() {
  local unit="$1"
  if [ -f "$ROOT_MNT/lib/systemd/system/$unit" ] || [ -f "$ROOT_MNT/usr/lib/systemd/system/$unit" ]; then
    sudo ln -sf /dev/null "$ROOT_MNT/etc/systemd/system/$unit"
    echo "[build] masked $unit"
  fi
}

echo "[build] trimming non-essential services (safe mask)"
mask_unit apt-daily.service
mask_unit apt-daily.timer
mask_unit apt-daily-upgrade.service
mask_unit apt-daily-upgrade.timer
mask_unit man-db.timer
mask_unit bluetooth.service
mask_unit hciuart.service
mask_unit triggerhappy.service

# Prefer starting Docker early at boot
if [ "$PREINSTALL_DOCKER" = "1" ]; then
  echo "[build] chroot: pre-install docker + OLED libs to speed first boot"
  # Prepare chroot mounts
  for d in dev dev/pts proc sys; do
    sudo mount --bind "/$d" "$ROOT_MNT/$d"
  done
  # Ensure DNS works inside chroot
  if [ -f /etc/resolv.conf ]; then
    sudo cp -f /etc/resolv.conf "$ROOT_MNT/etc/resolv.conf"
  fi
  host_arch=$(uname -m || true)
  if [ "$host_arch" != "aarch64" ] && command -v qemu-aarch64-static >/dev/null 2>&1; then
    echo "[build] using qemu-aarch64-static for cross-arch chroot"
    sudo install -m 0755 "$(command -v qemu-aarch64-static)" "$ROOT_MNT/usr/bin/qemu-aarch64-static"
    sudo chroot "$ROOT_MNT" /usr/bin/qemu-aarch64-static /bin/bash -euxo pipefail -c '
      export DEBIAN_FRONTEND=noninteractive
      export LANG=C.UTF-8 LC_ALL=C.UTF-8 LANGUAGE=C
      apt-get update
      # Install locales first and generate en_US.UTF-8 to avoid apt/perl warnings
      apt-get install -y --no-install-recommends locales
      sed -i "s/^# *en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/" /etc/locale.gen || true
      locale-gen en_US.UTF-8
      update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
      export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
      apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg \
        i2c-tools python3 python3-pil python3-luma.oled
      # Add Docker official apt repo for compose plugin
      install -m 0755 -d /etc/apt/keyrings
      curl -fsSL https://download.docker.com/linux/debian/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
      chmod a+r /etc/apt/keyrings/docker.gpg
      arch=$(dpkg --print-architecture)
      codename=$( . /etc/os-release; echo "$VERSION_CODENAME" )
      echo "deb [arch=${arch} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian ${codename} stable" > /etc/apt/sources.list.d/docker.list
      apt-get update
      # Prefer docker-ce + compose plugin; fallback to docker.io if repo unavailable
      if ! apt-get install -y --no-install-recommends docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin; then
        apt-get install -y --no-install-recommends docker.io || true
      fi
      # Minimize image size
      apt-get clean
      rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*
      mkdir -p /etc/systemd/system/multi-user.target.wants
      ln -sf /lib/systemd/system/docker.service /etc/systemd/system/multi-user.target.wants/docker.service || true
    '
  elif [ "$host_arch" = "aarch64" ]; then
    echo "[build] native aarch64 host; using direct chroot"
    sudo chroot "$ROOT_MNT" /bin/bash -euxo pipefail -c '
      export DEBIAN_FRONTEND=noninteractive
      export LANG=C.UTF-8 LC_ALL=C.UTF-8 LANGUAGE=C
      apt-get update
      apt-get install -y --no-install-recommends locales
      sed -i "s/^# *en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/" /etc/locale.gen || true
      locale-gen en_US.UTF-8
      update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
      export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
      apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg \
        i2c-tools python3 python3-pil python3-luma.oled
      install -m 0755 -d /etc/apt/keyrings
      curl -fsSL https://download.docker.com/linux/debian/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
      chmod a+r /etc/apt/keyrings/docker.gpg
      arch=$(dpkg --print-architecture)
      codename=$( . /etc/os-release; echo "$VERSION_CODENAME" )
      echo "deb [arch=${arch} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian ${codename} stable" > /etc/apt/sources.list.d/docker.list
      apt-get update
      if ! apt-get install -y --no-install-recommends docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin; then
        apt-get install -y --no-install-recommends docker.io || true
      fi
      apt-get clean
      rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*
      mkdir -p /etc/systemd/system/multi-user.target.wants
      ln -sf /lib/systemd/system/docker.service /etc/systemd/system/multi-user.target.wants/docker.service || true
    '
  else
    echo "[build] WARN: qemu-aarch64-static not found on non-aarch64 host; skipping preinstall"
  fi
  # Unmount chroot binds
  for d in dev/pts dev proc sys; do
    sudo umount "$ROOT_MNT/$d"
  done
fi

# Enable I2C on Raspberry Pi OS (config.txt)
if [ -f "$BOOT_MNT/config.txt" ]; then
  grep -Eq '^(dtparam=i2c_arm=on|dtoverlay=i2c1)' "$BOOT_MNT/config.txt" || \
    echo "dtparam=i2c_arm=on" | sudo tee -a "$BOOT_MNT/config.txt" >/dev/null
fi

echo "[build] unmounting and finalizing"
sync
sudo umount -R "$BOOT_MNT"
sudo umount -R "$ROOT_MNT"
sudo losetup -d "$LOOP"
trap - EXIT

echo "[build] done -> $WORK_IMG"
