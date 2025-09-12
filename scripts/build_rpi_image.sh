#!/usr/bin/env bash
# Build a Raspberry Pi OS Lite image with first-boot provisioning for a host.
set -euo pipefail

HOST=${1:?"usage: $0 <host>"}
IMG_URL=${IMG_URL:-https://downloads.raspberrypi.com/raspios_lite_arm64_latest}
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
OUT_DIR=${OUT_DIR:-$REPO_ROOT/images}
WORK_DIR=$(mktemp -d)
CACHE_DIR=${CACHE_DIR:-$OUT_DIR/cache}
mkdir -p "$OUT_DIR" "$CACHE_DIR" "$WORK_DIR"

ARCHIVE="$CACHE_DIR/base.img.xz"
BASE_IMG="$WORK_DIR/base.img"
if [ ! -f "$ARCHIVE" ]; then
  echo "[build] downloading base image"
  curl -fSL -o "$ARCHIVE" "$IMG_URL"
fi
xz -dkc "$ARCHIVE" > "$BASE_IMG"

LOOP=$(sudo losetup -fP --show "$BASE_IMG")
BOOT=$(mktemp -d)
ROOT=$(mktemp -d)
sudo mount "${LOOP}p1" "$BOOT"
sudo mount "${LOOP}p2" "$ROOT"

sudo mkdir -p "$ROOT/opt/psyche/scripts"
sudo cp "$REPO_ROOT/scripts/setup_host.py" "$ROOT/opt/psyche/setup_host.py"
sudo cp "$REPO_ROOT/firstboot/firstboot.sh" "$ROOT/opt/psyche/firstboot.sh"
sudo chmod +x "$ROOT/opt/psyche/firstboot.sh"
sudo cp "$REPO_ROOT/firstboot/psyche-firstboot.service" "$ROOT/etc/systemd/system/psyche-firstboot.service"
# Copy configuration and helper scripts used at first boot
sudo cp "$REPO_ROOT/hosts.toml" "$ROOT/opt/psyche/hosts.toml"
sudo cp "$REPO_ROOT/firstboot/oled_log.py" "$ROOT/opt/psyche/oled_log.py"
# Deploy service scripts so units can run without a network clone
sudo cp -r "$REPO_ROOT/scripts/"*.py "$ROOT/opt/psyche/scripts/"
sudo chroot "$ROOT" systemctl enable psyche-firstboot.service >/dev/null 2>&1 || true

sudo umount "$BOOT" "$ROOT"
sudo losetup -d "$LOOP"

FINAL_IMG="$OUT_DIR/${HOST}.img"
mv "$BASE_IMG" "$FINAL_IMG"
rm -rf "$BOOT" "$ROOT" "$WORK_DIR"
echo "[build] image ready: $FINAL_IMG"
