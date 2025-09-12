#!/usr/bin/env bash
set -euo pipefail

# Build an Ubuntu Server image (24.04) for a Raspberry Pi host.
# Usage: build_ubuntu_image.sh <host> <model>

HOST=${1:?"usage: $0 <host> <model>"}
MODEL=${2:?"usage: $0 <host> <model>"}

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
OUT_DIR=${OUT_DIR:-$REPO_ROOT/images}
CACHE_DIR=${CACHE_DIR:-$OUT_DIR/cache}
WORK_DIR=$(mktemp -d)
mkdir -p "$OUT_DIR" "$CACHE_DIR" "$WORK_DIR"

# Default images per model (override with IMG_URL)
case "$MODEL" in
  pi5)
    DEFAULT_URL="https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04.1-preinstalled-server-arm64+raspi.img.xz" ;;
  zero2w|zero-2-w|pi0-2w|pi-zero-2w)
    DEFAULT_URL="" ;;
  *)
    DEFAULT_URL="" ;;
esac

IMG_URL=${IMG_URL:-$DEFAULT_URL}
if [[ -z "$IMG_URL" ]]; then
  echo "[build] No default Ubuntu image for model '$MODEL'. Set IMG_URL explicitly." >&2
  exit 2
fi

ARCHIVE="$CACHE_DIR/$(basename "$IMG_URL")"
BASE_IMG="$WORK_DIR/base.img"
if [ ! -f "$ARCHIVE" ]; then
  echo "[build] downloading $(basename "$IMG_URL")"
  curl -fSL -o "$ARCHIVE" "$IMG_URL"
fi
xz -dkc "$ARCHIVE" > "$BASE_IMG"

LOOP=$(sudo losetup -fP --show "$BASE_IMG")
BOOT=$(mktemp -d)
ROOT=$(mktemp -d)
sudo mount "${LOOP}p1" "$BOOT"
sudo mount "${LOOP}p2" "$ROOT"

echo "[build] staging provisioning files"
sudo mkdir -p "$ROOT/opt/psyche"
sudo cp "$REPO_ROOT/scripts/setup_host.py" "$ROOT/opt/psyche/setup_host.py"
sudo cp "$REPO_ROOT/firstboot/firstboot.sh" "$ROOT/opt/psyche/firstboot.sh"
sudo cp "$REPO_ROOT/firstboot/psyche-firstboot.service" "$ROOT/etc/systemd/system/psyche-firstboot.service"
sudo cp "$REPO_ROOT/firstboot/oled_log.py" "$ROOT/opt/psyche/oled_log.py"
sudo cp "$REPO_ROOT/hosts.toml" "$ROOT/opt/psyche/hosts.toml"
sudo mkdir -p "$ROOT/opt/psyche/scripts"
sudo cp -r "$REPO_ROOT/scripts/"*.py "$ROOT/opt/psyche/scripts/"
sudo chmod +x "$ROOT/opt/psyche/firstboot.sh"

sudo mkdir -p "$ROOT/etc/systemd/system/multi-user.target.wants"
sudo ln -sf /etc/systemd/system/psyche-firstboot.service \
  "$ROOT/etc/systemd/system/multi-user.target.wants/psyche-firstboot.service"

sudo umount "$BOOT" "$ROOT"
sudo losetup -d "$LOOP"

FINAL_IMG="$OUT_DIR/${HOST}.img"
mv "$BASE_IMG" "$FINAL_IMG"
rm -rf "$BOOT" "$ROOT" "$WORK_DIR"
echo "[build] image ready: $FINAL_IMG"

