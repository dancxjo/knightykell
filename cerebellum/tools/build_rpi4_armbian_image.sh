#!/usr/bin/env bash
# Build a Raspberry Pi 4 Armbian image with cerebellum first-boot + Docker compose
set -euo pipefail

# You can override these via env vars
IMG_URL_DEFAULT="https://mirrors.edge.kernel.org/armbian/dl/rpi4b/archive/Armbian_24.5.1_Rpi4b_noble_current_6.6.30_minimal.img.xz"
IMG_URL="${IMG_URL:-$IMG_URL_DEFAULT}"
OUT_DIR="${OUT_DIR:-$(pwd)/out}"
WORK_DIR="${WORK_DIR:-$(pwd)/work}"
REPO_ROOT="${REPO_ROOT:-$(cd "$(dirname "$0")"/../.. && pwd)}"
CE_DIR="$REPO_ROOT/cerebellum"

echo "[build] Using image: $IMG_URL"
mkdir -p "$OUT_DIR" "$WORK_DIR"

cd "$WORK_DIR"
IMG_COMPRESSED=$(basename "$IMG_URL")
if [ ! -f "$IMG_COMPRESSED" ]; then
  echo "[build] downloading base image..."
  curl -L "$IMG_URL" -o "$IMG_COMPRESSED"
else
  echo "[build] using cached $IMG_COMPRESSED"
fi

# Decompress if needed
case "$IMG_COMPRESSED" in
  *.xz) IMG_FILE="${IMG_COMPRESSED%.xz}"; xz -dkf "$IMG_COMPRESSED" ;;
  *.zip) IMG_FILE="${IMG_COMPRESSED%.zip}.img"; unzip -o "$IMG_COMPRESSED" ;;
  *.img) IMG_FILE="$IMG_COMPRESSED" ;;
  *) echo "[build] Unsupported image compression" >&2; exit 1 ;;
esac

WORK_IMG="$OUT_DIR/armbian-rpi4-cerebellum.img"
cp -f "$IMG_FILE" "$WORK_IMG"
sync

echo "[build] setting up loop device"
LOOP=$(sudo losetup --show -fP "$WORK_IMG")
trap 'sudo losetup -d "$LOOP" || true' EXIT
sleep 1

# Identify partitions (assume p1=boot, p2=root)
BOOT_PART="${LOOP}p1"
ROOT_PART="${LOOP}p2"

BOOT_MNT="$WORK_DIR/mnt_boot"
ROOT_MNT="$WORK_DIR/mnt_root"
mkdir -p "$BOOT_MNT" "$ROOT_MNT"

echo "[build] mounting partitions"
sudo mount "$BOOT_PART" "$BOOT_MNT"
sudo mount "$ROOT_PART" "$ROOT_MNT"
trap 'sudo umount -R "$BOOT_MNT" 2>/dev/null || true; sudo umount -R "$ROOT_MNT" 2>/dev/null || true; sudo losetup -d "$LOOP" || true' EXIT

echo "[build] injecting cerebellum payload"
sudo mkdir -p "$ROOT_MNT/opt/cerebellum"
sudo rsync -a --delete "$CE_DIR/docker/" "$ROOT_MNT/opt/cerebellum/docker/"
sudo install -m 0755 "$CE_DIR/firstboot/firstboot.sh" "$ROOT_MNT/opt/cerebellum/firstboot.sh"
sudo install -m 0644 "$CE_DIR/firstboot/cerebellum-firstboot.service" "$ROOT_MNT/etc/systemd/system/cerebellum-firstboot.service"
sudo mkdir -p "$ROOT_MNT/opt/cerebellum/oled"
sudo rsync -a --delete "$CE_DIR/host/oled/" "$ROOT_MNT/opt/cerebellum/oled/"
sudo install -m 0644 "$CE_DIR/firstboot/oled-statusd.service" "$ROOT_MNT/etc/systemd/system/oled-statusd.service"
sudo chown -R root:root "$ROOT_MNT/opt/cerebellum" "$ROOT_MNT/etc/systemd/system/cerebellum-firstboot.service"

echo "[build] enabling firstboot and oled services"
sudo mkdir -p "$ROOT_MNT/etc/systemd/system/multi-user.target.wants"
sudo ln -sf ../cerebellum-firstboot.service \
  "$ROOT_MNT/etc/systemd/system/multi-user.target.wants/cerebellum-firstboot.service"
sudo mkdir -p "$ROOT_MNT/etc/systemd/system/basic.target.wants"
sudo ln -sf ../oled-statusd.service \
  "$ROOT_MNT/etc/systemd/system/basic.target.wants/oled-statusd.service"

# Optionally seed timezone and SSH (Armbian normally handles first login; skip here)

echo "[build] unmounting and finalizing"
sync
sudo umount -R "$BOOT_MNT"
sudo umount -R "$ROOT_MNT"
sudo losetup -d "$LOOP"
trap - EXIT

echo "[build] done -> $WORK_IMG"
