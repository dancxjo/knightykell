#!/usr/bin/env bash
set -euo pipefail

IMG=${1:-out/rpios-rpi4-cerebellum.img}
MACHINE=${MACHINE:-raspi3b}
RAM=${RAM:-1024}
SMP=${SMP:-4}

if ! command -v qemu-system-aarch64 >/dev/null 2>&1; then
  echo "[qemu] ERROR: qemu-system-aarch64 not found. Install qemu-system-aarch64." >&2
  exit 1
fi

if [ ! -f "$IMG" ]; then
  echo "[qemu] ERROR: image not found: $IMG" >&2
  exit 1
fi

WORKDIR=$(mktemp -d)
cleanup() {
  set +e
  sudo umount "$WORKDIR/boot" 2>/dev/null || true
  rmdir "$WORKDIR/boot" 2>/dev/null || true
  rm -rf "$WORKDIR"
}
trap cleanup EXIT

mkdir -p "$WORKDIR/boot"

# Determine boot partition offset and mount to extract kernel/dtb
SECTOR_SIZE=512
START_SECTOR=$(fdisk -l "$IMG" | awk '/^\s*Device\s+Boot/{flag=1;next} flag && /img1/{print $2; exit}')
if [ -z "$START_SECTOR" ]; then
  echo "[qemu] ERROR: could not locate first partition start sector" >&2
  exit 1
fi
OFFSET=$((START_SECTOR * SECTOR_SIZE))

sudo mount -o loop,offset=$OFFSET "$IMG" "$WORKDIR/boot"

KERNEL="$WORKDIR/boot/kernel8.img"
if [ ! -f "$KERNEL" ]; then
  echo "[qemu] ERROR: kernel8.img not found in boot partition" >&2
  exit 1
fi

# Prefer a Pi 3B+ DTB if present; otherwise Pi 3B
DTB=""
for cand in bcm2710-rpi-3-b-plus.dtb bcm2710-rpi-3-b.dtb; do
  if [ -f "$WORKDIR/boot/$cand" ]; then
    DTB="$WORKDIR/boot/$cand"
    break
  fi
done
if [ -z "$DTB" ]; then
  echo "[qemu] ERROR: Pi 3 DTB not found in boot partition" >&2
  exit 1
fi

echo "[qemu] Using machine=$MACHINE ram=${RAM}M smp=$SMP"
echo "[qemu] Kernel: $KERNEL"
echo "[qemu] DTB:    $DTB"

# Note: QEMU's raspi3* machines emulate an SD controller; root is mmcblk0p2
exec qemu-system-aarch64 \
  -M "$MACHINE" \
  -smp "$SMP" -m "$RAM" \
  -serial mon:stdio -display none \
  -kernel "$KERNEL" \
  -dtb "$DTB" \
  -append "console=ttyAMA0,115200 root=/dev/mmcblk0p2 rootfstype=ext4 rw" \
  -sd "$IMG"

