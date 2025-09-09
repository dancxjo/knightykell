#!/usr/bin/env bash
set -euo pipefail

LOG=/var/log/cerebellum-growroot.log
mkdir -p /var/lib/cerebellum
exec >>"$LOG" 2>&1
echo "[growroot] $(date -Is) start"

resolve_root() {
  local src
  src=$(findmnt -nr -o SOURCE / || true)
  if [ -z "$src" ]; then
    # fallback via lsblk
    src=$(lsblk -nrpo NAME,MOUNTPOINT | awk '$2=="/"{print $1; exit}')
  fi
  if [ "$src" = "/dev/root" ] && [ -L /dev/root ]; then
    src=$(readlink -f /dev/root || true)
  fi
  echo "$src"
}

ROOTDEV=$(resolve_root)
if [ -z "$ROOTDEV" ]; then
  echo "[growroot] ERROR: cannot determine root device"
  exit 0
fi
BASEDISK=$(lsblk -no PKNAME "$ROOTDEV" 2>/dev/null | head -n1 || true)
PARTNUM=$(lsblk -no PARTNUM "$ROOTDEV" 2>/dev/null | head -n1 || true)
if [ -z "$BASEDISK" ] || [ -z "$PARTNUM" ]; then
  echo "[growroot] INFO: fallback to mmcblk0p2 assumptions"
  BASEDISK=mmcblk0
  PARTNUM=2
fi

echo "[growroot] root=$ROOTDEV base=/dev/$BASEDISK part=$PARTNUM"

# Grow the partition to fill device (prefer sfdisk; fallback to growpart if present)
if command -v sfdisk >/dev/null 2>&1; then
  echo "[growroot] using sfdisk to expand partition /dev/$BASEDISK p$PARTNUM"
  # Expand partition to end (',+')
  printf ",+\n" | sfdisk -N "$PARTNUM" "/dev/$BASEDISK" || true
  partprobe "/dev/$BASEDISK" 2>/dev/null || true
  blockdev --rereadpt "/dev/$BASEDISK" 2>/dev/null || true
  udevadm settle || true
elif command -v growpart >/dev/null 2>&1; then
  echo "[growroot] using growpart /dev/$BASEDISK $PARTNUM"
  growpart "/dev/$BASEDISK" "$PARTNUM" || true
else
  echo "[growroot] WARN: no sfdisk/growpart available; skipping partition resize"
fi

# Grow the filesystem online (ext4 supports online resize)
echo "[growroot] running resize2fs $ROOTDEV"
resize2fs "$ROOTDEV" || true

touch /var/lib/cerebellum/growroot.done
echo "[growroot] $(date -Is) done"
