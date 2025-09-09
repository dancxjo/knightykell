#!/usr/bin/env bash
set -euo pipefail

LOG=/var/log/cerebellum-growroot.log
mkdir -p /var/lib/cerebellum
exec >>"$LOG" 2>&1
echo "[growroot] $(date -Is) start"

# Detect root partition and base disk
ROOTDEV=$(findmnt -nr -o SOURCE /)
if [ -z "$ROOTDEV" ]; then
  echo "[growroot] ERROR: cannot determine root device"
  exit 0
fi
BASEDISK=$(lsblk -no PKNAME "$ROOTDEV" | head -n1)
PARTNUM=$(lsblk -no PARTNUM "$ROOTDEV" | head -n1)
if [ -z "$BASEDISK" ] || [ -z "$PARTNUM" ]; then
  echo "[growroot] INFO: fallback to mmcblk0p2 assumptions"
  BASEDISK=mmcblk0
  PARTNUM=2
fi

echo "[growroot] root=$ROOTDEV base=/dev/$BASEDISK part=$PARTNUM"

# Ensure growpart available
if ! command -v growpart >/dev/null 2>&1; then
  echo "[growroot] installing cloud-guest-utils for growpart"
  apt-get update || true
  apt-get install -y --no-install-recommends cloud-guest-utils || true
fi

# Grow the partition to fill device (best effort)
if command -v growpart >/dev/null 2>&1; then
  echo "[growroot] running growpart /dev/$BASEDISK $PARTNUM"
  growpart "/dev/$BASEDISK" "$PARTNUM" || true
else
  echo "[growroot] WARN: growpart not available"
fi

# Grow the filesystem online (ext4 supports online resize)
echo "[growroot] running resize2fs $ROOTDEV"
resize2fs "$ROOTDEV" || true

touch /var/lib/cerebellum/growroot.done
echo "[growroot] $(date -Is) done"

