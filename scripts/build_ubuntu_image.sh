#!/usr/bin/env bash
set -euo pipefail

# Cleanup handler to detach loop and unmount on error or exit
cleanup() {
  set +e
  if [ -n "${BOOT:-}" ] && mountpoint -q "$BOOT" 2>/dev/null; then sudo umount "$BOOT"; fi
  if [ -n "${ROOT:-}" ] && mountpoint -q "$ROOT" 2>/dev/null; then sudo umount "$ROOT"; fi
  if [ -n "${LOOP:-}" ]; then sudo losetup -d "$LOOP" 2>/dev/null || true; fi
  rm -rf "${BOOT:-}" "${ROOT:-}" "${WORK_DIR:-}" 2>/dev/null || true
}
trap cleanup EXIT

# Build an Ubuntu Server image (24.04) for a Raspberry Pi host.
# Usage: build_ubuntu_image.sh <host> <model>

HOST=${1:?"usage: $0 <host> <model>"}
MODEL=${2:?"usage: $0 <host> <model>"}

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
OUT_DIR=${OUT_DIR:-$REPO_ROOT/images}
CACHE_DIR=${CACHE_DIR:-$OUT_DIR/cache}
mkdir -p "$OUT_DIR" "$CACHE_DIR"
# Use a work dir under OUT_DIR to avoid small /tmp partitions
WORK_DIR=${WORK_DIR:-$(mktemp -d "$OUT_DIR/work.XXXXXX")}
mkdir -p "$WORK_DIR"

# Default images per model (override with IMG_URL)
case "$MODEL" in
  pi5)
    # Try a list of known release URLs (newest first). Users can override with IMG_URL.
    CANDIDATES=(
      "https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04.4-preinstalled-server-arm64+raspi.img.xz"
      "https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04.3-preinstalled-server-arm64+raspi.img.xz"
      "https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04.2-preinstalled-server-arm64+raspi.img.xz"
      "https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04.1-preinstalled-server-arm64+raspi.img.xz"
      "https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04-preinstalled-server-arm64+raspi.img.xz"
    )
    ;;
  zero2w|zero-2-w|pi0-2w|pi-zero-2w)
    CANDIDATES=() ;;
  *)
    CANDIDATES=() ;;
esac

IMG_URL=${IMG_URL:-}
ARCHIVE=""
# Decompress into OUT_DIR to reduce tmp usage
BASE_IMG="$OUT_DIR/${HOST}.base.img"

download_ok=0
if [ -n "$IMG_URL" ]; then
  ARCHIVE="$CACHE_DIR/$(basename "$IMG_URL")"
  if [ ! -f "$ARCHIVE" ]; then
    echo "[build] downloading $(basename "$IMG_URL")"
    if curl -fSL -o "$ARCHIVE" "$IMG_URL"; then
      download_ok=1
    else
      echo "[build] failed to download IMG_URL; falling back to defaults" >&2
      rm -f "$ARCHIVE"
    fi
  else
    download_ok=1
  fi
fi

if [ "$download_ok" -ne 1 ]; then
  for u in "${CANDIDATES[@]}"; do
    ARCHIVE="$CACHE_DIR/$(basename "$u")"
    if [ -f "$ARCHIVE" ]; then
      download_ok=1
      break
    fi
    echo "[build] trying $(basename "$u")"
    if curl -fSL -o "$ARCHIVE" "$u"; then
      download_ok=1
      IMG_URL="$u"
      break
    fi
  done
fi

if [ "$download_ok" -ne 1 ]; then
  echo "[build] No working Ubuntu image URL for model '$MODEL'. Set IMG_URL explicitly." >&2
  exit 2
fi

# Sanity check free space where we'll write the decompressed image (need ~8G)
NEEDED_GB=${NEEDED_GB:-8}
FREE_KB=$(df -Pk "$OUT_DIR" | awk 'NR==2 {print $4}') || FREE_KB=0
FREE_GB=$(( FREE_KB / 1024 / 1024 ))
if [ "$FREE_GB" -lt "$NEEDED_GB" ]; then
  echo "[build] Warning: Low free space in $OUT_DIR (have ${FREE_GB}G, need ~${NEEDED_GB}G)." >&2
fi

if [ ! -s "$ARCHIVE" ]; then
  echo "[build] Error: archive $ARCHIVE missing or empty." >&2
  exit 2
fi

echo "[build] decompressing $(basename "$ARCHIVE") to $(basename "$BASE_IMG")"
xz -dkc "$ARCHIVE" > "$BASE_IMG"

LOOP=$(sudo losetup -fP --show "$BASE_IMG")
BOOT=$(mktemp -d "$WORK_DIR/boot.XXXXXX")
ROOT=$(mktemp -d "$WORK_DIR/root.XXXXXX")
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
sudo cp "$REPO_ROOT/scripts/provision_image.py" "$ROOT/opt/psyche/provision_image.py"
sudo chmod +x "$ROOT/opt/psyche/firstboot.sh"

# Optionally pre-seed large assets (models, voices, caches) for first boot
if [ -n "${ASSETS_SEED_DIR:-}" ] && [ -d "$ASSETS_SEED_DIR" ]; then
  echo "[build] seeding assets from $ASSETS_SEED_DIR"
  sudo mkdir -p "$ROOT/opt/psyche/assets_seed"
  if command -v rsync >/dev/null 2>&1; then
    sudo rsync -a --delete "$ASSETS_SEED_DIR"/ "$ROOT/opt/psyche/assets_seed/"
  else
    # Fallback to cp -a without --delete behavior
    sudo cp -a "$ASSETS_SEED_DIR"/. "$ROOT/opt/psyche/assets_seed/"
  fi
fi

# Minimal chroot prep (bind mounts); apt ops are optional and best-effort
if [ "${CHROOT_PROVISION:-1}" = "1" ]; then
  echo "[build] preparing chroot and provisioning heavy packages"
  if command -v qemu-aarch64-static >/dev/null 2>&1; then
    sudo mkdir -p "$ROOT/usr/bin"
    sudo cp "$(command -v qemu-aarch64-static)" "$ROOT/usr/bin/"
  else
    # If host arch differs and qemu is missing, skip chroot provisioning
    HOST_ARCH=$(uname -m || true)
    if [ "$HOST_ARCH" != "aarch64" ] && [ "$HOST_ARCH" != "arm64" ]; then
      echo "[build] qemu-aarch64-static not found on non-ARM host; skipping chroot provisioning (set CHROOT_PROVISION=0 to silence)." >&2
      CHROOT_PROVISION=0
    fi
  fi
  if [ "${CHROOT_PROVISION:-1}" = "1" ]; then
    sudo mount --bind /dev "$ROOT/dev" || true
    sudo mount --bind /proc "$ROOT/proc" || true
    sudo mount --bind /sys "$ROOT/sys" || true
    # Ensure DNS works inside chroot (copy host resolv and hosts)
    if [ -f /etc/resolv.conf ]; then
      sudo cp /etc/resolv.conf "$ROOT/etc/resolv.conf" || true
    fi
    # On systemd-resolved hosts, real resolv content is here
    if [ -f /run/systemd/resolve/resolv.conf ]; then
      sudo cp /run/systemd/resolve/resolv.conf "$ROOT/etc/resolv.conf" || true
    fi
    if [ -f /etc/hosts ]; then
      sudo cp /etc/hosts "$ROOT/etc/hosts" || true
    fi
    if [ -f /etc/nsswitch.conf ]; then
      sudo cp /etc/nsswitch.conf "$ROOT/etc/nsswitch.conf" || true
    fi
    # Minimal tools + run image provisioner, with retry on apt operations
    APT_OK=0
    if sudo chroot "$ROOT" bash -lc 'for i in 1 2 3; do apt-get update && exit 0 || sleep 5; done; exit 1'; then
      # Install a minimal set of tools; skip venv here to avoid version skew, finish on first boot
      sudo chroot "$ROOT" bash -lc 'DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends curl ca-certificates ffmpeg alsa-utils || true'
      APT_OK=1
    else
      echo "[build] chroot apt-get update failed; skipping in-image provisioning (will finish on first boot)" >&2
    fi
    # Determine host name from hosts.toml if not passed explicitly
    HOST_IN=${HOST_IN:-$HOST}
    if [ "$APT_OK" -eq 1 ]; then
      echo "[build] chroot provisioning for host $HOST_IN"
      sudo chroot "$ROOT" python3 /opt/psyche/provision_image.py "$HOST_IN" || echo "[build] chroot provision_image failed; will finish on first boot" >&2
    else
      echo "[build] skipping provision_image (APT not available in chroot)" >&2
    fi
    sudo umount "$ROOT/dev" || true
    sudo umount "$ROOT/proc" || true
    sudo umount "$ROOT/sys" || true
  fi
fi

sudo mkdir -p "$ROOT/etc/systemd/system/multi-user.target.wants"
sudo ln -sf /etc/systemd/system/psyche-firstboot.service \
  "$ROOT/etc/systemd/system/multi-user.target.wants/psyche-firstboot.service"

sudo umount "$BOOT" "$ROOT"
sudo losetup -d "$LOOP"

FINAL_IMG="$OUT_DIR/${HOST}.img"
mv "$BASE_IMG" "$FINAL_IMG"
rm -rf "$BOOT" "$ROOT" "$WORK_DIR"
echo "[build] image ready: $FINAL_IMG"
