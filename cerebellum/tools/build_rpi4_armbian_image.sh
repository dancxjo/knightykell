#!/usr/bin/env bash
# Build a Raspberry Pi 4 Armbian image with cerebellum first-boot + Docker compose
set -euo pipefail

# You can override these via env vars
# Prefer a working Armbian mirror and validate downloads to avoid caching HTML errors.
# Default to a current, available RPi4B Noble minimal image. Override with IMG_URL or IMG_FILE.
IMG_FILE_DEFAULT="Armbian_25.8.1_Rpi4b_noble_current_6.12.41_minimal.img.xz"
IMG_FILE="${IMG_FILE:-$IMG_FILE_DEFAULT}"
MIRRORS=(
  "https://armbian.systemonachip.net/dl/rpi4b/archive"
  "https://fi.mirror.armbian.de/dl/rpi4b/archive"
)
IMG_URL="${IMG_URL:-${MIRRORS[0]}/$IMG_FILE}"
OUT_DIR="${OUT_DIR:-$(pwd)/out}"
WORK_DIR="${WORK_DIR:-$(pwd)/work}"
REPO_ROOT="${REPO_ROOT:-$(cd "$(dirname "$0")"/../.. && pwd)}"
CE_DIR="$REPO_ROOT/cerebellum"

echo "[build] Using image: $IMG_URL"
mkdir -p "$OUT_DIR" "$WORK_DIR"

cd "$WORK_DIR"
IMG_COMPRESSED=$(basename "$IMG_URL")

ensure_valid_xz() {
  local f="$1"
  if ! xz -t "$f" >/dev/null 2>&1; then
    echo "[build] ERROR: '$f' is not a valid xz archive (likely a cached HTML error)." >&2
    file "$f" || true
    rm -f "$f" "$f.sha" 2>/dev/null || true
    return 1
  fi
  return 0
}

download_and_verify() {
  local url="$1"
  local fname
  fname=$(basename "$url")
  echo "[build] fetching: $url"
  curl -fSL --retry 3 --retry-connrefused -o "$fname.tmp" "$url" || return 1
  mv -f "$fname.tmp" "$fname"

  # Try checksum verification if available
  if curl -fsSL -o "$fname.sha.tmp" "$url.sha"; then
    mv -f "$fname.sha.tmp" "$fname.sha"
    if ! sha256sum -c "$fname.sha"; then
      echo "[build] checksum verification FAILED for $fname from $url" >&2
      rm -f "$fname" "$fname.sha"
      return 1
    fi
  else
    # Fallback to xz integrity test
    case "$fname" in
      *.xz) ensure_valid_xz "$fname" || return 1 ;;
    esac
  fi
  echo "[build] verified: $fname"
  return 0
}

if [ -f "$IMG_COMPRESSED" ]; then
  echo "[build] using cached $IMG_COMPRESSED"
  case "$IMG_COMPRESSED" in
    *.xz) ensure_valid_xz "$IMG_COMPRESSED" || echo "[build] cached file invalid; will re-download" ;;
  esac
fi

if [ ! -f "$IMG_COMPRESSED" ]; then
  echo "[build] downloading base image with mirror fallback..."
  # Build candidate URL list: honor explicit IMG_URL first; otherwise try mirrors
  CANDIDATES=("$IMG_URL")
  if [ -z "${IMG_URL+x}" ] || [[ "$IMG_URL" == *"$IMG_FILE" ]]; then
    for m in "${MIRRORS[@]}"; do
      CANDIDATES+=("$m/$IMG_FILE")
    done
  fi
  # Deduplicate candidates while preserving order
  uniq_candidates=()
  for c in "${CANDIDATES[@]}"; do
    skip="false"
    for u in "${uniq_candidates[@]}"; do [ "$u" = "$c" ] && skip="true" && break; done
    [ "$skip" = "false" ] && uniq_candidates+=("$c")
  done

  success=0
  for u in "${uniq_candidates[@]}"; do
    if download_and_verify "$u"; then success=1; break; fi
  done
  if [ "$success" -ne 1 ]; then
    echo "[build] ERROR: failed to download and verify image from all candidates" >&2
    exit 1
  fi
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
