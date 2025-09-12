#!/usr/bin/env bash
# PSYCHE one-line installer served via GitHub Pages.
# Defaults to this repository; override with OWNER/REPO/REF env vars or flags.
# Example (no args):
#   curl -fsSL https://dancxjo.github.io/knightykell/install.sh | sudo bash

set -euo pipefail

owner=${OWNER:-dancxjo}
repo=${REPO:-knightykell}
ref=${REF:-main}
zip_url=""

while [ $# -gt 0 ]; do
  case "$1" in
    -o|--owner) owner="$2"; shift 2 ;;
    -r|--repo) repo="$2"; shift 2 ;;
    -b|--branch) ref="$2"; shift 2 ;;
    -u|--url) zip_url="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: [sudo] bash install.sh [-o <owner>] [-r <repo>] [-b <branch>]"
      echo "   or: [sudo] bash install.sh -u <zip_url>"
      exit 0
      ;;
    *) echo "Unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [[ -z "$zip_url" ]]; then
  zip_url="https://codeload.github.com/${owner}/${repo}/zip/refs/heads/${ref}"
fi

if [[ $EUID -ne 0 ]]; then
  echo "Please run with sudo or as root" >&2
  exit 1
fi

apt-get update -y >/dev/null 2>&1 || true
apt-get install -y curl unzip >/dev/null 2>&1 || true

workdir=$(mktemp -d)
trap 'rm -rf "$workdir"' EXIT
cd "$workdir"

echo "[psyche] downloading $zip_url"
curl -fsSL -o src.zip "$zip_url"
unzip -q src.zip
src_dir=$(find . -mindepth 1 -maxdepth 1 -type d | head -n1)
if [[ -z "$src_dir" ]]; then
  echo "error: failed to extract source" >&2
  exit 1
fi

echo "[psyche] provisioning from $src_dir"
env PSYCHE_SRC="$(cd "$src_dir" && pwd)" python3 "$src_dir/scripts/setup_host.py"

echo "[psyche] provisioning complete"

