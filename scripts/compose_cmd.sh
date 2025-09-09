#!/usr/bin/env bash
set -euo pipefail

# Usage: compose_cmd.sh -f <compose-file>
# Prints a compose command (docker compose|docker-compose|podman compose) with -f <abs_path>
# Enhancements:
# - If SSH_AUTH_SOCK is unset or not a socket, auto-strip any SSH_AUTH_SOCK mounts from a temp override

COMPOSE_FILE=""
while getopts ":f:" opt; do
  case "$opt" in
    f) COMPOSE_FILE="$OPTARG" ;;
    *) ;;
  esac
done

if [[ -z "${COMPOSE_FILE}" ]]; then
  echo ""; exit 0
fi

# Resolve to absolute path based on repo root or current dir
if git_root=$(git rev-parse --show-toplevel 2>/dev/null); then
  base_dir="$git_root"
else
  base_dir="$(pwd)"
fi

# If COMPOSE_FILE is already absolute, keep it; else join with base_dir
case "$COMPOSE_FILE" in
  /*) abs_file="$COMPOSE_FILE" ;;
  *) abs_file="${base_dir}/${COMPOSE_FILE}" ;;
esac
abs_file="$(cd "$(dirname "$abs_file")" && pwd)/$(basename "$abs_file")"

# If SSH agent is not available, create a sanitized override with mount lines removed
use_file="$abs_file"
if [[ -z "${SSH_AUTH_SOCK:-}" || ! -S "${SSH_AUTH_SOCK:-/nonexistent}" ]]; then
  tmpdir="${XDG_RUNTIME_DIR:-/tmp}"
  override_file="${tmpdir}/$(basename "$abs_file" .yml).noagent.$$.yml"
  # Remove lines containing SSH_AUTH_SOCK (in volumes or environment)
  # Keep YAML valid; removing a single list item line is safe.
  grep -v 'SSH_AUTH_SOCK' "$abs_file" > "$override_file" || cp "$abs_file" "$override_file"
  use_file="$override_file"
fi

if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
  echo "docker compose -f ${use_file}"
elif command -v docker-compose >/dev/null 2>&1; then
  echo "docker-compose -f ${use_file}"
elif command -v podman >/dev/null 2>&1; then
  echo "podman compose -f ${use_file}"
else
  echo ""
fi
