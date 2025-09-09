#!/usr/bin/env bash
set -euo pipefail

# Usage: compose_cmd.sh -f <compose-file>
# Prints a compose command (docker compose|docker-compose|podman compose) with -f <abs_path>

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

if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
  echo "docker compose -f ${abs_file}"
elif command -v docker-compose >/dev/null 2>&1; then
  echo "docker-compose -f ${abs_file}"
elif command -v podman >/dev/null 2>&1; then
  echo "podman compose -f ${abs_file}"
else
  echo ""
fi
