#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

choose_compose() {
  if docker compose version >/dev/null 2>&1; then
    echo "docker compose"
  elif command -v docker-compose >/dev/null 2>&1; then
    echo "docker-compose"
  else
    echo ""; return 1
  fi
}

CMD=$(choose_compose) || { echo "[compose] ERROR: no docker compose found" >&2; exit 1; }

# Prefer not to build on-device if image exists
if docker image inspect Knightykell/cerebellum:latest >/dev/null 2>&1; then
  $CMD -f compose.yml up -d --no-build
else
  $CMD -f compose.yml up -d --build
fi

