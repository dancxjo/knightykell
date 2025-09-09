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

CMD=$(choose_compose) || { echo "[compose] WARN: no docker compose found" >&2; exit 0; }
$CMD -f compose.yml down || true

