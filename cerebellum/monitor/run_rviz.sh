#!/usr/bin/env bash
set -euo pipefail

choose_compose() {
  if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    echo "docker compose -f monitor/docker/compose.yml"
  elif command -v docker-compose >/dev/null 2>&1; then
    echo "docker-compose -f monitor/docker/compose.yml"
  elif command -v podman >/dev/null 2>&1; then
    echo "podman compose -f monitor/docker/compose.yml"
  else
    echo ""
  fi
}

COMPOSE=${COMPOSE:-$(choose_compose)}
if [ -z "$COMPOSE" ]; then
  echo "[rviz] ERROR: No compose tool found. Install Docker (compose) or Podman." >&2
  exit 127
fi

# Allow X clients (you can restrict later with xhost -local:)
if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

# Prefer Wayland if available
if [ -n "${WAYLAND_DISPLAY:-}" ] && [ -n "${XDG_RUNTIME_DIR:-}" ]; then
  echo "[rviz] Using Wayland: $WAYLAND_DISPLAY"
  $COMPOSE run --rm -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR monitor rviz2 "$@"
else
  echo "[rviz] Using X11: $DISPLAY"
  $COMPOSE run --rm -e DISPLAY monitor rviz2 "$@"
fi
