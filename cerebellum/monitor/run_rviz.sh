#!/usr/bin/env bash
set -euo pipefail

COMPOSE=${COMPOSE:-docker compose -f monitor/docker/compose.yml}

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

