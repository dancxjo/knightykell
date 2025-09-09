#!/usr/bin/env bash
set -euo pipefail

COMPOSE=${COMPOSE:-docker compose -f monitor/docker/compose.yml}

if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

if [ -n "${WAYLAND_DISPLAY:-}" ] && [ -n "${XDG_RUNTIME_DIR:-}" ]; then
  echo "[rqt] Using Wayland: $WAYLAND_DISPLAY"
  $COMPOSE run --rm -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR monitor rqt "$@"
else
  echo "[rqt] Using X11: $DISPLAY"
  $COMPOSE run --rm -e DISPLAY monitor rqt "$@"
fi

