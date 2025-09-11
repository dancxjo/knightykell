#!/usr/bin/env bash
set -euo pipefail
LOG=/var/log/banks-firstboot.log
exec >>"$LOG" 2>&1
echo "[firstboot] $(date -Is) starting"
python3 /opt/banks/setup_host.py || true
systemctl disable banks-firstboot.service || true
echo "[firstboot] done at $(date -Is)"
