#!/usr/bin/env bash
set -euo pipefail

# Wrapper to run firstboot inside a detached GNU screen session so it can be
# attached over SSH: screen -r ck-firstboot

SESSION_NAME=${SESSION_NAME:-ck-firstboot}
SCRIPT=${SCRIPT:-/opt/cerebellum/firstboot.sh}
LOG=${LOG:-/var/log/cerebellum-firstboot.log}

mkdir -p /run/screen || true

echo "[firstboot-wrapper] starting screen session '${SESSION_NAME}' for ${SCRIPT}" | tee -a "$LOG" || true

# Start the script in a detached screen session; run under bash -lc to ensure env/profile
if screen -list | grep -q "\.${SESSION_NAME}\b"; then
  echo "[firstboot-wrapper] session already exists; not starting another" | tee -a "$LOG" || true
else
  screen -DmS "$SESSION_NAME" bash -lc "$SCRIPT" || true
fi

# Wait until the script exits (session disappears)
for i in $(seq 1 720); do # up to 2 hours @10s
  if screen -list | grep -q "\.${SESSION_NAME}\b"; then
    sleep 10
  else
    echo "[firstboot-wrapper] session '${SESSION_NAME}' ended" | tee -a "$LOG" || true
    exit 0
  fi
done

echo "[firstboot-wrapper] timeout waiting for session to finish; leaving it running" | tee -a "$LOG" || true
exit 0

