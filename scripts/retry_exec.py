#!/usr/bin/env python3
"""Run a command in a loop with exponential backoff on exit.

Useful for wrapping fragile hardware drivers to keep retrying connections
without relying solely on systemd restarts.

Examples:
    Retry a command with a backoff between 1 and 60 seconds::

        $ python3 scripts/retry_exec.py --min 1 --max 60 -- echo hello  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import random
import subprocess
import sys
import time


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--min", type=float, default=1.0, dest="min_delay",
                        help="minimum backoff seconds")
    parser.add_argument("--max", type=float, default=60.0, dest="max_delay",
                        help="maximum backoff seconds")
    parser.add_argument("--factor", type=float, default=2.0,
                        help="exponential backoff factor")
    parser.add_argument("--jitter", type=float, default=0.2,
                        help="random jitter fraction (0..1)")
    parser.add_argument("--", dest="sep", action="store_true")
    parser.add_argument("cmd", nargs=argparse.REMAINDER, help="command to run")
    ns = parser.parse_args(argv)
    if not ns.cmd:
        print("retry_exec: no command provided", file=sys.stderr)
        return 2
    delay = float(ns.min_delay)
    while True:
        try:
            rc = subprocess.call(ns.cmd)
        except KeyboardInterrupt:
            return 130
        if rc == 0:
            # If command exited cleanly, reset delay before next run
            delay = ns.min_delay
        # Sleep with jitter
        jitter = 1.0 + random.uniform(-ns.jitter, ns.jitter)
        time.sleep(max(0.1, min(ns.max_delay, delay)) * jitter)
        delay = min(ns.max_delay, max(ns.min_delay, delay * ns.factor))


if __name__ == "__main__":
    raise SystemExit(main())

