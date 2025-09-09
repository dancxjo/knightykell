#!/usr/bin/env python3
import os
import sys
import json
import socket

SOCK_PATH = os.environ.get("OLED_SOCK", "/run/oled/statusd.sock")


def send(header=None, *lines, ttl=None):
    msg = {}
    if header:
        msg["header"] = header
    if lines:
        msg["lines"] = list(lines)
    if ttl is not None:
        msg["ttl"] = float(ttl)
    data = json.dumps(msg).encode("utf-8")
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    s.connect(SOCK_PATH)
    s.send(data)
    s.close()


def main(argv):
    if not argv or argv[0] in ("-h", "--help"):
        print("Usage: oled_client.py [--ttl SEC] HEADER [LINE ...]")
        return 0
    ttl = None
    if argv and argv[0] == "--ttl" and len(argv) >= 3:
        try:
            ttl = float(argv[1])
            argv = argv[2:]
        except Exception:
            pass
    header = argv[0]
    lines = argv[1:]
    try:
        send(header, *lines, ttl=ttl)
        return 0
    except Exception as e:
        print(f"oled_client: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
