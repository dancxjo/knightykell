#!/usr/bin/env python3
"""Play random tunes on an iRobot Create (1) via OI song banks.

This attempts to open the serial ``port`` (default ``/dev/ttyUSB0``) at
``baud`` (default 57600), program a short random melody into song bank 0,
and play it. It repeats roughly once a minute. If the serial port is not
available, it quietly waits and retries.

It opens the port only long enough to send the song, minimizing conflicts
with other processes. If another process (e.g., a ROS 2 driver) has the
port open, this will simply skip that cycle.

Examples:
    $ python3 scripts/create_singer.py --port /dev/ttyUSB0 --period 60  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import os
import random
import time
import fcntl
import termios
import tty


def _open_serial(path: str, baud: int = 57600):
    # Open serial port non-blocking
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    # Clear O_NONBLOCK after open
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    # input flags
    attrs[0] = termios.IGNPAR
    # output flags
    attrs[1] = 0
    # control flags: 8N1, enable receiver, local mode
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    # local flags
    attrs[3] = 0
    # baud
    speed_map = {
        57600: termios.B57600,
        115200: termios.B115200,
        19200: termios.B19200,
        38400: termios.B38400,
    }
    speed = speed_map.get(int(baud), termios.B57600)
    termios.cfsetispeed(attrs, speed)
    termios.cfsetospeed(attrs, speed)
    # read timeout
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def _write(fd: int, data: bytes) -> None:
    os.write(fd, data)
    os.fsync(fd)


def _oi(bytes_list):
    return bytes(bytes_list)


def _melody() -> list[tuple[int, int]]:
    # Random small melodies from a pentatonic scale
    scales = [
        [60, 62, 65, 67, 69, 72],  # C major-ish
        [57, 60, 62, 64, 67, 69],  # A minor pentatonic
    ]
    notes = random.choice(scales)
    length = random.randint(6, 12)
    out: list[tuple[int, int]] = []
    for _ in range(length):
        n = random.choice(notes)
        d = random.choice([8, 8, 8, 12, 16])  # ticks of 1/64s -> 125-250ms
        out.append((n, d))
    return out


def _program_and_play(fd: int, song_id: int = 0) -> None:
    # Start (128), Safe (131)
    _write(fd, _oi([128]))
    time.sleep(0.05)
    _write(fd, _oi([131]))
    time.sleep(0.05)
    # Build song bytes: [140, id, length, n1, d1, n2, d2, ...]
    melody = _melody()
    pairs = []
    for n, d in melody[:16]:  # max 16 notes per song
        # Clamp to OI ranges
        n = max(31, min(127, int(n)))
        d = max(1, min(255, int(d)))
        pairs.extend([n, d])
    song = [140, song_id, len(pairs) // 2] + pairs
    _write(fd, _oi(song))
    time.sleep(0.02)
    # Play song (141, id)
    _write(fd, _oi([141, song_id]))


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=os.getenv("CREATE_PORT", "/dev/ttyUSB0"))
    p.add_argument("--baud", type=int, default=int(os.getenv("CREATE_BAUD", "57600")))
    p.add_argument("--period", type=float, default=60.0, help="seconds between songs")
    ns = p.parse_args(argv)
    last = 0.0
    while True:
        now = time.monotonic()
        if now - last >= ns.period:
            last = now
            try:
                fd = _open_serial(ns.port, ns.baud)
            except Exception:
                fd = None
            if fd is not None:
                try:
                    _program_and_play(fd, song_id=0)
                except Exception:
                    pass
                finally:
                    try:
                        os.close(fd)
                    except Exception:
                        pass
        time.sleep(0.5)


if __name__ == "__main__":
    raise SystemExit(main())

