#!/usr/bin/env python3
"""Play random tunes on an iRobot Create via the driver's topics.

Publishes ``create_msgs/DefineSong`` to ``define_song`` and then
``create_msgs/PlaySong`` to ``play_song`` every ``period`` seconds.
This avoids touching the serial port directly and works alongside the
create_robot driver.

Examples:
    $ python3 scripts/create_singer.py --period 60  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import random
import time

import rclpy
from rclpy.node import Node
from create_msgs.msg import DefineSong, PlaySong


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


class Singer(Node):
    def __init__(self, period: float = 60.0, song_id: int = 0) -> None:
        super().__init__("create_singer")
        self._pub_def = self.create_publisher(DefineSong, "define_song", 10)
        self._pub_play = self.create_publisher(PlaySong, "play_song", 10)
        self._song_id = int(song_id) % 4
        self._period = float(period)
        self._last = 0.0
        self.create_timer(0.5, self._tick)

    def _tick(self) -> None:
        now = time.monotonic()
        if now - self._last < self._period:
            return
        self._last = now
        mel = _melody()
        notes = []
        durs = []
        for n, d in mel[:16]:
            notes.append(int(max(31, min(127, n))))
            # convert 1/64th tick d to seconds (e.g., 8 -> 0.125s)
            durs.append(max(1, int(d)) / 64.0)
        msg = DefineSong()
        msg.song = self._song_id
        msg.length = min(16, len(notes))
        msg.notes = notes[: msg.length]
        msg.durations = durs[: msg.length]
        self._pub_def.publish(msg)
        play = PlaySong()
        play.song = self._song_id
        # slight delay to ensure driver defines first
        def _later():
            self._pub_play.publish(play)
        self.create_timer(0.2, _later)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--period", type=float, default=60.0)
    p.add_argument("--song-id", type=int, default=0)
    ns = p.parse_args(argv)
    rclpy.init()
    node = Singer(period=ns.period, song_id=ns.song_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
