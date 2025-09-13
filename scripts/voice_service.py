#!/usr/bin/env python3
"""Queue speech from the ``voice`` topic using ``espeak-ng``.

Examples:
    Run the service::

        $ python3 voice_service.py  # doctest: +SKIP

    Publish text::

        $ ros2 topic pub --once voice std_msgs/String '{data: "hello"}'  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import os
import queue
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

VOICE = "mb-en1"


class VoiceNode(Node):
    """Speak queued messages via ``espeak-ng``.

    Args:
        voice: espeak-ng voice identifier (e.g., ``"mb-en1"``).

    Examples:
        >>> VoiceNode(voice="mb-en1")  # doctest: +SKIP
    """

    def __init__(self, voice: str = VOICE) -> None:
        super().__init__("voice")
        self.voice = voice
        self._queue: queue.Queue[str] = queue.Queue()
        self._proc: subprocess.Popen | None = None
        self.create_subscription(String, "voice", self.enqueue, 10)
        self.create_subscription(String, "voice_interrupt", self.interrupt, 10)
        threading.Thread(target=self._worker, daemon=True).start()

    def enqueue(self, msg: String) -> None:
        """Add ``msg`` to the speech queue."""
        self._queue.put(msg.data)

    def interrupt(self, msg: String | None = None) -> None:
        """Stop current speech and clear the queue."""
        if self._proc:
            self._proc.terminate()
        with self._queue.mutex:  # type: ignore[attr-defined]
            self._queue.queue.clear()  # type: ignore[attr-defined]

    def _worker(self) -> None:
        """Continuously speak queued messages."""
        while rclpy.ok():
            text = self._queue.get()
            self._proc = subprocess.Popen(["espeak-ng", "-v", self.voice, text])
            self._proc.wait()
            self._proc = None


def main() -> None:
    """Start the voice node.

    Respects ``VOICE`` environment variable and ``--voice`` CLI argument.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--voice", default=os.getenv("VOICE", VOICE))
    ns = parser.parse_args()
    rclpy.init()
    node = VoiceNode(voice=ns.voice)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.interrupt()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
