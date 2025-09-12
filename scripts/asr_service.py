#!/usr/bin/env python3
"""Continuously publish transcribed microphone audio via Whisper.

Examples:
    Run the service::

        $ python3 asr_service.py  # doctest: +SKIP

    Echo transcripts::

        $ ros2 topic echo asr  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import queue
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import webrtcvad
import whisper

SAMPLE_RATE = 16_000
FRAME_DURATION = 30  # ms
FRAME_BYTES = int(SAMPLE_RATE * FRAME_DURATION / 1000) * 2  # 16-bit mono


class AsrNode(Node):
    """Transcribe speech from the default microphone.

    Args:
        model: Whisper model name (e.g. ``"tiny"``, ``"base"``).

    Examples:
        >>> AsrNode(model="tiny")  # doctest: +SKIP
    """

    def __init__(self, model: str = "tiny") -> None:
        super().__init__("asr")
        self._pub = self.create_publisher(String, "asr", 10)
        self._model = whisper.load_model(model)
        self._vad = webrtcvad.Vad(2)
        self._frames: deque[bytes] = deque()
        self._buffer = bytearray()
        self._queue: queue.Queue[bytes] = queue.Queue()
        self._stream = sd.RawInputStream(
            samplerate=SAMPLE_RATE,
            blocksize=FRAME_BYTES,
            channels=1,
            dtype="int16",
            callback=self._callback,
        )
        self._stream.start()
        threading.Thread(target=self._worker, daemon=True).start()

    def _callback(self, indata: bytes, frames: int, time, status) -> None:
        """Collect voiced frames based on VAD."""
        if status:
            self.get_logger().warning(str(status))
        if self._vad.is_speech(indata, SAMPLE_RATE):
            self._buffer.extend(indata)
            self._frames.clear()
        else:
            self._frames.append(indata)
            if len(self._frames) * FRAME_DURATION > 800:
                if self._buffer:
                    self._queue.put(bytes(self._buffer))
                    self._buffer.clear()
                self._frames.clear()

    def _worker(self) -> None:
        """Process queued audio chunks and publish transcripts."""
        while rclpy.ok():
            audio = self._queue.get()
            arr = np.frombuffer(audio, np.int16).astype(np.float32) / 32768.0
            result = self._model.transcribe(arr, fp16=False)
            msg = String()
            msg.data = result.get("text", "").strip()
            if msg.data:
                self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Start the ASR node."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="tiny", help="Whisper model size")
    ns = parser.parse_args(args=args)
    rclpy.init()
    node = AsrNode(model=ns.model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
