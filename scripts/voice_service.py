#!/usr/bin/env python3
"""Queue speech from the ``voice`` topic using Piper TTS.

Piper is a high‑quality, lightweight neural TTS. This service reads text from
the ``voice`` topic, synthesizes speech with Piper, and plays audio via ALSA
(``aplay``).

Examples:
    Run the service::

        $ python3 voice_service.py  # doctest: +SKIP

    Publish text::

        $ ros2 topic pub --once voice std_msgs/String '{data: "hello"}'  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Default Piper voice model (downloaded during provisioning)
PIPER_VOICES_DIR = os.getenv("PIPER_VOICES_DIR", "/opt/piper/voices")
PIPER_MODEL = os.getenv("PIPER_MODEL", "en_US-lessac-high")


class VoiceNode(Node):
    """Speak queued messages via Piper TTS.

    Args:
        model: Piper voice model basename (e.g., ``"en_US-amy-medium"``).
        voices_dir: Directory containing ``.onnx`` and ``.onnx.json`` files.

    Examples:
        >>> VoiceNode(model="en_US-amy-medium")  # doctest: +SKIP
    """

    def __init__(self, model: str = PIPER_MODEL, voices_dir: str = PIPER_VOICES_DIR) -> None:
        super().__init__("voice")
        self.model = model
        self.voices_dir = voices_dir
        self.model_path = os.path.join(self.voices_dir, f"{self.model}.onnx")
        self.config_path = os.path.join(self.voices_dir, f"{self.model}.onnx.json")
        self.sample_rate = self._load_sample_rate(self.config_path)
        self._queue: queue.Queue[str] = queue.Queue()
        # Track both piper and aplay for interruption
        self._procs: list[subprocess.Popen] = []
        self.create_subscription(String, "voice", self.enqueue, 10)
        self.create_subscription(String, "voice_interrupt", self.interrupt, 10)
        threading.Thread(target=self._worker, daemon=True).start()

    @staticmethod
    def _load_sample_rate(config_path: str) -> int:
        """Return sample rate from a Piper model JSON config.

        Falls back to 22050 Hz if unavailable.
        """
        try:
            with open(config_path, "r", encoding="utf-8") as fh:
                cfg = json.load(fh)
            # Typical locations: top-level or under "audio"
            return int(cfg.get("sample_rate") or cfg.get("audio", {}).get("sample_rate") or 22050)
        except Exception:
            return 22050

    def enqueue(self, msg: String) -> None:
        """Add ``msg`` to the speech queue."""
        self._queue.put(msg.data)

    def interrupt(self, msg: String | None = None) -> None:
        """Stop current speech and clear the queue."""
        for p in self._procs:
            try:
                p.terminate()
            except Exception:
                pass
        self._procs.clear()
        with self._queue.mutex:  # type: ignore[attr-defined]
            self._queue.queue.clear()  # type: ignore[attr-defined]

    def _worker(self) -> None:
        """Continuously speak queued messages."""
        while rclpy.ok():
            text = self._queue.get()
            # Launch Piper to produce RAW PCM to stdout, then pipe into aplay
            try:
                piper = subprocess.Popen(
                    [
                        "piper",
                        "--model",
                        self.model_path,
                        "--config",
                        self.config_path,
                        "--output_raw",
                    ],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                )
            except FileNotFoundError:
                # Piper not installed; log once and skip
                self.get_logger().error("piper not found on PATH; install piper")
                continue
            aplay = subprocess.Popen(
                [
                    "aplay",
                    "-q",
                    "-r",
                    str(self.sample_rate),
                    "-f",
                    "S16_LE",
                    "-t",
                    "raw",
                    "-",
                ],
                stdin=piper.stdout,
            )
            self._procs = [aplay, piper]
            # Send text to Piper
            try:
                assert piper.stdin is not None
                piper.stdin.write((text + "\n").encode("utf-8"))
                piper.stdin.flush()
                piper.stdin.close()
            except Exception:
                pass
            # Wait for playback to finish
            aplay.wait()
            try:
                piper.terminate()
            except Exception:
                pass
            self._procs.clear()


def main() -> None:
    """Start the voice node.

    Respects ``PIPER_MODEL`` and ``PIPER_VOICES_DIR`` environment variables
    and ``--model``/``--voices-dir`` CLI arguments.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=os.getenv("PIPER_MODEL", PIPER_MODEL))
    parser.add_argument("--voices-dir", default=os.getenv("PIPER_VOICES_DIR", PIPER_VOICES_DIR))
    ns = parser.parse_args()
    rclpy.init()
    node = VoiceNode(model=ns.model, voices_dir=ns.voices_dir)
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
