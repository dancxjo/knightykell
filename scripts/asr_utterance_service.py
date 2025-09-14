#!/usr/bin/env python3
"""High-accuracy ASR on utterance segments.

Subscribes to ``audio/utterance`` (Int16MultiArray PCM16 mono @ 16kHz) and
performs a slower, more accurate Whisper transcription (e.g., ``base`` or
``small``). Publishes text to ``asr_long``.

Examples:
    Run the service::

        $ python3 scripts/asr_utterance_service.py --model base  # doctest: +SKIP

    Echo transcripts::

        $ ros2 topic echo asr_long  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String
import whisper

SAMPLE_RATE = 16_000


class UtteranceASRNode(Node):
    """Transcribe utterance-level PCM arrays with higher accuracy.

    Args:
        model: Whisper model name (e.g. ``"base"``, ``"small"``).

    Examples:
        >>> UtteranceASRNode(model="base")  # doctest: +SKIP
    """

    def __init__(self, model: str = "base") -> None:
        super().__init__("asr_long")
        self._pub = self.create_publisher(String, "asr_long", 10)
        self._model = whisper.load_model(model)
        self.create_subscription(Int16MultiArray, "audio/utterance", self._on_utterance, 10)

    def _on_utterance(self, msg: Int16MultiArray) -> None:
        try:
            arr16 = np.array(msg.data, dtype=np.int16)
            audio = arr16.astype(np.float32) / 32768.0
            result = self._model.transcribe(audio, fp16=False)
            text = (result.get("text", "") or "").strip()
            if not text:
                return
            out = String()
            out.data = text
            self._pub.publish(out)
        except Exception as e:
            self.get_logger().warning(f"asr_long error: {e}")


def main(args: List[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="base", help="Whisper model (e.g., base, small)")
    ns = parser.parse_args(args=args)
    rclpy.init()
    node = UtteranceASRNode(model=ns.model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

