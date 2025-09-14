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
from std_msgs.msg import String, Bool
from std_msgs.msg import Int16MultiArray
import sounddevice as sd
import webrtcvad
import whisper

SAMPLE_RATE = 16_000
FRAME_DURATION = 30  # ms
# Number of frames per VAD window (10/20/30 ms supported)
FRAME_SAMPLES = SAMPLE_RATE * FRAME_DURATION // 1000
# Raw bytes per frame window for int16 mono
FRAME_BYTES = FRAME_SAMPLES * 2


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
        self._pub_vad = self.create_publisher(Bool, "vad", 10)
        self._pub_frames = self.create_publisher(Int16MultiArray, "audio/frames", 10)
        self._pub_utter = self.create_publisher(Int16MultiArray, "audio/utterance", 10)
        self._model = whisper.load_model(model)
        self._vad = webrtcvad.Vad(2)
        self._frames: deque[bytes] = deque()
        self._buffer = bytearray()
        # Queue of complete utterance bytes for Whisper
        self._utt_q: queue.Queue[bytes] = queue.Queue()
        # Queue of captured frames for VAD + PCM publishing (bytes per frame)
        self._frame_q: queue.Queue[bytes] = queue.Queue(maxsize=256)
        # Use a raw stream so VAD receives 16-bit PCM bytes.
        # blocksize must be specified in FRAMES, not bytes. Using bytes here
        # doubles the duration and triggers webrtcvad "Error while processing frame".
        self._stream = sd.RawInputStream(
            samplerate=SAMPLE_RATE,
            blocksize=FRAME_SAMPLES,
            channels=1,
            dtype="int16",
            callback=self._callback,
        )
        self._stream.start()
        # Background workers: frame/VAD processing and utterance ASR
        threading.Thread(target=self._frame_worker, daemon=True).start()
        threading.Thread(target=self._utt_worker, daemon=True).start()

    def _callback(self, indata: bytes, frames: int, time, status) -> None:
        """Audio callback: enqueue frame quickly; avoid heavy work here."""
        if status:
            try:
                self.get_logger().warning(str(status))
            except Exception:
                pass
        try:
            self._frame_q.put_nowait(bytes(indata))
        except queue.Full:
            # Drop frame if overloaded
            pass

    def _frame_worker(self) -> None:
        """Process audio frames: VAD, publish PCM/vad, segment utterances."""
        while rclpy.ok():
            frame = self._frame_q.get()
            try:
                # VAD decision
                is_speech = self._vad.is_speech(frame, SAMPLE_RATE)
            except Exception:
                is_speech = False
            # Publish frame PCM and VAD
            try:
                arr = np.frombuffer(frame, np.int16)
                self._pub_frames.publish(Int16MultiArray(data=arr.tolist()))
                self._pub_vad.publish(Bool(data=bool(is_speech)))
            except Exception:
                pass
            # Segment utterances
            if is_speech:
                self._buffer.extend(frame)
                self._frames.clear()
            else:
                self._frames.append(frame)
                if len(self._frames) * FRAME_DURATION > 800:
                    if self._buffer:
                        # Emit full utterance PCM for accurate downstream ASR
                        try:
                            utt_arr = np.frombuffer(self._buffer, np.int16)
                            self._pub_utter.publish(Int16MultiArray(data=utt_arr.tolist()))
                        except Exception:
                            pass
                        try:
                            self._utt_q.put_nowait(bytes(self._buffer))
                        except Exception:
                            pass
                        self._buffer.clear()
                    self._frames.clear()

    def _utt_worker(self) -> None:
        """Transcribe queued utterances and publish text."""
        while rclpy.ok():
            audio = self._utt_q.get()
            try:
                arr = np.frombuffer(audio, np.int16).astype(np.float32) / 32768.0
                result = self._model.transcribe(arr, fp16=False)
                text = (result.get("text", "") or "").strip()
                if not text:
                    continue
                self._pub.publish(String(data=text))
            except Exception as e:
                try:
                    self.get_logger().warning(f"asr error: {e}")
                except Exception:
                    pass


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
