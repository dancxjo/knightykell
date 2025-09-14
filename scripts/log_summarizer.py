#!/usr/bin/env python3
"""Summarize buffered log lines with a local LLM and publish to voice.

This node subscribes to the ``logs`` topic, buffers messages for a short
interval, prompts a local LLM to produce a concise, plain-English summary,
and publishes the result to the ``voice`` topic for the voice service.

Model backends (auto-detected in this order):
- Ollama (``ollama run $OLLAMA_MODEL``; defaults to ``gpt-oss:20b``)
- llama-cpp-python (``LLAMA_MODEL_PATH`` must point to a GGUF file)
- Fallback (simple heuristic summary when no LLM is available)

Environment variables:
- ``OLLAMA_MODEL``: Ollama model tag (default: ``gpt-oss:20b``)
- ``LLAMA_MODEL_PATH``: Path to a GGUF file for llama-cpp
- ``SUMMARY_INTERVAL``: Seconds between summaries (default: 20)
- ``SUMMARY_MAX_LINES``: Max buffered lines to include (default: 200)

Examples:
    Run the service::

        $ python3 log_summarizer.py  # doctest: +SKIP

    With Ollama installed::

        $ export OLLAMA_MODEL=gpt-oss:20b
        $ python3 log_summarizer.py  # doctest: +SKIP

    With llama-cpp-python::

        $ export LLAMA_MODEL_PATH=/models/TinyLlama-1.1B-Chat.Q4_K_M.gguf
        $ python3 log_summarizer.py  # doctest: +SKIP
"""
from __future__ import annotations

import os
import queue
import shlex
import shutil
import subprocess
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _has_cmd(name: str) -> bool:
    """Return True if ``name`` is on PATH.

    Examples:
        >>> isinstance(_has_cmd('bash'), bool)
        True
    """
    return shutil.which(name) is not None


class _LLMBackend:
    """Abstract LLM backend interface."""

    def summarize(self, text: str) -> str:
        raise NotImplementedError


class _OllamaBackend(_LLMBackend):
    """Use ``ollama run`` to generate a summary.

    Args:
        model: Ollama model tag (e.g., ``"gpt-oss:20b"``).
    """

    def __init__(self, model: str = "gpt-oss:20b") -> None:
        self.model = model

    def summarize(self, text: str) -> str:
        prompt = (
            "Summarize the following system logs into a concise, plain-English "
            "update suitable for speaking out loud. Use 1-2 sentences, "
            "highlight notable events, avoid timestamps and noise.\n\n" + text
        )
        # Use non-interactive generation to get a single response
        try:
            proc = subprocess.run(
                ["ollama", "run", self.model, "-p", prompt],
                check=True,
                text=True,
                capture_output=True,
            )
            out = proc.stdout.strip()
            return out or "No significant log events."
        except Exception as e:
            return f"Log update: unable to generate summary via Ollama ({e})."


class _LlamaCppBackend(_LLMBackend):
    """Use ``llama-cpp-python`` with a local GGUF model.

    Args:
        model_path: Path to GGUF model file.
    """

    def __init__(self, model_path: str) -> None:
        from llama_cpp import Llama  # type: ignore

        # Keep context small to reduce memory by default
        self.llm = Llama(model_path=model_path, n_ctx=2048)

    def summarize(self, text: str) -> str:
        prompt = (
            "You are a helpful system that produces concise spoken summaries. "
            "Summarize the following logs in 1-2 sentences highlighting only "
            "notable events. Avoid timestamps, PIDs, and noisy details.\n\n" + text
        )
        try:
            out = self.llm.create_completion(prompt=prompt, temperature=0.2, max_tokens=128)
            text = out.get("choices", [{}])[0].get("text", "").strip()
            return text or "No significant log events."
        except Exception as e:
            return f"Log update: unable to generate summary via llama.cpp ({e})."


class _FallbackBackend(_LLMBackend):
    """Fallback summarizer when no LLM is available.

    Produces a compact heuristic summary by extracting unique keywords.
    """

    def summarize(self, text: str) -> str:
        # Extract a few tokens as a rough summary
        words = [w.strip(";,:.()[]{}<>") for w in text.split() if len(w) > 3]
        uniq = []
        seen = set()
        for w in words:
            lw = w.lower()
            if lw not in seen and lw.isalpha():
                uniq.append(w)
                seen.add(lw)
            if len(uniq) >= 12:
                break
        if not uniq:
            return "No significant log events."
        return "Log update: " + ", ".join(uniq[:12]) + "."


def _select_backend() -> _LLMBackend:
    """Choose the best available LLM backend based on environment.

    Preference order: Ollama -> llama-cpp -> fallback.

    Examples:
        >>> isinstance(_select_backend(), _LLMBackend)
        True
    """
    model = os.getenv("OLLAMA_MODEL", "gpt-oss:20b")
    if _has_cmd("ollama"):
        return _OllamaBackend(model=model)
    mpath = os.getenv("LLAMA_MODEL_PATH")
    if mpath and os.path.exists(mpath):
        try:
            return _LlamaCppBackend(mpath)
        except Exception:
            pass
    return _FallbackBackend()


class LogSummarizer(Node):
    """Summarize buffered log lines and publish to the ``voice`` topic.

    Buffers messages from ``logs`` and emits summaries every
    ``SUMMARY_INTERVAL`` seconds.

    Args:
        interval: Seconds between summaries.
        max_lines: Maximum lines to include per summary window.

    Examples:
        >>> LogSummarizer(interval=10, max_lines=50)  # doctest: +SKIP
    """

    def __init__(self, interval: float = 20.0, max_lines: int = 200) -> None:
        super().__init__("log_summarizer")
        self._pub = self.create_publisher(String, "voice", 10)
        self._buf: list[str] = []
        self._lock = threading.Lock()
        self._backend = _select_backend()
        self.create_subscription(String, "logs", self._on_log, 50)
        self._interval = interval
        self._max_lines = max_lines
        threading.Thread(target=self._worker, daemon=True).start()

    def _on_log(self, msg: String) -> None:
        """Append a log line to the buffer."""
        with self._lock:
            if len(self._buf) < self._max_lines:
                self._buf.append(msg.data)
            else:
                # Drop oldest to keep window bounded
                self._buf.pop(0)
                self._buf.append(msg.data)

    def _drain(self) -> Optional[str]:
        """Drain buffered lines and return a joined block or ``None``."""
        with self._lock:
            if not self._buf:
                return None
            lines = self._buf[:]
            self._buf.clear()
        # De-duplicate adjacent lines, trim whitespace
        cleaned: list[str] = []
        prev: Optional[str] = None
        for ln in lines:
            t = ln.strip()
            if t and t != prev:
                cleaned.append(t)
            prev = t
        if not cleaned:
            return None
        # Keep last N lines if over budget (bias to recent events)
        if len(cleaned) > self._max_lines:
            cleaned = cleaned[-self._max_lines :]
        return "\n".join(cleaned)

    def _worker(self) -> None:
        """Periodically summarize buffered logs and publish to ``voice``."""
        while rclpy.ok():
            time.sleep(self._interval)
            block = self._drain()
            if not block:
                continue
            summary = self._backend.summarize(block).strip()
            if not summary:
                continue
            msg = String()
            msg.data = summary
            self._pub.publish(msg)


def main() -> None:
    """Start the log summarizer node.

    Respects ``SUMMARY_INTERVAL`` and ``SUMMARY_MAX_LINES`` environment variables.
    """
    interval = float(os.getenv("SUMMARY_INTERVAL", "20"))
    max_lines = int(os.getenv("SUMMARY_MAX_LINES", "200"))
    rclpy.init()
    node = LogSummarizer(interval=interval, max_lines=max_lines)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

