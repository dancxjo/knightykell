#!/usr/bin/env python3
"""Chat service that maintains a conversation and reacts to ASR.

This node keeps a running list of messages in the format
``{"role": "user|assistant|system", "content": str}``. It listens to the
``asr`` topic for user utterances; when a new user message arrives it is added
to the conversation and sent to a local LLM for a chat completion. The
assistant's reply is then published to the ``voice`` topic for speaking and to
the ``chat`` topic for display/consumption by other nodes.

Backends (auto-detected in this order):
- Ollama (``ollama run $OLLAMA_MODEL``) — conversation is flattened into a
  single prompt
- llama-cpp-python (``LLAMA_MODEL_PATH`` must point to a GGUF model)

Environment variables:
- ``CHAT_PROMPT``: Optional system prompt added at conversation start
- ``OLLAMA_MODEL``: Optional Ollama model tag (e.g., ``llama3.2:1b-instruct``)
- ``LLAMA_MODEL_PATH``: GGUF model path for llama.cpp backend
- ``LLAMA_CTX``/``LLAMA_THREADS``: Optional llama.cpp params

Examples:
    Run the service::

        $ python3 chat_service.py  # doctest: +SKIP

    Configure a system prompt via env::

        $ export CHAT_PROMPT="You are a concise robot assistant."
        $ python3 chat_service.py  # doctest: +SKIP
"""
from __future__ import annotations

import os
import shlex
import shutil
import subprocess
from typing import List, TypedDict
import pathlib

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _has_cmd(name: str) -> bool:
    return shutil.which(name) is not None


class Message(TypedDict):
    role: str
    content: str


class _ChatBackend:
    def complete(self, messages: List[Message]) -> str:
        raise NotImplementedError


class _OllamaBackend(_ChatBackend):
    def __init__(self, model: str) -> None:
        self.model = model

    def complete(self, messages: List[Message]) -> str:
        # Flatten messages into a simple instruct-style prompt
        parts: list[str] = []
        for m in messages:
            role = m.get("role", "user")
            content = m.get("content", "")
            if not content:
                continue
            if role == "system":
                parts.append(f"System: {content}")
            elif role == "assistant":
                parts.append(f"Assistant: {content}")
            else:
                parts.append(f"User: {content}")
        parts.append("Assistant:")
        prompt = "\n\n".join(parts)
        try:
            timeout = float(os.getenv("CHAT_OLLAMA_TIMEOUT", "30"))
            proc = subprocess.run(
                ["ollama", "run", self.model, "-p", prompt],
                check=True,
                text=True,
                capture_output=True,
                timeout=timeout,
            )
            return proc.stdout.strip()
        except Exception as e:
            return f"(ollama error: {e})"


class _LlamaCppBackend(_ChatBackend):
    def __init__(self, model_path: str) -> None:
        from llama_cpp import Llama  # type: ignore

        n_ctx = int(os.getenv("LLAMA_CTX", "2048"))
        n_threads = int(os.getenv("LLAMA_THREADS", str(os.cpu_count() or 2)))
        self.llm = Llama(model_path=model_path, n_ctx=n_ctx, n_threads=n_threads)

    def complete(self, messages: List[Message]) -> str:
        # Prefer chat completion API when available
        try:
            out = self.llm.create_chat_completion(messages=messages)
            content = out.get("choices", [{}])[0].get("message", {}).get("content", "")
            return str(content).strip()
        except Exception:
            # Fallback to simple completion using the most recent user prompt
            last_user = next((m["content"] for m in reversed(messages) if m["role"] == "user"), "")
            prompt = (
                "You are a helpful assistant. Respond to the user request succinctly.\n\n"
                + last_user
            )
            out = self.llm.create_completion(prompt=prompt, max_tokens=256)
            text = out.get("choices", [{}])[0].get("text", "")
            return str(text).strip()


def _select_backend() -> _ChatBackend | None:
    # Prefer Ollama if installed; default to Llama 3.2 small
    model = os.getenv("OLLAMA_MODEL", "llama3.2:1b-instruct")
    if _has_cmd("ollama"):
        return _OllamaBackend(model=model)
    mp = os.getenv("LLAMA_MODEL_PATH")
    if mp and os.path.exists(mp):
        try:
            return _LlamaCppBackend(model_path=mp)
        except Exception:
            return None
    return None


class ChatNode(Node):
    """Maintain conversation state and generate replies to ASR utterances.

    Examples:
        >>> ChatNode()  # doctest: +SKIP
    """

    def __init__(self) -> None:
        super().__init__("chat")
        self._messages: List[Message] = []
        self._pending: List[str] = []
        self._last_error: str | None = None
        prompt = os.getenv("CHAT_PROMPT")
        if prompt:
            self._messages.append({"role": "system", "content": prompt})
        self._backend = _select_backend()
        # If backend is unavailable, try to fetch a local GGUF and reselect
        if not self._backend:
            try:
                self._ensure_llama_model()
                self._backend = _select_backend()
            except Exception:
                pass
        if isinstance(self._backend, _OllamaBackend):
            self.get_logger().info(f"chat: backend=ollama model={self._backend.model}")
        elif isinstance(self._backend, _LlamaCppBackend):
            self.get_logger().info("chat: backend=llama.cpp (LLAMA_MODEL_PATH)")
        else:
            self.get_logger().warning("chat: backend unavailable; set OLLAMA_MODEL or LLAMA_MODEL_PATH")
        self._pub_voice = self.create_publisher(String, "voice", 10)
        self._pub_chat = self.create_publisher(String, "chat", 10)
        self._pub_status = self.create_publisher(String, "status/chat", 10)
        self.create_subscription(String, "asr", self._on_asr, 10)
        self.create_subscription(String, "voice_done", self._on_voice_done, 10)
        # Periodic status heartbeat
        try:
            interval = float(os.getenv("CHAT_STATUS_INTERVAL", "10"))
        except Exception:
            interval = 10.0
        self.create_timer(interval, self._publish_status)

    def _ensure_llama_model(self) -> None:
        """Ensure a llama.cpp GGUF is present if using local backend.

        Attempts to download a default Llama 3.2 1B Instruct Q4_K_M model when
        ``LLAMA_MODEL_PATH`` is unset or points to a missing file. Honors
        ``LLAMA_MODELS_DIR`` for destination. If ``OLLAMA_MODEL`` is set and
        Ollama is available, this method does nothing (Ollama handles pulls).
        """
        # If Ollama is available, skip (it will pull as needed)
        if _has_cmd("ollama"):
            return
        mp = os.getenv("LLAMA_MODEL_PATH")
        if mp and os.path.exists(mp):
            return
        # Choose destination directory
        models_dir = pathlib.Path(os.getenv("LLAMA_MODELS_DIR", "/opt/llama/models"))
        try:
            models_dir.mkdir(parents=True, exist_ok=True)
        except Exception:
            return
        # Determine URL (allow override via LLAMA_GGUF_URL)
        url = os.getenv(
            "LLAMA_GGUF_URL",
            "https://huggingface.co/TheBloke/Meta-Llama-3.2-1B-Instruct-GGUF/resolve/main/Meta-Llama-3.2-1B-Instruct.Q4_K_M.gguf?download=true",
        )
        fname = url.split("/")[-1].split("?")[0] or "model.gguf"
        out = models_dir / fname
        if not out.exists():
            try:
                subprocess.run(["curl", "-fsSL", "-o", str(out), url], check=True)
            except Exception as e:
                self._last_error = f"download failed: {e}"
                return
        os.environ["LLAMA_MODEL_PATH"] = str(out)

    def _on_asr(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text:
            return
        # Add user message and attempt a reply
        self._messages.append({"role": "user", "content": text})
        if not self._backend:
            self.get_logger().warning("chat: backend unavailable; skipping response")
            self._last_error = "backend unavailable"
            return
        try:
            reply = self._backend.complete(self._messages)
        except Exception as e:
            self._last_error = str(e)
            reply = f"(llm error: {e})"
        reply = (reply or "").strip()
        if not reply:
            return
        # Publish to voice and chat topics, but defer logging until voice completes
        m = String()
        m.data = reply
        self.get_logger().info("chat: sending reply to voice and chat topics")
        self._pub_voice.publish(m)
        self._pub_chat.publish(m)
        # Track pending assistant message until announcement completes
        self._pending.append(reply)

    def _on_voice_done(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text:
            return
        # Find matching pending reply (first occurrence)
        try:
            idx = self._pending.index(text)
        except ValueError:
            idx = -1
        if idx != -1:
            # Commit to conversation log in order of completion
            self._messages.append({"role": "assistant", "content": text})
            del self._pending[idx]

    def _publish_status(self) -> None:
        """Publish a lightweight status line to ``status/chat``.

        Includes backend type, model hint (if any), count of pending
        voice announcements, and the last error message (if any).
        """
        backend = (
            "ollama" if isinstance(self._backend, _OllamaBackend) else
            "llama.cpp" if isinstance(self._backend, _LlamaCppBackend) else
            "unavailable"
        )
        model = ""
        if isinstance(self._backend, _OllamaBackend):
            model = getattr(self._backend, "model", "")
        msg = String()
        err = self._last_error or ""
        msg.data = f"backend={backend} model={model} pending={len(self._pending)} last_error={err}"
        self._pub_status.publish(msg)


def main() -> None:
    """Start the chat node."""
    rclpy.init()
    node = ChatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
