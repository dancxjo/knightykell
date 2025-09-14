#!/usr/bin/env python3
"""Fetch Whisper and Llama models into the standard assets layout.

Defaults to the PSYCHE assets seed layout so results can be baked into an
image and later moved to the NVMe mount on first boot.

Destinations (override via env/flags):
- Llama: ``$LLAMA_MODELS_DIR`` or ``/opt/psyche/assets_seed/models/llama``
- Whisper cache: ``$XDG_CACHE_HOME`` or ``/opt/psyche/assets_seed/cache``

Examples:
    Fetch defaults (TinyLlama GGUF + Whisper tiny)::

        $ python3 fetch_models.py --defaults  # doctest: +SKIP

    Fetch specific models::

        $ python3 fetch_models.py \
            --llama tinyllama-q4_k_m \
            --whisper tiny base  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import os
import pathlib
import subprocess
from typing import Iterable


KNOWN_LLAMA: dict[str, str] = {
    "tinyllama-q4_k_m": "https://huggingface.co/TheBloke/TinyLlama-1.1B-Chat-v1.0-GGUF/resolve/main/tinyllama-1.1b-chat-v1.0.Q4_K_M.gguf?download=true",
}


def run(cmd: list[str], check: bool = True, **kwargs) -> None:
    subprocess.run(cmd, check=check, **kwargs)


def fetch_llama(url_or_name: str, dest: str | os.PathLike[str]) -> None:
    url = KNOWN_LLAMA.get(url_or_name.lower(), url_or_name)
    dest = pathlib.Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    fname = url.split("/")[-1].split("?")[0]
    if not fname.endswith(".gguf"):
        fname = "model.gguf"
    out = dest / fname
    if out.exists():
        return
    run(["curl", "-fsSL", "-o", str(out), url])


def fetch_whisper(names: Iterable[str], cache_dir: str | os.PathLike[str]) -> None:
    cache_dir = pathlib.Path(cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["XDG_CACHE_HOME"] = str(cache_dir)
    py = os.environ.get("PYTHON", "python3")
    code = (
        "import sys; import whisper;\n"
        "mods = sys.argv[1:];\n"
        "[whisper.load_model(m) for m in mods];\n"
        "print('ok')\n"
    )
    run([py, "-c", code, *list(names)], env=env)


def main() -> None:
    parser = argparse.ArgumentParser(description="Fetch Llama/Whisper models")
    parser.add_argument("--llama", nargs="*", default=[], help="Llama GGUF URLs or known names")
    parser.add_argument("--whisper", nargs="*", default=[], help="Whisper model names, e.g. tiny base small")
    parser.add_argument("--defaults", action="store_true", help="Fetch TinyLlama Q4_K_M and Whisper tiny")
    parser.add_argument("--llama-dir", default=os.getenv("LLAMA_MODELS_DIR", "/opt/psyche/assets_seed/models/llama"))
    parser.add_argument("--cache-dir", default=os.getenv("XDG_CACHE_HOME", "/opt/psyche/assets_seed/cache"))
    args = parser.parse_args()

    llama_items = list(args.llama)
    whisper_items = list(args.whisper)
    if args.defaults:
        llama_items.append("tinyllama-q4_k_m")
        whisper_items.append("tiny")

    for item in llama_items:
        fetch_llama(item, args.llama_dir)
    if whisper_items:
        fetch_whisper(whisper_items, args.cache_dir)


if __name__ == "__main__":
    main()

