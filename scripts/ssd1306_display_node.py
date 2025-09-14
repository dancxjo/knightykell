#!/usr/bin/env python3
"""Subscribe to topics and display pages with a ticker on an OLED.

Examples:
    $ python3 scripts/ssd1306_display_node.py /sensor/range  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Dict, List
from collections import deque
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306, sh1106, ssd1309
    from PIL import ImageDraw, ImageFont, Image
    HAS_LUMA = True
except Exception:
    HAS_LUMA = False


@dataclass
class _Page:
    topic: str
    text: str = ""
    offset: int = 0  # ticker pixel offset
    mode: str = "ticker"  # 'ticker' or 'terminal'
    lines: deque[str] | None = None  # for terminal mode


class DisplayNode(Node):
    """Show latest messages from topics as rotating pages with a ticker.

    - Rotates through subscribed topics every few seconds.
    - For long messages, scrolls horizontally like a ticker.
    """

    def __init__(self, topics: list[str], *, driver: str, width: int | None,
                 height: int | None, port: int, address: int,
                 page_seconds: float = 6.0, tick_interval: float = 0.10) -> None:
        super().__init__("display")
        if not HAS_LUMA:
            raise RuntimeError("luma.oled not available; install it first")
        self._topics = topics or ["/sensor/range"]
        self._serial = i2c(port=port, address=address)
        drv_map = {"ssd1306": ssd1306, "sh1106": sh1106, "ssd1309": ssd1309}
        drv = drv_map.get(driver.lower(), ssd1306)
        kw = {}
        if width:
            kw["width"] = int(width)
        if height:
            kw["height"] = int(height)
        self._device = drv(self._serial, **kw)
        # Prefer a nicer TrueType mono font if available
        tt_paths = [
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/freefont/FreeMono.ttf",
        ]
        self._font = None
        for p in tt_paths:
            try:
                if os.path.exists(p):
                    self._font = ImageFont.truetype(p, 12)
                    break
            except Exception:
                continue
        if self._font is None:
            self._font = ImageFont.load_default()
        self._pages: List[_Page] = []
        for t in self._topics:
            mode = "terminal" if t.strip().lower().endswith("/logs") else "ticker"
            lines = deque(maxlen=300) if mode == "terminal" else None
            self._pages.append(_Page(t, mode=mode, lines=lines))
        self._page_index = 0
        self._last_switch = time.monotonic()
        self._page_seconds = float(page_seconds)
        self._tick_interval = float(tick_interval)
        qos = QoSProfile(depth=10)
        for t in self._topics:
            self.create_subscription(String, t, self._make_cb(t), qos)
        # Timers: ticker and page rotation
        self.create_timer(self._tick_interval, self._tick)
        self._splash("Display ready")

    def _make_cb(self, topic: str):
        def _cb(msg: String) -> None:
            for p in self._pages:
                if p.topic == topic:
                    if p.mode == "terminal" and p.lines is not None:
                        for ln in str(msg.data).splitlines():
                            p.lines.append(ln)
                    else:
                        p.text = msg.data
                        p.offset = 0
                    break
            self._render()
        return _cb

    def _tick(self) -> None:
        # Advance ticker offset and possibly switch page
        now = time.monotonic()
        cur = self._pages[self._page_index]
        # Measure text width
        tw = int(self._font.getlength(cur.text)) if hasattr(self._font, "getlength") else ImageDraw.Draw(Image.new("1", self._device.size)).textlength(cur.text, font=self._font)
        w, _ = self._device.size
        if tw > w:
            cur.offset = (cur.offset + 2) % (tw + 10)
        else:
            cur.offset = 0
        if now - self._last_switch >= self._page_seconds:
            self._page_index = (self._page_index + 1) % len(self._pages)
            self._last_switch = now
        self._render()

    def _splash(self, text: str) -> None:
        try:
            img = Image.new("1", self._device.size)
            draw = ImageDraw.Draw(img)
            draw.text((0, 0), text, font=self._font, fill=255)
            self._device.display(img)
        except Exception:
            pass

    def _render(self) -> None:
        img = Image.new("1", self._device.size)
        draw = ImageDraw.Draw(img)
        w, h = self._device.size
        page = self._pages[self._page_index]
        title = f"{page.topic}"
        # Title bar inverted
        draw.rectangle((0, 0, w, 14), outline=255, fill=255)
        try:
            tw = int(self._font.getlength(title)) if hasattr(self._font, "getlength") else draw.textlength(title, font=self._font)
        except Exception:
            tw = len(title) * 6
        tx = max(0, min(2, (w - tw) // 2))
        draw.text((tx, 2), title, font=self._font, fill=0)
        if page.mode == "terminal" and page.lines is not None:
            # Render a terminal-like scrolling log under the title bar
            y0 = 16
            avail_h = h - y0
            line_h = max(8, self._font.getsize("A")[1]) if hasattr(self._font, "getsize") else 12
            max_lines = max(1, avail_h // line_h)
            # Wrap recent lines to fit width
            wrapped: list[str] = []
            for ln in list(page.lines)[-200:]:
                wrapped.extend(self._wrap_text(draw, ln, w - 4))
            # Take the last lines that fit
            to_draw = wrapped[-max_lines:]
            y = y0
            for ln in to_draw:
                draw.text((2, y), ln, font=self._font, fill=255)
                y += line_h
        else:
            text = page.text or ""
            if not text:
                draw.text((2, 18), "(no data)", font=self._font, fill=255)
            else:
                # Ticker: shift drawing rect to the left by offset
                y = 16
                tw = int(self._font.getlength(text)) if hasattr(self._font, "getlength") else draw.textlength(text, font=self._font)
                if tw <= w:
                    draw.text((2, y), text, font=self._font, fill=255)
                else:
                    off = page.offset
                    draw.text((2 - off, y), text, font=self._font, fill=255)
                    draw.text((2 - off + tw + 20, y), text, font=self._font, fill=255)
        self._device.display(img)

    def _wrap_text(self, draw: "ImageDraw", text: str, max_w: int) -> List[str]:
        # Simple word-wrapping by pixel width
        words = text.split()
        if not words:
            return [""]
        lines: List[str] = []
        cur = words[0]
        for w in words[1:]:
            trial = cur + " " + w
            try:
                tw = int(self._font.getlength(trial)) if hasattr(self._font, "getlength") else draw.textlength(trial, font=self._font)
            except Exception:
                tw = len(trial) * 6
            if tw <= max_w:
                cur = trial
            else:
                lines.append(cur)
                cur = w
        lines.append(cur)
        return lines


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("topics", nargs="*", default=["/sensor/range"])
    parser.add_argument("--driver", default="ssd1306",
                        help="oled driver: ssd1306|sh1106|ssd1309")
    parser.add_argument("--width", type=int, default=128)
    parser.add_argument("--height", type=int, default=64)
    parser.add_argument("--i2c-port", type=int, default=1)
    parser.add_argument("--i2c-address", type=lambda x: int(x, 0), default=0x3C)
    parser.add_argument("--page-seconds", type=float, default=6.0)
    parser.add_argument("--tick-interval", type=float, default=0.10)
    ns = parser.parse_args(argv)
    rclpy.init()
    node = DisplayNode(
        ns.topics,
        driver=ns.driver,
        width=ns.width,
        height=ns.height,
        port=ns.i2c_port,
        address=ns.i2c_address,
        page_seconds=ns.page_seconds,
        tick_interval=ns.tick_interval,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
