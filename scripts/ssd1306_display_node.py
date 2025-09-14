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


class DisplayNode(Node):
    """Show latest messages from topics as rotating pages with a ticker.

    - Rotates through subscribed topics every few seconds.
    - For long messages, scrolls horizontally like a ticker.
    """

    def __init__(self, topics: list[str], *, driver: str, width: int | None,
                 height: int | None, port: int, address: int) -> None:
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
        self._font = ImageFont.load_default()
        self._pages: List[_Page] = [ _Page(t) for t in self._topics ]
        self._page_index = 0
        self._last_switch = time.monotonic()
        self._page_seconds = 3.0
        self._tick_interval = 0.15
        qos = QoSProfile(depth=10)
        for t in self._topics:
            self.create_subscription(String, t, self._on_string, qos)
        # Timers: ticker and page rotation
        self.create_timer(self._tick_interval, self._tick)
        self._splash("Display ready")

    def _on_string(self, msg: String) -> None:
        topic = msg._topic_name if hasattr(msg, "_topic_name") else None  # rclpy fills it
        # rclpy doesn't expose topic on the message in all versions; map by callback context
        # Update the page for this topic
        if topic is None:
            # Fallback: update all pages with latest text (last writer wins)
            for p in self._pages:
                p.text = msg.data
                p.offset = 0
        else:
            for p in self._pages:
                if p.topic == topic:
                    p.text = msg.data
                    p.offset = 0
                    break
        self._render()

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
        # Render title small at top
        draw.text((0, 0), title, font=self._font, fill=255)
        text = page.text or ""
        if not text:
            draw.text((0, 16), "(no data)", font=self._font, fill=255)
        else:
            # Ticker: shift drawing rect to the left by offset
            # Use a clipping rectangle inside the screen
            y = 16
            # Compute text length in pixels
            tw = int(self._font.getlength(text)) if hasattr(self._font, "getlength") else draw.textlength(text, font=self._font)
            if tw <= w:
                draw.text((0, y), text, font=self._font, fill=255)
            else:
                # Draw twice to create seamless wraparound
                off = page.offset
                draw.text((-off, y), text, font=self._font, fill=255)
                draw.text((-off + tw + 10, y), text, font=self._font, fill=255)
        self._device.display(img)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("topics", nargs="*", default=["/sensor/range"])
    parser.add_argument("--driver", default="ssd1306",
                        help="oled driver: ssd1306|sh1106|ssd1309")
    parser.add_argument("--width", type=int, default=128)
    parser.add_argument("--height", type=int, default=64)
    parser.add_argument("--i2c-port", type=int, default=1)
    parser.add_argument("--i2c-address", type=lambda x: int(x, 0), default=0x3C)
    ns = parser.parse_args(argv)
    rclpy.init()
    node = DisplayNode(
        ns.topics,
        driver=ns.driver,
        width=ns.width,
        height=ns.height,
        port=ns.i2c_port,
        address=ns.i2c_address,
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
