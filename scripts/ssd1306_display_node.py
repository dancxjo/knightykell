#!/usr/bin/env python3
"""Subscribe to topics and display lines on an SSD1306 OLED.

Examples:
    $ python3 scripts/ssd1306_display_node.py /sensor/range  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
from collections import deque

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


class DisplayNode(Node):
    """Show the latest messages from given topics on the OLED."""

    def __init__(self, topics: list[str], *, driver: str, width: int | None,
                 height: int | None, port: int, address: int) -> None:
        super().__init__("display")
        if not HAS_LUMA:
            raise RuntimeError("luma.oled not available; install it first")
        self._buf: deque[str] = deque(maxlen=5)
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
        qos = QoSProfile(depth=10)
        for t in self._topics:
            self.create_subscription(String, t, self._on_string, qos)
        # Show a small splash so the OLED visibly turns on even before messages
        try:
            self._buf.append("Display ready")
            self._render()
        except Exception:
            # Non-fatal if initial render fails; subscriptions may still work
            pass

    def _on_string(self, msg: String) -> None:
        self._buf.append(msg.data)
        self._render()

    def _render(self) -> None:
        img = Image.new("1", self._device.size)
        draw = ImageDraw.Draw(img)
        draw.text((0, 0), "\n".join(self._buf), font=self._font, fill=255)
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
