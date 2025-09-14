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
import socket
import pathlib
import termios

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
    offset: int = 0  # ticker pixel offset (legacy horizontal)
    mode: str = "ticker"  # 'ticker' or 'terminal'
    lines: deque[str] | None = None  # for terminal mode
    vfirst: int = 0  # first visible wrapped line (body vertical scroll)
    vpix: int = 0    # pixel offset within a line for smooth vertical scroll
    looped: bool = False  # whether a full vertical scroll cycle completed
    vfirst: int = 0  # first visible wrapped line (body vertical scroll)
    vpix: int = 0    # pixel offset within a line for smooth vertical scroll


class DisplayNode(Node):
    """Show latest messages from topics as rotating pages with a ticker.

    - Rotates through subscribed topics every few seconds.
    - For long messages, scrolls horizontally like a ticker.
    """

    def __init__(self, topics: list[str], *, driver: str, width: int | None,
                 height: int | None, port: int, address: int,
                 page_seconds: float = 8.0, tick_interval: float = 0.10,
                 extra: str = "") -> None:
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
        # Prefer TrueType fonts; pick sizes for title/body/status
        tt_candidates = [
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
            "/usr/share/fonts/truetype/freefont/FreeMono.ttf",
        ]
        def load_font(size: int):
            for p in tt_candidates:
                try:
                    if os.path.exists(p):
                        return ImageFont.truetype(p, size)
                except Exception:
                    continue
            return ImageFont.load_default()
        self._font_title = load_font(10)
        self._font_body = load_font(10)
        self._font_status = load_font(8)
        self._pages: List[_Page] = []
        for t in self._topics:
            mode = "terminal" if t.strip().lower().endswith("/logs") else "ticker"
            lines = deque(maxlen=300) if mode == "terminal" else None
            self._pages.append(_Page(t, mode=mode, lines=lines))
        self._page_index = 0
        self._last_switch = time.monotonic()
        self._page_seconds = float(page_seconds)
        self._tick_interval = float(tick_interval)
        # Bottom status/extra lines (ticker-style)
        self._extra_text = str(extra or "")
        self._status_off = 0
        self._extra_off = 0
        self._last_ip = None
        self._last_ip_ts = 0.0
        self._last_wifi = None
        self._last_wifi_ts = 0.0
        self._last_temp = None
        self._last_temp_ts = 0.0
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
                        p.vfirst = 0
                        p.vpix = 0
                        p.looped = False
                    break
            self._render()
        return _cb

    def _tick(self) -> None:
        # Advance ticker offset and possibly switch page
        now = time.monotonic()
        cur = self._pages[self._page_index]
        # Compute available body area and advance vertical scroll if needed
        w, h = self._device.size
        draw = ImageDraw.Draw(Image.new("1", self._device.size))
        title_h = 12
        body_line_h = max(8, self._font_body.getsize("A")[1]) if hasattr(self._font_body, "getsize") else 10
        status_h = max(8, self._font_status.getsize("A")[1]) if hasattr(self._font_status, "getsize") else 9
        content_h = max(1, h - title_h - (status_h + 2))
        max_lines = max(1, content_h // body_line_h)
        if cur.mode == "ticker":
            wrapped = self._wrap_text_with_font(draw, (cur.text or "")+"\n\n\n\n\n", w - 4, self._font_body)
            if len(wrapped) > max_lines:
                cur.vpix += 1
                if cur.vpix >= body_line_h:
                    cur.vpix = 0
                    # Add gap so text scrolls fully off with some blank space
                    gap_lines = max_lines + 2
                    total_positions = (len(wrapped) - max_lines) + gap_lines
                    cur.vfirst += 1
                    if cur.vfirst >= total_positions:
                        cur.vfirst = 0
                        cur.looped = True
            else:
                cur.vpix = 0
                cur.vfirst = 0
                cur.looped = True  # nothing to scroll implies immediate cycle
        # Bottom status ticker offsets
        status = self._status_text()
        if self._extra_text:
            status = f"{status}  •  {self._extra_text}"
        stw = int(self._font_status.getlength(status)) if hasattr(self._font_status, "getlength") else draw.textlength(status, font=self._font_status)
        if stw > w:
            self._status_off = (self._status_off + 2) % (stw + 20)
        else:
            self._status_off = 0
        # Switch pages more slowly: wait until a full scroll cycle completes
        # and also respect the minimum dwell time.
        if now - self._last_switch >= self._page_seconds:
            should_switch = True
            if cur.mode == "ticker":
                # Prefer to finish a full scroll cycle, but never block forever.
                # If not looped within 3x dwell time, switch anyway.
                should_switch = cur.looped or (now - self._last_switch >= self._page_seconds * 3)
            if should_switch and len(self._pages) > 1:
                self._page_index = (self._page_index + 1) % len(self._pages)
                # Reset next page cycle marker
                self._pages[self._page_index].looped = False
                self._last_switch = now
        self._render()

    def _splash(self, text: str) -> None:
        try:
            img = Image.new("1", self._device.size)
            draw = ImageDraw.Draw(img)
            draw.text((0, 0), text, font=self._font_title, fill=255)
            self._device.display(img)
        except Exception:
            pass

    def _render(self) -> None:
        img = Image.new("1", self._device.size)
        draw = ImageDraw.Draw(img)
        w, h = self._device.size
        page = self._pages[self._page_index]
        title = f"{page.topic}"
        # Title bar (slim) inverted
        title_h = 12
        draw.rectangle((0, 0, w, title_h), outline=255, fill=255)
        try:
            tw = int(self._font_title.getlength(title)) if hasattr(self._font_title, "getlength") else draw.textlength(title, font=self._font_title)
        except Exception:
            tw = len(title) * 6
        tx = max(0, min(2, (w - tw) // 2))
        draw.text((tx, 1), title, font=self._font_title, fill=0)
        # Bottom info area: single status line (bottom-aligned)
        status_h = max(8, self._font_status.getsize("A")[1]) if hasattr(self._font_status, "getsize") else 9
        y_bottom = h - status_h
        draw.line((0, y_bottom - 2, w, y_bottom - 2), fill=255)
        status = self._status_text()
        if self._extra_text:
            status = f"{status}  •  {self._extra_text}"
        stw = int(self._font_status.getlength(status)) if hasattr(self._font_status, "getlength") else draw.textlength(status, font=self._font_status)
        off = self._status_off
        if stw <= w:
            draw.text((1, y_bottom), status, font=self._font_status, fill=255)
        else:
            draw.text((1 - off, y_bottom), status, font=self._font_status, fill=255)
            draw.text((1 - off + stw + 40, y_bottom), status, font=self._font_status, fill=255)
        if page.mode == "terminal" and page.lines is not None:
            # Render a terminal-like scrolling log under the title bar
            y0 = title_h + 2
            avail_h = (y_bottom - 3) - y0
            line_h = max(8, self._font_body.getsize("A")[1]) if hasattr(self._font_body, "getsize") else 10
            max_lines = max(1, avail_h // line_h)
            # Wrap recent lines to fit width
            wrapped: list[str] = []
            for ln in list(page.lines)[-200:]:
                wrapped.extend(self._wrap_text_with_font(draw, ln, w - 4, self._font_body))
            # Take the last lines that fit
            to_draw = wrapped[-max_lines:]
            y = y0
            for ln in to_draw:
                draw.text((2, y), ln, font=self._font_body, fill=255)
                y += line_h
        else:
            text = page.text or ""
            if not text:
                draw.text((2, title_h + 2), "(no data)", font=self._font_body, fill=255)
            else:
                # Multi-line body with smooth vertical scroll when needed
                y0 = title_h + 2
                avail_h = (y_bottom - 3) - y0
                line_h = max(8, self._font_body.getsize("A")[1]) if hasattr(self._font_body, "getsize") else 10
                max_lines = max(1, avail_h // line_h)
                wrapped = self._wrap_text_with_font(draw, text, w - 4, self._font_body)
                if len(wrapped) <= max_lines:
                    y = y0
                    for ln in wrapped:
                        draw.text((2, y), ln, font=self._font_body, fill=255)
                        y += line_h
                else:
                    s = page.vfirst
                    vp = page.vpix
                    y = y0 - vp
                    for ln in wrapped[s : s + max_lines + 1]:
                        draw.text((2, y), ln, font=self._font_body, fill=255)
                        y += line_h
        self._device.display(img)

    def _status_text(self) -> str:
        """Return compact status: time/date, IP, host, Wi‑Fi, temp, ROS domain.

        Format: ``HH:MM  YYYY-MM-DD  192.168.x.x  host  WiFi:-58dBm  CPU:47C  RD:0``
        """
        # Time and date
        t = time.strftime("%H:%M")
        d = time.strftime("%Y-%m-%d")
        host = self._safe_hostname()
        ip = self._primary_ip()
        wifi = self._wifi_rssi()
        temp = self._cpu_temp()
        rd = os.getenv("ROS_DOMAIN_ID", "0")
        parts = [f"{t}", d, ip, host]
        if wifi:
            parts.append(f"WiFi:{wifi}")
        if temp:
            parts.append(f"CPU:{temp}")
        parts.append(f"RD:{rd}")
        return "  ".join(parts)

    def _safe_hostname(self) -> str:
        try:
            return socket.gethostname()
        except Exception:
            return "(host)"

    def _primary_ip(self) -> str:
        # Refresh IP at most every 5 seconds
        now = time.monotonic()
        if self._last_ip and now - self._last_ip_ts < 5.0:
            return self._last_ip
        ip = "0.0.0.0"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.1)
            s.connect(("1.1.1.1", 80))
            ip = s.getsockname()[0]
            s.close()
        except Exception:
            try:
                ip = socket.gethostbyname(socket.gethostname())
            except Exception:
                pass
        self._last_ip = ip
        self._last_ip_ts = now
        return ip

    def _wifi_rssi(self) -> str | None:
        # Parse /proc/net/wireless for wlan0 level in dBm
        now = time.monotonic()
        if self._last_wifi and now - self._last_wifi_ts < 5.0:
            return self._last_wifi
        path = pathlib.Path("/proc/net/wireless")
        val: str | None = None
        try:
            txt = path.read_text()
            for line in txt.splitlines():
                if line.strip().startswith("wlan0:"):
                    parts = line.split()
                    if len(parts) >= 4:
                        lvl = parts[3]
                        # Expect like -58.
                        try:
                            n = float(lvl)
                            val = f"{int(n)}dBm"
                        except Exception:
                            val = lvl.strip('.').strip()
                        break
        except Exception:
            val = None
        self._last_wifi = val
        self._last_wifi_ts = now
        return val

    def _cpu_temp(self) -> str | None:
        # Try common thermal zones
        now = time.monotonic()
        if self._last_temp and now - self._last_temp_ts < 5.0:
            return self._last_temp
        def read_temp(p: pathlib.Path) -> float | None:
            try:
                raw = p.read_text().strip()
                v = float(raw)
                return v / 1000.0 if v > 200 else v
            except Exception:
                return None
        temp_c: float | None = None
        for n in range(0, 8):
            p = pathlib.Path(f"/sys/class/thermal/thermal_zone{n}/temp")
            if p.exists():
                v = read_temp(p)
                if v is not None:
                    temp_c = v
                    break
        self._last_temp = f"{int(temp_c)}C" if temp_c is not None else None
        self._last_temp_ts = now
        return self._last_temp

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
                tw = int(self._font_body.getlength(trial)) if hasattr(self._font_body, "getlength") else draw.textlength(trial, font=self._font_body)
            except Exception:
                tw = len(trial) * 6
            if tw <= max_w:
                cur = trial
            else:
                lines.append(cur)
                cur = w
        lines.append(cur)
        return lines

    def _wrap_text_with_font(self, draw: "ImageDraw", text: str, max_w: int, font) -> List[str]:
        words = text.split()
        if not words:
            return [""]
        lines: List[str] = []
        cur = words[0]
        for w in words[1:]:
            trial = cur + " " + w
            try:
                tw = int(font.getlength(trial)) if hasattr(font, "getlength") else draw.textlength(trial, font=font)
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
    parser.add_argument("--extra", default="", help="extra bottom-line text")
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
        extra=ns.extra,
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
