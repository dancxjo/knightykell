#!/usr/bin/env python3
import os
import sys
import json
import socket
import signal
import threading
import time
import subprocess
from datetime import datetime

from PIL import Image, ImageDraw, ImageFont

try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import sh1106, ssd1306
except Exception as e:
    i2c = sh1106 = ssd1306 = None

# e-Paper support removed


SOCK_DIR = "/run/oled"
SOCK_PATH = os.path.join(SOCK_DIR, "statusd.sock")
STATUS_PATH = os.path.join(SOCK_DIR, "statusd.json")
DISPLAY_WIDTH = 128
DISPLAY_HEIGHT = 64


def parse_rotate_from_env(default=0):
    val = os.environ.get("OLED_ORIENTATION", "").lower()
    mapping = {
        "landscape": 0,
        "portrait": 1,
        "inverted": 2,
        "portrait-inverted": 3,
    }
    if val in mapping:
        return mapping[val]
    try:
        return int(os.environ.get("OLED_ROTATE", default))
    except Exception:
        return default


class OLED:
    def __init__(self, rotate: int = None, i2c_port: int = None, address: int = None, controller: str = None, h_offset: int = None):
        if rotate is None:
            rotate = parse_rotate_from_env(0)
        if i2c_port is None:
            i2c_port = int(os.environ.get("OLED_I2C_PORT", "1"))
        if controller is None:
            controller = os.environ.get("OLED_CONTROLLER", "auto").lower()
        # address may be provided as hex string or int; when unset or 'auto' try common addresses
        addr_env = os.environ.get("OLED_ADDR")
        if address is None:
            if addr_env and addr_env.lower() != "auto":
                try:
                    address = int(addr_env, 0)
                except Exception:
                    address = None
        if h_offset is None and os.environ.get("OLED_H_OFFSET"):
            try:
                h_offset = int(os.environ.get("OLED_H_OFFSET"))
            except Exception:
                h_offset = None

        self.rotate = rotate
        self.i2c_port = i2c_port
        self.address = address
        self.controller = controller
        self.h_offset = h_offset
        self.device = None
        self.last_init_err = None
        # optional flips to fine-tune physical orientation
        def _tf(k):
            return os.environ.get(k, "0").lower() in ("1", "true", "yes", "on")
        self.flip_x = _tf("OLED_FLIP_X")
        self.flip_y = _tf("OLED_FLIP_Y")

        # drawing resources
        self.image = Image.new("1", (DISPLAY_WIDTH, DISPLAY_HEIGHT), 0)
        self.draw = ImageDraw.Draw(self.image)
        # Prefer clear, readable fonts for small OLEDs
        self.font_small = None
        self.font_header = None
        self.font_tiny = None
        self.font_footer = None
        self.font_ticker = None
        font_path_env = os.environ.get("OLED_FONT")
        candidate_fonts = []
        if font_path_env:
            candidate_fonts.append(font_path_env)
        # Prefer Noto families exclusively for consistent glyph coverage
        candidate_fonts += [
            "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
            "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf",
            "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
            "/usr/share/fonts/truetype/noto/NotoSansSymbols-Regular.ttf",
            "/usr/share/fonts/truetype/noto/NotoSansSymbols2-Regular.ttf",
            "/usr/share/fonts/truetype/noto/NotoColorEmoji.ttf",
        ]
        # Slightly smaller body font for more room; override with OLED_FONT_SIZE
        font_size = 12
        try:
            font_size = int(os.environ.get("OLED_FONT_SIZE", font_size))
        except Exception:
            pass
        for fp in candidate_fonts:
            try:
                if os.path.isfile(fp):
                    self.font_small = ImageFont.truetype(fp, size=font_size)
                    break
            except Exception:
                continue
        if self.font_small is None:
            try:
                self.font_small = ImageFont.load_default()
            except Exception:
                self.font_small = None

        # derive a reasonable line height for layout early, before any use
        self.line_height = 10
        try:
            bbox = (self.font_small.getbbox("Ag") if self.font_small else None)
            if bbox:
                self.line_height = (bbox[3] - bbox[1]) + 2
        except Exception:
            pass

        # Header font: try to use a bold variant slightly larger; fall back to body font
        header_size = max(12, int(round(font_size * 1.1)))
        for fp in candidate_fonts:
            try:
                if os.path.isfile(fp) and ("Bold" in os.path.basename(fp) or fp.lower().endswith("-bold.ttf")):
                    self.font_header = ImageFont.truetype(fp, size=header_size)
                    break
            except Exception:
                continue
        if self.font_header is None:
            try:
                # Use same family at a touch larger if available
                base = candidate_fonts[0] if candidate_fonts else None
                if base and os.path.isfile(base):
                    self.font_header = ImageFont.truetype(base, size=header_size)
            except Exception:
                self.font_header = self.font_small

        # If header/logo may contain Shavian code points, prefer Noto Sans Shavian for coverage
        try:
            shavian_paths = [
                "/usr/share/fonts/truetype/noto/NotoSansShavian-Regular.ttf",
                "/usr/share/fonts/truetype/noto/NotoSansShavian.ttf",
            ]
            for up in shavian_paths:
                if os.path.isfile(up):
                    self.font_shavian = ImageFont.truetype(up, size=header_size)
                    break
        except Exception:
            self.font_shavian = None

        # Tiny font for top-right logo
        tiny_size = max(8, min(10, font_size - 2))
        for fp in candidate_fonts:
            try:
                if os.path.isfile(fp):
                    self.font_tiny = ImageFont.truetype(fp, size=tiny_size)
                    break
            except Exception:
                continue
        if self.font_tiny is None:
            self.font_tiny = self.font_small

        # Footer font (smaller to fit more info on one line)
        try:
            footer_size = max(8, int(os.environ.get("OLED_FOOTER_SIZE", "10")))
        except Exception:
            footer_size = max(8, (self.line_height - 2))
        self.font_footer = None
        for fp in candidate_fonts:
            try:
                if os.path.isfile(fp):
                    self.font_footer = ImageFont.truetype(fp, size=footer_size)
                    break
            except Exception:
                continue
        if self.font_footer is None:
            self.font_footer = self.font_tiny or self.font_small

        # Body font for compact three-line display (smaller than body)
        try:
            ticker_size = max(8, int(os.environ.get("OLED_TICKER_SIZE", str(max(8, int(self.line_height-2))))) )
        except Exception:
            ticker_size = max(8, int(self.line_height-2))
        self.font_ticker = None
        for fp in candidate_fonts:
            try:
                if os.path.isfile(fp):
                    self.font_ticker = ImageFont.truetype(fp, size=10)
                    break
            except Exception:
                continue
        if self.font_ticker is None:
            self.font_ticker = self.font_small

        # line_height already computed above

        # Ticker configuration (disabled: we render static three-line body)
        def _get_int(k, d):
            try:
                return int(os.environ.get(k, d))
            except Exception:
                return d
        self.ticker_enabled = os.environ.get("OLED_TICKER", "0").lower() in ("1", "true", "yes", "on")
        self.ticker_speed = _get_int("OLED_TICKER_SPEED", 5)  # pixels per frame
        self._ticker_text = ""
        self._ticker_width = 0
        self._ticker_pos = 0
        # multi-line ticker
        self.ticker_lines = _get_int("OLED_TICKER_LINES", 3)
        # frame delay (seconds)
        try:
            self.frame_delay = float(os.environ.get("OLED_FRAME_DELAY", "0.1"))
        except Exception:
            self.frame_delay = 0.1

        # Bottom-line ticker state (independent of page; maintains position across pages)
        try:
            self.bottom_ticker_speed = int(os.environ.get("OLED_BOTTOM_TICKER_SPEED", str(self.ticker_speed)))
        except Exception:
            self.bottom_ticker_speed = self.ticker_speed
        self._bottom_ticker_text = ""
        self._bottom_ticker_width = 0
        self._bottom_ticker_pos = 0

        self._try_init()

    def _display_frame(self):
        if not self.device:
            return
        frame = self.image
        try:
            if self.flip_x:
                frame = frame.transpose(Image.FLIP_LEFT_RIGHT)
            if self.flip_y:
                frame = frame.transpose(Image.FLIP_TOP_BOTTOM)
        except Exception:
            pass
        self.device.display(frame)

    def _W(self) -> int:
        try:
            return int(self.image.size[0])
        except Exception:
            return DISPLAY_WIDTH

    def _H(self) -> int:
        try:
            return int(self.image.size[1])
        except Exception:
            return DISPLAY_HEIGHT

    def splash(self, name: str = None, sub: str = None):
        if not name:
            name = os.environ.get("OLED_NAME", "Pete Knightykell")
        if not sub:
            sub = os.environ.get("OLED_SUBTEXT", "Booting…")
        # Draw a simple splash frame
        if not self.device:
            self._try_init()
            if not self.device:
                return
        self.draw.rectangle((0, 0, self._W() - 1, self._H() - 1), outline=255, fill=0)
        try:
            f = self.font_small
            y = 6
            self.draw.text((4, y), str(name)[:20], fill=255, font=f); y += self.line_height
            self.draw.text((4, y), str(sub)[:22], fill=255, font=f); y += self.line_height
            self.draw.text((4, y), "Starting…", fill=255, font=f)
        except Exception:
            pass
        try:
            self._display_frame()
        except Exception:
            pass

    def _make_device(self, address: int, controller: str):
        serial = i2c(port=self.i2c_port, address=address)
        if controller == "sh1106":
            # SH1106 often needs horizontal offset in landscape
            if self.h_offset is None:
                eff_h = 2 if self.rotate in (0, 2) else 0
            else:
                eff_h = self.h_offset
            return sh1106(serial, rotate=self.rotate, h_offset=eff_h)
        elif controller == "ssd1306":
            return ssd1306(serial, rotate=self.rotate)
        elif controller == "auto":
            # Try SH1106 first, then SSD1306
            try:
                if self.h_offset is None:
                    eff_h = 2 if self.rotate in (0, 2) else 0
                else:
                    eff_h = self.h_offset
                return sh1106(serial, rotate=self.rotate, h_offset=eff_h)
            except Exception:
                return ssd1306(serial, rotate=self.rotate)
        else:
            return ssd1306(serial, rotate=self.rotate)

    def _try_init(self):
        if i2c is None or (sh1106 is None and ssd1306 is None):
            print("[oled] luma.oled not available; running in no-display mode", file=sys.stderr)
            return
        addrs = [self.address] if isinstance(self.address, int) else [0x3C, 0x3D]
        ctrls = [self.controller] if self.controller != "auto" else ["sh1106", "ssd1306"]
        last_err = None
        for a in addrs:
            for c in ctrls:
                try:
                    self.device = self._make_device(a, c)
                    print(f"[oled] initialized {c} at 0x{a:02X} rotate={self.rotate}")
                    self.last_init_err = None
                    # Ensure backing image matches device size/orientation
                    try:
                        self.image = Image.new("1", self.device.size, 0)
                        self.draw = ImageDraw.Draw(self.image)
                    except Exception:
                        pass
                    return
                except Exception as e:
                    last_err = e
                    continue
        self.device = None
        self.last_init_err = last_err
        if last_err:
            print(f"[oled] init failed: {last_err}", file=sys.stderr)

    def clear(self):
        if not self.device:
            self._try_init()
            if not self.device:
                return
        self.draw.rectangle((0, 0, self._W(), self._H()), outline=0, fill=0)
        self._display_frame()

    def _text_width(self, text: str, font=None) -> int:
        try:
            # Pillow 8.0+: ImageFont.getlength via ImageDraw.textlength
            if hasattr(self.draw, 'textlength'):
                return int(self.draw.textlength(text, font=(font or self.font_small)))
            # Fallback: bbox width
            bbox = self.draw.textbbox((0,0), text, font=(font or self.font_small))
            return max(0, bbox[2] - bbox[0])
        except Exception:
            try:
                w, _ = self.draw.textsize(text, font=(font or self.font_small))
                return int(w)
            except Exception:
                return len(text) * 6

    def _render_ticker(self, header: str, lines):
        # Prepare device if needed
        if not self.device:
            self._try_init()
            if not self.device:
                return
        self.draw.rectangle((0, 0, self._W(), self._H()), outline=0, fill=0)
        y = 0
        # Top-right tiny logo (use Shavian-capable font when available)
        try:
            # Include Shavian namer dot (·) to mark a proper name
            logo = os.environ.get("OLED_LOGO_TEXT", "𐑯𐑲𐑑𐑦𐑒𐑧𐑤")
            font_logo = self.font_shavian or self.font_tiny or self.font_small
            if logo and font_logo:
                lw = int(self.draw.textlength(logo, font=font_logo)) if hasattr(self.draw, 'textlength') else self.draw.textbbox((0,0), logo, font=font_logo)[2]
                self.draw.text((self._W() - lw - 1, 0), logo, font=font_logo, fill=255)
        except Exception:
            pass

        if header:
            # Optional: render headline in Shavian if enabled
            header_src = header
            if os.environ.get("OLED_HEADLINE_SHAVIAN", "1").lower() in ("1", "true", "yes", "on"):
                try:
                    header = shavian_transliterate(header)
                except Exception:
                    header = header_src

            # Header slightly darker/bolder: use header font (or Unifont if Shavian), and overdraw once for weight
            use_font = self.font_header or self.font_small
            try:
                # If header contains Shavian block, prefer Noto Sans Shavian for coverage
                if any(0x10450 <= ord(ch) <= 0x1047F for ch in header) and getattr(self, 'font_shavian', None):
                    use_font = self.font_shavian
            except Exception:
                pass
            try:
                self.draw.text((0, y), header[:20], font=use_font, fill=255)
                # Overdraw offset by 1px right for thicker look (monochrome)
                self.draw.text((1, y), header[:20], font=use_font, fill=255)
            except Exception:
                self.draw.text((0, y), header[:20], font=self.font_small, fill=255)
            # move to next line and draw a separator
            try:
                hbbox = self.draw.textbbox((0, y), header[:20], font=self.font_header or self.font_small)
                h = (hbbox[3] - hbbox[1]) if hbbox else self.line_height
            except Exception:
                h = self.line_height
            y += max(self.line_height, h)
            self.draw.line((0, y, self._W(), y), fill=255)
            y += 1
        # Build ticker string
        parts = [str(x).strip() for x in (lines or []) if str(x).strip()]
        if not parts:
            self._display_frame()
            return
        text = ('   •   ').join(parts)
        # Update ticker metrics/state if content changed
        if text != self._ticker_text:
            self._ticker_text = text
            self._ticker_width = self._text_width(text, font=self.font_ticker or self.font_small)
            # Start just off the right edge for smooth entry
            self._ticker_pos = self._W()
        # Draw scrolling text across multiple lines using a smaller ticker font
        x = self._W() - self._ticker_pos
        # compute ticker line height
        try:
            tbbox = (self.font_ticker or self.font_small).getbbox("Ag")
            ticker_lh = (tbbox[3] - tbbox[1]) + 2
        except Exception:
            ticker_lh = self.line_height
        # Reserve space for footer at bottom
        footer_font = self.font_footer or self.font_small
        try:
            fb = footer_font.getbbox("Ag")
            footer_lh = (fb[3] - fb[1]) + 2
        except Exception:
            footer_lh = self.line_height
        max_y = self._H() - footer_lh - 1
        lines_avail = max(1, min(self.ticker_lines, max(0, (max_y - y)) // ticker_lh))
        for i in range(lines_avail):
            yy = y + i * ticker_lh
            try:
                self.draw.text((x, yy), text, font=(self.font_ticker or self.font_small), fill=255)
            except Exception:
                pass

        # Footer: bottom-line ticker (clock, uptime, host, WiFi)
        try:
            now = datetime.now().strftime(os.environ.get("OLED_CLOCK_FMT", "%H:%M:%S"))
            ip_v4, ssid, rssi = get_ip_info()
            # compute y position bottom line
            y_footer = self._H() - (footer_lh)
            # Build ticker content
            # uptime
            try:
                us = float(subprocess.check_output("awk '{print $1}' /proc/uptime", shell=True, text=True).strip())
                s = int(us)
                h, rem = divmod(s, 3600)
                m, rems = divmod(rem, 60)
                if h > 0:
                    up_txt = f"{h}h{m:02d}m"
                elif m > 0:
                    up_txt = f"{m}m{rems:02d}s"
                else:
                    up_txt = f"{s}s"
            except Exception:
                up_txt = "?"
            host = os.uname().nodename
            items = [f"time={now}", f"up={up_txt}", f"host={host}"]
            if ssid:
                items.append(f"ssid={ssid}")
            if rssi:
                items.append(f"rssi={rssi}")
            btext = '   |   '.join(items)
            if btext != self._bottom_ticker_text:
                self._bottom_ticker_text = btext
                self._bottom_ticker_width = self._text_width(btext, font=footer_font)
                cycle = self._bottom_ticker_width + self._W() + 10
                if cycle > 0 and self._bottom_ticker_pos > cycle:
                    self._bottom_ticker_pos = self._bottom_ticker_pos % cycle
                self._ticker_width = self._bottom_ticker_width
                self._ticker_pos = self._bottom_ticker_pos
            bx = self._W() - self._bottom_ticker_pos
            self.draw.rectangle((0, y_footer - 1, self._W(), self._H()), outline=0, fill=0)
            try:
                self.draw.text((bx, y_footer), self._bottom_ticker_text, font=footer_font, fill=255)
            except Exception:
                pass
            self._bottom_ticker_pos += max(1, int(self.bottom_ticker_speed))
            cycle = self._bottom_ticker_width + self._W() + 10
            if self._bottom_ticker_pos > cycle:
                self._bottom_ticker_pos = 0
            self._ticker_pos = self._bottom_ticker_pos
        except Exception:
            pass
        self._display_frame()
        # Advance for next frame
        self._ticker_pos += max(1, int(self.ticker_speed))
        if self._ticker_pos > (self._ticker_width + self._W() + 10):
            self._ticker_pos = 0

    def render_lines(self, header: str, lines):
        if not self.device:
            # Retry init occasionally in case I2C appears later in boot
            self._try_init()
            if not self.device:
                return
        # Always render a static three-line view, smaller font
        self.draw.rectangle((0, 0, self._W(), self._H()), outline=0, fill=0)
        y = 0
        # Top-right tiny Shavian logo
        try:
            logo = os.environ.get("OLED_LOGO_TEXT", "𐑯𐑲𐑑𐑦𐑒𐑧𐑤")
            font_logo = self.font_shavian or self.font_tiny or self.font_small
            if logo and font_logo:
                lw = int(self.draw.textlength(logo, font=font_logo)) if hasattr(self.draw, 'textlength') else self.draw.textbbox((0,0), logo, font=font_logo)[2]
                self.draw.text((self._W() - lw - 1, 0), logo, font=font_logo, fill=255)
        except Exception:
            pass
        # Header
        if header:
            try:
                header_src = header
                if os.environ.get("OLED_HEADLINE_SHAVIAN", "1").lower() in ("1", "true", "yes", "on"):
                    try:
                        header = shavian_transliterate(header)
                    except Exception:
                        header = header_src
                self.draw.text((0, y), header[:18], font=(self.font_header or self.font_small), fill=255)
                y += max(self.line_height, 11)
                self.draw.line((0, y, self._W(), y), fill=255)
                y += 1
            except Exception:
                pass
        # Body: up to 3 lines in compact font
        body_font = self.font_ticker or self.font_small
        # Merge lines with pipe, wrap to fit display width
        merged = " | ".join(str(ln).replace("\n", " ") for ln in (lines or []) if str(ln).strip())
        max_width = self._W() - 2
        # Simple word wrap
        words = merged.split()
        wrapped_lines = []
        line = ""
        for word in words:
            test_line = f"{line} {word}".strip()
            if self._text_width(test_line, font=body_font) <= max_width:
                line = test_line
            else:
                if line:
                    wrapped_lines.append(line)
                line = word
        if line:
            wrapped_lines.append(line)
        for wl in wrapped_lines[:3]:
            try:
                self.draw.text((0, y), wl, font=body_font, fill=255)
            except Exception:
                pass
            y += max(9, int(self.line_height - 1))
        # Footer: bottom-line ticker showing time, uptime, hostname, and WiFi info
        try:
            footer_font = self.font_footer or body_font
            # Determine footer line height
            try:
                fb = footer_font.getbbox("Ag")
                footer_lh = (fb[3] - fb[1]) + 2
            except Exception:
                footer_lh = max(8, int(self.line_height - 2))
            y_footer = self._H() - footer_lh
            # Build ticker content (ASCII only)
            now = datetime.now().strftime(os.environ.get("OLED_CLOCK_FMT", "%H:%M:%S"))
            # uptime in seconds -> concise human form
            up_txt = ""
            try:
                us = float(subprocess.check_output("awk '{print $1}' /proc/uptime", shell=True, text=True).strip())
                s = int(us)
                h, rem = divmod(s, 3600)
                m, rems = divmod(rem, 60)
                if h > 0:
                    up_txt = f"{h}h{m:02d}m"
                elif m > 0:
                    up_txt = f"{m}m{rems:02d}s"
                else:
                    up_txt = f"{s}s"
            except Exception:
                up_txt = "?"
            host = os.uname().nodename
            ip_v4, ssid, rssi = get_ip_info()
            items = [f"time={now}", f"up={up_txt}", f"host={host}"]
            if ssid:
                items.append(f"ssid={ssid}")
            if rssi:
                items.append(f"rssi={rssi}")
            text = '   |   '.join(items)
            # Update ticker metrics/state without resetting position
            if text != self._bottom_ticker_text:
                self._bottom_ticker_text = text
                self._bottom_ticker_width = self._text_width(text, font=footer_font)
                cycle = self._bottom_ticker_width + self._W() + 10
                if cycle > 0 and self._bottom_ticker_pos > cycle:
                    self._bottom_ticker_pos = self._bottom_ticker_pos % cycle
                # expose width/pos for external timing heuristics
                self._ticker_width = self._bottom_ticker_width
                self._ticker_pos = self._bottom_ticker_pos
            x = self._W() - self._bottom_ticker_pos
            # clear footer area to avoid overdraw artifacts
            self.draw.rectangle((0, y_footer - 1, self._W(), self._H()), outline=0, fill=0)
            try:
                self.draw.text((x, y_footer), self._bottom_ticker_text, font=footer_font, fill=255)
            except Exception:
                pass
            # Advance for next frame and wrap
            self._bottom_ticker_pos += max(1, int(self.bottom_ticker_speed))
            cycle = self._bottom_ticker_width + self._W() + 10
            if self._bottom_ticker_pos > cycle:
                self._bottom_ticker_pos = 0
            # keep mirrored values in sync
            self._ticker_pos = self._bottom_ticker_pos
        except Exception:
            pass
        self._display_frame()



    # (Duplicate methods removed; see earlier definitions above)


def get_ip_info():
    def run(cmd):
        try:
            out = subprocess.check_output(cmd, shell=True, text=True).strip()
            return out
        except Exception:
            return ""

    ip_v4 = run("""ip -4 -o addr show scope global | awk '{print $2":"$4}' || true""")
    if not ip_v4:
        ip_v4 = run("hostname -I 2>/dev/null | awk '{print $1}' || true")
    ssid = run("iwgetid -r || true")
    rssi = run("iw dev wlan0 link | awk -F' signal ' 'NF>1{print $2}' | awk '{print $1}' || true")
    return ip_v4, ssid, rssi


def shavian_transliterate(text: str) -> str:
    """Best-effort transliteration to Shavian.
    If a mapping file is provided via OLED_SHAVIAN_MAP (JSON dict), use it.
    Otherwise, return input unchanged to avoid incorrect output.
    """
    try:
      mp_path = os.environ.get("OLED_SHAVIAN_MAP")
      if mp_path and os.path.isfile(mp_path):
          with open(mp_path, 'r', encoding='utf-8') as f:
              import json as _json
              mapper = _json.load(f) or {}
          # simple token replacement by longest keys first
          s = text
          for k in sorted(mapper.keys(), key=len, reverse=True):
              s = s.replace(k, mapper.get(k, k))
          return s
    except Exception:
      pass
    # Default: no-op unless explicitly allowed to pass through a naive map
    return text


class StatusDaemon:
    def __init__(self):
        os.makedirs(SOCK_DIR, exist_ok=True)
        # Ensure previous socket removed
        try:
            os.unlink(SOCK_PATH)
        except FileNotFoundError:
            pass
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(SOCK_PATH)
        os.chmod(SOCK_PATH, 0o666)
        self.oled = OLED(rotate=0)
        # Manual overlay content (with TTL)
        self.overlay = None  # {header, lines, expires}

        # Pager state
        self.page_index = 0
        # Slow down paging by default for readability
        self.page_interval = float(os.environ.get("OLED_PAGE_INTERVAL", "10"))
        self._last_page_ts = 0.0

        # Baseline content used by overview page
        self.header = "BOOT"
        self.lines = ["Starting…", datetime.now().strftime("%H:%M:%S")]
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.started_at = datetime.now().isoformat(timespec="seconds")

        # Show splash briefly (non-blocking if no device)
        try:
            self.oled.splash(os.environ.get("OLED_NAME", "Pete Knightykell"), os.uname().nodename)
        except Exception:
            pass

        # cache for expensive calls
        self._cache = {}
        # Boot journal follow toggle
        self.boot_journal = os.environ.get("OLED_BOOT_JOURNAL", "1").lower() in ("1", "true", "yes", "on")

    def shutdown(self):
        self.stop_event.set()
        try:
            self.oled.clear()
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass
        try:
            os.unlink(SOCK_PATH)
        except Exception:
            pass

    def _cached(self, key, ttl_sec, supplier):
        now = time.monotonic()
        ent = self._cache.get(key)
        if not ent or (now - ent[0]) > ttl_sec:
            try:
                val = supplier()
            except Exception:
                val = None
            self._cache[key] = (now, val)
            return val
        return ent[1]

    def _page_overview(self):
        header = self.header
        lines = list(self.lines)
        # Remove IP address from body content; focus on other info and avoid emoji icons
        ip_v4, ssid, rssi = get_ip_info()
        if ssid:
            lines.append(f"SSID: {ssid}"[:21])
        if rssi:
            lines.append(f"RSSI: {rssi}"[:21])
        try:
            uptime = subprocess.check_output("awk '{print $1}' /proc/uptime", shell=True, text=True).strip()
            lines.append(f"uptime: {float(uptime):.0f}s")
        except Exception:
            pass
        return (header, lines)


    def _systemd_summary(self):
        def supplier():
            state = subprocess.check_output(
                ["bash", "-lc", "systemctl is-system-running 2>/dev/null || true"], text=True
            ).strip()
            failed = subprocess.check_output(
                ["bash", "-lc", "systemctl --failed --no-legend 2>/dev/null | awk '{print $1}' | head -n 3 | xargs -r echo"], text=True
            ).strip()
            return state, failed
        return self._cached("systemd", 5.0, supplier)

    def _page_systemd(self):
        state, failed = self._systemd_summary() or ("", "")
        lines = [f"state: {state}"]
        if failed:
            parts = failed.split()
            lines.append(f"failed: {len(parts)}")
            for u in parts[:2]:
                lines.append(f"x {u}")
        return ("system", lines)

    def _dmesg_tail(self):
        return self._cached(
            "dmesg", 5.0,
            lambda: subprocess.check_output(["bash", "-lc", "dmesg -T 2>/dev/null | tail -n 3 || true"], text=True)
        )

    def _page_dmesg(self):
        out = self._dmesg_tail() or ""
        lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
        return ("DMESG", lines[:4])

    def _journal_warnings(self):
        return self._cached(
            "journal_warn", 6.0,
            lambda: subprocess.check_output(["bash", "-lc", "journalctl -b -p warning --no-pager -n 3 -o short 2>/dev/null || true"], text=True)
        )

    def _page_journal(self):
        out = self._journal_warnings() or ""
        lines = []
        for ln in out.splitlines():
            if ";" in ln:
                ln = ln.split(";", 1)[-1]
            lines.append(ln.strip())
        return ("WARN", lines[:4])

    def _render_current_page(self):
        pages = [self._page_overview, self._page_systemd, self._page_dmesg, self._page_journal]
        now = time.monotonic()
        # honor dynamic minimum interval (e.g., allow full ticker pass)
        min_interval = max(getattr(self, 'dynamic_min_interval', 0.0), self.page_interval)
        if (now - self._last_page_ts) > min_interval:
            self.page_index = (self.page_index + 1) % len(pages)
            self._last_page_ts = now
        return pages[self.page_index]()

    def sender_loop(self):
        while not self.stop_event.is_set():
            with self.lock:
                ov = self.overlay
                if ov and time.monotonic() > ov.get("expires", 0):
                    self.overlay = ov = None
                if ov:
                    header, lines = ov.get("header", ""), ov.get("lines", [])
                else:
                    # adjust dynamic minimum page interval (estimate based on display width)
                    try:
                        cycle = (self.oled._ticker_width + self.oled._W() + 10) / max(1, int(self.oled.ticker_speed))
                        self.dynamic_min_interval = max(2.0, cycle * max(0.05, float(getattr(self.oled, 'frame_delay', 0.25))))
                    except Exception:
                        self.dynamic_min_interval = 0.0
                    header, lines = self._render_current_page()

            try:
                self.oled.render_lines(header, lines)
            except Exception:
                pass
            # e-paper support removed
            # Write health file atomically
            try:
                status = {
                    "started_at": self.started_at,
                    "last_update": datetime.now().isoformat(timespec="seconds"),
                    "device_ready": bool(self.oled.device),
                    "last_error": (str(self.oled.last_init_err) if getattr(self.oled, 'last_init_err', None) else None),
                    "page": getattr(self, 'page_index', 0),
                }
                tmp = STATUS_PATH + ".tmp"
                with open(tmp, "w") as f:
                    json.dump(status, f)
                os.replace(tmp, STATUS_PATH)
            except Exception:
                pass
            # frame delay for smoother/faster ticker
            time.sleep(getattr(self.oled, 'frame_delay', 0.25))

    def receiver_loop(self):
        while not self.stop_event.is_set():
            try:
                data, _ = self.sock.recvfrom(4096)
            except Exception:
                if self.stop_event.is_set():
                    break
                continue
            try:
                msg = json.loads(data.decode('utf-8', 'ignore'))
                header = msg.get('header') or msg.get('h') or ""
                lines = msg.get('lines') or msg.get('l') or []
                ttl = float(msg.get('ttl') or msg.get('t') or 6)
                if isinstance(lines, str):
                    lines = [lines]
                with self.lock:
                    if header or lines:
                        self.overlay = {
                            "header": header[:20] if header else "",
                            "lines": [str(x) for x in lines][:4],
                            "expires": time.monotonic() + max(1.0, min(30.0, ttl)),
                        }
                    if header:
                        self.header = header[:20]
                    if lines:
                        self.lines = [str(x) for x in lines][:4]
            except Exception:
                # ignore malformed
                pass

    def journal_alerts_loop(self):
        cmd = [
            "bash", "-lc",
            "journalctl -b -f -p warning --no-pager -o short 2>/dev/null"
        ]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        except Exception:
            return
        for line in proc.stdout or []:
            ln = line.strip()
            if not ln:
                continue
            if ";" in ln:
                ln = ln.split(";", 1)[-1].strip()
            with self.lock:
                self.overlay = {
                    "header": "WARN",
                    "lines": [ln[:21]],
                    "expires": time.monotonic() + 8.0,
                }

    def journal_boot_loop(self):
        if not self.boot_journal:
            return
        # Follow overall systemd journal during boot and render the latest lines
        cmd = [
            "bash", "-lc",
            "journalctl -b -f --no-pager -o short 2>/dev/null"
        ]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        except Exception:
            return
        last_lines = []
        last_set = 0.0
        while not self.stop_event.is_set():
            # stop condition: system reaches running or degraded
            st = self._systemd_summary() or ("", "")
            state = st[0] if isinstance(st, tuple) else ""
            if state in ("running", "degraded") and (time.monotonic() - self._last_page_ts) > 10:
                break
            try:
                ln = (proc.stdout.readline() if proc.stdout else "")
            except Exception:
                break
            if not ln:
                time.sleep(0.05)
                continue
            ln = ln.strip()
            if not ln:
                continue
            # Prefer systemd messages; otherwise include occasionally
            show = ("systemd[" in ln) or ("systemd " in ln) or ((time.monotonic() - last_set) > 2.0)
            if not show:
                continue
            # Keep the last 3 trimmed lines
            if ";" in ln:
                ln = ln.split(";", 1)[-1].strip()
            last_lines.append(ln[:50])
            last_lines = last_lines[-3:]
            with self.lock:
                self.overlay = {
                    "header": "BOOT",
                    "lines": last_lines[-2:] if len(last_lines) > 1 else last_lines,
                    "expires": time.monotonic() + 3.0,
                }
            last_set = time.monotonic()
        try:
            if proc and proc.poll() is None:
                proc.terminate()
        except Exception:
            pass

    def run(self):
        def on_sig(signum, frame):
            self.shutdown()
            sys.exit(0)
        signal.signal(signal.SIGTERM, on_sig)
        signal.signal(signal.SIGINT, on_sig)
        t1 = threading.Thread(target=self.sender_loop, daemon=True)
        t2 = threading.Thread(target=self.receiver_loop, daemon=True)
        t3 = threading.Thread(target=self.journal_alerts_loop, daemon=True)
        t4 = threading.Thread(target=self.journal_boot_loop, daemon=True)
        t1.start(); t2.start(); t3.start(); t4.start()
        t1.join(); t2.join(); t3.join(); t4.join()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--clear":
        OLED().clear()
        return 0
    d = StatusDaemon()
    d.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
