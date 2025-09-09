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
        font_path_env = os.environ.get("OLED_FONT")
        candidate_fonts = []
        if font_path_env:
            candidate_fonts.append(font_path_env)
        candidate_fonts += [
            # Lato (clean sans)
            "/usr/share/fonts/truetype/lato/Lato-Regular.ttf",
            "/usr/share/fonts/truetype/lato/Lato-Bold.ttf",
            # Noto (broad glyph coverage, good hinting)
            "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
            "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf",
            "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
            # Fira Code (mono) as a crisp option if present
            "/usr/share/fonts/opentype/firacode/FiraCode-Regular.otf",
            "/usr/share/fonts/truetype/firacode/FiraCode-Regular.ttf",
            # DejaVu fallbacks
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/unifont/unifont.ttf",
            "/usr/share/fonts/opentype/unifont/unifont.otf",
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

        # If header may contain Shavian code points, prefer Unifont for coverage
        try:
            unifont_paths = [
                "/usr/share/fonts/truetype/unifont/unifont.ttf",
                "/usr/share/fonts/opentype/unifont/unifont.otf",
            ]
            for up in unifont_paths:
                if os.path.isfile(up):
                    self.font_unifont = ImageFont.truetype(up, size=header_size)
                    break
        except Exception:
            self.font_unifont = None

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

        # derive a reasonable line height for layout
        self.line_height = 10
        try:
            bbox = (self.font_small.getbbox("Ag") if self.font_small else None)
            if bbox:
                self.line_height = (bbox[3] - bbox[1]) + 2
        except Exception:
            pass

        # Ticker configuration
        def _get_int(k, d):
            try:
                return int(os.environ.get(k, d))
            except Exception:
                return d
        self.ticker_enabled = os.environ.get("OLED_TICKER", "1").lower() in ("1", "true", "yes", "on")
        self.ticker_speed = _get_int("OLED_TICKER_SPEED", 2)  # pixels per frame (~0.5s)
        self._ticker_text = ""
        self._ticker_width = 0
        self._ticker_pos = 0

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
        self.draw.rectangle((0, 0, DISPLAY_WIDTH - 1, DISPLAY_HEIGHT -1), outline=255, fill=0)
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
        self.draw.rectangle((0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT), outline=0, fill=0)
        self._display_frame()

    def _text_width(self, text: str) -> int:
        try:
            # Pillow 8.0+: ImageFont.getlength via ImageDraw.textlength
            if hasattr(self.draw, 'textlength'):
                return int(self.draw.textlength(text, font=self.font_small))
            # Fallback: bbox width
            bbox = self.draw.textbbox((0,0), text, font=self.font_small)
            return max(0, bbox[2] - bbox[0])
        except Exception:
            try:
                w, _ = self.draw.textsize(text, font=self.font_small)
                return int(w)
            except Exception:
                return len(text) * 6

    def _render_ticker(self, header: str, lines):
        # Prepare device if needed
        if not self.device:
            self._try_init()
            if not self.device:
                return
        self.draw.rectangle((0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT), outline=0, fill=0)
        y = 0
        # Top-right tiny logo
        try:
            # Include Shavian namer dot (·) to mark a proper name
            logo = os.environ.get("OLED_LOGO_TEXT", "\u00B7Knightykell")
            if logo and self.font_tiny:
                lw = int(self.draw.textlength(logo, font=self.font_tiny)) if hasattr(self.draw, 'textlength') else self.draw.textbbox((0,0), logo, font=self.font_tiny)[2]
                self.draw.text((DISPLAY_WIDTH - lw - 1, 0), logo, font=self.font_tiny, fill=255)
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
                # If header contains Shavian block, prefer Unifont for coverage
                if any(0x10450 <= ord(ch) <= 0x1047F for ch in header) and getattr(self, 'font_unifont', None):
                    use_font = self.font_unifont
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
            self.draw.line((0, y, DISPLAY_WIDTH, y), fill=255)
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
            self._ticker_width = self._text_width(text)
            # Start just off the right edge for smooth entry
            self._ticker_pos = DISPLAY_WIDTH
        # Draw scrolling text on the second line (below header)
        x = DISPLAY_WIDTH - self._ticker_pos
        try:
            self.draw.text((x, y), text, font=self.font_small, fill=255)
        except Exception:
            pass

        # Footer: clock and IP address on bottom two lines
        try:
            now = datetime.now().strftime(os.environ.get("OLED_CLOCK_FMT", "%H:%M:%S"))
            ip_v4, ssid, rssi = get_ip_info()
            ip_short = ""
            if ip_v4:
                first = str(ip_v4).split()[0]
                if ":" in first:
                    first = first.split(":", 1)[-1]
                if "/" in first:
                    first = first.split("/", 1)[0]
                ip_short = first
            # compute y positions from bottom
            lh = self.line_height
            y_ip = DISPLAY_HEIGHT - lh
            y_clk = DISPLAY_HEIGHT - (2 * lh)
            # clear footer area to avoid overdraw artifacts
            self.draw.rectangle((0, y_clk - 1, DISPLAY_WIDTH, DISPLAY_HEIGHT), outline=0, fill=0)
            # Draw clock (left) and ssid or RSSI (right)
            self.draw.text((0, y_clk), now, font=self.font_small, fill=255)
            right_text = ip_short or ssid or ""
            if right_text:
                try:
                    rw = int(self.draw.textlength(right_text, font=self.font_small)) if hasattr(self.draw, 'textlength') else self.draw.textbbox((0,0), right_text, font=self.font_small)[2]
                except Exception:
                    rw = len(right_text) * 6
                self.draw.text((DISPLAY_WIDTH - rw, y_ip), right_text, font=self.font_small, fill=255)
        except Exception:
            pass
        self._display_frame()
        # Advance for next frame
        self._ticker_pos += max(1, int(self.ticker_speed))
        if self._ticker_pos > (self._ticker_width + DISPLAY_WIDTH + 10):
            self._ticker_pos = 0

    def render_lines(self, header: str, lines):
        if not self.device:
            # Retry init occasionally in case I2C appears later in boot
            self._try_init()
            if not self.device:
                return
        if self.ticker_enabled:
            return self._render_ticker(header, lines)
        # Fallback: show at most two lines with larger line spacing
        self.draw.rectangle((0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT), outline=0, fill=0)
        y = 0
        if header:
            self.draw.text((0, y), header[:18], font=self.font_small, fill=255)
            y += self.line_height
            self.draw.line((0, y, DISPLAY_WIDTH, y), fill=255)
            y += 2
        for ln in (lines or [])[:2]:
            self.draw.text((0, y), str(ln)[:20], font=self.font_small, fill=255)
            y += self.line_height
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
        ip_v4, ssid, rssi = get_ip_info()
        if ip_v4:
            lines.append(ip_v4.split("\n")[0][:21])
        if ssid:
            lines.append(f"📶 {ssid}"[:21])
        if rssi:
            lines.append(f"📡 {rssi}"[:21])
        try:
            uptime = subprocess.check_output("awk '{print $1}' /proc/uptime", shell=True, text=True).strip()
            lines.append(f"🕒 {float(uptime):.0f}s")
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
        icon = {
            "running": "✅",
            "degraded": "⚠️",
            "starting": "⏳",
            "maintenance": "🛠",
            "stopping": "⏹",
            "offline": "⚫",
            "failed": "❌",
        }.get(state, "ℹ️")
        lines = [f"State: {state}"]
        if failed:
            parts = failed.split()
            lines.append(f"Failed: {len(parts)}")
            for u in parts[:2]:
                lines.append(f"✗ {u}")
        return (f"SYSTEM {icon}", lines)

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
        if (now - self._last_page_ts) > self.page_interval:
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
            time.sleep(0.5)

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
                    "header": "⚠️ WARN",
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
