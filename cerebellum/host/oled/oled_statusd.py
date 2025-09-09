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
        font_path_env = os.environ.get("OLED_FONT")
        candidate_fonts = []
        if font_path_env:
            candidate_fonts.append(font_path_env)
        candidate_fonts += [
            # Lato (clean sans)
            "/usr/share/fonts/truetype/lato/Lato-Regular.ttf",
            # Noto (broad glyph coverage, good hinting)
            "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
            "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
            # Fira Code (mono) as a crisp option if present
            "/usr/share/fonts/opentype/firacode/FiraCode-Regular.otf",
            "/usr/share/fonts/truetype/firacode/FiraCode-Regular.ttf",
            # DejaVu fallbacks
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/unifont/unifont.ttf",
            "/usr/share/fonts/opentype/unifont/unifont.otf",
        ]
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

        # derive a reasonable line height for layout
        self.line_height = 10
        try:
            bbox = (self.font_small.getbbox("Ag") if self.font_small else None)
            if bbox:
                self.line_height = (bbox[3] - bbox[1]) + 2
        except Exception:
            pass

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

    def render_lines(self, header: str, lines):
        if not self.device:
            # Retry init occasionally in case I2C appears later in boot
            self._try_init()
            if not self.device:
                return
        self.draw.rectangle((0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT), outline=0, fill=0)
        y = 0
        if header:
            self.draw.text((0, y), header[:20], font=self.font_small, fill=255)
            y += self.line_height
            self.draw.line((0, y, DISPLAY_WIDTH, y), fill=255)
            y += 2
        for ln in lines[:5]:
            self.draw.text((0, y), str(ln)[:21], font=self.font_small, fill=255)
            y += self.line_height
        self._display_frame()



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
            self.draw.text((4, 8), str(name)[:20], fill=255, font=f)
            self.draw.text((4, 24), str(sub)[:22], fill=255, font=f)
            self.draw.text((4, 48), "Starting…", fill=255, font=f)
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

    def render_lines(self, header: str, lines):
        if not self.device:
            # Retry init occasionally in case I2C appears later in boot
            self._try_init()
            if not self.device:
                return
        self.draw.rectangle((0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT), outline=0, fill=0)
        y = 0
        if header:
            self.draw.text((0, y), header[:20], font=self.font_small, fill=255)
            y += 10
            self.draw.line((0, y, DISPLAY_WIDTH, y), fill=255)
            y += 2
        for ln in lines[:5]:
            self.draw.text((0, y), str(ln)[:21], font=self.font_small, fill=255)
            y += 10
        self._display_frame()


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
        self.page_interval = float(os.environ.get("OLED_PAGE_INTERVAL", "4"))
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

    def run(self):
        def on_sig(signum, frame):
            self.shutdown()
            sys.exit(0)
        signal.signal(signal.SIGTERM, on_sig)
        signal.signal(signal.SIGINT, on_sig)
        t1 = threading.Thread(target=self.sender_loop, daemon=True)
        t2 = threading.Thread(target=self.receiver_loop, daemon=True)
        t3 = threading.Thread(target=self.journal_alerts_loop, daemon=True)
        t1.start(); t2.start(); t3.start()
        t1.join(); t2.join(); t3.join()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--clear":
        OLED().clear()
        return 0
    d = StatusDaemon()
    d.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
