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
    from luma.oled.device import sh1106
except Exception as e:
    i2c = sh1106 = None


SOCK_DIR = "/run/oled"
SOCK_PATH = os.path.join(SOCK_DIR, "statusd.sock")
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
    def __init__(self, rotate: int = None, i2c_port: int = None, address: int = None):
        if rotate is None:
            rotate = parse_rotate_from_env(0)
        if i2c_port is None:
            i2c_port = int(os.environ.get("OLED_I2C_PORT", "1"))
        if address is None:
            address = int(os.environ.get("OLED_ADDR", "0x3C"), 16)
        self.rotate = rotate
        self.device = None
        if i2c is None or sh1106 is None:
            print("[oled] luma.oled not available; running in no-display mode", file=sys.stderr)
            return
        try:
            serial = i2c(port=i2c_port, address=address)
            self.device = sh1106(serial, rotate=rotate)
        except Exception as e:
            print(f"[oled] failed to init display: {e}", file=sys.stderr)
            self.device = None

        self.image = Image.new("1", (DISPLAY_WIDTH, DISPLAY_HEIGHT), 0)
        self.draw = ImageDraw.Draw(self.image)
        try:
            self.font_small = ImageFont.load_default()
        except Exception:
            self.font_small = None

    def clear(self):
        if not self.device:
            return
        self.draw.rectangle((0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT), outline=0, fill=0)
        self.device.display(self.image)

    def render_lines(self, header: str, lines):
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
        self.device.display(self.image)


def get_ip_info():
    def run(cmd):
        try:
            out = subprocess.check_output(cmd, shell=True, text=True).strip()
            return out
        except Exception:
            return ""

    ip_v4 = run("ip -4 -o addr show scope global | awk '{print $2":"$4}'")
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
        self.header = "BOOT"
        self.lines = ["Starting…", datetime.now().strftime("%H:%M:%S")]
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

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

    def sender_loop(self):
        while not self.stop_event.is_set():
            with self.lock:
                header = self.header
                lines = list(self.lines)
            # append dynamic system info
            ip_v4, ssid, rssi = get_ip_info()
            dyn = []
            if ip_v4:
                dyn.append(ip_v4.split("\n")[0][:21])
            if ssid:
                dyn.append(f"WiFi: {ssid}"[:21])
            if rssi:
                dyn.append(f"RSSI: {rssi}"[:21])
            uptime = subprocess.check_output("awk '{print $1}' /proc/uptime", shell=True, text=True).strip()
            dyn.append(f"Up: {float(uptime):.0f}s")

            try:
                self.oled.render_lines(header, lines + dyn)
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
                if isinstance(lines, str):
                    lines = [lines]
                with self.lock:
                    if header:
                        self.header = header[:20]
                    if lines:
                        self.lines = [str(x) for x in lines][:4]
            except Exception:
                # ignore malformed
                pass

    def run(self):
        def on_sig(signum, frame):
            self.shutdown()
            sys.exit(0)
        signal.signal(signal.SIGTERM, on_sig)
        signal.signal(signal.SIGINT, on_sig)
        t1 = threading.Thread(target=self.sender_loop, daemon=True)
        t2 = threading.Thread(target=self.receiver_loop, daemon=True)
        t1.start(); t2.start()
        t1.join(); t2.join()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--clear":
        OLED().clear()
        return 0
    d = StatusDaemon()
    d.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
