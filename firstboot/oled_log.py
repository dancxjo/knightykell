#!/usr/bin/env python3
"""
Display log lines on SSD1306 OLED during first boot.

Usage:
    tail -f /var/log/syslog | python3 /opt/psyche/oled_log.py
"""
import os
import sys
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306, sh1106, ssd1309
from PIL import ImageDraw, ImageFont, Image

port = int(os.getenv("OLED_I2C_PORT", "1"))
addr = int(os.getenv("OLED_I2C_ADDR", "0x3C"), 0)
drv = os.getenv("OLED_DRIVER", "ssd1306").lower()
drv_map = {"ssd1306": ssd1306, "sh1106": sh1106, "ssd1309": ssd1309}
Device = drv_map.get(drv, ssd1306)
width = int(os.getenv("OLED_WIDTH", "128"))
height = int(os.getenv("OLED_HEIGHT", "64"))
serial = i2c(port=port, address=addr)
device = Device(serial, width=width, height=height)
font = ImageFont.load_default()

MAX_LINES = 5

buffer = []
def display(text):
    img = Image.new("1", device.size)
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), text, font=font, fill=255)
    device.display(img)

for line in sys.stdin:
    buffer.append(line.strip())
    if len(buffer) > MAX_LINES:
        buffer.pop(0)
    display("\n".join(buffer))
