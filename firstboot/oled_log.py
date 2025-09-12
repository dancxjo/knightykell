#!/usr/bin/env python3
"""
Display log lines on SSD1306 OLED during first boot.

Usage:
    tail -f /var/log/syslog | python3 /opt/psyche/oled_log.py
"""
import sys
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import ImageDraw, ImageFont, Image

serial = i2c(port=1, address=0x3C)
device = ssd1306(serial)
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
