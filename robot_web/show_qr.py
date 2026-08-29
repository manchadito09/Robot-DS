#!/usr/bin/env python3
# Build a "scan me" card (QR + URL) so a phone on the same WiFi can open the
# robot web app. The Jetson has no working browser, so we show the QR instead
# and the phone does the browsing. Prints the PNG path on stdout.
import socket
import segno
from PIL import Image, ImageDraw, ImageFont

PORT = 8000


def lan_ip():
    """This machine's address on the office WiFi (no packet actually sent)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def font(size, bold=False):
    base = "/usr/share/fonts/truetype/dejavu/"
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(base + name, size)
    except Exception:
        return ImageFont.load_default()


url = "http://%s:%d" % (lan_ip(), PORT)
segno.make(url, error="m").save("/tmp/_qr_only.png", scale=11, border=2,
                                dark="#0b1f12", light="#ffffff")
qr = Image.open("/tmp/_qr_only.png").convert("RGB")

W = max(qr.width + 80, 560)
top = 92
H = top + qr.height + 44
card = Image.new("RGB", (W, H), (255, 255, 255))
card.paste(qr, ((W - qr.width) // 2, top))
d = ImageDraw.Draw(card)
d.text((W // 2, 48), "Scan to control the robot", fill=(11, 31, 18),
       anchor="mm", font=font(34, bold=True))
out = "/tmp/robot_qr.png"
card.save(out)
print(out)
