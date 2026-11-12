"""Generate background and card assets for the OKO MCHS presentation."""
from __future__ import annotations

import math
import random
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "docs" / "presentation" / "assets"
OUT.mkdir(parents=True, exist_ok=True)

W, H = 2560, 1440
S = 2  # supersampling for vector-ish elements


def _lerp(c1, c2, t):
    return tuple(int(round(a + (b - a) * t)) for a, b in zip(c1, c2))


def base_layer(size, top_left, bottom_right, glow):
    w, h = size
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    tx = xx / (w - 1)
    ty = yy / (h - 1)
    t = np.clip(0.18 * tx + 0.82 * ty, 0, 1)[..., None]
    c1 = np.array(top_left, dtype=np.float32)
    c2 = np.array(bottom_right, dtype=np.float32)
    img = c1 + (c2 - c1) * t
    # radial glows
    for cx, cy, radius, color, strength in glow:
        d = np.sqrt(((xx - cx * w) / (radius * w)) ** 2 + ((yy - cy * h) / (radius * h)) ** 2)
        g = np.clip(1 - d, 0, 1) ** 2 * strength
        img += np.array(color, dtype=np.float32) * g[..., None]
    img = np.clip(img, 0, 255)
    return img


def add_dot_grid(img, spacing=48, color=(30, 45, 70), alpha=95, start=24):
    h, w = img.shape[:2]
    layer = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    for y in range(start, h, spacing):
        for x in range(start, w, spacing):
            d.ellipse((x - 1.1, y - 1.1, x + 1.1, y + 1.1), fill=(*color, alpha))
    return Image.alpha_composite(Image.fromarray(img).convert("RGBA"), layer)


def add_radar(img, center, max_r=900, rings=7, color=(42, 66, 98)):
    w, h = img.size
    layer = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    cx, cy = center
    for i in range(1, rings + 1):
        r = max_r * i / rings
        a = int(26 + 72 * (1 - i / (rings + 1)))
        d.ellipse((cx - r, cy - r, cx + r, cy + r), outline=(*color, a), width=2)
    # crosshair
    a = 60
    d.line((cx - max_r, cy, cx + max_r, cy), fill=(*color, a), width=2)
    d.line((cx, cy - max_r, cx, cy + max_r), fill=(*color, a), width=2)
    # sector ticks
    for ang in range(0, 360, 30):
        r0, r1 = max_r * 0.92, max_r
        x0 = cx + r0 * math.cos(math.radians(ang))
        y0 = cy + r0 * math.sin(math.radians(ang))
        x1 = cx + r1 * math.cos(math.radians(ang))
        y1 = cy + r1 * math.sin(math.radians(ang))
        d.line((x0, y0, x1, y1), fill=(*color, 55), width=2)
    # blips with glow
    glow = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    gd = ImageDraw.Draw(glow)
    blips = [
        (cx + 300, cy - 170, (255, 122, 26)),
        (cx - 420, cy + 220, (56, 189, 248)),
        (cx + 130, cy + 340, (192, 132, 252)),
        (cx - 250, cy - 330, (255, 176, 32)),
    ]
    for bx, by, col in blips:
        for rr, aa in ((26, 60), (13, 120), (5, 255)):
            gd.ellipse((bx - rr, by - rr, bx + rr, by + rr), fill=(*col, aa))
    glow = glow.filter(ImageFilter.GaussianBlur(9))
    layer = Image.alpha_composite(layer, glow)
    return Image.alpha_composite(img, layer)


def finish(img, file, vignette=0.38, noise=1.8):
    arr = np.asarray(img.convert("RGB"), dtype=np.float32)
    h, w = arr.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    dx = (xx - w / 2) / (w / 2)
    dy = (yy - h / 2) / (h / 2)
    d = np.sqrt(dx ** 2 + dy ** 2) / math.sqrt(2)
    vig = 1 - vignette * (d ** 2)
    arr *= vig[..., None]
    if noise:
        rng = np.random.default_rng(7)
        arr += rng.normal(0, noise, arr.shape)
    arr = np.clip(arr, 0, 255).astype(np.uint8)
    Image.fromarray(arr).save(OUT / file, optimize=True)


def make_content_bg():
    img = base_layer(
        (W, H), (10, 15, 27), (12, 22, 39),
        [(0.98, 0.06, 0.55, (255, 122, 26), 14), (0.04, 0.96, 0.6, (37, 99, 235), 10)],
    )
    img = add_dot_grid(img, spacing=52, alpha=80)
    finish(Image.fromarray(img.astype(np.uint8)).convert("RGB"), "bg_content.png")


def make_cover_bg():
    img = base_layer(
        (W, H), (8, 12, 23), (12, 23, 42),
        [(0.9, 0.1, 0.65, (255, 122, 26), 20), (0.02, 0.95, 0.7, (37, 99, 235), 12)],
    )
    img = add_dot_grid(img, spacing=54, alpha=70)
    im = add_radar(Image.fromarray(img.astype(np.uint8)).convert("RGBA"), (2030, 620), max_r=860, rings=7)
    finish(im, "bg_cover.png", vignette=0.46)


def rounded_card(src, dst, radius=26, border=(45, 64, 94, 255), border_width=3, size=None):
    im = Image.open(src).convert("RGB")
    if size:
        im = im.resize(size, Image.LANCZOS)
    w, h = im.size
    im = im.convert("RGBA")
    mask = Image.new("L", (w, h), 0)
    md = ImageDraw.Draw(mask)
    md.rounded_rectangle((0, 0, w - 1, h - 1), radius=radius, fill=255)
    out = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    out.paste(im, (0, 0), mask)
    d = ImageDraw.Draw(out)
    d.rounded_rectangle((0, 0, w - 1, h - 1), radius=radius, outline=border, width=border_width)
    out.save(dst, optimize=True)


def main():
    random.seed(4)
    make_content_bg()
    make_cover_bg()
    rounded_card(ROOT / "docs" / "screenshots" / "operator_console.png", OUT / "console.png", radius=30)
    rounded_card(
        ROOT / "reports" / "incidents" / "INC-2026-002_115930.jpg",
        OUT / "frame_multi.jpg", radius=22, border=(74, 52, 48, 255), border_width=3,
    )
    print("assets done:", [p.name for p in sorted(OUT.iterdir())])


if __name__ == "__main__":
    main()
