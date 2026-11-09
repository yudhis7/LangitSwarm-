"""Generate the Langit Swarm brand assets from the master logo artwork.

    python tools/make_brand_assets.py assets/brand/logo_master.png

The master is the square logo card (cream background, orange emblem, "langit" script,
"Swarm." wordmark, tagline).  The script separates the artwork from the background
(anti-aliased edges become alpha) and writes:

  web/brand/logo_full_dark.png    full logo for dark screens (dark text turned cream)
  web/brand/logo_full_light.png   full logo, original colours, transparent background
  web/brand/emblem.png            winged emblem only, orange
  web/brand/lockup_dark.png       emblem + wordmark side by side for the title bar (dark UI)
  web/brand/favicon.png           64 px app icon
  assets/brand/langitswarm.ico    Windows icon 16..256 px (used by build_windows.bat)
  assets/brand/icon_256.png
  langit/show/brand_data.py       emblem as a data URL for the demo show's drone formation
"""
from __future__ import annotations

import base64
import io
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BG = np.array([229, 229, 212.0])         # card background (cream)
ORANGE = (225, 56, 14)                   # emblem
SCRIPT = (254, 51, 1)                    # "langit"
DARK = (34, 34, 34)                      # "Swarm." and tagline
CREAM = (229, 229, 212)


def separate(path):
    """alpha (H,W) and a boolean 'orange family' mask from the master artwork."""
    src = np.array(Image.open(path).convert("RGB")).astype(float)
    d = src - BG
    red = (src[..., 0] - src[..., 1]) > 40

    def proj(F):
        v = np.array(F, float) - BG
        return np.clip((d @ v) / (v @ v), 0, 1)

    alpha = np.where(red, proj(ORANGE), proj(DARK))
    alpha[np.linalg.norm(d, axis=2) < 6] = 0
    return alpha, red


def bands(alpha):
    rows = (alpha > 0.5).sum(1)
    out, inside, y0 = [], False, 0
    for y, r in enumerate(rows):
        if r and not inside:
            inside, y0 = True, y
        elif not r and inside:
            inside = False
            out.append((y0, y))
    return out


def render(alpha, red, box, colours):
    """RGBA crop; colours(kind, y) -> rgb for kind in ('red', 'dark')."""
    x0, y0, x1, y1 = box
    a = alpha[y0:y1, x0:x1]
    r = red[y0:y1, x0:x1]
    out = np.zeros((y1 - y0, x1 - x0, 4), np.uint8)
    yy = np.arange(y0, y1)[:, None].repeat(x1 - x0, 1)
    for kind, m in (("red", r), ("dark", ~r)):
        for y_lo, y_hi, col in colours(kind):
            sel = m & (yy >= y_lo) & (yy < y_hi)
            out[sel, :3] = col
    out[..., 3] = np.round(a * 255).astype(np.uint8)
    return Image.fromarray(out, "RGBA")


def trim(im, pad=0):
    bb = im.getchannel("A").point(lambda v: 255 if v > 8 else 0).getbbox()
    im = im.crop(bb)
    if pad:
        c = Image.new("RGBA", (im.width + 2 * pad, im.height + 2 * pad), (0, 0, 0, 0))
        c.paste(im, (pad, pad))
        im = c
    return im


def fit(im, w=None, h=None):
    if w and not h:
        h = round(im.height * w / im.width)
    if h and not w:
        w = round(im.width * h / im.height)
    return im.resize((w, h), Image.LANCZOS)


def main(master):
    alpha, red = separate(master)
    b = bands(alpha)
    if len(b) < 3:
        raise SystemExit(f"unexpected artwork layout: {b}")
    emb_y = b[0]
    word_y = b[1]
    tag_y = (b[2][0], b[-1][1])
    H, W = alpha.shape
    xs = np.nonzero((alpha > 0.5).any(0))[0]
    X0, X1 = int(xs.min()) - 4, int(xs.max()) + 5

    def cols_dark(kind):
        if kind == "red":
            return [(emb_y[0], emb_y[1] + 8, ORANGE), (word_y[0] - 8, H, SCRIPT)]
        return [(0, H, CREAM)]

    def cols_light(kind):
        if kind == "red":
            return [(emb_y[0], emb_y[1] + 8, ORANGE), (word_y[0] - 8, H, SCRIPT)]
        return [(0, H, DARK)]

    full_box = (X0, emb_y[0] - 4, X1, tag_y[1] + 5)
    full_dark = trim(render(alpha, red, full_box, cols_dark), 8)
    full_light = trim(render(alpha, red, full_box, cols_light), 8)
    emblem = trim(render(alpha, red, (X0, emb_y[0] - 4, X1, emb_y[1] + 8), cols_dark))
    word_dark = trim(render(alpha, red, (X0, word_y[0] - 4, X1, word_y[1] + 4), cols_dark))

    web = os.path.join(ROOT, "web", "brand")
    assets = os.path.join(ROOT, "assets", "brand")
    os.makedirs(web, exist_ok=True)
    os.makedirs(assets, exist_ok=True)
    fit(full_dark, w=720).save(os.path.join(web, "logo_full_dark.png"), optimize=True)
    fit(full_light, w=720).save(os.path.join(web, "logo_full_light.png"), optimize=True)
    fit(full_light, w=480).save(os.path.join(assets, "logo_readme.png"), optimize=True)
    fit(emblem, w=480).save(os.path.join(web, "emblem.png"), optimize=True)

    # title-bar lockup: emblem | wordmark, 2x resolution (shown 32 px high)
    h = 64
    e = fit(emblem, h=h - 6)
    wm = fit(word_dark, h=h)
    gap = 16
    lock = Image.new("RGBA", (e.width + gap + wm.width, h), (0, 0, 0, 0))
    lock.paste(e, (0, (h - e.height) // 2), e)
    lock.paste(wm, (e.width + gap, 0), wm)
    lock.save(os.path.join(web, "lockup_dark.png"), optimize=True)

    # icons: cream rounded tile with the emblem; tiny sizes use the eye only
    eye_box, circle = _eye(alpha, red, emb_y)
    eye = render(alpha, red, eye_box, cols_dark)
    ea = np.array(eye)
    ea[..., 3] = np.where(circle, ea[..., 3], 0)
    eye = trim(Image.fromarray(ea, "RGBA"))

    def tile(size):
        s = size * 4
        im = Image.new("RGBA", (s, s), (0, 0, 0, 0))
        ImageDraw.Draw(im).rounded_rectangle((0, 0, s - 1, s - 1), radius=int(s * 0.22), fill=CREAM + (255,))
        mark = fit(eye, h=int(s * 0.72)) if size <= 24 else fit(emblem, w=int(s * 0.86))
        im.alpha_composite(mark, ((s - mark.width) // 2, (s - mark.height) // 2))
        return im.resize((size, size), Image.LANCZOS)

    sizes = [16, 24, 32, 48, 64, 128, 256]
    tiles = {z: tile(z) for z in sizes}
    tiles[64].save(os.path.join(web, "favicon.png"), optimize=True)
    tiles[256].save(os.path.join(assets, "icon_256.png"), optimize=True)
    tiles[256].save(os.path.join(assets, "langitswarm.ico"), sizes=[(z, z) for z in sizes],
                    append_images=[tiles[z] for z in sizes[:-1]])

    # demo-show drone formation: emblem, orange, transparent, <= 360 px wide
    form = fit(emblem, w=360)
    buf = io.BytesIO()
    form.save(buf, "PNG", optimize=True)
    url = "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()
    with open(os.path.join(ROOT, "langit", "show", "brand_data.py"), "w", encoding="utf-8") as f:
        f.write('"""Generated by tools/make_brand_assets.py - the Langit Swarm emblem for the demo show."""\n\n')
        f.write("EMBLEM_PNG = (\n")
        for k in range(0, len(url), 100):
            f.write(f'    "{url[k:k + 100]}"\n')
        f.write(")\n")
    print("brand assets written:", web, assets, f"(emblem formation {form.size}, {len(url)} chars)")


def _eye(alpha, red, emb_y):
    """The eye (ring + pupil) cut out with a circular mask: centre and size come from the
    pupil (a separate blob), the outer radius from scanning across the ring."""
    from scipy import ndimage
    y0, y1 = emb_y
    m = (alpha > 0.5) & red
    m[:y0] = False
    m[y1:] = False
    lab, n = ndimage.label(m)
    sizes = ndimage.sum(m, lab, range(1, n + 1))
    comps = sorted(range(1, n + 1), key=lambda k: -sizes[k - 1])
    pupil = None
    for k in comps[1:4]:                          # biggest blob = ring + wings; the pupil is next
        ys, xs = np.nonzero(lab == k)
        if 0.6 < (np.ptp(xs) + 1) / (np.ptp(ys) + 1) < 1.6:
            pupil = (xs, ys)
            break
    xs, ys = pupil
    cx, cy = int(round(xs.mean())), int(round((ys.min() + ys.max()) / 2))
    x = xs.max() + 1
    while x < m.shape[1] and not m[cy, x]:        # gap between pupil and ring
        x += 1
    while x < m.shape[1] and m[cy, x]:            # the ring
        x += 1
    R = x - cx
    box = (cx - R - 2, cy - R - 2, cx + R + 3, cy + R + 3)
    yy, xx = np.mgrid[box[1]:box[3], box[0]:box[2]]
    circle = np.hypot(xx - cx, yy - cy) <= R + 0.5
    return box, circle


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "assets", "brand", "logo_master.png"))
