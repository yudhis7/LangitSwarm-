"""Formation generators.

Every generator returns exactly ``m`` points whose nearest-neighbour distance is
at least ``spacing`` (measured in the downwash-scaled metric).  Shapes are sized
automatically: the smallest size that fits the requested number of drones at
the requested spacing, unless the formation asks for a larger size.
"""
from __future__ import annotations

import base64
import io
import math
from dataclasses import dataclass, field

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont
from scipy.spatial import cKDTree

SQRT3 = math.sqrt(3.0)
GOLDEN_ANGLE = math.pi * (3.0 - math.sqrt(5.0))

SHAPES_2D = ("text", "image", "disk", "ring", "heart", "star", "wall")
SHAPES_3D = ("sphere", "torus", "cube", "cylinder", "cone", "wave", "points")
ALL_SHAPES = SHAPES_2D + SHAPES_3D


@dataclass
class Formation:
    points: np.ndarray                      # (m, 3) show frame
    colors: np.ndarray | None = None        # (m, 3) float 0..1, optional per-point colours
    warnings: list = field(default_factory=list)


def resolve_count(count, n: int) -> int:
    if count is None:
        return n
    c = float(count)
    m = int(round(c * n)) if c <= 1.0 else int(round(c))
    return max(1, min(n, m))


# ----------------------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------------------
def _scaled(p: np.ndarray, zscale: float) -> np.ndarray:
    if zscale == 1.0:
        return p
    q = p.copy()
    q[:, 2] = q[:, 2] / zscale
    return q


def min_nn_distance(p: np.ndarray, zscale: float = 1.0) -> float:
    if len(p) < 2:
        return float("inf")
    d, _ = cKDTree(_scaled(p, zscale)).query(_scaled(p, zscale), k=2)
    return float(d[:, 1].min())


def _drop_evenly(p: np.ndarray, m: int, extra: np.ndarray | None = None):
    """Keep exactly m rows, removing surplus rows spread evenly through the array."""
    k = len(p) - m
    if k <= 0:
        return p, extra
    drop = np.unique(np.round(np.linspace(0, len(p) - 1, k)).astype(int))
    # unique() may merge duplicates for tiny arrays; top up from the end
    if len(drop) < k:
        rest = np.setdiff1d(np.arange(len(p)), drop)[::-1][: k - len(drop)]
        drop = np.concatenate([drop, rest])
    keep = np.ones(len(p), dtype=bool)
    keep[drop] = False
    return p[keep], (extra[keep] if extra is not None else None)


def hex_grid_2d(w: float, h: float, s: float) -> np.ndarray:
    dy = s * SQRT3 / 2.0
    rows = max(1, int(h / dy) + 2)
    cols = max(1, int(w / s) + 2)
    ys = (np.arange(rows) - (rows - 1) / 2.0) * dy
    xs = (np.arange(cols) - (cols - 1) / 2.0) * s
    X, Y = np.meshgrid(xs, ys)
    X[1::2] += s / 2.0
    return np.stack([X.ravel(), Y.ravel()], axis=1)


def _crop(mask: np.ndarray, rgb: np.ndarray | None = None):
    ys, xs = np.nonzero(mask)
    if len(ys) == 0:
        raise ValueError("shape is empty (nothing to draw)")
    y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
    return mask[y0:y1, x0:x1], (rgb[y0:y1, x0:x1] if rgb is not None else None)


def sample_mask(mask: np.ndarray, m: int, spacing: float, height: float | None = None,
                rgb: np.ndarray | None = None, vstretch: float = 1.0):
    """Place m points on a hex lattice inside a raster mask.

    mask: bool (H, W), row 0 = top.  Returns (uv (m,2) metres centred, colours|None, warnings)
    vstretch > 1 stretches the vertical axis (used for downwash-aware vertical planes).
    """
    warnings = []
    mask, rgb = _crop(mask, rgb)
    if vstretch != 1.0:
        h_new = max(2, int(round(mask.shape[0] / vstretch)))
        im = Image.fromarray(mask.astype(np.uint8) * 255).resize((mask.shape[1], h_new), Image.NEAREST)
        mask = np.array(im) > 127
        if rgb is not None:
            rgb = np.array(Image.fromarray(rgb).resize((mask.shape[1], h_new), Image.BILINEAR))
    H, W = mask.shape
    area_px = float(mask.sum())

    def lattice(k, s):
        pts = hex_grid_2d(W * k, H * k, s)
        px = np.floor((pts[:, 0] + W * k / 2.0) / k).astype(int)
        py = np.floor((H * k / 2.0 - pts[:, 1]) / k).astype(int)
        ok = (px >= 0) & (px < W) & (py >= 0) & (py < H)
        idx = np.nonzero(ok)[0]
        inside = mask[py[idx], px[idx]]
        idx = idx[inside]
        return pts[idx], px[idx], py[idx]

    s = spacing
    if height:
        k = float(height) / (H * vstretch)
        pts, px, py = lattice(k, s)
        if len(pts) < m:
            warnings.append(f"requested height {height:.0f} m is too small for {m} drones at "
                            f"{spacing:.2f} m spacing; formation enlarged")
            height = None
        else:   # spread the drones out: largest spacing that still fits m points
            lo, hi = s, s * math.sqrt(len(pts) / m) * 1.5 + 1e-6
            for _ in range(30):
                mid = 0.5 * (lo + hi)
                if len(lattice(k, mid)[0]) >= m:
                    lo = mid
                else:
                    hi = mid
            s = lo
            pts, px, py = lattice(k, s)
    if not height:
        k0 = math.sqrt(m * (SQRT3 / 2.0) * s * s / area_px)
        lo, hi = k0 * 0.25, k0 * 1.2
        while len(lattice(hi, s)[0]) < m:
            hi *= 1.5
        for _ in range(32):
            mid = 0.5 * (lo + hi)
            if len(lattice(mid, s)[0]) >= m:
                hi = mid
            else:
                lo = mid
        pts, px, py = lattice(hi, s)

    cols = None
    if rgb is not None:
        cols = rgb[py, px, :3].astype(np.float32) / 255.0
    pts, cols = _drop_evenly(pts, m, cols)
    pts = pts - (pts.max(0) + pts.min(0)) / 2.0
    pts[:, 1] *= vstretch
    return pts, cols, warnings


# ----------------------------------------------------------------------------------------
# rasterised 2D shapes
# ----------------------------------------------------------------------------------------
_FONT_CANDIDATES = ("arialbd.ttf", "Arial Bold.ttf", "DejaVuSans-Bold.ttf",
                    "LiberationSans-Bold.ttf", "arial.ttf", "DejaVuSans.ttf")


def _font(px: int, name: str | None = None):
    names = ((name,) if name else ()) + _FONT_CANDIDATES
    for n in names:
        try:
            return ImageFont.truetype(n, px)
        except (OSError, TypeError):
            continue
    return ImageFont.load_default(size=px)


def text_mask(text: str, style: str = "fill", font: str | None = None, px: int = 160) -> np.ndarray:
    lines = [ln for ln in (text or " ").split("\n")] or [" "]
    f = _font(px, font)
    boxes = [f.getbbox(ln or " ") for ln in lines]
    widths = [b[2] - b[0] for b in boxes]
    line_h = int(px * 1.18)
    pad = px // 4
    img = Image.new("L", (max(widths) + 2 * pad, line_h * len(lines) + 2 * pad), 0)
    d = ImageDraw.Draw(img)
    for i, (ln, b) in enumerate(zip(lines, boxes)):
        x = pad + (max(widths) - (b[2] - b[0])) // 2 - b[0]
        d.text((x, pad + i * line_h), ln, fill=255, font=f)
    if style == "outline":
        t = max(3, int(px * 0.09)) | 1
        eroded = img.filter(ImageFilter.MinFilter(t))
        mask = (np.array(img) > 127) & ~(np.array(eroded) > 127)
    else:
        mask = np.array(img) > 127
    return mask


def _load_image(src: str) -> Image.Image:
    if not src:
        raise ValueError("image formation has no image")
    if src.startswith("data:"):
        raw = base64.b64decode(src.split(",", 1)[1])
        im = Image.open(io.BytesIO(raw))
    else:
        im = Image.open(src)
    im = im.convert("RGBA")
    im.thumbnail((360, 360))
    return im


def image_mask(params: dict):
    im = _load_image(params.get("image", ""))
    a = np.array(im)
    rgb = a[:, :, :3]
    alpha = a[:, :, 3]
    if alpha.min() < 250:                       # transparent background -> use alpha
        mask = alpha > 127
    else:
        lum = (0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]) / 255.0
        thr = float(params.get("threshold", 0.5))
        mask = lum < thr if params.get("invert") else lum > thr
    return mask, rgb


def implicit_mask(shape: str, params: dict, res: int = 420) -> np.ndarray:
    if shape == "wall":
        aspect = float(params.get("aspect", 1.6))
        w = res
        h = max(4, int(res / aspect))
        return np.ones((h, w), dtype=bool)
    if shape == "star":
        img = Image.new("L", (res, res), 0)
        pts = []
        spikes = int(params.get("points", 5))
        inner = float(params.get("inner", 0.42))
        for i in range(spikes * 2):
            r = (res * 0.5) * (1.0 if i % 2 == 0 else inner)
            a = math.pi / 2 + i * math.pi / spikes
            pts.append((res / 2 + r * math.cos(a), res / 2 - r * math.sin(a)))
        ImageDraw.Draw(img).polygon(pts, fill=255)
        return np.array(img) > 127
    u = np.linspace(-1.35, 1.35, res)
    v = np.linspace(1.35, -1.35, res)
    X, Y = np.meshgrid(u, v)
    if shape == "disk":
        return X * X + Y * Y <= 1.0
    if shape == "ring":
        inner = float(params.get("inner", 0.6))
        r2 = X * X + Y * Y
        return (r2 <= 1.0) & (r2 >= inner * inner)
    if shape == "heart":
        Yh = Y + 0.15
        return (X * X + Yh * Yh - 1.0) ** 3 - X * X * Yh ** 3 <= 0.0
    raise ValueError(f"unknown 2D shape {shape}")


# ----------------------------------------------------------------------------------------
# 3D shapes (unit size; scaled afterwards)
# ----------------------------------------------------------------------------------------
def _fib_sphere(m: int) -> np.ndarray:
    i = np.arange(m) + 0.5
    z = 1.0 - 2.0 * i / m
    r = np.sqrt(np.maximum(0.0, 1.0 - z * z))
    th = GOLDEN_ANGLE * i
    return np.stack([r * np.cos(th), r * np.sin(th), z], axis=1)


def _torus(m: int, ratio: float) -> np.ndarray:
    """Near-uniform torus sampling: every ring around the tube gets a number of points
    proportional to its circumference, so the inner side is not over-crowded."""
    R, r = 1.0, max(0.05, min(0.9, ratio))

    def build(s):
        n_v = max(3, int(round(2 * math.pi * r / (s * SQRT3 / 2))))
        pts = []
        for k in range(n_v):
            v = 2 * math.pi * k / n_v
            rad = R + r * math.cos(v)
            n_u = max(3, int(round(2 * math.pi * rad / s)))
            u = (np.arange(n_u) + 0.5 * (k % 2)) * (2 * math.pi / n_u)
            pts.append(np.stack([rad * np.cos(u), rad * np.sin(u), np.full(n_u, r * math.sin(v))], axis=1))
        return np.vstack(pts)

    lo, hi = 1e-4, 2.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if len(build(mid)) >= m:
            lo = mid
        else:
            hi = mid
    return _drop_evenly(build(lo), m)[0]


def _cube(m: int) -> np.ndarray:
    k = 2
    while k ** 3 - max(0, k - 2) ** 3 < m:
        k += 1
    g = np.linspace(-1, 1, k)
    X, Y, Z = np.meshgrid(g, g, g, indexing="ij")
    p = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
    surf = np.any(np.isclose(np.abs(p), 1.0), axis=1)
    return _drop_evenly(p[surf], m)[0]


def _cylinder(m: int, aspect: float) -> np.ndarray:
    i = np.arange(m) + 0.5
    h = 2.0 * aspect
    z = (i / m - 0.5) * h
    th = GOLDEN_ANGLE * np.arange(m)
    return np.stack([np.cos(th), np.sin(th), z], axis=1)


def _cone(m: int, aspect: float) -> np.ndarray:
    i = np.arange(m) + 4.0             # offset avoids a crowded apex
    u = np.sqrt(i / (m + 3.0))         # area-uniform distance from the apex
    h = 2.0 * aspect
    z = h * (0.5 - u)
    r = u
    th = GOLDEN_ANGLE * np.arange(m)
    return np.stack([r * np.cos(th), r * np.sin(th), z], axis=1)


def _wave(m: int, spacing: float, params: dict) -> np.ndarray:
    side = math.sqrt(m * SQRT3 / 2.0) * spacing * 1.05
    pts = hex_grid_2d(side, side, spacing)
    # keep the m points closest to the centre (round footprint)
    order = np.argsort((pts ** 2).sum(1))
    pts = pts[order[:m]]
    amp = float(params.get("amplitude", side * 0.08))
    lam = float(params.get("wavelength", side * 0.5))
    z = amp * np.sin(2 * math.pi * pts[:, 0] / lam) * np.cos(2 * math.pi * pts[:, 1] / lam)
    return np.stack([pts[:, 0], pts[:, 1], z], axis=1)


def _points_csv(text: str):
    rows = []
    for line in (text or "").splitlines():
        line = line.strip()
        if not line or line[0] in "#xX":
            continue
        vals = [float(v) for v in line.replace(";", ",").split(",") if v.strip()]
        if len(vals) >= 3:
            rows.append(vals[:6] + [math.nan] * (6 - len(vals[:6])))
    if not rows:
        raise ValueError("point list is empty (expected lines of x,y,z[,r,g,b])")
    a = np.array(rows, dtype=np.float64)
    cols = None
    if not np.isnan(a[:, 3:6]).any():
        cols = a[:, 3:6] / (255.0 if a[:, 3:6].max() > 1.0 else 1.0)
    return a[:, :3], cols


# ----------------------------------------------------------------------------------------
# transforms
# ----------------------------------------------------------------------------------------
def rotation_matrix(yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0) -> np.ndarray:
    y, p, r = (math.radians(a) for a in (yaw_deg, pitch_deg, roll_deg))
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])
    Ry = np.array([[math.cos(r), 0, math.sin(r)], [0, 1, 0], [-math.sin(r), 0, math.cos(r)]])
    return Rz @ Rx @ Ry


def _enforce_spacing(p: np.ndarray, spacing: float, zscale: float, size: float | None,
                     warnings: list) -> np.ndarray:
    p = p - (p.max(0) + p.min(0)) / 2.0
    d = min_nn_distance(p, zscale)
    scale = spacing / d if d > 0 else 1.0
    if size:
        extent = float((p.max(0) - p.min(0)).max())
        want = float(size) / extent if extent > 0 else 1.0
        if want < scale * 0.999:
            warnings.append(f"size {size:.0f} m is too small for this many drones; enlarged to "
                            f"{extent * scale:.0f} m")
        scale = max(scale, want)
    return p * scale


def build_formation(item: dict, n: int, settings: dict) -> Formation:
    """Build the points of one timeline item for an n-drone show (show frame)."""
    shape = item.get("shape", "sphere")
    params = item.get("params") or {}
    m = resolve_count(item.get("count", 1.0), n)
    spacing = float(settings["formation_spacing"])
    zscale = float(settings.get("downwash_factor", 1.0))
    size = params.get("size") or params.get("height")
    size = float(size) if size else None
    warnings: list = []
    colors = None

    if shape in SHAPES_2D:
        vertical = item.get("plane", "vertical") != "horizontal"
        vstretch = zscale if vertical else 1.0
        rgb = None
        if shape == "text":
            mask = text_mask(params.get("text", "HELLO"), params.get("style", "fill"), params.get("font"))
        elif shape == "image":
            mask, rgb = image_mask(params)
        else:
            mask = implicit_mask(shape, params)
        uv, colors, w = sample_mask(mask, m, spacing, height=size, rgb=rgb, vstretch=vstretch)
        warnings += w
        if vertical:
            local = np.stack([uv[:, 0], np.zeros(len(uv)), uv[:, 1]], axis=1)
        else:
            local = np.stack([uv[:, 0], uv[:, 1], np.zeros(len(uv))], axis=1)
    else:
        if shape == "sphere":
            local = _fib_sphere(m)
        elif shape == "torus":
            local = _torus(m, float(params.get("ratio", 0.33)))
        elif shape == "cube":
            local = _cube(m)
        elif shape == "cylinder":
            local = _cylinder(m, float(params.get("aspect", 1.2)))
        elif shape == "cone":
            local = _cone(m, float(params.get("aspect", 1.0)))
        elif shape == "wave":
            local = _wave(m, spacing, params)
        elif shape == "points":
            local, colors = _points_csv(params.get("csv", ""))
            if len(local) != m:
                if len(local) > m:
                    local, colors = _drop_evenly(local, m, colors)
                else:
                    warnings.append(f"point list has {len(local)} points; {m - len(local)} drones parked")
            if params.get("absolute"):
                size = None
        else:
            raise ValueError(f"unknown shape '{shape}'")
        if shape == "points" and params.get("absolute"):
            d = min_nn_distance(local, zscale)
            if d < spacing * 0.999:
                warnings.append(f"imported points are only {d:.2f} m apart; scaled up to keep spacing")
                local = local * (spacing / d)
        else:
            local = _enforce_spacing(local, spacing, zscale, size, warnings)

    rot = item.get("rotation") or [0, 0, 0]
    R = rotation_matrix(*[float(a) for a in (list(rot) + [0, 0, 0])[:3]])
    pts = local @ R.T
    d = min_nn_distance(pts, zscale)
    if d < spacing * 0.999:          # rotation mixed z into the plane (downwash metric)
        pts = pts * (spacing / d)
    center = np.array((list(item.get("center") or [0, 0, 80]) + [0, 0, 0])[:3], dtype=np.float64)
    pts = pts + center
    return Formation(points=pts, colors=colors, warnings=warnings)
