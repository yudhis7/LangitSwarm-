"""LED light effects (vectorised over all drones)."""
from __future__ import annotations

import math

import numpy as np

EFFECTS = ("solid", "image", "gradient", "split", "rainbow", "pulse", "twinkle", "wave", "off")


def hex_rgb(h, default=(1.0, 1.0, 1.0)) -> np.ndarray:
    try:
        h = str(h).lstrip("#")
        if len(h) == 3:
            h = "".join(c * 2 for c in h)
        return np.array([int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)], dtype=np.float32)
    except (ValueError, IndexError):
        return np.array(default, dtype=np.float32)


def hsv_to_rgb(h, s, v) -> np.ndarray:
    h = np.mod(h, 1.0) * 6.0
    i = np.floor(h).astype(int) % 6
    f = h - np.floor(h)
    p = v * (1 - s)
    q = v * (1 - s * f)
    t = v * (1 - s * (1 - f))
    r = np.choose(i, [v, q, p, p, t, v])
    g = np.choose(i, [t, v, v, q, p, p])
    b = np.choose(i, [p, p, t, v, v, q])
    return np.stack([r, g, b], axis=-1).astype(np.float32)


def drone_phase(idx: np.ndarray) -> np.ndarray:
    x = np.sin(idx.astype(np.float64) * 12.9898 + 78.233) * 43758.5453
    return (x - np.floor(x)).astype(np.float32)


def axis_norm(P: np.ndarray, axis: str, active: np.ndarray) -> np.ndarray:
    if axis == "radial":
        c = P[active].mean(0) if active.any() else P.mean(0)
        r = np.linalg.norm(P - c, axis=1)
        top = r[active].max() if active.any() else r.max()
        return (r / top if top > 0 else r * 0).astype(np.float32)
    k = {"x": 0, "y": 1, "z": 2}.get(axis, 0)
    v = P[:, k]
    ref = v[active] if active.any() else v
    lo, hi = float(ref.min()), float(ref.max())
    if hi - lo < 1e-9:
        return np.zeros(len(v), dtype=np.float32)
    return ((v - lo) / (hi - lo)).astype(np.float32)


def effect_colors(light: dict, P: np.ndarray, active: np.ndarray, tau: float,
                  idx: np.ndarray, point_colors: np.ndarray | None = None) -> np.ndarray:
    """Colours (N,3) float 0..1 for a formation `tau` seconds after it was reached."""
    light = light or {}
    eff = light.get("effect", "solid")
    c1 = hex_rgb(light.get("color", "#ffffff"))
    c2 = hex_rgb(light.get("color2", "#ff0000"))
    n = len(P)
    axis = light.get("axis", "x")
    speed = float(light.get("speed", 0.3))
    cycles = float(light.get("cycles", 1.0))

    if eff == "off":
        out = np.zeros((n, 3), np.float32)
    elif eff == "image":
        out = point_colors.astype(np.float32) if point_colors is not None else np.tile(c1, (n, 1))
    elif eff == "gradient":
        a = axis_norm(P, axis, active)[:, None]
        out = c1 * (1 - a) + c2 * a
    elif eff == "split":
        a = axis_norm(P, axis, active)
        out = np.where((a >= 0.5)[:, None], c1, c2).astype(np.float32)
        ripple = float(light.get("ripple", 0.0))
        if ripple > 0:
            xr = axis_norm(P, "x", active)
            out = out * (1.0 - ripple * (0.5 + 0.5 * np.sin(2 * math.pi * (1.5 * xr - 0.5 * tau))))[:, None]
    elif eff == "rainbow":
        a = axis_norm(P, axis, active)
        out = hsv_to_rgb(a * cycles + tau * speed, np.ones(n), np.ones(n))
    elif eff == "pulse":
        rate = float(light.get("rate", 1.0))
        low = float(light.get("low", 0.3))
        b = low + (1 - low) * (0.5 + 0.5 * math.cos(2 * math.pi * rate * tau))
        out = np.tile(c1 * b, (n, 1))
    elif eff == "twinkle":
        dens = max(0.02, min(1.0, float(light.get("density", 0.15))))
        rate = float(light.get("rate", 0.7))
        x = np.mod(tau * rate + drone_phase(idx), 1.0)
        bump = np.exp(-((x - 0.5) / (dens * 0.5)) ** 2)
        b = 0.3 + 0.7 * bump
        out = c1 * b[:, None]
    elif eff == "wave":
        a = axis_norm(P, axis, active)
        w = (0.5 + 0.5 * np.sin(2 * math.pi * (a * cycles - tau * speed)))[:, None]
        out = c1 * (1 - w) + c2 * w
    else:   # solid
        out = np.tile(c1, (n, 1))

    out = np.asarray(out, dtype=np.float32) * float(light.get("brightness", 1.0))
    out[~active] = 0.0
    return np.clip(out, 0.0, 1.0)
