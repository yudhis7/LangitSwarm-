"""Synthetic Skybrush Studio shows for tests (.skyc, show.json, CSV .zip).

The show: drones on a pad grid take off row by row (staggered Skybrush takeoffTime,
trajectories starting at 1 m like real exports), climb, expand into a wider grid,
turn 90 deg about the vertical axis in four cubic-Bezier arcs, contract back over the
pads and land row by row (trajectories end at 1 m).  Lights are Skybrush LED bytecode
with a loop.  The reference functions (ref_position / ref_color) are written
independently of the importer so the tests can compare against ground truth.
"""
from __future__ import annotations

import base64
import io
import json
import math
import zipfile

import numpy as np

SP = 3.0          # pad spacing (m)


def pads(n):
    cols = int(math.ceil(math.sqrt(n)))
    k = np.arange(n)
    r, c = k // cols, k % cols
    P = np.zeros((n, 3))
    P[:, 0] = (c - (cols - 1) / 2) * SP + 40.0       # Skybrush coordinates, deliberately off-centre
    P[:, 1] = r * SP + 15.0
    return P, r


def _arc_ctrl(p0, center, ang0, ang1, z):
    """Cubic Bezier approximation of a circular arc (radius from p0) - standard 4/3 tan(a/4)."""
    r = math.hypot(p0[0] - center[0], p0[1] - center[1])
    a0, a1 = ang0, ang1
    k = 4.0 / 3.0 * math.tan((a1 - a0) / 4.0)
    P0 = np.array([center[0] + r * math.cos(a0), center[1] + r * math.sin(a0), z])
    P3 = np.array([center[0] + r * math.cos(a1), center[1] + r * math.sin(a1), z])
    P1 = P0 + k * r * np.array([-math.sin(a0), math.cos(a0), 0.0])
    P2 = P3 - k * r * np.array([-math.sin(a1), math.cos(a1), 0.0])
    return P1, P2, P3


def build(n=64, scale_time=1.0, extra_turns=0, expand=1.6):
    """Returns a list of drones: dict(name, home, takeoff, points [(t_abs, xyz, [ctrl...])], lights_events)."""
    P, rows = pads(n)
    C = np.array([P[:, 0].mean(), P[:, 1].mean()])
    stagger = min(0.25, 4.0 / max(1, int(rows.max()) + 1))
    drones = []
    T = lambda x: x * scale_time  # noqa: E731
    for i in range(n):
        p = P[i]
        r = int(rows[i])
        tko = 3.0 + stagger * r
        pts = [(tko, np.array([p[0], p[1], 1.0]), [])]
        pts.append((T(14.0), np.array([p[0], p[1], 12.0]), []))
        pts.append((T(17.0), np.array([p[0], p[1], 12.0]), []))
        e = np.array([C[0] + expand * (p[0] - C[0]), C[1] + expand * (p[1] - C[1]), 30.0])
        pts.append((T(27.0), e, []))
        a0 = math.atan2(e[1] - C[1], e[0] - C[0])
        t = T(27.0)
        nseg = 4 * (1 + extra_turns)
        for k in range(nseg):
            a_s = a0 + (math.pi / 8) * k
            a_e = a0 + (math.pi / 8) * (k + 1)
            P1, P2, P3 = _arc_ctrl(e, C, a_s, a_e, 30.0)
            t += T(5.0)
            pts.append((t, P3, [P1.tolist(), P2.tolist()]))
        t_hold = t + T(5.0)
        pts.append((t_hold, pts[-1][1].copy(), []))
        pts.append((t_hold + T(10.0), np.array([p[0], p[1], 12.0]), []))
        pts.append((t_hold + T(12.0), np.array([p[0], p[1], 12.0]), []))
        t_end = t_hold + T(23.0) + stagger * r
        pts.append((t_end, np.array([p[0], p[1], 1.0]), []))
        # lights: (t_start, kind, color, dur)
        if i % 2 == 0:
            ev = [("set_gray", 20, 0.0), ("wait_until", T(14.0)), ("fade", (255, 120, 0), 3.0),
                  ("loop", 5, [("fade", (255, 255, 255), 1.0), ("fade", (0, 128, 255), 1.0)]),
                  ("set", (255, 0, 0), 5.0), ("fade", (0, 0, 0), 2.0)]
        else:
            ev = [("set", (0, 0, 40), 2.0), ("wait_until", T(16.0)), ("set", (255, 255, 255), 4.0),
                  ("fade", (0, 255, 0), 6.0), ("sleep", 3.0), ("fade", (255, 0, 255), 4.0)]
        drones.append({"name": f"Drone {i + 1}", "home": [float(p[0]), float(p[1]), 0.0], "takeoff": tko,
                       "points": pts, "lights": ev})
    return drones


# ---- light program encoding -----------------------------------------------------------------
def _varint(v: int) -> bytes:
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        if v:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def _d(sec):
    return _varint(int(round(sec * 50)))


def encode_lights(ev) -> bytes:
    out = bytearray()
    for e in ev:
        k = e[0]
        if k == "set":
            out += bytes([0x04, *e[1]]) + _d(e[2])
        elif k == "set_gray":
            out += bytes([0x05, e[1]]) + _d(e[2])
        elif k == "fade":
            c = e[1]
            if c == (0, 0, 0):
                out += bytes([0x0A]) + _d(e[2])
            elif c == (255, 255, 255):
                out += bytes([0x0B]) + _d(e[2])
            else:
                out += bytes([0x08, *c]) + _d(e[2])
        elif k == "sleep":
            out += bytes([0x02]) + _d(e[1])
        elif k == "wait_until":
            out += bytes([0x03]) + _d(e[1])
        elif k == "loop":
            out += bytes([0x0C, e[1]]) + encode_lights(e[2])[:-1] + bytes([0x0D])
    out += bytes([0x00])
    return bytes(out)


def ref_color(ev, t):
    """Independent reference evaluator of the light events at show time t."""
    keys = [(0.0, (0, 0, 0))]
    clock = 0.0
    cur = (0, 0, 0)

    def run(evs):
        nonlocal clock, cur
        for e in evs:
            k = e[0]
            if k in ("set", "set_gray"):
                col = e[1] if k == "set" else (e[1],) * 3
                keys.append((clock, cur))
                keys.append((clock, col))
                cur = col
                clock += e[2]
            elif k == "fade":
                keys.append((clock, cur))
                clock += e[2]
                keys.append((clock, e[1]))
                cur = e[1]
            elif k == "sleep":
                clock += e[1]
            elif k == "wait_until":
                clock = max(clock, e[1])
            elif k == "loop":
                for _ in range(e[1]):
                    run(e[2])
    run(ev)
    ts = [k[0] for k in keys]
    j = 0
    for j in range(len(keys) - 1, -1, -1):
        if ts[j] <= t:
            break
    if j == len(keys) - 1:
        return np.array(keys[-1][1], float)
    (t0, c0), (t1, c1) = keys[j], keys[j + 1]
    if t1 <= t0:
        return np.array(c1, float)
    w = (t - t0) / (t1 - t0)
    return np.array(c0, float) * (1 - w) + np.array(c1, float) * w


def ref_position(d, t):
    """Independent reference: pad until take-off, the Bezier path (abs times), the pad after."""
    pts = d["points"]
    home = np.array(d["home"], float)
    if t <= pts[0][0]:
        # importer adds a 1 m/s climb from the pad in front of the first point (1 m)
        t_climb0 = pts[0][0] - 1.0
        if t <= t_climb0:
            return home.copy()
        u = (t - t_climb0) / 1.0
        return home + u * (pts[0][1] - home)
    for k in range(1, len(pts)):
        t0, A, _ = pts[k - 1]
        t1, B, cps = pts[k]
        if t <= t1:
            u = (t - t0) / (t1 - t0)
            if not cps:
                return A + u * (B - A)
            C1, C2 = np.array(cps[0]), np.array(cps[1])
            v = 1 - u
            return v ** 3 * A + 3 * v * v * u * C1 + 3 * v * u * u * C2 + u ** 3 * B
    last = pts[-1][1]
    t_last = pts[-1][0]
    if t <= t_last + 1.0:                       # importer adds a 1 m/s landing
        u = (t - t_last) / 1.0
        return last + u * (np.array([last[0], last[1], 0.0]) - last)
    return np.array([last[0], last[1], 0.0])


# ---- writers ----------------------------------------------------------------------------------
def show_json(drones, title="Synthetic test show", cues=None):
    sw = []
    for d in drones:
        tko = d["takeoff"]
        pts = [[round(t - tko, 3), [round(float(x), 4) for x in p], cps] for t, p, cps in d["points"]]
        sw.append({"type": "generic", "settings": {
            "name": d["name"], "home": d["home"],
            "trajectory": {"version": 1, "points": pts, "takeoffTime": tko},
            "lights": {"version": 1, "data": base64.b64encode(encode_lights(d["lights"])).decode()}}})
    return {"version": 1, "meta": {"title": title},
            "settings": {"cues": {"version": 1, "items": cues or [{"name": "Expand", "time": 17.0},
                                                                   {"name": "Turn", "time": 27.0}]},
                         "validation": {"minDistance": 1.5, "maxVelocityXY": 6}},
            "environment": {"type": "outdoor"}, "swarm": {"drones": sw}}


def write_skyc(path, drones, **kw):
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("show.json", json.dumps(show_json(drones, **kw)))


def write_csv_zip(path, drones, fps=4.0):
    end = max(d["points"][-1][0] for d in drones) + 1.5
    times = np.arange(0.0, end, 1.0 / fps)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for i, d in enumerate(drones):
            buf = io.StringIO()
            buf.write("Time [msec],x [m],y [m],z [m],Red,Green,Blue\n")
            for t in times:
                p = ref_position(d, t)
                c = ref_color(d["lights"], t)
                buf.write(f"{int(round(t * 1000))},{p[0]:.3f},{p[1]:.3f},{p[2]:.3f},"
                          f"{int(round(c[0]))},{int(round(c[1]))},{int(round(c[2]))}\n")
            z.writestr(f"{d['name']}.csv", buf.getvalue())
