"""Import drone shows designed in Blender with Skybrush Studio.

Supported files
  .skyc   Skybrush compiled show (Skybrush Studio for Blender: Export > Skybrush > .skyc).
          A ZIP archive holding show.json with, per drone, a trajectory made of Bezier
          segments and a light program in Skybrush LED bytecode.
  .json   the show.json taken out of a .skyc (plain or gzip compressed).
  .zip    Skybrush CSV export: one CSV per drone with columns time (ms), x, y, z (m),
          red, green, blue (0-255).

Coordinates
  Skybrush / Blender world:  X, Y, Z in metres, Z up, Z = 0 is the ground.
  LangitSwarm show frame:    x right, y away from the audience, z up.
  With the default mapping the Blender Front view (looking along +Y) is the audience
  view, so coordinates are taken over 1:1.  The import settings add a rotation about
  the vertical axis and a horizontal offset to place the show on the field.

Time
  Show time 0 is the start of the Skybrush show clock.  A drone waits on its pad until
  its trajectory starts (Skybrush takeoffTime), so take-offs can be staggered.  If a
  trajectory starts in the air (Skybrush starts trajectories at ~1 m), a vertical climb
  at 1 m/s from the pad is added in front of it; if it ends in the air, a vertical
  landing is added.  If an added climb would start before time 0, the whole show
  (trajectories, lights, cues) is shifted later.

Flying it
  The imported trajectories are flown as MAVLink waypoint missions like compiled shows:
  every drone's path is simplified (synchronised Euclidean distance, default 0.25 m) to
  timed legs flown at constant speed, with the ground station starting each drone at its
  own take-off time.  Timing is approximate (autopilot acceleration); for exact
  playback an onboard trajectory player is the professional option (see README).
"""
from __future__ import annotations

import base64
import gzip
import hashlib
import io
import json
import math
import os
import re
import threading
import zipfile
from collections import OrderedDict
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from .trajectory import Keyframe, Segment

CACHE_VERSION = 3
BIG = 1.0e5            # per-drone time key separation (s); shows must be shorter than BIG / 2
LBIG = 1 << 32         # per-drone light key separation (20 ms units)
GROUND = 0.3           # m: a drone below this height is on the ground
TKO_MIN_ALT = 2.5      # m: missions use TAKEOFF / LAND only below this height (PX4 MIS_TAKEOFF_ALT default)
ADDED_CLIMB_SPEED = 1.0  # m/s for climbs / landings added to trajectories that start / end in the air
SAMPLE_DT = 0.2        # s: sampling step for validation and mission generation
MISSION_MIN_SPEED = 0.1  # m/s: lowest DO_CHANGE_SPEED used (ArduPilot WPNAV_SPEED >= 10 cm/s)
VERT_TOL = 0.3         # m: horizontal drift still counted as a vertical climb / descent
MAX_LISTED = 40


class ShowImportError(ValueError):
    """The file cannot be imported (message is shown to the operator)."""


# =====================================================================================
#  file locations and caches
# =====================================================================================
def imports_dir() -> str:
    d = os.environ.get("LANGIT_HOME") or os.path.join(os.path.expanduser("~"), "LangitSwarm")
    d = os.path.join(d, "imports")
    os.makedirs(d, exist_ok=True)
    return d


def file_sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


_RAW_CACHE: "OrderedDict[str, RawShow]" = OrderedDict()
_RAW_LOCK = threading.Lock()
_SEP_CACHE: "OrderedDict[tuple, dict]" = OrderedDict()


# =====================================================================================
#  raw (untransformed) show
# =====================================================================================
@dataclass
class RawShow:
    """All drones of an imported show in Skybrush coordinates, packed for vectorised
    evaluation.  Points of drone i are pp[pfirst[i]:pfirst[i+1]] at times
    tk[...] - i*BIG; the segment ending at point k is a straight line if cidx[k] < 0,
    otherwise a cubic Bezier with inner control points cc[cidx[k]].  Light keys of drone
    i are lc[lfirst[i]:lfirst[i+1]] at times (lk - i*LBIG) * 20 ms, linearly interpolated
    (a step is two keys at the same time)."""
    n: int
    names: list
    tk: np.ndarray
    pp: np.ndarray
    pfirst: np.ndarray
    cidx: np.ndarray
    cc: np.ndarray
    lk: np.ndarray
    lc: np.ndarray
    lfirst: np.ndarray
    phases: dict          # per-drone arrays: t_lift, t_climb, z_climb, t_start, z_start, t_end, t_land, t_down
    meta: dict
    sha256: str = ""

    @property
    def duration(self) -> float:
        return float(self.meta["duration"])

    # ---- persistence ------------------------------------------------------------------
    def save(self, path: str):
        tmp = path + ".tmp.npz"
        np.savez(tmp, tk=self.tk, pp=self.pp, pfirst=self.pfirst, cidx=self.cidx, cc=self.cc,
                 lk=self.lk, lc=self.lc, lfirst=self.lfirst,
                 **{"ph_" + k: v for k, v in self.phases.items()},
                 meta=np.array(json.dumps({"version": CACHE_VERSION, "names": self.names, "meta": self.meta})))
        os.replace(tmp, path)

    @classmethod
    def load(cls, path: str, sha: str) -> "RawShow | None":
        try:
            with np.load(path, allow_pickle=False) as z:
                m = json.loads(str(z["meta"]))
                if m.get("version") != CACHE_VERSION:
                    return None
                ph = {k[3:]: z[k] for k in z.files if k.startswith("ph_")}
                return cls(n=len(z["pfirst"]) - 1, names=m["names"], tk=z["tk"], pp=z["pp"], pfirst=z["pfirst"],
                           cidx=z["cidx"], cc=z["cc"], lk=z["lk"], lc=z["lc"], lfirst=z["lfirst"],
                           phases=ph, meta=m["meta"], sha256=sha)
        except (OSError, ValueError, KeyError):
            return None


# =====================================================================================
#  Skybrush light program (LED bytecode, libskybrush src/lights/commands.h)
# =====================================================================================
def decode_light_program(buf: bytes, cap20: int):
    """Decode Skybrush LED bytecode into colour keys.

    Durations and WAIT_UNTIL deadlines are unsigned LEB128 varints in 20 ms units.
    SET_* commands change the colour at once and then wait; FADE_* fade linearly from the
    current colour over the duration.  Loops are unrolled (an infinite loop stops at
    cap20).  The initial colour is black.
    Returns (t20 int64 (L,), rgb uint8 (L,3), note or None)."""
    T = [0]
    C = [(0, 0, 0)]
    cur = (0, 0, 0)
    clock = 0
    pos = 0
    n = len(buf)
    loops = []          # [body_start, remaining (None = infinite), clock_at_iteration_start]
    note = None

    def varint():
        nonlocal pos
        v = 0
        shift = 0
        while True:
            b = buf[pos]
            pos += 1
            v |= (b & 0x7F) << shift
            if b < 0x80:
                return v
            shift += 7

    try:
        while pos < n and clock <= cap20:
            op = buf[pos]
            pos += 1
            if op == 0x00:                      # END
                break
            if op == 0x01:                      # NOP
                continue
            if op == 0x02:                      # SLEEP
                clock += varint()
                continue
            if op == 0x03:                      # WAIT_UNTIL
                clock = max(clock, varint())
                continue
            if 0x04 <= op <= 0x0B:
                if op in (0x04, 0x08):
                    col = (buf[pos], buf[pos + 1], buf[pos + 2])
                    pos += 3
                elif op in (0x05, 0x09):
                    g = buf[pos]
                    pos += 1
                    col = (g, g, g)
                elif op in (0x06, 0x0A):
                    col = (0, 0, 0)
                else:
                    col = (255, 255, 255)
                d = varint()
                if op <= 0x07 or d == 0:        # SET_*: step now, then wait
                    T.append(clock)
                    C.append(cur)
                    T.append(clock)
                    C.append(col)
                    clock += d
                else:                           # FADE_*: ramp over d
                    T.append(clock)
                    C.append(cur)
                    clock += d
                    T.append(clock)
                    C.append(col)
                cur = col
                continue
            if op == 0x0C:                      # LOOP_BEGIN
                it = buf[pos]
                pos += 1
                loops.append([pos, it if it > 0 else None, clock])
                if len(loops) > 32:
                    note = "light program loops nested too deep"
                    break
                continue
            if op == 0x0D:                      # LOOP_END
                if not loops:
                    continue
                top = loops[-1]
                if clock == top[2]:             # iteration without duration: skip the rest
                    loops.pop()
                    if top[1] is None:
                        break
                    continue
                if top[1] is None:
                    pos = top[0]
                    top[2] = clock
                    continue
                top[1] -= 1
                if top[1] > 0:
                    pos = top[0]
                    top[2] = clock
                else:
                    loops.pop()
                continue
            note = f"light command 0x{op:02X} is not in the Skybrush LED format - the program stops there, as on the drone"
            break
    except IndexError:
        note = "light program truncated"
    t = np.asarray(T, np.int64)
    c = np.asarray(C, np.uint8).reshape(-1, 3)
    return t, c, note


def _compress_keys(t: np.ndarray, c: np.ndarray):
    """Drop keys that change nothing (duplicates and the middle of constant runs)."""
    if len(t) <= 2:
        return t, c
    keep = np.ones(len(t), bool)
    same_prev = (t[1:] == t[:-1]) & (c[1:] == c[:-1]).all(1)
    keep[1:] &= ~same_prev
    t, c = t[keep], c[keep]
    if len(t) <= 2:
        return t, c
    mid = (c[1:-1] == c[:-2]).all(1) & (c[1:-1] == c[2:]).all(1)
    keep = np.ones(len(t), bool)
    keep[1:-1] = ~mid
    return t[keep], c[keep]


def _light_keys_from_list(data):
    """Studio's uncompiled light list: [[t, [r, g, b], is_fade], ...] (is_fade = fade into
    this colour from the previous key, otherwise a step at t)."""
    T = [0]
    C = [(0, 0, 0)]
    cur = (0, 0, 0)
    for row in data:
        t = int(round(float(row[0]) * 50.0))
        col = tuple(int(max(0, min(255, round(float(v))))) for v in row[1][:3])
        fade = len(row) > 2 and bool(row[2])
        if not fade:
            T.append(t)
            C.append(cur)
        T.append(t)
        C.append(col)
        cur = col
    order = np.argsort(np.asarray(T), kind="stable")
    return np.asarray(T, np.int64)[order], np.asarray(C, np.uint8).reshape(-1, 3)[order]


# =====================================================================================
#  path simplification (synchronised Euclidean distance Ramer-Douglas-Peucker)
# =====================================================================================
def sed_rdp(t: np.ndarray, X: np.ndarray, tol: float) -> np.ndarray:
    """Indices of samples to keep so that linear interpolation IN TIME between kept
    samples stays within tol of every dropped sample (position at the same moment)."""
    m = len(t)
    if m <= 2:
        return np.arange(m)
    keep = np.zeros(m, bool)
    keep[0] = keep[-1] = True
    tol2 = tol * tol
    stack = [(0, m - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        ta, tb = t[a], t[b]
        u = (t[a + 1:b] - ta) / (tb - ta) if tb > ta else np.zeros(b - a - 1)
        d = X[a + 1:b] - (X[a] + u[:, None] * (X[b] - X[a]))
        d2 = (d * d).sum(1)
        k = int(d2.argmax())
        if d2[k] > tol2:
            j = a + 1 + k
            keep[j] = True
            stack.append((a, j))
            stack.append((j, b))
    return np.nonzero(keep)[0]


# =====================================================================================
#  parsing
# =====================================================================================
def _natural_key(s: str):
    return [int(p) if p.isdigit() else p.lower() for p in re.split(r"(\d+)", s)]


def _to_cubic(p0, cps, p1, notes):
    """Inner control points (c1, c2) of the cubic equal to a Bezier of degree len(cps)+1."""
    k = len(cps)
    if k == 1:
        q = np.asarray(cps[0], float)
        return p0 + (2.0 / 3.0) * (q - p0), p1 + (2.0 / 3.0) * (q - p1)
    if k == 2:
        return np.asarray(cps[0], float), np.asarray(cps[1], float)
    notes["high_degree"] = notes.get("high_degree", 0) + 1      # keep end tangents
    f = (k + 1) / 3.0
    return p0 + f * (np.asarray(cps[0], float) - p0), p1 + f * (np.asarray(cps[-1], float) - p1)


def _trajectory_arrays(traj: dict, notes: dict):
    """(times (m,), points (m,3), {point index: (c1, c2)}) in absolute show time."""
    if not traj:
        return None
    ver = int(traj.get("version", 1))
    t_off = float(traj.get("takeoffTime", 0.0) or 0.0)
    pts = traj.get("points")
    ctrl = {}
    if ver == 2 or isinstance(pts, str):
        a = np.frombuffer(base64.b64decode(pts), dtype="<f4").astype(float).reshape(-1, 4)
        t, P = a[:, 0], a[:, 1:4]
    elif ver == 0 or (pts and len(pts[0]) == 4 and not isinstance(pts[0][1], list)):
        a = np.asarray(pts, float).reshape(-1, 4)
        t, P = a[:, 0], a[:, 1:4]
    elif ver == 1:
        if not pts:
            return None
        t = np.fromiter((float(p[0]) for p in pts), float, len(pts))
        P = np.asarray([p[1] for p in pts], float).reshape(-1, 3)
        for k, p in enumerate(pts):
            if k > 0 and len(p) > 2 and p[2]:
                ctrl[k] = _to_cubic(P[k - 1], p[2], P[k], notes)
    else:
        raise ShowImportError(f"unsupported Skybrush trajectory version {ver}")
    if len(t) == 0:
        return None
    t = t + t_off
    # drop zero / negative length steps (keep the later point as a jump)
    good = np.ones(len(t), bool)
    good[1:] = np.diff(t) > 1e-6
    if not good.all():
        jumps = np.nonzero(~good)[0]
        if (np.linalg.norm(P[jumps] - P[jumps - 1], axis=1) > 0.05).any():
            notes["jumps"] = notes.get("jumps", 0) + 1
        idx = np.nonzero(good)[0]
        remap = {int(o): j for j, o in enumerate(idx)}
        for k in jumps:                                    # the jump target replaces its predecessor
            P[k - 1] = P[k]
        t, P = t[idx], P[idx]
        ctrl = {remap[k]: v for k, v in ctrl.items() if k in remap and remap[k] > 0}
    return t, P, ctrl


def _finish_drone(t, P, ctrl, home, notes):
    """Add a vertical climb in front of a trajectory that starts in the air and a vertical
    landing after one that ends in the air.  Returns (t, P, ctrl)."""
    t = np.asarray(t, float)
    P = np.asarray(P, float)
    ground = 0.0
    if home is not None and abs(float(home[2])) > 0.05:
        notes["home_z"] = notes.get("home_z", 0) + 1
    if P[0, 2] > ground + GROUND:
        start = np.array([P[0, 0], P[0, 1], ground])
        if home is not None and math.hypot(home[0] - P[0, 0], home[1] - P[0, 1]) <= 1.0:
            start[:2] = home[:2]
        dur = max(0.5, (P[0, 2] - ground) / ADDED_CLIMB_SPEED)
        t = np.concatenate([[t[0] - dur], t])
        P = np.vstack([start, P])
        ctrl = {k + 1: v for k, v in ctrl.items()}
        notes["added_climb"] = notes.get("added_climb", 0) + 1
    if P[-1, 2] > ground + GROUND:
        dur = max(0.5, (P[-1, 2] - ground) / ADDED_CLIMB_SPEED)
        t = np.concatenate([t, [t[-1] + dur]])
        P = np.vstack([P, [P[-1, 0], P[-1, 1], ground]])
        notes["added_landing"] = notes.get("added_landing", 0) + 1
    if len(t) == 1:
        t = np.concatenate([t, [t[0] + 1e-3]])
        P = np.vstack([P, P])
    return t, P, ctrl


def _phases(t, P, ctrl):
    """Per-drone take-off / landing times from the trajectory points."""
    z = P[:, 2]
    m = len(t)
    above = np.nonzero(z > GROUND)[0]
    inf = math.inf
    if len(above) == 0:
        return dict(t_lift=inf, t_climb=inf, z_climb=0.0, t_start=inf, z_start=0.0, t_end=inf,
                    t_land=inf, t_down=inf)
    k = int(above[0])
    k0 = max(k - 1, 0)
    t_lift = float(t[k0])
    pad = P[k0, :2]

    def near(q, ref):
        return math.hypot(float(q[0] - ref[0]), float(q[1] - ref[1])) <= VERT_TOL

    def vertical(j, ref, rising):
        """segment j-1 -> j is a vertical climb (rising) / descent above ref"""
        if not (near(P[j, :2], ref) and near(P[j - 1, :2], ref)):
            return False
        if rising and not z[j] > z[j - 1] + 0.02:
            return False
        if not rising and not z[j - 1] > z[j] + 0.02:
            return False
        c = ctrl.get(j)
        if c is not None and not (near(c[0], ref) and near(c[1], ref)):
            return False
        return True

    j = k0 + 1
    while j < m and vertical(j, pad, True):
        j += 1
    ce = j - 1
    t_climb, z_climb = float(t[ce]), float(z[ce])
    kl = int(above[-1])
    kd = min(kl + 1, m - 1)
    t_down = float(t[kd])
    fin = P[kd, :2]
    j = kd
    while j - 1 > ce and vertical(j, fin, False):
        j -= 1
    t_land = float(t[j])
    return dict(t_lift=t_lift, t_climb=t_climb, z_climb=z_climb, t_start=t_climb, z_start=z_climb,
                t_end=t_land, t_land=t_land, t_down=t_down)


class _Packer:
    def __init__(self):
        self.T, self.P, self.CI, self.CC = [], [], [], []
        self.LT, self.LC = [], []
        self.nc = 0
        self.ph = []

    def add(self, t, P, ctrl, lt, lc):
        self.T.append(np.asarray(t, float))
        self.P.append(np.asarray(P, np.float32))
        ci = np.full(len(t), -1, np.int32)
        for k in sorted(ctrl):
            ci[k] = self.nc
            self.CC.append(np.stack(ctrl[k]).astype(np.float32))
            self.nc += 1
        self.CI.append(ci)
        self.LT.append(np.asarray(lt, np.int64))
        self.LC.append(np.asarray(lc, np.uint8).reshape(-1, 3))
        self.ph.append(_phases(t, np.asarray(P, float), ctrl))

    def build(self, names, meta, sha="") -> RawShow:
        n = len(self.T)
        cnt = np.array([len(x) for x in self.T])
        pfirst = np.concatenate([[0], np.cumsum(cnt)]).astype(np.int64)
        off = np.repeat(np.arange(n) * BIG, cnt)
        tk = np.concatenate(self.T) + off
        lcnt = np.array([len(x) for x in self.LT])
        lfirst = np.concatenate([[0], np.cumsum(lcnt)]).astype(np.int64)
        lk = np.concatenate(self.LT) + np.repeat(np.arange(n, dtype=np.int64) * LBIG, lcnt)
        keys = ("t_lift", "t_climb", "z_climb", "t_start", "z_start", "t_end", "t_land", "t_down")
        phases = {k: np.array([p[k] for p in self.ph], float) for k in keys}
        return RawShow(n=n, names=names, tk=tk, pp=np.concatenate(self.P), pfirst=pfirst,
                       cidx=np.concatenate(self.CI),
                       cc=np.stack(self.CC) if self.CC else np.zeros((0, 2, 3), np.float32),
                       lk=lk, lc=np.concatenate(self.LC), lfirst=lfirst, phases=phases, meta=meta, sha256=sha)


def _parse_show_dict(d: dict, fmt: str, progress=None):
    if "swarm" not in d and isinstance(d.get("input"), dict):      # Studio render request payload
        d = d["input"].get("data", d)
    drones = (d.get("swarm") or {}).get("drones")
    if not drones:
        raise ShowImportError("the file contains no drones (expected swarm.drones in show.json)")
    if len(drones) > 10000:
        raise ShowImportError(f"{len(drones)} drones - LangitSwarm supports up to 10000")
    notes: dict = {}
    per = []
    for i, dr in enumerate(drones):
        st = dr.get("settings", dr) if isinstance(dr, dict) else {}
        tr = _trajectory_arrays(st.get("trajectory"), notes)
        home = st.get("home")
        home = np.asarray(home, float)[:3] if home is not None and len(home) >= 3 else None
        if tr is None:
            if home is None:
                raise ShowImportError(f"drone {i + 1} has neither a trajectory nor a home position")
            tr = (np.array([0.0]), home[None, :].copy(), {})
            notes["no_trajectory"] = notes.get("no_trajectory", 0) + 1
        per.append((st.get("name") or f"Drone {i + 1}", tr, home, st.get("lights")))
        if st.get("yawControl"):
            notes["yaw"] = notes.get("yaw", 0) + 1
        if st.get("pyro"):
            notes["pyro"] = notes.get("pyro", 0) + 1
        if progress and i % 200 == 0:
            progress(0.05 + 0.25 * i / len(drones), f"Reading trajectories {i}/{len(drones)}")
    finished = [(nm, _finish_drone(*tr, home, notes), lights) for nm, tr, home, lights in per]
    t_first = min(float(f[1][0][0]) for f in finished)
    shift = 0.0 if t_first >= 0 else math.ceil(-t_first * 50.0) / 50.0
    end = max(float(f[1][0][-1]) for f in finished) + shift
    cap20 = int(end * 50) + 500
    pk = _Packer()
    light_notes: dict = {}
    for i, (nm, (t, P, ctrl), lights) in enumerate(finished):
        lt, lc = _decode_lights(lights, cap20, light_notes)
        if shift:
            lt = lt + int(round(shift * 50))
        lt, lc = _compress_keys(lt, lc)
        pk.add(t + shift, P, ctrl, lt, lc)
        if progress and i % 100 == 0:
            progress(0.3 + 0.6 * i / len(finished), f"Decoding light programs {i}/{len(finished)}")
    settings = d.get("settings") or {}
    cues = []
    for c in ((settings.get("cues") or {}).get("items") or []):
        try:
            cues.append([round(float(c["time"]) + shift, 3), str(c.get("name", ""))])
        except (KeyError, TypeError, ValueError):
            pass
    meta_in = d.get("meta") or {}
    segs = {}
    for k, v in (meta_in.get("segments") or {}).items():
        try:
            segs[str(k)] = [float(v[0]) + shift, float(v[1]) + shift]
        except (TypeError, ValueError, IndexError):
            pass
    media = d.get("media") or {}
    loc = (d.get("environment") or {}).get("location") or {}
    location = None
    try:
        if loc.get("origin"):
            o = loc["origin"]
            location = {"lat": o[0] / 1e7, "lon": o[1] / 1e7, "amsl": (o[2] / 1e3) if len(o) > 2 else None,
                        "orientation_deg": float(loc.get("orientation", 0.0))}
    except (TypeError, ValueError, IndexError, AttributeError):
        location = None
    meta = {
        "format": fmt, "title": str(meta_in.get("title") or ""), "drones": len(finished),
        "duration": round(end, 3), "time_shift": shift, "cues": sorted(cues), "segments": segs,
        "design_limits": settings.get("validation") or {},
        "has_audio": bool(media.get("audio")),
        "location": location,
        "notes": _notes_text(notes, light_notes, shift, len(finished)),
    }
    if location:
        meta["notes"].append(f"the file carries a real-world location ({location['lat']:.6f}, {location['lon']:.6f}, "
                             f"X axis {location['orientation_deg']:.0f} deg); LangitSwarm places the show with the "
                             f"origin in Safety & field settings")
    names = [f[0] for f in finished]
    return pk, names, meta


def _decode_lights(lights, cap20, light_notes):
    if not lights:
        light_notes["none"] = light_notes.get("none", 0) + 1
        return np.zeros(1, np.int64), np.zeros((1, 3), np.uint8)
    data = lights.get("data") if isinstance(lights, dict) else None
    if isinstance(data, str):
        t, c, note = decode_light_program(base64.b64decode(data), cap20)
        if note:
            light_notes[note] = light_notes.get(note, 0) + 1
        return t, c
    if isinstance(data, list):
        return _light_keys_from_list(data)
    light_notes["none"] = light_notes.get("none", 0) + 1
    return np.zeros(1, np.int64), np.zeros((1, 3), np.uint8)


def _notes_text(notes, light_notes, shift, n):
    out = []
    if notes.get("added_climb"):
        out.append(f"{notes['added_climb']} drones start their trajectory in the air: a {ADDED_CLIMB_SPEED:.0f} m/s "
                   f"vertical climb from the pad was added")
    if notes.get("added_landing"):
        out.append(f"{notes['added_landing']} drones end their trajectory in the air: a {ADDED_CLIMB_SPEED:.0f} m/s "
                   f"vertical landing was added")
    if shift:
        out.append(f"show time shifted by +{shift:.2f} s so the added climbs start at or after 0")
    if notes.get("no_trajectory"):
        out.append(f"{notes['no_trajectory']} drones have no trajectory and stay on their pads")
    if notes.get("jumps"):
        out.append(f"{notes['jumps']} drones have instant position jumps in their trajectory")
    if notes.get("high_degree"):
        out.append(f"{notes['high_degree']} Bezier segments above cubic were approximated by cubics")
    if notes.get("home_z"):
        out.append(f"{notes['home_z']} drones have a home height other than 0 - the ground is taken as z = 0")
    if notes.get("yaw"):
        out.append(f"{notes['yaw']} drones have yaw setpoints - ignored (drones keep their heading)")
    if notes.get("pyro"):
        out.append(f"{notes['pyro']} drones have pyro programs - ignored (LangitSwarm does not fire pyro)")
    if light_notes.get("none"):
        out.append(f"{light_notes['none']} drones have no light program (dark)")
    for k, v in light_notes.items():
        if k != "none":
            out.append(f"{v} drones: {k}")
    return out


def _read_csv_text(text: str, name: str):
    lines = [ln for ln in text.splitlines() if ln.strip()]
    if not lines:
        raise ShowImportError(f"{name} is empty")
    first = lines[0]
    sep = ";" if first.count(";") > first.count(",") else ("\t" if "\t" in first else ",")
    header = None
    if re.search(r"[A-Za-z]", first):
        header = [h.strip().strip('"').lower() for h in first.split(sep)]
        lines = lines[1:]
    try:
        a = np.loadtxt(io.StringIO("\n".join(lines)), delimiter=sep, ndmin=2)
    except ValueError as e:
        raise ShowImportError(f"{name}: cannot read numbers ({e})") from e
    col = {"t": 0, "x": 1, "y": 2, "z": 3, "r": 4, "g": 5, "b": 6}
    ms = None
    if header:
        found = {}
        for j, h in enumerate(header):
            hh = re.sub(r"[^a-z]", "", h)
            if "time" in hh or hh in ("t", "ms", "msec", "sec", "s"):
                found.setdefault("t", j)
                if "msec" in h or "[ms]" in h or "_ms" in h or "(ms)" in h or hh in ("timems", "ms", "msec"):
                    ms = True
                elif "sec" in h or "[s]" in h or "(s)" in h:
                    ms = False
            elif hh.startswith("x"):
                found.setdefault("x", j)
            elif hh.startswith("y"):
                found.setdefault("y", j)
            elif hh.startswith("z"):
                found.setdefault("z", j)
            elif hh.startswith("red") or hh == "r":
                found.setdefault("r", j)
            elif hh.startswith("green") or hh == "g":
                found.setdefault("g", j)
            elif hh.startswith("blue") or hh == "b":
                found.setdefault("b", j)
        if all(k in found for k in ("t", "x", "y", "z")):
            col.update(found)
    if a.shape[1] < 4:
        raise ShowImportError(f"{name}: expected at least time, x, y, z columns")
    t = a[:, col["t"]].astype(float)
    if ms is None:
        steps = np.diff(t)
        ms = bool(len(steps) and np.median(steps) >= 5.0) or (t.max(initial=0) > 7200)
    if ms:
        t = t / 1000.0
    P = a[:, [col["x"], col["y"], col["z"]]].astype(float)
    if a.shape[1] >= 7 and max(col["r"], col["g"], col["b"]) < a.shape[1]:
        C = a[:, [col["r"], col["g"], col["b"]]].astype(float)
        if C.max(initial=0) <= 1.0:
            C = C * 255.0
    else:
        C = np.full((len(t), 3), 255.0)
    order = np.argsort(t, kind="stable")
    return t[order], P[order], np.clip(C[order], 0, 255)


def _parse_csv_zip(zf: zipfile.ZipFile, progress=None):
    names = sorted([n for n in zf.namelist() if n.lower().endswith(".csv") and "__MACOSX" not in n
                    and not os.path.basename(n).startswith(".")], key=_natural_key)
    if not names:
        raise ShowImportError("the ZIP holds neither show.json nor CSV files")
    if len(names) > 10000:
        raise ShowImportError(f"{len(names)} drones - LangitSwarm supports up to 10000")
    notes: dict = {}
    per = []
    for i, nm in enumerate(names):
        t, P, C = _read_csv_text(zf.read(nm).decode("utf-8-sig", errors="replace"), nm)
        if len(t) == 0:
            raise ShowImportError(f"{nm} has no rows")
        keep = sed_rdp(t, P, 0.02)
        tp, Pp = t[keep], P[keep]
        good = np.ones(len(tp), bool)
        good[1:] = np.diff(tp) > 1e-6
        tp, Pp = tp[good], Pp[good]
        kc = sed_rdp(t, C, 2.0)
        lt = np.round(t[kc] * 50.0).astype(np.int64)
        lc = np.round(C[kc]).astype(np.uint8)
        stem = os.path.splitext(os.path.basename(nm))[0]
        per.append((stem, _finish_drone(tp, Pp, {}, None, notes), lt, lc))
        if progress and i % 50 == 0:
            progress(0.05 + 0.85 * i / len(names), f"Reading CSV {i}/{len(names)}")
    t_first = min(float(f[1][0][0]) for f in per)
    shift = 0.0 if t_first >= 0 else math.ceil(-t_first * 50.0) / 50.0
    end = max(float(f[1][0][-1]) for f in per) + shift
    pk = _Packer()
    for nm, (t, P, ctrl), lt, lc in per:
        lt = lt + int(round(shift * 50))
        if lt[0] > 0:                       # colour before the first row: the first row's colour
            lt = np.concatenate([[0], lt])
            lc = np.vstack([lc[:1], lc])
        lt, lc = _compress_keys(lt, lc)
        pk.add(t + shift, P, ctrl, lt, lc)
    meta = {"format": "csv", "title": "", "drones": len(per), "duration": round(end, 3), "time_shift": shift,
            "cues": [], "segments": {}, "design_limits": {}, "has_audio": False,
            "notes": _notes_text(notes, {}, shift, len(per))}
    return pk, [p[0] for p in per], meta


def parse_file(path: str, progress=None, sha: str = "") -> RawShow:
    """Parse a .skyc / show.json / CSV .zip file into a RawShow."""
    with open(path, "rb") as f:
        head = f.read(4)
    if head[:2] == b"PK":
        try:
            zf = zipfile.ZipFile(path)
        except zipfile.BadZipFile as e:
            raise ShowImportError(f"not a valid ZIP / .skyc file ({e})") from e
        with zf:
            jsons = [n for n in zf.namelist() if n.lower().endswith(".json") and "__MACOSX" not in n]
            jsons.sort(key=lambda n: (os.path.basename(n).lower() != "show.json", len(n)))
            for n in jsons:
                try:
                    d = json.loads(zf.read(n).decode("utf-8"))
                except (ValueError, UnicodeDecodeError):
                    continue
                if isinstance(d, dict) and ("swarm" in d or "input" in d):
                    if progress:
                        progress(0.05, "Reading show.json")
                    pk, names, meta = _parse_show_dict(d, "skyc", progress)
                    break
            else:
                pk, names, meta = _parse_csv_zip(zf, progress)
    else:
        with open(path, "rb") as f:
            raw = f.read()
        if raw[:2] == b"\x1f\x8b":
            raw = gzip.decompress(raw)
        try:
            d = json.loads(raw.decode("utf-8-sig"))
        except (ValueError, UnicodeDecodeError) as e:
            raise ShowImportError("not a Skybrush .skyc, show.json or CSV .zip file") from e
        if not isinstance(d, dict):
            raise ShowImportError("show.json must be a JSON object")
        pk, names, meta = _parse_show_dict(d, "json", progress)
    if meta["duration"] >= BIG / 2:
        raise ShowImportError("show is too long")
    raw = pk.build(names, meta, sha)
    if progress:
        progress(0.95, "Indexing")
    return raw


def load_raw(path: str, sha: str | None = None, progress=None) -> RawShow:
    """Parsed show for a stored import file (memory cache, then disk cache, then parse)."""
    if not os.path.isfile(path):
        raise ShowImportError(f"imported show file not found: {path} - import the show again")
    sha = sha or file_sha256(path)
    with _RAW_LOCK:
        if sha in _RAW_CACHE:
            _RAW_CACHE.move_to_end(sha)
            return _RAW_CACHE[sha]
    cache = os.path.join(imports_dir(), f"{sha}.npz")
    raw = RawShow.load(cache, sha) if os.path.isfile(cache) else None
    if raw is None:
        raw = parse_file(path, progress, sha)
        try:
            raw.save(cache)
        except OSError:
            pass
    with _RAW_LOCK:
        _RAW_CACHE[sha] = raw
        while len(_RAW_CACHE) > 2:
            _RAW_CACHE.popitem(last=False)
    return raw


# =====================================================================================
#  imported show (same interface as CompiledShow)
# =====================================================================================
def _gather(first: np.ndarray, sel: np.ndarray):
    lens = first[sel + 1] - first[sel]
    nf = np.concatenate([[0], np.cumsum(lens)]).astype(np.int64)
    idx = np.repeat(first[sel] - nf[:-1], lens) + np.arange(nf[-1])
    return idx, nf, lens


class ImportedShow:
    imported = True

    def __init__(self, raw: RawShow, settings: dict, name: str = "", rotate_deg: float = 0.0,
                 offset=(0.0, 0.0), tolerance: float = 0.25, sel: np.ndarray | None = None):
        self.raw = raw
        self.settings = settings
        self.name = name or raw.meta.get("title") or "Imported show"
        self.rotate_deg = float(rotate_deg)
        self.offset = np.array([float(offset[0]), float(offset[1])])
        self.tolerance = max(0.05, float(tolerance))
        self.sel = np.arange(raw.n) if sel is None else np.asarray(sel, np.int64)
        self.is_subset = sel is not None
        n = len(self.sel)
        self.n = n
        self.idx = np.arange(n)
        if self.is_subset:
            ip, self.pfirst, plen = _gather(raw.pfirst, self.sel)
            self.tk = raw.tk[ip] + np.repeat((np.arange(n) - self.sel) * BIG, plen)
            pp = raw.pp[ip]
            self.cidx = raw.cidx[ip]
            il, self.lfirst, llen = _gather(raw.lfirst, self.sel)
            self.lk = raw.lk[il] + np.repeat((np.arange(n, dtype=np.int64) - self.sel) * LBIG, llen)
            self.lc = raw.lc[il]
        else:
            self.pfirst, self.tk, pp, self.cidx = raw.pfirst, raw.tk, raw.pp, raw.cidx
            self.lfirst, self.lk, self.lc = raw.lfirst, raw.lk, raw.lc
        R, off = transform_matrix(raw, self.rotate_deg, self.offset)
        ident = abs(self.rotate_deg) < 1e-12 and not self.offset.any()
        self.pp = pp if ident else (pp @ R.T + off).astype(np.float32)
        self.cc = raw.cc if ident else (raw.cc @ R.T + off).astype(np.float32)
        self.ph = {k: v[self.sel] for k, v in raw.phases.items()}
        self.duration = float(raw.duration)
        self.start_positions = self.pp[self.pfirst[:-1]].astype(float)
        self.start_positions[:, 2] = 0.0
        self.final_positions = self.pp[self.pfirst[1:] - 1].astype(float)
        self.final_positions[:, 2] = 0.0
        flies = np.isfinite(self.ph["t_lift"])
        self.flies = flies
        self.takeoff_offsets = np.where(flies, self.ph["t_lift"], np.inf)
        act = np.ones(n, bool)
        self.keyframes = [Keyframe(self.start_positions, act, None, {}, "Pads", -1),
                          Keyframe(self.final_positions, act, None, {}, "Landed", -1)]
        self.segments = self._scenes()
        self.formation_start = {}
        self.formation_names = []
        self.landing_slots = None
        self.warnings = list(raw.meta.get("notes", []))
        if not flies.all():
            self.warnings.append(f"{int((~flies).sum())} drones never leave the ground")
        self._wp: dict = {}
        self.mission_info: dict = {}

    # ---- structure ------------------------------------------------------------------------
    def _scenes(self) -> list:
        ph, fl = self.ph, self.flies
        T = self.duration
        meta = self.raw.meta
        if fl.any():
            to0 = float(ph["t_lift"][fl].min())
            to1 = float(ph["t_climb"][fl].max())
            la0 = float(ph["t_land"][fl].min())
            la1 = float(ph["t_down"][fl].max())
        else:
            to0 = to1 = la0 = la1 = 0.0
        sg = meta.get("segments") or {}
        if "takeoff" in sg:
            to0, to1 = sg["takeoff"]
        if "landing" in sg:
            la0, la1 = sg["landing"]
        to0 = min(max(0.0, to0), T)
        to1 = min(max(to0, to1), T)
        la0 = min(max(to1, la0), T)
        la1 = T
        out = []
        if to0 > 0.05:
            out.append(Segment("hold", 0.0, to0, 0, 0, -1, "On the pads"))
        out.append(Segment("takeoff", to0, to1, 0, 1, -1, "Take-off"))
        marks = [(t, nm) for t, nm in meta.get("cues", []) if to1 + 0.5 < t < la0 - 0.5]
        bounds = [to1] + [t for t, _ in marks] + [la0]
        labels = ["Show"] + [nm or "Cue" for _, nm in marks]
        for a, b, lb in zip(bounds[:-1], bounds[1:], labels):
            if b - a > 1e-3:
                out.append(Segment("path", a, b, 0, 1, -1, lb))
        out.append(Segment("landing", la0, max(la1, la0 + 1e-3), 0, 1, -1, "Landing"))
        return out

    def subset(self, indices) -> "ImportedShow":
        idx = np.asarray(indices, np.int64)
        return ImportedShow(self.raw, self.settings, self.name, self.rotate_deg, self.offset, self.tolerance,
                            sel=self.sel[idx])

    def timeline_summary(self) -> list:
        return [{"kind": s.kind, "t0": round(s.t0, 2), "t1": round(s.t1, 2), "formation": -1, "label": s.label}
                for s in self.segments]

    def seg_index(self, t: float) -> int:
        for k, s in enumerate(self.segments):
            if t < s.t1:
                return k
        return len(self.segments) - 1

    # ---- evaluation ------------------------------------------------------------------------
    def eval_positions(self, times, drones) -> np.ndarray:
        times = np.asarray(times, float)
        drones = np.asarray(drones, np.int64)
        q = np.clip(times, -1.0, self.duration + 1.0) + drones * BIG
        j = np.searchsorted(self.tk, q, side="right")
        j = np.clip(j, self.pfirst[drones] + 1, self.pfirst[drones + 1] - 1)
        j0 = j - 1
        t0 = self.tk[j0]
        tau = np.clip((q - t0) / np.maximum(self.tk[j] - t0, 1e-9), 0.0, 1.0)
        A = self.pp[j0].astype(float)
        B = self.pp[j].astype(float)
        P = A + tau[:, None] * (B - A)
        ci = self.cidx[j]
        cur = ci >= 0
        if cur.any():
            u = tau[cur][:, None]
            v = 1.0 - u
            C = self.cc[ci[cur]].astype(float)
            P[cur] = v ** 3 * A[cur] + 3 * v * v * u * C[:, 0] + 3 * v * u * u * C[:, 1] + u ** 3 * B[cur]
        return P

    def eval_colors(self, times, drones) -> np.ndarray:
        """float32 (m,3) in 0..1"""
        times = np.asarray(times, float)
        drones = np.asarray(drones, np.int64)
        q = np.clip(times, -0.1, self.duration + 1.0) * 50.0
        base = drones * LBIG
        qi = np.floor(q).astype(np.int64) + base
        j = np.searchsorted(self.lk, qi, side="right") - 1
        lo = self.lfirst[drones]
        hi = self.lfirst[drones + 1] - 1
        j = np.clip(j, lo, hi)
        j1 = np.minimum(j + 1, hi)
        k0 = (self.lk[j] - base).astype(float)
        k1 = (self.lk[j1] - base).astype(float)
        w = np.where(k1 > k0, np.clip((q - k0) / np.maximum(k1 - k0, 1e-9), 0.0, 1.0), 0.0)
        c = self.lc[j].astype(np.float32) * (1.0 - w[:, None]) + self.lc[j1].astype(np.float32) * w[:, None]
        return (c / 255.0).astype(np.float32)

    def positions(self, t: float) -> np.ndarray:
        return self.eval_positions(np.full(self.n, float(t)), self.idx)

    def positions_for(self, t: float, slots: np.ndarray) -> np.ndarray:
        slots = np.asarray(slots, np.int64)
        return self.eval_positions(np.full(len(slots), float(t)), slots)

    def colors(self, t: float) -> np.ndarray:
        return self.eval_colors(np.full(self.n, float(t)), self.idx)

    def colors_u8(self, t: float) -> np.ndarray:
        return (self.colors(t) * 255.0 + 0.5).astype(np.uint8)

    def drone_light_frames(self, i: int, fps: int) -> np.ndarray:
        F = int(math.ceil(self.duration * fps)) + 1
        c = self.eval_colors(np.arange(F) / float(fps), np.full(F, int(i)))
        return (c * 255.0 + 0.5).astype(np.uint8)

    def path_deviation(self, t: float, slots: np.ndarray, P: np.ndarray, window: float = 5.0) -> np.ndarray:
        """Distance of each drone from its planned path within +-window s of t (tolerates the
        timing drift of drones flying waypoint missions)."""
        slots = np.asarray(slots, np.int64)
        m = len(slots)
        if m == 0:
            return np.zeros(0)
        offs = np.arange(-window, window + 1e-9, 1.0)       # chords between 1 s samples: < 0.4 m sag
        k = len(offs)
        Q = self.eval_positions((t + offs)[None, :].repeat(m, 0).ravel(), np.repeat(slots, k)).reshape(m, k, 3)
        A, B = Q[:, :-1], Q[:, 1:]
        AB = B - A
        L2 = (AB * AB).sum(-1)
        R = np.asarray(P, float)[:, None, :] - A
        u = np.clip((R * AB).sum(-1) / np.maximum(L2, 1e-9), 0.0, 1.0)
        d = np.linalg.norm(R - u[..., None] * AB, axis=-1)
        return d.min(1)

    # ---- missions --------------------------------------------------------------------------
    def _mission_window(self, i: int):
        """(t_start, z_start, t_end): the part of drone i's path above TKO_MIN_ALT, flown as
        timed waypoints.  Below it the autopilot's TAKEOFF / LAND take over, so the planned
        climb and descent speeds are kept everywhere except the last metres."""
        t0, t1 = float(self.ph["t_lift"][i]), float(self.ph["t_down"][i])
        tt = np.arange(t0, t1 + 1e-9, 0.05)
        z = self.eval_positions(tt, np.full(len(tt), int(i)))[:, 2]
        up = np.nonzero(z >= TKO_MIN_ALT)[0]
        if len(up) == 0:                      # never reaches TKO_MIN_ALT: take off, hover at the top, land
            k = int(z.argmax())
            return float(tt[k]), TKO_MIN_ALT, float(tt[k])
        k0, k1 = int(up[0]), int(up[-1])
        ts = float(tt[k0])
        if k0 > 0 and z[k0] > z[k0 - 1]:      # interpolate the crossing
            ts = float(tt[k0 - 1] + (TKO_MIN_ALT - z[k0 - 1]) / (z[k0] - z[k0 - 1]) * (tt[k0] - tt[k0 - 1]))
        te = float(tt[k1])
        if k1 + 1 < len(tt) and z[k1] > z[k1 + 1]:
            te = float(tt[k1] + (z[k1] - TKO_MIN_ALT) / (z[k1] - z[k1 + 1]) * (tt[k1 + 1] - tt[k1]))
        return ts, TKO_MIN_ALT, max(te, ts)

    def waypoints(self, i: int, tol: float | None = None):
        """Timed waypoints (times (k,), positions (k,3)) of drone i's airborne path."""
        tol = self.tolerance if tol is None else tol
        key = (int(i), round(tol, 4))
        if key in self._wp:
            return self._wp[key]
        if not self.flies[i]:
            res = (np.zeros(0), np.zeros((0, 3)), 0.0)
        else:
            ts, zs, te = self._mission_window(i)
            a, b = int(self.pfirst[i]), int(self.pfirst[i + 1])
            knots = self.tk[a:b] - i * BIG                    # exact corners / hover ends of the path
            knots = knots[(knots > ts + 1e-3) & (knots < te - 1e-3)]
            grid = np.arange(ts, te, SAMPLE_DT)
            if len(knots) and len(grid):
                near = np.abs(grid[:, None] - knots[None, :]).min(1) < 0.02 if len(grid) * len(knots) < 4e6 \
                    else np.zeros(len(grid), bool)
                grid = grid[~near | (grid == ts)]
            tt = np.union1d(grid, knots)
            tt = np.append(tt, te) if (len(tt) == 0 or te - tt[-1] > 1e-6) else tt
            P = self.eval_positions(tt, np.full(len(tt), int(i)))
            keep = sed_rdp(tt, P, tol)
            res = (tt[keep], P[keep], zs)
        self._wp[key] = res
        return res

    def _items(self, i, origin, sync_mode, start_utc, tol, pad):
        from .mission import (CMD_CHANGE_SPEED, CMD_DELAY, CMD_LAND, CMD_TAKEOFF, CMD_WAYPOINT, MAV_FRAME_GLOBAL,
                              _item, _utc_parts)
        MIN_SPEED = MISSION_MIN_SPEED

        def wgs(p):
            a, b, c = origin.show_to_wgs84(np.asarray(p, float))
            return float(a), float(b), float(c)

        lat, lon, _ = wgs(pad)
        items = [_item(CMD_WAYPOINT, lat, lon, origin.alt_amsl, frame=MAV_FRAME_GLOBAL)]
        if not self.flies[i]:
            return items
        T, W, z_to = self.waypoints(i, tol)
        s = self.settings
        use_delay = sync_mode == "utc_delay" and start_utc is not None
        t_lift = float(self.ph["t_lift"][i])
        climb_v = float(np.clip(z_to / max(T[0] - t_lift, 0.1), 0.5, float(s["max_speed_up"])))
        items.append(_item(CMD_CHANGE_SPEED, p1=2, p2=climb_v, p3=-1))
        la, lo, _ = wgs(pad)
        items.append(_item(CMD_TAKEOFF, la, lo, z_to))
        la, lo, al = wgs(W[0])
        items.append(_item(CMD_WAYPOINT, la, lo, al))
        last_wp = items[-1]
        cur = {1: None, 2: None, 3: None}
        hover = 0.0          # accumulated hover time at the last waypoint
        carry = 0.0          # short hovers folded into the next leg

        def set_speed(kind, v):
            v = max(MIN_SPEED, float(v))
            c = cur[kind]
            if c is None or abs(v - c) > 0.02 * c:
                items.append(_item(CMD_CHANGE_SPEED, p1=kind, p2=v, p3=-1))
                cur[kind] = v

        def flush_hover(t_end):
            nonlocal hover, carry
            if hover >= 1.0:
                if use_delay:
                    h, m_, sec = _utc_parts(start_utc + t_end)
                    items.append(_item(CMD_DELAY, p1=-1, p2=h, p3=m_, p4=sec))
                else:
                    last_wp["p1"] = float(round(hover, 2))
            elif hover > 0:
                carry += hover
            hover = 0.0

        for k in range(1, len(T)):
            dt = float(T[k] - T[k - 1])
            d = W[k] - W[k - 1]
            if float(np.linalg.norm(d)) < 0.05:
                hover += dt
                continue
            flush_hover(float(T[k - 1]))
            dt += carry
            carry = 0.0
            dh = math.hypot(float(d[0]), float(d[1]))
            dz = float(d[2])
            set_speed(1, dh / dt)
            if abs(dz) > 0.3:
                set_speed(2 if dz > 0 else 3, abs(dz) / dt)
            la, lo, al = wgs(W[k])
            items.append(_item(CMD_WAYPOINT, la, lo, al))
            last_wp = items[-1]
            # time the leg really takes with the speeds in effect (minimum speeds make slow
            # legs short): wait out the rest at the waypoint so later legs stay on time
            vz = cur[2 if dz > 0 else 3] or 2.5
            slack = dt - max(dh / (cur[1] or 5.0), abs(dz) / vz)
            if slack > 0.02:
                hover += slack
        flush_hover(float(T[-1]))
        fin = self.final_positions[i]
        la, lo, _ = wgs(fin)
        items.append(_item(CMD_LAND, la, lo, 0.0))
        return items

    def mission_items(self, i: int, origin, sync_mode: str = "hold_time", start_utc: float | None = None,
                      start_slot_xyz=None) -> list:
        pad = self.start_positions[i] if start_slot_xyz is None else np.asarray(start_slot_xyz, float)
        tol = self.mission_info.get("tol", {}).get(int(i), self.tolerance)
        items = self._items(i, origin, sync_mode, start_utc, tol, pad)
        limit = int(self.settings.get("mission_max_items", 700))
        tries = 0
        while len(items) > limit and tries < 6:
            tol *= 1.6
            tries += 1
            items = self._items(i, origin, sync_mode, start_utc, tol, pad)
        if tries:
            self.mission_info.setdefault("tol", {})[int(i)] = tol
        for n, it in enumerate(items):
            it["seq"] = n
        return items

    def prepare_missions(self, origin, sync_mode="hold_time", start_utc=None, progress=None) -> dict:
        """Build every drone's mission once (slow part: path simplification) and report sizes."""
        sizes = np.zeros(self.n, int)
        for i in range(self.n):
            sizes[i] = len(self.mission_items(i, origin, sync_mode, start_utc))
            if progress and i % 100 == 0:
                progress(i / max(1, self.n), f"Building missions {i}/{self.n}")
        tols = self.mission_info.get("tol", {})
        self.mission_info.update(max_items=int(sizes.max(initial=0)), mean_items=float(sizes.mean()) if self.n else 0,
                                 loosened=len(tols), max_tol=round(max(tols.values(), default=self.tolerance), 3))
        return self.mission_info

    # ---- export ----------------------------------------------------------------------------
    def trajectory_csv(self, i: int, origin) -> str:
        a, b = int(self.pfirst[i]), int(self.pfirst[i + 1])
        t = self.tk[a:b] - i * BIG
        P = self.pp[a:b].astype(float)
        lines = ["# LangitSwarm imported trajectory v1 - show frame metres (x right, y away from audience, z up)",
                 "# each row is a cubic Bezier segment: P(u) = (1-u)^3 P0 + 3(1-u)^2 u C1 + 3(1-u) u^2 C2 + u^3 P1,"
                 " u = (t-t0)/(t1-t0); before t0 of the first row the drone waits on its pad",
                 "t0,t1,x0,y0,z0,c1x,c1y,c1z,c2x,c2y,c2z,x1,y1,z1,lat1,lon1,alt1"]
        lat, lon, alt = origin.show_to_wgs84(P)
        for k in range(1, len(t)):
            A, B = P[k - 1], P[k]
            ci = int(self.cidx[a + k])
            if ci >= 0:
                C1, C2 = self.cc[ci].astype(float)
            else:
                C1, C2 = A + (B - A) / 3.0, A + 2.0 * (B - A) / 3.0
            lines.append(f"{t[k - 1]:.3f},{t[k]:.3f},{A[0]:.3f},{A[1]:.3f},{A[2]:.3f},{C1[0]:.3f},{C1[1]:.3f},"
                         f"{C1[2]:.3f},{C2[0]:.3f},{C2[1]:.3f},{C2[2]:.3f},{B[0]:.3f},{B[1]:.3f},{B[2]:.3f},"
                         f"{float(lat[k]):.8f},{float(lon[k]):.8f},{float(alt[k]):.3f}")
        return "\n".join(lines) + "\n"

    def npz_bytes(self) -> bytes:
        buf = io.BytesIO()
        np.savez_compressed(buf, t=(self.tk - np.repeat(np.arange(self.n) * BIG, np.diff(self.pfirst))),
                            points=self.pp, first=self.pfirst, curve=self.cidx, ctrl=self.cc,
                            light_t20=(self.lk - np.repeat(np.arange(self.n, dtype=np.int64) * LBIG,
                                                          np.diff(self.lfirst))),
                            light_rgb=self.lc, light_first=self.lfirst, takeoff=self.takeoff_offsets)
        return buf.getvalue()

    def summary(self) -> dict:
        m = self.raw.meta
        return {"format": m.get("format"), "title": m.get("title"), "drones": self.n, "duration": self.duration,
                "notes": self.warnings, "cues": m.get("cues", []), "design_limits": m.get("design_limits", {}),
                "first_takeoff": float(self.ph["t_lift"][self.flies].min()) if self.flies.any() else None,
                "last_takeoff": float(self.ph["t_lift"][self.flies].max()) if self.flies.any() else None,
                "has_audio": m.get("has_audio", False), "location": m.get("location")}


# =====================================================================================
#  building from a spec
# =====================================================================================
def raw_pads(raw: RawShow) -> np.ndarray:
    P = raw.pp[raw.pfirst[:-1]].astype(float)
    P[:, 2] = 0.0
    return P


def grid_centre(raw: RawShow) -> np.ndarray:
    """Centre (x, y) of the pad grid's bounding box in Skybrush coordinates."""
    P = raw_pads(raw)[:, :2]
    return 0.5 * (P.min(0) + P.max(0))


def transform_matrix(raw: RawShow, rotate_deg: float, offset):
    """Placement: rotate about the pad-grid centre, then move by offset.
    show = P @ R.T + t  (R (3,3), t (3,), float32)."""
    a = math.radians(float(rotate_deg))
    R = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
    c = np.r_[grid_centre(raw), 0.0]
    t = c + np.r_[float(offset[0]), float(offset[1]), 0.0] - R @ c
    return R.astype(np.float32), t.astype(np.float32)


def centre_offset(raw: RawShow, rotate_deg: float, center_xy) -> list:
    """Offset that puts the pad-grid centre on center_xy (rotation keeps the centre)."""
    c = grid_centre(raw)
    return [round(float(center_xy[0] - c[0]), 3), round(float(center_xy[1] - c[1]), 3)]


def show_from_spec(spec: dict, settings: dict, progress=None) -> ImportedShow:
    imp = spec["imported"]
    raw = load_raw(resolve_file(imp), imp.get("sha256"), progress)
    return ImportedShow(raw, settings, spec.get("name") or "", imp.get("rotate_deg", 0.0),
                        imp.get("offset", [0.0, 0.0]), imp.get("tolerance", 0.25))


def resolve_file(imp: dict) -> str:
    p = imp.get("file", "")
    if p and os.path.isfile(p):
        return p
    alt = os.path.join(imports_dir(), os.path.basename(p)) if p else ""
    if alt and os.path.isfile(alt):
        return alt
    raise ShowImportError(f"imported show file not found ({p or 'none'}) - import the show again")


def build_imported_show(spec: dict, settings: dict, start_positions=None, progress=None) -> ImportedShow:
    show = show_from_spec(spec, settings, progress)
    if start_positions is None:
        return show
    start = np.asarray(start_positions, float)
    d, k = cKDTree(show.start_positions[:, :2]).query(start[:, :2], k=1)
    tol = float(settings.get("max_slot_offset", 0.5)) + 1e-6
    if (d > tol).any():
        raise ShowImportError(f"{int((d > tol).sum())} drones are not on a pad of the imported show")
    if len(np.unique(k)) != len(k):
        raise ShowImportError("two drones are matched to the same pad of the imported show")
    return show.subset(k)


def transformed_pads(spec: dict) -> np.ndarray:
    imp = spec["imported"]
    raw = load_raw(resolve_file(imp), imp.get("sha256"))
    R, t = transform_matrix(raw, imp.get("rotate_deg", 0.0), imp.get("offset", [0.0, 0.0]))
    P = raw_pads(raw) @ R.T.astype(float) + t.astype(float)
    P[:, 2] = 0.0
    return P


def field_layout_imported(spec: dict, settings: dict, spares: int):
    """Pads of an imported show plus `spares` spare pads in rows behind the grid."""
    pads = transformed_pads(spec)
    is_spare = np.zeros(len(pads), bool)
    if spares > 0:
        sp = float(settings.get("takeoff_spacing", 2.2))
        x0, x1 = pads[:, 0].min(), pads[:, 0].max()
        per_row = max(1, int((x1 - x0) / sp) + 1)
        k = np.arange(spares)
        row, col = k // per_row, k % per_row
        width = (min(spares, per_row) - 1) * sp
        cx = 0.5 * (x0 + x1)
        S = np.zeros((spares, 3))
        S[:, 0] = cx - width / 2 + col * sp
        S[:, 1] = pads[:, 1].max() + 2 * sp + row * sp
        pads = np.vstack([pads, S])
        is_spare = np.concatenate([is_spare, np.ones(spares, bool)])
    return pads, is_spare


# =====================================================================================
#  validation
# =====================================================================================
def _lin_min(A0, A1, i, j):
    d0 = A0[i] - A0[j]
    b = (A1[i] - A1[j]) - d0
    bb = (b * b).sum(1)
    s = np.where(bb > 1e-12, np.clip(-(d0 * b).sum(1) / np.maximum(bb, 1e-12), 0.0, 1.0), 0.0)
    return np.linalg.norm(d0 + s[:, None] * b, axis=1), s


def _separation_scan(show: ImportedShow, D: float, zs: float, progress=None):
    """Closest approaches over the whole show (raw drone indices).  Drones closer than
    D are listed; pairs where both drones are on the ground are ignored."""
    T = show.duration
    steps = max(2, int(math.ceil(T / SAMPLE_DT)) + 1)
    times = np.linspace(0.0, T, steps)
    n = show.n
    sc = np.array([1.0, 1.0, 1.0 / zs])
    prev = show.positions(times[0]) * sc
    best = (math.inf, None)
    viol = []
    cap = 20000
    trunc = False
    for k in range(1, steps):
        cur = show.positions(times[k]) * sc
        disp = np.linalg.norm(cur - prev, axis=1)
        r = D + 2.0 * float(disp.max(initial=0.0))
        p = cKDTree(prev).query_pairs(r, output_type="ndarray")
        if len(p):
            i, j = p[:, 0], p[:, 1]
            air = ((prev[i, 2] > 0.5) | (cur[i, 2] > 0.5)) & ((prev[j, 2] > 0.5) | (cur[j, 2] > 0.5))
            i, j = i[air], j[air]
            if len(i):
                dist, s = _lin_min(prev, cur, i, j)
                m = int(dist.argmin())
                if dist[m] < best[0]:
                    best = (float(dist[m]), (float(times[k - 1] + s[m] * SAMPLE_DT), int(show.sel[i[m]]),
                                             int(show.sel[j[m]])))
                bad = np.nonzero(dist < D - 1e-6)[0]
                for b in bad:
                    if len(viol) >= cap:
                        trunc = True
                        break
                    viol.append((round(float(times[k - 1] + s[b] * SAMPLE_DT), 2), int(show.sel[i[b]]),
                                 int(show.sel[j[b]]), round(float(dist[b]), 3)))
        prev = cur
        if progress and k % 50 == 0:
            progress(k / steps, f"Checking separation {times[k]:.0f}/{T:.0f} s")
    return {"best": best, "viol": viol, "trunc": trunc}


def validate_imported(show: ImportedShow, progress=None) -> dict:
    s = show.settings
    D = float(s["min_separation"])
    zs = float(s.get("downwash_factor", 1.0))
    checks, violations = [], []
    stats = {"drones": show.n, "duration_s": round(show.duration, 1)}

    def add(cid, name, status, detail):
        checks.append({"id": cid, "name": name, "status": status, "detail": detail})

    def rep(f, m):
        if progress:
            progress(f, m)

    # ---- separation (cached per file: rotation / offset do not change distances) -----------
    key = (show.raw.sha256, round(D, 4), round(zs, 4))
    sep = _SEP_CACHE.get(key) if show.raw.sha256 else None
    if sep is None or (show.is_subset and sep["trunc"]):
        full = show if not show.is_subset else ImportedShow(show.raw, s, show.name)
        sep = _separation_scan(full, D, zs, progress=lambda f, m: rep(0.7 * f, m))
        if show.raw.sha256:
            _SEP_CACHE[key] = sep
            while len(_SEP_CACHE) > 8:
                _SEP_CACHE.popitem(last=False)
    pos_of = {int(r): k for k, r in enumerate(show.sel)}
    viol = [v for v in sep["viol"] if v[1] in pos_of and v[2] in pos_of]
    for t_, a, b, dd in viol[:MAX_LISTED]:
        violations.append({"t": t_, "a": pos_of[a], "b": pos_of[b], "dist": dd, "segment": _scene_at(show, t_)})
    best = sep["best"]
    if best[1] is None:
        stats["min_separation_m"] = None
        add("separation", "Minimum separation", "pass",
            f"no two airborne drones come within {D + 0.0:.2f} m of each other")
    elif viol:
        md = min(v[3] for v in viol)
        stats["min_separation_m"] = round(md, 3)
        v0 = min(viol, key=lambda v: v[3])
        add("separation", "Minimum separation", "fail",
            f"{len(viol)}{'+' if sep['trunc'] else ''} close passes; closest {md:.2f} m (drones {pos_of[v0[1]] + 1} "
            f"& {pos_of[v0[2]] + 1} at t={v0[0]:.1f} s); required >= {D:.2f} m")
    else:
        d, (t_at, a, b) = best
        if a in pos_of and b in pos_of:
            stats["min_separation_m"] = round(d, 3)
            add("separation", "Minimum separation", "pass",
                f"closest approach {d:.2f} m (drones {pos_of[a] + 1} & {pos_of[b] + 1} at t={t_at:.1f} s); "
                f"required >= {D:.2f} m")
        else:
            stats["min_separation_m"] = round(d, 3)
            add("separation", "Minimum separation", "pass",
                f"closest approach >= {max(d, D):.2f} m (full show {d:.2f} m); required >= {D:.2f} m")

    # ---- dynamics, extent, altitude ---------------------------------------------------------
    T = show.duration
    steps = max(3, int(math.ceil(T / SAMPLE_DT)) + 1)
    times = np.linspace(0.0, T, steps)
    h = times[1] - times[0]
    ph = show.ph
    win0 = np.where(show.flies, ph["t_climb"], np.inf)
    win1 = np.where(show.flies, ph["t_land"], -np.inf)
    hist = []
    vh = vu = vd = acc = 0.0
    at = {"h": (0.0, 0), "u": (0.0, 0), "d": (0.0, 0), "a": (0.0, 0)}
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    zmin_show = math.inf
    zmin_at = (0.0, 0)
    for k in range(steps):
        P = show.positions(times[k])
        lo = np.minimum(lo, P.min(0))
        hi = np.maximum(hi, P.max(0))
        ins = (times[k] >= win0) & (times[k] <= win1)
        if ins.any():
            zi = np.where(ins, P[:, 2], np.inf)
            j = int(zi.argmin())
            if zi[j] < zmin_show:
                zmin_show, zmin_at = float(zi[j]), (float(times[k]), j)
        hist.append(P)
        if len(hist) >= 2:
            v = (hist[-1] - hist[-2]) / h
            sh = np.hypot(v[:, 0], v[:, 1])
            j = int(sh.argmax())
            if sh[j] > vh:
                vh, at["h"] = float(sh[j]), (float(times[k]), j)
            j = int(v[:, 2].argmax())
            if v[j, 2] > vu:
                vu, at["u"] = float(v[j, 2]), (float(times[k]), j)
            j = int(v[:, 2].argmin())
            if -v[j, 2] > vd:
                vd, at["d"] = float(-v[j, 2]), (float(times[k]), j)
        if len(hist) == 7:                   # acceleration over +-0.6 s (sampled paths have kinks)
            a = np.linalg.norm(hist[6] - 2 * hist[3] + hist[0], axis=1) / (3 * h) ** 2
            j = int(a.argmax())
            if a[j] > acc:
                acc, at["a"] = float(a[j]), (float(times[k - 3]), j)
            hist.pop(0)
        if progress and k % 100 == 0:
            rep(0.7 + 0.25 * k / steps, f"Checking dynamics {times[k]:.0f}/{T:.0f} s")
    stats.update(max_speed_h=round(vh, 2), max_speed_up=round(vu, 2), max_speed_down=round(vd, 2),
                 max_accel=round(acc, 2))
    lim = (float(s["max_speed_h"]), float(s["max_speed_up"]), float(s["max_speed_down"]), float(s["max_accel"]))
    tol = 1.02
    ok = vh <= lim[0] * tol and vu <= lim[1] * tol and vd <= lim[2] * tol
    acc_ok = acc <= lim[3] * tol
    worst = []
    for kk, name, v, li in (("h", "horizontal", vh, lim[0]), ("u", "climb", vu, lim[1]),
                            ("d", "descent", vd, lim[2]), ("a", "accel", acc, lim[3])):
        if v > li * tol:
            worst.append(f"{name} drone {at[kk][1] + 1} at t={at[kk][0]:.1f} s")
    add("dynamics", "Speed & acceleration", "fail" if not ok else ("pass" if acc_ok else "warn"),
        f"peak horizontal {vh:.1f}/{lim[0]:.1f} m/s, climb {vu:.1f}/{lim[1]:.1f}, descent {vd:.1f}/{lim[2]:.1f}, "
        f"accel ~{acc:.1f}/{lim[3]:.1f} m/s² (over 1.2 s)" + (f" - worst: {'; '.join(worst)}" if worst else ""))
    g = s["geofence"]
    inside = (lo[0] >= g["x"][0] and hi[0] <= g["x"][1] and lo[1] >= g["y"][0] and hi[1] <= g["y"][1]
              and lo[2] >= g["z"][0] - 1e-6 and hi[2] <= g["z"][1])
    stats["extent"] = {"x": [round(float(lo[0]), 1), round(float(hi[0]), 1)],
                       "y": [round(float(lo[1]), 1), round(float(hi[1]), 1)],
                       "z": [round(float(lo[2]), 1), round(float(hi[2]), 1)]}
    add("geofence", "Geofence", "pass" if inside else "fail",
        f"show uses x {lo[0]:.0f}..{hi[0]:.0f}, y {lo[1]:.0f}..{hi[1]:.0f}, z {lo[2]:.0f}..{hi[2]:.0f} m; "
        f"fence x {g['x'][0]:.0f}..{g['x'][1]:.0f}, y {g['y'][0]:.0f}..{g['y'][1]:.0f}, z ..{g['z'][1]:.0f}"
        + ("" if inside else " - move it with the import offset / rotation or widen the fence"))
    limit_y = float(s["audience_y"]) + float(s["audience_buffer"])
    add("audience", "Audience safety distance", "pass" if lo[1] >= limit_y else "fail",
        f"closest drone to the audience is at y={lo[1]:.0f} m; must stay behind y={limit_y:.0f} m "
        f"(audience line {float(s['audience_y']):.0f} m + {float(s['audience_buffer']):.0f} m buffer)")
    zmax = float(hi[2])
    zmin_ok = zmin_show >= float(s["min_altitude"]) - 1e-6 or not math.isfinite(zmin_show)
    ok = zmin_ok and zmax <= float(s["max_altitude"]) + 1e-6
    lowest = (f"lowest {zmin_show:.1f} m (drone {zmin_at[1] + 1} at t={zmin_at[0]:.0f} s)"
              if math.isfinite(zmin_show) else "no airborne phase")
    add("altitude", "Altitude limits", "pass" if ok else "fail",
        f"between take-off climb and landing: {lowest}, highest {zmax:.0f} m; allowed "
        f"{float(s['min_altitude']):.0f}..{float(s['max_altitude']):.0f} m")

    # ---- pads ------------------------------------------------------------------------------
    pads = show.start_positions[:, :2]
    if len(pads) > 1:
        dd, _ = cKDTree(pads).query(pads, k=2)
        pmin = float(dd[:, 1].min())
    else:
        pmin = math.inf
    need = 2.0 * float(s.get("max_slot_offset", 0.5))
    st_ = "fail" if pmin < need else ("warn" if pmin < D else "pass")
    stats["pad_spacing_m"] = round(pmin, 2) if math.isfinite(pmin) else None
    add("spacing", "Pad spacing", st_,
        f"closest pads {pmin:.2f} m apart; pads must be >= {need:.2f} m apart to tell drones apart "
        f"(2 x max pad offset){' and should be >= ' + format(D, '.2f') + ' m for vertical take-off' if st_ == 'warn' else ''}")

    # ---- battery ---------------------------------------------------------------------------------
    usable = float(s["battery_minutes"]) * 60.0 * (1.0 - float(s["battery_reserve_pct"]) / 100.0)
    fl = show.flies
    ft = float((ph["t_down"][fl] - ph["t_lift"][fl]).max()) if fl.any() else 0.0
    st_ = "pass" if ft <= usable else ("warn" if ft <= float(s["battery_minutes"]) * 60.0 else "fail")
    stats["flight_time_s"] = round(ft, 1)
    add("battery", "Battery endurance", st_,
        f"longest flight {ft / 60:.1f} min; usable {usable / 60:.1f} min "
        f"({float(s['battery_minutes']):.0f} min with {float(s['battery_reserve_pct']):.0f}% reserve)")
    if show.warnings:
        add("compiler", "Import notes", "warn" if any("jump" in w or "unsupported" in w or "truncated" in w
                                                      for w in show.warnings) else "pass",
            "; ".join(show.warnings[:6]))
    ok = all(c["status"] != "fail" for c in checks)
    rep(1.0, "Validation done")
    return {"ok": ok, "checks": checks, "violations": violations, "stats": stats}


def _scene_at(show, t):
    for sg in show.segments:
        if sg.t0 <= t < sg.t1:
            return sg.label
    return ""
