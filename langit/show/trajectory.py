"""Compiled show: synchronised trajectories + light programme for every drone.

All drones in a segment share the same time profile s(t) (minimum-jerk), which
is what makes the CAPT collision guarantee hold.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field

import numpy as np

from .lights import effect_colors, hex_rgb

V_PEAK = 1.875           # peak velocity of a min-jerk move = 1.875 * d / T
A_PEAK = 5.7735          # peak acceleration = 5.7735 * d / T^2


def minjerk(tau):
    t = np.clip(tau, 0.0, 1.0)
    return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)


def rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


AXES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}


@dataclass
class Keyframe:
    pos: np.ndarray                      # (N,3)
    active: np.ndarray                   # (N,) bool - lit drones
    point_colors: np.ndarray | None      # (N,3) float or None
    light: dict
    label: str
    formation: int                       # timeline index, -1 take-off/landing grid


@dataclass
class Segment:
    kind: str                            # takeoff | hold | transition | spin | landing
    t0: float
    t1: float
    k0: int
    k1: int
    formation: int = -1                  # formation shown at the END of this segment
    label: str = ""
    spin_axis: tuple | None = None
    spin_center: tuple | None = None
    spin_angle: float = 0.0              # radians

    @property
    def duration(self) -> float:
        return self.t1 - self.t0


@dataclass
class CompiledShow:
    name: str
    settings: dict
    start_positions: np.ndarray          # (N,3) ground pads, z = 0
    keyframes: list
    segments: list
    formation_start: dict                # formation index -> time it is reached
    formation_names: list
    warnings: list = field(default_factory=list)
    landing_slots: np.ndarray | None = None   # (N,) index into start_positions where each drone lands

    def __post_init__(self):
        self._t0s = [s.t0 for s in self.segments]
        self.n = len(self.start_positions)
        self.idx = np.arange(self.n)
        self._takeoff_rgb = hex_rgb(self.settings.get("takeoff_color", "#3a3a3a"))
        self._fkf = {s.formation: self.keyframes[s.k1] for s in self.segments
                     if s.kind == "transition" and s.formation >= 0}

    imported = False

    @property
    def duration(self) -> float:
        return self.segments[-1].t1 if self.segments else 0.0

    @property
    def final_positions(self) -> np.ndarray:
        return self.keyframes[self.segments[-1].k1].pos

    def seg_index(self, t: float) -> int:
        i = bisect.bisect_right(self._t0s, t) - 1
        return max(0, min(len(self.segments) - 1, i))

    # -- positions --------------------------------------------------------------------------
    def _seg_pos(self, seg: Segment, t: float) -> np.ndarray:
        K0 = self.keyframes[seg.k0].pos
        if seg.kind == "hold" or seg.k0 == seg.k1 and seg.kind != "spin":
            return K0
        tau = (t - seg.t0) / seg.duration if seg.duration > 0 else 1.0
        s = float(minjerk(tau))
        if seg.kind == "spin":
            c = np.asarray(seg.spin_center)
            R = rodrigues(np.asarray(seg.spin_axis), seg.spin_angle * s)
            m = self.keyframes[seg.k0].active
            out = K0.copy()
            out[m] = (K0[m] - c) @ R.T + c
            return out
        K1 = self.keyframes[seg.k1].pos
        return K0 + s * (K1 - K0)

    def positions(self, t: float) -> np.ndarray:
        if t <= 0:
            return self.keyframes[0].pos
        if t >= self.duration:
            return self.keyframes[self.segments[-1].k1].pos
        seg = self.segments[self.seg_index(t)]
        return self._seg_pos(seg, t)

    def positions_for(self, t: float, slots: np.ndarray) -> np.ndarray:
        return self.positions(t)[slots]

    def path_deviation(self, t: float, slots: np.ndarray, P: np.ndarray) -> np.ndarray:
        """Distance of each drone (positions P, show slots `slots`) from its planned PATH
        around time t (current and previous segment).  Unlike the distance to the planned
        position this tolerates timing differences, e.g. drones flying stock missions at
        constant speed instead of the min-jerk profile."""
        i = self.seg_index(t)
        best = np.full(len(slots), np.inf)
        for k in (i - 1, i, i + 1):
            if not 0 <= k < len(self.segments):
                continue
            seg = self.segments[k]
            A = self.keyframes[seg.k0].pos[slots]
            if seg.kind == "spin":
                d = np.full(len(slots), np.inf)
                for sv in np.linspace(0.0, 1.0, 25):
                    d = np.minimum(d, np.linalg.norm(P - self._seg_pos(seg, seg.t0 + sv * seg.duration)[slots], axis=1))
            else:
                B = self.keyframes[seg.k1].pos[slots]
                AB = B - A
                L2 = (AB * AB).sum(1)
                u = np.where(L2 > 1e-9, np.clip(((P - A) * AB).sum(1) / np.maximum(L2, 1e-9), 0, 1), 0.0)
                d = np.linalg.norm(P - (A + u[:, None] * AB), axis=1)
            best = np.minimum(best, d)
        return best

    # -- lights -------------------------------------------------------------------------------
    def _formation_colors(self, j: int, P: np.ndarray, t: float) -> np.ndarray:
        if j < 0:
            return np.tile(self._takeoff_rgb, (self.n, 1))
        kf = self._kf_of_formation(j)
        tau = max(0.0, t - self.formation_start.get(j, t))
        return effect_colors(kf.light, P, kf.active, tau, self.idx, kf.point_colors)

    def _kf_of_formation(self, j: int) -> Keyframe:
        return self._fkf[j]

    def colors(self, t: float) -> np.ndarray:
        """(N,3) float colours at show time t."""
        if t < 0 or t > self.duration:
            return np.zeros((self.n, 3), np.float32)
        i = self.seg_index(t)
        seg = self.segments[i]
        tau = (t - seg.t0) / seg.duration if seg.duration > 0 else 1.0
        if seg.kind == "takeoff":
            ramp = min(1.0, max(0.0, (t - seg.t0) / 2.0))
            return np.tile(self._takeoff_rgb * ramp, (self.n, 1))
        if seg.kind == "landing":
            return np.tile(self._takeoff_rgb * (1.0 - 0.8 * tau), (self.n, 1))
        if seg.kind in ("hold", "spin"):
            P = self._seg_pos(seg, t)
            return self._formation_colors(seg.formation, P, t)
        # transition: blend previous formation colours -> next formation colours, dimmed
        prev = self.segments[i - 1] if i > 0 else None
        P0 = self.keyframes[seg.k0].pos
        P1 = self.keyframes[seg.k1].pos
        j_prev = prev.formation if prev is not None else -1
        c0 = self._formation_colors(j_prev, P0, seg.t0)
        if j_prev >= 0:     # drones not lit in the previous formation stay dark
            c0[~self._kf_of_formation(j_prev).active] = 0
        if seg.formation >= 0:
            c1 = self._formation_colors(seg.formation, P1, self.formation_start[seg.formation])
        else:
            c1 = np.tile(self._takeoff_rgb, (self.n, 1))
        s = float(minjerk(tau))
        dim = float(self.settings.get("transition_dim", 0.35))
        b = 1.0 - (1.0 - dim) * math.sin(math.pi * tau)
        return np.clip((c0 * (1 - s) + c1 * s) * b, 0.0, 1.0).astype(np.float32)

    def colors_u8(self, t: float) -> np.ndarray:
        return (self.colors(t) * 255.0 + 0.5).astype(np.uint8)

    # -- summaries ----------------------------------------------------------------------------
    def timeline_summary(self) -> list:
        out = []
        for s in self.segments:
            out.append({"kind": s.kind, "t0": round(s.t0, 2), "t1": round(s.t1, 2),
                        "formation": s.formation, "label": s.label})
        return out
