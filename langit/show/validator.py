"""Safety validation of a compiled show.

Separation is checked rigorously for straight-line segments: every segment is
sampled densely enough that any pair which comes closer than the minimum
separation must appear as a candidate, then each candidate pair's closest
approach is computed analytically.  Spins are sampled at <= D/4 arc steps.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree

from .trajectory import A_PEAK, V_PEAK, minjerk, rodrigues

MAX_LISTED = 40


def _sc(P: np.ndarray, zs: float) -> np.ndarray:
    if zs == 1.0:
        return P
    Q = P.copy()
    Q[:, 2] = Q[:, 2] / zs
    return Q


def _pairs(P: np.ndarray, r: float) -> np.ndarray:
    return cKDTree(P).query_pairs(r, output_type="ndarray")


def _invert_minjerk(s: float) -> float:
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if minjerk(mid) < s:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _linear_segment_min(A0: np.ndarray, A1: np.ndarray, D: float):
    """Closest approaches below 1.25 D for synchronised straight-line motion A0 -> A1.
    Returns (pairs (k,2), dmin (k,), s_at_min (k,))."""
    n = len(A0)
    disp = np.linalg.norm(A1 - A0, axis=1)
    dmax = float(disp.max(initial=0.0))
    r = 1.25 * D
    if dmax < 1e-9:
        p = _pairs(A0, r)
        if len(p) == 0:
            return p, np.zeros(0), np.zeros(0)
        d = np.linalg.norm(A0[p[:, 0]] - A0[p[:, 1]], axis=1)
        return p, d, np.zeros(len(p))
    K = int(math.ceil(4.0 * dmax / D)) + 1
    found = []
    for s in np.linspace(0.0, 1.0, K):
        p = _pairs(A0 + s * (A1 - A0), r)
        if len(p):
            found.append(p[:, 0].astype(np.int64) * n + p[:, 1])
    if not found:
        return np.zeros((0, 2), int), np.zeros(0), np.zeros(0)
    code = np.unique(np.concatenate(found))
    i, j = code // n, code % n
    d0 = A0[i] - A0[j]
    b = (A1[i] - A1[j]) - d0
    bb = (b * b).sum(1)
    s = np.where(bb > 1e-12, np.clip(-(d0 * b).sum(1) / np.maximum(bb, 1e-12), 0.0, 1.0), 0.0)
    dist = np.linalg.norm(d0 + s[:, None] * b, axis=1)
    return np.stack([i, j], axis=1), dist, s


def validate(show, progress=None) -> dict:
    if getattr(show, "imported", False):
        from .importer import validate_imported
        return validate_imported(show, progress)
    s = show.settings
    D = float(s["min_separation"])
    zs = float(s.get("downwash_factor", 1.0))
    checks = []
    violations = []
    stats = {"drones": show.n, "duration_s": round(show.duration, 1)}

    def add(cid, name, status, detail):
        checks.append({"id": cid, "name": name, "status": status, "detail": detail})

    # ---- separation -------------------------------------------------------------------------
    global_min = (math.inf, None)
    nsegs = len(show.segments)
    for si, seg in enumerate(show.segments):
        if progress:
            progress(si / nsegs, f"Checking separation: {seg.label}")
        K0 = show.keyframes[seg.k0].pos
        K1 = show.keyframes[seg.k1].pos
        if seg.kind == "spin":
            act = show.keyframes[seg.k0].active
            c = np.asarray(seg.spin_center)
            axis = np.asarray(seg.spin_axis)
            rel = K0[act] - c
            rmax = float(np.linalg.norm(rel - np.outer(rel @ axis, axis), axis=1).max(initial=0))
            K = int(math.ceil(4.0 * rmax * abs(seg.spin_angle) / D)) + 1
            best = []
            for sv in np.linspace(0.0, 1.0, max(2, K)):
                P = K0.copy()
                P[act] = rel @ rodrigues(axis, seg.spin_angle * sv).T + c
                Q = _sc(P, zs)
                p = _pairs(Q, D * 1.25)
                if len(p):
                    d = np.linalg.norm(Q[p[:, 0]] - Q[p[:, 1]], axis=1)
                    k = int(d.argmin())
                    best.append((float(d[k]), sv, int(p[k, 0]), int(p[k, 1])))
                    for (a, b_), dd in zip(p[d < D], d[d < D]):
                        if len(violations) < MAX_LISTED:
                            violations.append({"t": round(seg.t0 + seg.duration * _invert_minjerk(sv), 2),
                                               "a": int(a), "b": int(b_), "dist": round(float(dd), 3),
                                               "segment": seg.label})
            if best:
                d, sv, a, b_ = min(best)
                if d < global_min[0]:
                    global_min = (d, (seg.t0 + seg.duration * _invert_minjerk(sv), a, b_, seg.label))
            continue
        pairs, dist, sat = _linear_segment_min(_sc(K0, zs), _sc(K1, zs), D)
        if len(dist):
            k = int(dist.argmin())
            if dist[k] < global_min[0]:
                t_at = seg.t0 + seg.duration * _invert_minjerk(float(sat[k]))
                global_min = (float(dist[k]), (t_at, int(pairs[k, 0]), int(pairs[k, 1]), seg.label))
            bad = np.nonzero(dist < D - 1e-6)[0]
            for k in bad[: max(0, MAX_LISTED - len(violations))]:
                violations.append({"t": round(seg.t0 + seg.duration * _invert_minjerk(float(sat[k])), 2),
                                   "a": int(pairs[k, 0]), "b": int(pairs[k, 1]),
                                   "dist": round(float(dist[k]), 3), "segment": seg.label})
            if len(bad) and len(violations) >= MAX_LISTED:
                pass
    if global_min[1] is None:
        add("separation", "Minimum separation", "pass",
            f"no two drones come within {1.25 * D:.2f} m of each other at any time")
        stats["min_separation_m"] = None
    else:
        d, (t_at, a, b_, label) = global_min
        stats["min_separation_m"] = round(d, 3)
        if d < D - 1e-6:
            add("separation", "Minimum separation", "fail",
                f"{len(violations)}{'+' if len(violations) >= MAX_LISTED else ''} conflicts; closest "
                f"{d:.2f} m (drones {a} & {b_} at t={t_at:.1f} s, {label}); required >= {D:.2f} m")
        else:
            add("separation", "Minimum separation", "pass",
                f"closest approach {d:.2f} m (t={t_at:.1f} s, {label}); required >= {D:.2f} m")

    # ---- dynamics ------------------------------------------------------------------------------
    vh = vu = vd = acc = 0.0
    worst_v = ""
    for seg in show.segments:
        if seg.duration <= 0 or seg.kind == "hold":
            continue
        K0 = show.keyframes[seg.k0].pos
        K1 = show.keyframes[seg.k1].pos
        T = seg.duration
        if seg.kind == "spin":
            act = show.keyframes[seg.k0].active
            axis = np.asarray(seg.spin_axis)
            rel = K0[act] - np.asarray(seg.spin_center)
            r = float(np.linalg.norm(rel - np.outer(rel @ axis, axis), axis=1).max(initial=0))
            v = V_PEAK * abs(seg.spin_angle) * r / T
            w = V_PEAK * abs(seg.spin_angle) / T
            a = math.hypot(A_PEAK * abs(seg.spin_angle) * r / T ** 2, w * w * r)
            if abs(axis[2]) > 0.99:
                if v > vh:
                    vh, worst_v = v, seg.label
            else:
                vu, vd = max(vu, v), max(vd, v)
            acc = max(acc, a)
            continue
        d = K1 - K0
        dh = np.hypot(d[:, 0], d[:, 1])
        v = V_PEAK * float(dh.max(initial=0)) / T
        if v > vh:
            vh, worst_v = v, seg.label
        vu = max(vu, V_PEAK * float(np.clip(d[:, 2], 0, None).max(initial=0)) / T)
        vd = max(vd, V_PEAK * float(np.clip(-d[:, 2], 0, None).max(initial=0)) / T)
        acc = max(acc, A_PEAK * float(np.linalg.norm(d, axis=1).max(initial=0)) / T ** 2)
    stats.update(max_speed_h=round(vh, 2), max_speed_up=round(vu, 2), max_speed_down=round(vd, 2),
                 max_accel=round(acc, 2))
    tol = 1.001
    lim = (float(s["max_speed_h"]), float(s["max_speed_up"]), float(s["max_speed_down"]), float(s["max_accel"]))
    ok = vh <= lim[0] * tol and vu <= lim[1] * tol and vd <= lim[2] * tol and acc <= lim[3] * tol
    add("dynamics", "Speed & acceleration", "pass" if ok else "fail",
        f"peak horizontal {vh:.1f}/{lim[0]:.1f} m/s ({worst_v}), climb {vu:.1f}/{lim[1]:.1f}, "
        f"descent {vd:.1f}/{lim[2]:.1f}, accel {acc:.1f}/{lim[3]:.1f} m/s²")

    # ---- geofence / audience / altitude ----------------------------------------------------------
    pts = [kf.pos for kf in show.keyframes]
    for seg in show.segments:              # spin sweeps
        if seg.kind == "spin":
            act = show.keyframes[seg.k0].active
            K0 = show.keyframes[seg.k0].pos[act]
            c = np.asarray(seg.spin_center)
            for sv in np.linspace(0, 1, 17):
                pts.append((K0 - c) @ rodrigues(np.asarray(seg.spin_axis), seg.spin_angle * sv).T + c)
    allp = np.vstack(pts)
    lo = allp.min(0)
    hi = allp.max(0)
    g = s["geofence"]
    inside = (lo[0] >= g["x"][0] and hi[0] <= g["x"][1] and lo[1] >= g["y"][0] and hi[1] <= g["y"][1]
              and lo[2] >= g["z"][0] - 1e-6 and hi[2] <= g["z"][1])
    stats["extent"] = {"x": [round(float(lo[0]), 1), round(float(hi[0]), 1)],
                       "y": [round(float(lo[1]), 1), round(float(hi[1]), 1)],
                       "z": [round(float(lo[2]), 1), round(float(hi[2]), 1)]}
    add("geofence", "Geofence", "pass" if inside else "fail",
        f"show uses x {lo[0]:.0f}..{hi[0]:.0f}, y {lo[1]:.0f}..{hi[1]:.0f}, z {lo[2]:.0f}..{hi[2]:.0f} m; "
        f"fence x {g['x'][0]:.0f}..{g['x'][1]:.0f}, y {g['y'][0]:.0f}..{g['y'][1]:.0f}, z ..{g['z'][1]:.0f}")
    limit_y = float(s["audience_y"]) + float(s["audience_buffer"])
    add("audience", "Audience safety distance", "pass" if lo[1] >= limit_y else "fail",
        f"closest drone to the audience is at y={lo[1]:.0f} m; must stay behind y={limit_y:.0f} m "
        f"(audience line {float(s['audience_y']):.0f} m + {float(s['audience_buffer']):.0f} m buffer)")
    form_kfs = [kf for kf in show.keyframes if kf.formation >= 0]
    zmin = min(float(kf.pos[:, 2].min()) for kf in form_kfs) if form_kfs else 0.0
    zmax = float(hi[2])
    ok = zmin >= float(s["min_altitude"]) - 1e-6 and zmax <= float(s["max_altitude"]) + 1e-6
    add("altitude", "Altitude limits", "pass" if ok else "fail",
        f"formations between {zmin:.0f} and {zmax:.0f} m; allowed {float(s['min_altitude']):.0f}.."
        f"{float(s['max_altitude']):.0f} m")

    # ---- spacing preconditions -----------------------------------------------------------------
    need = math.sqrt(2) * D
    worst = math.inf
    for kf in show.keyframes[1:]:
        Q = _sc(kf.pos, zs)
        dd, _ = cKDTree(Q).query(Q, k=2)
        worst = min(worst, float(dd[:, 1].min()))
    add("spacing", "Formation spacing", "pass" if worst >= need * 0.999 else "warn",
        f"smallest neighbour spacing {worst:.2f} m (collision-free guarantee needs >= {need:.2f} m)")

    # ---- battery ------------------------------------------------------------------------------------
    usable = float(s["battery_minutes"]) * 60.0 * (1.0 - float(s["battery_reserve_pct"]) / 100.0)
    dur = show.duration
    st = "pass" if dur <= usable else ("warn" if dur <= float(s["battery_minutes"]) * 60.0 else "fail")
    stats["flight_time_s"] = round(dur, 1)
    add("battery", "Battery endurance", st,
        f"flight time {dur / 60:.1f} min; usable {usable / 60:.1f} min "
        f"({float(s['battery_minutes']):.0f} min with {float(s['battery_reserve_pct']):.0f}% reserve)")

    if show.warnings:
        add("compiler", "Compiler notes", "warn", "; ".join(show.warnings[:6]))

    ok = all(c["status"] != "fail" for c in checks)
    if progress:
        progress(1.0, "Validation done")
    return {"ok": ok, "checks": checks, "violations": violations, "stats": stats}
