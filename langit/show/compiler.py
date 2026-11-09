"""Compile a show spec into synchronised, collision-free trajectories.

Pipeline
  1. Build the point set of every formation (plus a dark "parking wall" for drones
     a formation does not use) and the end set of every formation (after spin).
  2. Solve every transition's assignment between consecutive point SETS.  These are
     independent, so they run in parallel worker processes.
  3. Compose the permutations to get each drone's path, and lay out segments in time
     using the slowest drone of every move (min-jerk profile, speed/accel limits).
"""
from __future__ import annotations

import math
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from . import assignment
from .formations import build_formation, hex_grid_2d
from .spec import merged_settings
from .trajectory import A_PEAK, AXES, V_PEAK, CompiledShow, Keyframe, Segment, rodrigues


class CompileError(Exception):
    pass


def field_layout(n: int, spares: int, settings: dict):
    """Ground pads: a square grid, front row nearest the audience.  The last
    `spares` pads (back rows) hold spare drones.  Returns (slots (n+spares,3), is_spare)."""
    total = n + spares
    s = float(settings["takeoff_spacing"])
    cols = int(math.ceil(math.sqrt(total)))
    rows = int(math.ceil(total / cols))
    cx, cy = (list(settings.get("takeoff_center") or [0, 0]) + [0, 0])[:2]
    ix = np.arange(total) % cols
    iy = np.arange(total) // cols
    x = (ix - (cols - 1) / 2.0) * s + cx
    y = (iy - (rows - 1) / 2.0) * s + cy
    slots = np.stack([x, y, np.zeros(total)], axis=1)
    is_spare = np.zeros(total, dtype=bool)
    is_spare[n:] = True
    return slots, is_spare


def field_layout_for(spec: dict, n: int, spares: int, settings: dict):
    """Pad layout for a show spec: the generated grid, or the pads of an imported show."""
    if spec.get("imported"):
        from .importer import ShowImportError, field_layout_imported
        try:
            return field_layout_imported(spec, settings, spares)
        except ShowImportError as e:
            raise CompileError(str(e)) from e
    return field_layout(n, spares, settings)


def _round_up(t: float, step: float = 0.5) -> float:
    return math.ceil(t / step - 1e-9) * step


def min_move_time(P0: np.ndarray, P1: np.ndarray, s: dict) -> float:
    d = P1 - P0
    dh = np.hypot(d[:, 0], d[:, 1])
    up = np.clip(d[:, 2], 0, None)
    down = np.clip(-d[:, 2], 0, None)
    dist = np.linalg.norm(d, axis=1)
    T = max(
        V_PEAK * float(dh.max(initial=0)) / float(s["max_speed_h"]),
        V_PEAK * float(up.max(initial=0)) / float(s["max_speed_up"]),
        V_PEAK * float(down.max(initial=0)) / float(s["max_speed_down"]),
        math.sqrt(A_PEAK * float(dist.max(initial=0)) / float(s["max_accel"])),
        1.0,
    )
    return _round_up(T)


def spin_radius(P: np.ndarray, center: np.ndarray, axis: np.ndarray) -> float:
    a = axis / np.linalg.norm(axis)
    rel = P - center
    return float(np.linalg.norm(rel - np.outer(rel @ a, a), axis=1).max(initial=0))


def min_spin_time(P: np.ndarray, center: np.ndarray, axis: np.ndarray, angle: float, s: dict) -> float:
    r = spin_radius(P, center, axis)
    if r <= 0 or angle == 0:
        return 0.0
    horizontal = abs(axis[2]) > 0.99            # spinning about the vertical axis
    vmax = float(s["max_speed_h"]) if horizontal else min(
        float(s["max_speed_h"]), float(s["max_speed_up"]), float(s["max_speed_down"]))
    t_v = V_PEAK * abs(angle) * r / vmax
    t_a = math.sqrt(A_PEAK * abs(angle) * r / float(s["max_accel"]))
    t_c = V_PEAK * abs(angle) / math.sqrt(float(s["max_accel"]) / r)   # centripetal w^2 r <= amax
    return _round_up(max(t_v, t_a, t_c))


def parking_wall(count: int, formation: np.ndarray, spin_r: float, s: dict, warnings: list, name: str):
    """Hover positions for drones a formation does not use: a dark vertical wall
    behind the formation (away from the audience)."""
    sp = float(s["formation_spacing"])
    aspect = 2.5
    area = count * (math.sqrt(3) / 2) * sp * sp * 1.15
    W = math.sqrt(area * aspect)
    H = W / aspect
    pts = hex_grid_2d(W, H, sp)
    while len(pts) < count:
        W, H = W * 1.05, H * 1.05
        pts = hex_grid_2d(W, H, sp)
    order = np.argsort(np.maximum(np.abs(pts[:, 0]) / W, np.abs(pts[:, 1]) / H), kind="stable")
    pts = pts[order[:count]]
    c = formation.mean(0)
    depth = max(float(formation[:, 1].max() - c[1]), spin_r)
    y = c[1] + depth + 15.0
    gy = s["geofence"]["y"][1] - 5.0
    if y > gy:
        warnings.append(f"{name}: parking wall for {count} unlit drones is close to the back of the "
                        f"geofence (needs y={y:.0f} m)")
        y = gy
    zmin = float(s["min_altitude"]) + H / 2 + 3
    zmax = float(s["max_altitude"]) - H / 2 - 3
    z = min(max(c[2], zmin), max(zmin, zmax))
    return np.stack([pts[:, 0] + c[0], np.full(count, y), pts[:, 1] + z], axis=1)


def _solve_job(args):
    a, b, zs = args
    return assignment.solve(a, b, zs)


_POOL: ProcessPoolExecutor | None = None


def _pool(workers: int) -> ProcessPoolExecutor:
    global _POOL
    if _POOL is None or getattr(_POOL, "_max_workers", 0) != workers:
        if _POOL is not None:
            _POOL.shutdown(cancel_futures=True)
        _POOL = ProcessPoolExecutor(max_workers=workers)
    return _POOL


def _auto_workers(n: int, jobs: int) -> int:
    cpu = os.cpu_count() or 2
    mem_cap = 3 if n >= 4000 else 6      # each 5,000-drone job needs ~0.6 GB
    return max(1, min(jobs, cpu - 1 if cpu > 2 else cpu, mem_cap))


def solve_all(pairs: list, zs: float, n: int, progress=None, workers: int | None = None) -> list:
    keys = [assignment.set_key(a, b, zs) for a, b in pairs]
    out = [assignment.cache_get(k) for k in keys]
    todo = [i for i, o in enumerate(out) if o is None]
    if todo:
        w = workers if workers is not None else _auto_workers(n, len(todo))
        if w <= 1 or n < 1200:
            for c, i in enumerate(todo):
                if progress:
                    progress(c / len(todo), f"Assigning transition {c + 1}/{len(todo)}")
                out[i] = assignment.solve(pairs[i][0], pairs[i][1], zs)
        else:
            try:
                pool = _pool(w)
                futs = {i: pool.submit(_solve_job, (pairs[i][0], pairs[i][1], zs)) for i in todo}
                for c, i in enumerate(todo):
                    out[i] = futs[i].result()
                    if progress:
                        progress((c + 1) / len(todo), f"Assigned {c + 1}/{len(todo)} transitions")
            except Exception:   # noqa: BLE001 - e.g. no multiprocessing in this environment
                for i in todo:
                    if out[i] is None:
                        out[i] = assignment.solve(pairs[i][0], pairs[i][1], zs)
        for i in todo:
            assignment.cache_put(keys[i], out[i])
    return out


def compile_show(spec: dict, start_positions: np.ndarray | None = None, progress=None,
                 workers: int | None = None) -> CompiledShow:
    s = merged_settings(spec)
    if spec.get("imported"):
        from .importer import ShowImportError, build_imported_show
        try:
            return build_imported_show(spec, s, start_positions, progress)
        except ShowImportError as e:
            raise CompileError(str(e)) from e
    timeline = spec.get("timeline") or []
    if not timeline:
        raise CompileError("the timeline is empty - add at least one formation")
    if start_positions is None:
        n = int(spec.get("drones", 0))
        if n < 2:
            raise CompileError("a show needs at least 2 drones")
        start = field_layout(n, 0, s)[0][:n]
    else:
        start = np.array(start_positions, dtype=np.float64).copy()
        start[:, 2] = 0.0
        n = len(start)
    zs = float(s.get("downwash_factor", 1.0))
    warnings: list = []

    def report(frac, msg):
        if progress:
            progress(max(0.0, min(1.0, frac)), msg)

    lifted = start.copy()
    lifted[:, 2] = float(s["takeoff_altitude"])

    # ---- 1. point sets ---------------------------------------------------------------------
    report(0.02, "Building formations")
    forms = []
    for j, item in enumerate(timeline):
        name = item.get("name") or f"{item.get('shape', 'formation')} {j + 1}"
        try:
            f = build_formation(item, n, s)
        except Exception as e:   # noqa: BLE001 - surface generator errors to the user
            raise CompileError(f"formation {j + 1} ({name}): {e}") from e
        warnings += [f"{name}: {w}" for w in f.warnings]
        pts = f.points
        m = len(pts)
        act = np.ones(m, dtype=bool)
        cols = f.colors
        spin = item.get("spin") or {}
        angle = math.radians(float(spin.get("angle", 0.0) or 0.0))
        axis = np.array(AXES.get(spin.get("axis", "z"), AXES["z"]))
        center = pts.mean(0)
        r_spin = spin_radius(pts, center, axis) if angle else 0.0
        if m < n:
            park = parking_wall(n - m, pts, r_spin, s, warnings, name)
            pts = np.vstack([pts, park])
            act = np.concatenate([act, np.zeros(n - m, dtype=bool)])
            if cols is not None:
                cols = np.vstack([cols, np.zeros((n - m, 3), np.float32)])
        hold = float(item.get("hold", 5.0) or 0.0)
        end = pts
        if angle:
            t_need = min_spin_time(pts[act], center, axis, angle, s)
            if hold < t_need:
                warnings.append(f"{name}: hold extended to {t_need:.1f} s so the spin respects speed limits")
                hold = t_need
            end = pts.copy()
            end[act] = (pts[act] - center) @ rodrigues(axis, angle).T + center
        forms.append(dict(name=name, item=item, pts=pts, end=end, act=act, cols=cols, hold=hold,
                          angle=angle, axis=axis, center=center,
                          light=item.get("light") or {"effect": "solid", "color": "#ffffff"}))

    # ---- 2. assignments (parallel) -----------------------------------------------------------
    pairs = []
    src = lifted
    for f in forms:
        pairs.append((src, f["pts"]))
        src = f["end"]
    pairs.append((src, lifted))
    perms = solve_all(pairs, zs, n, progress=lambda fr, msg: report(0.08 + 0.85 * fr, msg),
                      workers=workers)

    # ---- 3. compose & lay out in time ---------------------------------------------------------
    report(0.95, "Timing segments")
    all_on = np.ones(n, dtype=bool)
    off = {"effect": "off"}
    kfs = [Keyframe(start.copy(), all_on, None, off, "Ground", -1),
           Keyframe(lifted, all_on, None, off, "Take-off layer", -1)]
    segs = []
    t = 0.0
    T = min_move_time(start, lifted, s)
    segs.append(Segment("takeoff", t, t + T, 0, 1, -1, "Take-off"))
    t += T
    segs.append(Segment("hold", t, t + 3.0, 1, 1, -1, "Take-off check"))
    t += 3.0

    q = np.arange(n)                  # q[i] = index of drone i in the current point set
    prev, prev_k = lifted, 1
    fstart = {}
    for j, f in enumerate(forms):
        q = perms[j][q]
        P = f["pts"][q]
        act = f["act"][q]
        cols = f["cols"][q] if f["cols"] is not None else None
        T_min = min_move_time(prev, P, s)
        T_req = float(f["item"].get("transition") or 0.0)
        T = max(T_min, _round_up(T_req)) if T_req else T_min
        kfs.append(Keyframe(P, act, cols, f["light"], f["name"], j))
        k_new = len(kfs) - 1
        segs.append(Segment("transition", t, t + T, prev_k, k_new, j, f"to {f['name']}"))
        t += T
        fstart[j] = t
        if f["angle"]:
            P_end = f["end"][q]
            kfs.append(Keyframe(P_end, act, cols, f["light"], f["name"], j))
            segs.append(Segment("spin", t, t + f["hold"], k_new, len(kfs) - 1, j, f["name"],
                                spin_axis=tuple(f["axis"]), spin_center=tuple(f["center"]),
                                spin_angle=f["angle"]))
            t += f["hold"]
            prev, prev_k = P_end, len(kfs) - 1
        else:
            if f["hold"] > 0:
                segs.append(Segment("hold", t, t + f["hold"], k_new, k_new, j, f["name"]))
                t += f["hold"]
            prev, prev_k = P, k_new

    q = perms[-1][q]                  # landing pad index of every drone
    P = lifted[q]
    kfs.append(Keyframe(P, all_on, None, off, "Landing layer", -1))
    T = min_move_time(prev, P, s)
    segs.append(Segment("transition", t, t + T, prev_k, len(kfs) - 1, -1, "Return to pads"))
    t += T
    segs.append(Segment("hold", t, t + 3.0, len(kfs) - 1, len(kfs) - 1, -1, "Landing check"))
    t += 3.0
    ground = P.copy()
    ground[:, 2] = 0.0
    kfs.append(Keyframe(ground, all_on, None, off, "Landed", -1))
    T = min_move_time(P, ground, s)
    segs.append(Segment("landing", t, t + T, len(kfs) - 2, len(kfs) - 1, -1, "Landing"))
    report(1.0, "Compiled")

    return CompiledShow(
        name=spec.get("name", "Show"), settings=s, start_positions=start, keyframes=kfs,
        segments=segs, formation_start=fstart, formation_names=[f["name"] for f in forms],
        warnings=warnings, landing_slots=q,
    )
