"""Show package export (.zip).

Contents
  manifest.json                    show summary, origin, per-drone pads, file formats
  show.json                        the editable show spec
  keyframes.npz                    every drone's keyframe positions (re-import / analysis)
  trajectories/drone_0001.csv      segments per drone (show frame + WGS84), min-jerk timing
  missions/drone_0001.waypoints    QGC WPL 110 mission (Mission Planner / QGroundControl)
  lights/drone_0001.lights         LED programme: 20-byte header + RGB bytes per frame

Imported (Skybrush) shows write imported_show.npz instead of keyframes.npz, and their
trajectory CSVs list the exact cubic Bezier segments of each drone.
"""
from __future__ import annotations

import datetime as dt
import io
import json
import math
import struct
import zipfile

import numpy as np

from .. import __version__
from ..geo import Origin
from .mission import mission_items, to_qgc_wpl

LIGHT_MAGIC = b"LSL1"


def light_frames(show, fps: int, progress=None) -> np.ndarray:
    F = int(math.ceil(show.duration * fps)) + 1
    out = np.zeros((F, show.n, 3), dtype=np.uint8)
    for f in range(F):
        out[f] = show.colors_u8(f / fps)
        if progress and f % 200 == 0:
            progress(f / F, f"Rendering lights {f}/{F}")
    return out


def light_file(frames_for_drone: np.ndarray, fps: int, index: int) -> bytes:
    hdr = LIGHT_MAGIC + struct.pack("<HHIII", int(fps), 1, len(frames_for_drone), int(index), 0)
    return hdr + np.ascontiguousarray(frames_for_drone, dtype=np.uint8).tobytes()


def trajectory_csv(show, i: int, origin: Origin) -> str:
    lines = ["# LangitSwarm trajectory v1 - show frame metres (x right, y away from audience, z up)",
             "# position(t) = P0 + s(tau)*(P1-P0), tau=(t-t0)/(t1-t0), s=10tau^3-15tau^4+6tau^5;"
             " spin rows rotate about axis through center",
             "t0,t1,kind,x0,y0,z0,x1,y1,z1,lat1,lon1,alt1,lit,spin_deg,axis,cx,cy,cz"]
    for seg in show.segments:
        P0 = show.keyframes[seg.k0].pos[i]
        P1 = show.keyframes[seg.k1].pos[i]
        la, lo, al = origin.show_to_wgs84(P1)
        lit = int(bool(show.keyframes[seg.k1].active[i]))
        if seg.kind == "spin":
            ax = "xyz"[int(np.argmax(np.abs(seg.spin_axis)))]
            c = seg.spin_center
            extra = f"{math.degrees(seg.spin_angle):.3f},{ax},{c[0]:.3f},{c[1]:.3f},{c[2]:.3f}"
        else:
            extra = "0,,,,"
        lines.append(f"{seg.t0:.3f},{seg.t1:.3f},{seg.kind},{P0[0]:.3f},{P0[1]:.3f},{P0[2]:.3f},"
                     f"{P1[0]:.3f},{P1[1]:.3f},{P1[2]:.3f},{float(la):.8f},{float(lo):.8f},{float(al):.3f},"
                     f"{lit},{extra}")
    return "\n".join(lines) + "\n"


def export_package(show, spec: dict, out_path: str, progress=None, include=("trajectories", "missions", "lights"),
                   start_utc: float | None = None) -> dict:
    s = show.settings
    origin = Origin.from_dict(s["origin"])
    fps = int(s.get("light_fps", 10))
    sync = s.get("mission_sync", "hold_time")
    n = show.n

    def rep(frac, msg):
        if progress:
            progress(frac, msg)

    imported = getattr(show, "imported", False)
    frames = None
    if "lights" in include and not imported:
        frames = light_frames(show, fps, progress=lambda f, m: rep(0.05 + 0.35 * f, m))
    if imported and "missions" in include:
        show.prepare_missions(origin, sync, start_utc, progress=lambda f, m: rep(0.05 + 0.35 * f, m))
    lat, lon, _ = origin.show_to_wgs84(show.start_positions)
    manifest = {
        "format": "langitswarm-show", "version": 1, "generator": f"Langit Swarm {__version__}",
        "name": show.name, "drones": n, "duration_s": round(show.duration, 3),
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
        "origin": s["origin"], "light_fps": fps, "mission_sync": sync,
        "start_utc": start_utc,
        "segments": show.timeline_summary(),
        "light_file": {"header": "4s magic 'LSL1', uint16 fps, uint16 version, uint32 frames, "
                                 "uint32 drone index, uint32 reserved (little endian)",
                       "frame": "3 bytes R,G,B; frame k plays at show time k/fps"},
        "takeoff_s": ([None if not math.isfinite(float(x)) else round(float(x), 3) for x in show.takeoff_offsets]
                      if imported else None),
        "imported": show.summary() if imported else None,
        "missions": dict(getattr(show, "mission_info", {}) or {}, tol=None) if imported else None,
        "pads": [{"drone": i + 1, "x": round(float(show.start_positions[i, 0]), 3),
                  "y": round(float(show.start_positions[i, 1]), 3),
                  "lat": round(float(lat[i]), 8), "lon": round(float(lon[i]), 8)} for i in range(n)],
    }
    with zipfile.ZipFile(out_path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=5) as z:
        z.writestr("manifest.json", json.dumps(manifest, indent=1))
        z.writestr("show.json", json.dumps(spec, indent=1))
        if imported:
            z.writestr("imported_show.npz", show.npz_bytes())
        else:
            buf = io.BytesIO()
            np.savez_compressed(buf, keyframes=np.stack([k.pos for k in show.keyframes]).astype(np.float32),
                                active=np.stack([k.active for k in show.keyframes]),
                                t0=np.array([sg.t0 for sg in show.segments]),
                                t1=np.array([sg.t1 for sg in show.segments]))
            z.writestr("keyframes.npz", buf.getvalue())
        for i in range(n):
            name = f"drone_{i + 1:04d}"
            if "trajectories" in include:
                z.writestr(f"trajectories/{name}.csv",
                           show.trajectory_csv(i, origin) if imported else trajectory_csv(show, i, origin))
            if "missions" in include:
                z.writestr(f"missions/{name}.waypoints",
                           to_qgc_wpl(mission_items(show, i, origin, sync, start_utc)))
            if frames is not None:
                z.writestr(f"lights/{name}.lights", light_file(frames[:, i], fps, i))
            elif imported and "lights" in include:
                z.writestr(f"lights/{name}.lights", light_file(show.drone_light_frames(i, fps), fps, i))
            if i % 100 == 0:
                rep(0.4 + 0.6 * i / n, f"Writing drone files {i}/{n}")
    rep(1.0, "Export complete")
    return {"path": out_path, "drones": n, "files": n * len(include) + 3}
