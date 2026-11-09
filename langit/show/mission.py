"""Per-drone MAVLink missions generated from a compiled show.

This is the portable way to fly a show on stock PX4 / ArduPilot: each drone gets
a waypoint mission that follows its straight-line show path.  Every transition is
preceded by a DO_CHANGE_SPEED so all drones arrive at the same time (drones move
proportionally along their lines, which keeps the CAPT collision guarantee).

sync modes
  hold_time  - hold durations are encoded as waypoint hold times (PX4 + ArduPilot).
               Timing can drift by the drones' acceleration differences.
  utc_delay  - ArduPilot only: every hold ends with NAV_DELAY until an absolute UTC
               time, re-synchronising all drones to the GPS clock at every formation.

For tight synchronisation of large fleets, an onboard show player that executes
the exported trajectory against GPS time (see docs) is the professional option.
"""
from __future__ import annotations

import datetime as dt
import math

import numpy as np

from ..geo import Origin

MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
CMD_WAYPOINT = 16
CMD_LAND = 21
CMD_TAKEOFF = 22
CMD_DELAY = 93
CMD_CHANGE_SPEED = 178
MIN_SPEED = 0.3


def _item(cmd, lat=0.0, lon=0.0, alt=0.0, p1=0.0, p2=0.0, p3=0.0, p4=0.0, frame=MAV_FRAME_GLOBAL_RELATIVE_ALT):
    return {"frame": frame, "command": cmd, "p1": float(p1), "p2": float(p2), "p3": float(p3),
            "p4": float(p4), "lat": float(lat), "lon": float(lon), "alt": float(alt)}


def _utc_parts(t_utc: float):
    d = dt.datetime.fromtimestamp(t_utc, tz=dt.timezone.utc)
    return d.hour, d.minute, d.second + d.microsecond / 1e6


def mission_items(show, i: int, origin: Origin, sync_mode: str = "hold_time",
                  start_utc: float | None = None, start_slot_xyz: np.ndarray | None = None) -> list:
    """Mission items (list of dicts) for show drone index i."""
    if getattr(show, "imported", False):
        return show.mission_items(i, origin, sync_mode, start_utc, start_slot_xyz)
    from .trajectory import rodrigues
    items = []
    pad = show.start_positions[i] if start_slot_xyz is None else start_slot_xyz
    lat, lon, _ = origin.show_to_wgs84(pad)
    items.append(_item(CMD_WAYPOINT, lat, lon, origin.alt_amsl, frame=MAV_FRAME_GLOBAL))   # home
    use_delay = sync_mode == "utc_delay" and start_utc is not None

    def wgs(p):
        a, b, c = origin.show_to_wgs84(np.asarray(p))
        return float(a), float(b), float(c)

    segs = show.segments
    k = 0
    while k < len(segs):
        seg = segs[k]
        P0 = show.keyframes[seg.k0].pos[i]
        P1 = show.keyframes[seg.k1].pos[i]
        T = max(seg.duration, 1e-3)
        if seg.kind == "takeoff":
            la, lo, al = wgs(P1)
            items.append(_item(CMD_TAKEOFF, la, lo, al))
        elif seg.kind == "transition":
            d = P1 - P0
            dh = math.hypot(d[0], d[1])
            items.append(_item(CMD_CHANGE_SPEED, p1=1, p2=max(MIN_SPEED, dh / T), p3=-1))
            if abs(d[2]) > 0.5:
                items.append(_item(CMD_CHANGE_SPEED, p1=2 if d[2] > 0 else 3,
                                   p2=max(MIN_SPEED, abs(d[2]) / T), p3=-1))
            hold = 0.0
            nxt = segs[k + 1] if k + 1 < len(segs) else None
            if nxt is not None and nxt.kind == "hold" and not use_delay:
                hold = nxt.duration
                k += 1
            la, lo, al = wgs(P1)
            items.append(_item(CMD_WAYPOINT, la, lo, al, p1=hold))
        elif seg.kind == "hold":
            if use_delay:
                h, m, sec = _utc_parts(start_utc + seg.t1)
                items.append(_item(CMD_DELAY, p1=-1, p2=h, p3=m, p4=sec))
            else:
                la, lo, al = wgs(P0)
                items.append(_item(CMD_WAYPOINT, la, lo, al, p1=seg.duration))
        elif seg.kind == "spin":
            kf = show.keyframes[seg.k0]
            if not kf.active[i]:                 # parked drone just waits
                la, lo, al = wgs(P0)
                items.append(_item(CMD_WAYPOINT, la, lo, al, p1=seg.duration))
            else:
                steps = max(2, int(math.ceil(abs(math.degrees(seg.spin_angle)) / 10.0)))
                c = np.asarray(seg.spin_center)
                axis = np.asarray(seg.spin_axis)
                r = np.linalg.norm((P0 - c) - ((P0 - c) @ axis) * axis)
                arc = abs(seg.spin_angle) * r
                items.append(_item(CMD_CHANGE_SPEED, p1=1, p2=max(MIN_SPEED, arc / T), p3=-1))
                for st in range(1, steps + 1):
                    p = (P0 - c) @ rodrigues(axis, seg.spin_angle * st / steps).T + c
                    la, lo, al = wgs(p)
                    items.append(_item(CMD_WAYPOINT, la, lo, al))
            if use_delay:
                h, m, sec = _utc_parts(start_utc + seg.t1)
                items.append(_item(CMD_DELAY, p1=-1, p2=h, p3=m, p4=sec))
        elif seg.kind == "landing":
            la, lo, _ = wgs(P1)
            items.append(_item(CMD_LAND, la, lo, 0.0))
        k += 1
    for n, it in enumerate(items):
        it["seq"] = n
    return items


def to_qgc_wpl(items: list) -> str:
    lines = ["QGC WPL 110"]
    for it in items:
        lines.append("\t".join([
            str(it["seq"]), "1" if it["seq"] == 0 else "0", str(it["frame"]), str(it["command"]),
            f"{it['p1']:.6f}", f"{it['p2']:.6f}", f"{it['p3']:.6f}", f"{it['p4']:.6f}",
            f"{it['lat']:.8f}", f"{it['lon']:.8f}", f"{it['alt']:.3f}", "1"]))
    return "\n".join(lines) + "\n"
