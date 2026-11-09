"""Pad mapping, flyer selection and pre-flight checks."""
from __future__ import annotations

import numpy as np

from scipy.spatial import cKDTree
from .state import (F_LINK, F_LOW_BATT, F_NO_MISSION, F_NO_RTK, F_OFF_PAD, F_SENSORS, FleetState)

CHECKS = [
    ("link", "Telemetry link", F_LINK),
    ("gps", "RTK / GPS fix", F_NO_RTK),
    ("battery", "Battery", F_LOW_BATT),
    ("sensors", "Sensors healthy", F_SENSORS),
    ("pad", "On assigned pad", F_OFF_PAD),
    ("mission", "Show uploaded", F_NO_MISSION),
]


def map_to_pads(st: FleetState, pads: np.ndarray, max_offset: float, now: float, link_timeout: float = 3.0):
    """Match every connected, landed drone to its nearest pad.  Pads are ~2 m apart and
    drones must sit within max_offset (~0.5 m) of a pad, so nearest-pad is unambiguous;
    if two drones claim one pad the closer one wins and the other is flagged off-pad.
    Writes st.pad (-1 = unmatched). Returns offsets (m,) metres (inf if unmatched)."""
    m = st.m
    st.pad[:m] = -1
    off = np.full(m, np.inf)
    seen = (now - st.last_seen[:m]) < link_timeout
    cand = np.nonzero(seen & ~np.isnan(st.pos[:m, 0]) & ~st.in_air[:m])[0]
    if len(cand) == 0:
        return off
    d, p = cKDTree(pads[:, :2]).query(st.pos[cand, :2], k=1)
    order = np.argsort(d, kind="stable")
    taken = np.zeros(len(pads), bool)
    for k in order:
        if not taken[p[k]]:
            taken[p[k]] = True
            st.pad[cand[k]] = p[k]
            off[cand[k]] = d[k]
    return off


def check_drones(st: FleetState, s: dict, now: float, offsets: np.ndarray, require_mission: bool):
    """Per-drone readiness.  Returns (ready mask (m,), flags (m,) uint32)."""
    m = st.m
    flags = np.zeros(m, np.uint32)
    link = (now - st.last_seen[:m]) < 3.0
    flags[~link] |= F_LINK
    need_fix = 6 if s.get("require_rtk", True) else 3
    gps = (st.fix[:m] >= need_fix) & (st.sats[:m] >= int(s.get("min_satellites", 12)))
    flags[~gps] |= F_NO_RTK
    batt = st.batt[:m] >= float(s.get("min_battery_preflight", 95.0))
    flags[~batt] |= F_LOW_BATT
    flags[~st.sensors_ok[:m]] |= F_SENSORS
    pad = (st.pad[:m] >= 0) & (offsets <= float(s.get("max_slot_offset", 0.5)))
    flags[~pad] |= F_OFF_PAD
    if require_mission:
        flags[~st.uploaded[:m]] |= F_NO_MISSION
    ready = flags == 0
    return ready, flags


def choose_flyers(st: FleetState, ready: np.ndarray, is_spare_pad: np.ndarray, n_needed: int, pads: np.ndarray,
                  use_spares: bool = True):
    """Pick the drones that fly: every ready drone on a primary pad, then spares
    closest to the empty primary pads.  Returns fleet indices (sorted by pad).
    Imported shows have a fixed path per pad, so spares cannot stand in (use_spares=False)."""
    m = st.m
    idx = np.nonzero(ready & (st.pad[:m] >= 0))[0]
    on_primary = idx[~is_spare_pad[st.pad[idx]]]
    spares = idx[is_spare_pad[st.pad[idx]]]
    chosen = list(on_primary[:n_needed])
    missing = n_needed - len(chosen)
    if missing > 0 and len(spares) and use_spares:
        used = set(int(st.pad[i]) for i in chosen)
        holes = np.array([p for p in range(len(is_spare_pad)) if not is_spare_pad[p] and p not in used])
        if len(holes):
            c = pads[holes, :2].mean(0)
            d = np.hypot(*(pads[st.pad[spares], :2] - c).T)
            spares = spares[np.argsort(d)]
        chosen += list(spares[:missing])
    chosen = np.array(sorted(chosen, key=lambda i: int(st.pad[i])), dtype=int)
    return chosen
