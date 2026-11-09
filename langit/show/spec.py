"""Show specification: settings, defaults and a demo show.

A show is a plain JSON document so it can be saved, versioned and shared:

{
  "name": "...",
  "drones": 3000,               # drones that fly the show
  "spares": 60,                 # extra drones placed on the field as replacements
  "settings": {...},            # see DEFAULT_SETTINGS
  "timeline": [ formation, ... ]
}

Each formation:
{
  "name": "Logo",
  "shape": "text" | "image" | "sphere" | "torus" | "cube" | "cylinder" | "cone" |
           "wave" | "wall" | "disk" | "ring" | "heart" | "star" | "points",
  "params": {...},              # shape specific (text, image data URL, size, ...)
  "count": 1.0,                 # fraction (<=1) or absolute number of drones used
  "center": [x, y, z],          # show frame metres
  "rotation": [yaw, pitch, roll] degrees,
  "plane": "vertical" | "horizontal"   (2D shapes only)
  "transition": 0,              # seconds; 0 = fastest safe time
  "hold": 8,                    # seconds the formation is held
  "spin": {"angle": 0, "axis": "z"},   # rotate while holding
  "light": {"effect": "solid", "color": "#ffffff", "color2": "#ff0000", ...}
}
"""
from __future__ import annotations

import copy
import math

from .brand_data import EMBLEM_PNG

DEFAULT_SETTINGS = {
    # --- safety distances (metres) ---------------------------------------------------
    "min_separation": 1.5,      # hard minimum distance between any two drone centres
    "formation_spacing": 2.2,   # neighbour spacing inside formations (>= sqrt(2)*min_sep)
    "takeoff_spacing": 2.2,     # pad spacing on the ground grid (>= sqrt(2)*min_sep)
    "downwash_factor": 1.0,     # >1 = treat vertical distance as shorter (ellipsoid)
    # --- dynamics -------------------------------------------------------------------------
    "max_speed_h": 6.0,         # m/s horizontal
    "max_speed_up": 4.0,        # m/s climb
    "max_speed_down": 3.0,      # m/s descent
    "max_accel": 2.5,           # m/s^2
    # --- altitudes ------------------------------------------------------------------------
    "takeoff_altitude": 8.0,    # first hover layer above the pads
    "min_altitude": 5.0,        # formations must stay above this
    "max_altitude": 120.0,      # ceiling AGL - 120 m (400 ft) is the usual limit; raise only with approval
    # --- field layout (show frame) ------------------------------------------------------
    "takeoff_center": [0.0, 0.0],
    "geofence": {"x": [-260.0, 260.0], "y": [-120.0, 220.0], "z": [0.0, 125.0]},
    "audience_y": -150.0,       # audience stands at y < audience_y
    "audience_buffer": 30.0,    # drones must stay this far behind audience_y
    # --- geo reference ------------------------------------------------------------------
    "origin": {"lat": -6.1214, "lon": 106.8441, "alt_amsl": 5.0, "heading_deg": 0.0},
    # --- power ----------------------------------------------------------------------------
    "battery_minutes": 18.0,    # hover endurance with lights on
    "battery_reserve_pct": 25.0,
    # --- lights -----------------------------------------------------------------------------
    "transition_dim": 0.35,     # brightness while drones travel between formations
    "takeoff_color": "#3a3a3a",
    "light_fps": 10,
    # --- live operations ------------------------------------------------------------------
    "deviation_warn": 2.0,      # m from planned position -> warning
    "deviation_abort": 5.0,     # m -> drone is taken out of the show (safe exit)
    "min_battery_preflight": 95.0,
    "critical_battery": 20.0,
    "require_rtk": True,
    "min_satellites": 12,
    "max_slot_offset": 0.5,     # m between a drone and its pad
    "min_ready_fraction": 0.97, # minimum share of drones that must be ready to fly
    "max_wind": 8.0,            # m/s operator go/no-go limit
    "mission_sync": "hold_time", # "hold_time" (PX4 + ArduPilot) or "utc_delay" (ArduPilot)
    "mission_max_items": 700,   # imported shows: path simplification is loosened to stay below this
}


DEMO_TIMELINE = [
    {
        "name": "Opening sphere",
        "shape": "sphere", "params": {}, "count": 1.0,
        "center": [0, 20, 64], "rotation": [0, 0, 0],
        "transition": 0, "hold": 16,
        "spin": {"angle": 90, "axis": "z"},
        "light": {"effect": "rainbow", "axis": "z", "speed": 0.15, "cycles": 1.0},
    },
    {
        "name": "INDONESIA",
        "shape": "text", "params": {"text": "INDONESIA", "style": "fill"}, "count": 0.35,
        "center": [0, 0, 64], "rotation": [0, 0, 0], "plane": "vertical",
        "transition": 0, "hold": 8,
        "light": {"effect": "wave", "color": "#ffffff", "color2": "#ff2a2a",
                  "axis": "x", "speed": 0.6},
    },
    {
        "name": "Merah Putih",
        "shape": "wall", "params": {"aspect": 1.5}, "count": 0.6,
        "center": [0, 0, 64], "rotation": [0, 0, 0], "plane": "vertical",
        "transition": 0, "hold": 8,
        "light": {"effect": "split", "color": "#ff1a1a", "color2": "#ffffff", "axis": "z",
                  "ripple": 0.25},
    },
    {
        "name": "Heart",
        "shape": "heart", "params": {}, "count": 0.4,
        "center": [0, 0, 64], "rotation": [0, 0, 0], "plane": "vertical",
        "transition": 0, "hold": 7,
        "light": {"effect": "pulse", "color": "#ff2d6f", "rate": 1.0, "low": 0.35},
    },
    {
        "name": "Torus",
        "shape": "torus", "params": {"ratio": 0.33}, "count": 0.6,
        "center": [0, 20, 64], "rotation": [0, 75, 0],
        "transition": 0, "hold": 18,
        "spin": {"angle": 60, "axis": "z"},
        "light": {"effect": "gradient", "color": "#00e5ff", "color2": "#ff00d4", "axis": "x"},
    },
    {
        "name": "2026",
        "shape": "text", "params": {"text": "2026", "style": "fill"}, "count": 0.3,
        "center": [0, 0, 64], "rotation": [0, 0, 0], "plane": "vertical",
        "transition": 0, "hold": 8,
        "light": {"effect": "twinkle", "color": "#ffc23d", "density": 0.12},
    },
    {
        "name": "Langit Swarm",
        "shape": "image", "params": {"image": EMBLEM_PNG}, "count": 0.45,
        "center": [0, 0, 64], "rotation": [0, 0, 0], "plane": "vertical",
        "transition": 0, "hold": 10,
        "light": {"effect": "solid", "color": "#ff4a12"},   # brand orange, brightened for LEDs
    },
]


def default_show(drones: int = 3000, spares: int | None = None) -> dict:
    if spares is None:
        spares = max(10, int(math.ceil(drones * 0.02)))
    return {
        "name": "Demo Show",
        "drones": int(drones),
        "spares": int(spares),
        "settings": copy.deepcopy(DEFAULT_SETTINGS),
        "timeline": copy.deepcopy(DEMO_TIMELINE),
    }


def merged_settings(spec: dict) -> dict:
    s = copy.deepcopy(DEFAULT_SETTINGS)
    for k, v in (spec.get("settings") or {}).items():
        if isinstance(v, dict) and isinstance(s.get(k), dict):
            s[k].update(v)
        else:
            s[k] = v
    # The collision-free guarantee needs start/goal spacing >= sqrt(2) * min_separation.
    need = math.sqrt(2.0) * float(s["min_separation"]) * 1.0001
    s["formation_spacing"] = max(float(s["formation_spacing"]), need)
    s["takeoff_spacing"] = max(float(s["takeoff_spacing"]), need)
    return s
