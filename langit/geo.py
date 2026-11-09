"""Coordinate frames.

Show frame (used everywhere inside the app), in metres:
    x  = to the right, as seen by the audience
    y  = depth, pointing away from the audience
    z  = up (above the take-off field)
The origin is the centre of the show on the ground.  ``heading_deg`` is the
compass direction the audience looks in (the direction of +y).

Conversion to WGS84 uses a local tangent-plane approximation, which is
accurate to a few millimetres over the ~1 km footprint of a light show.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

_A = 6378137.0                 # WGS84 semi-major axis
_E2 = 6.69437999014e-3         # WGS84 first eccentricity squared


@dataclass
class Origin:
    lat: float
    lon: float
    alt_amsl: float = 0.0
    heading_deg: float = 0.0

    @classmethod
    def from_dict(cls, d: dict) -> "Origin":
        return cls(float(d.get("lat", 0.0)), float(d.get("lon", 0.0)),
                   float(d.get("alt_amsl", 0.0)), float(d.get("heading_deg", 0.0)))

    def _radii(self):
        phi = math.radians(self.lat)
        s = math.sin(phi)
        w = math.sqrt(1.0 - _E2 * s * s)
        r_meridian = _A * (1.0 - _E2) / (w ** 3)
        r_normal = _A / w
        return r_meridian, r_normal * math.cos(phi)

    # -- show frame <-> local East/North --------------------------------------------------
    def show_to_en(self, x, y):
        h = math.radians(self.heading_deg)
        ch, sh = math.cos(h), math.sin(h)
        east = x * ch + y * sh
        north = -x * sh + y * ch
        return east, north

    def en_to_show(self, east, north):
        h = math.radians(self.heading_deg)
        ch, sh = math.cos(h), math.sin(h)
        x = east * ch - north * sh
        y = east * sh + north * ch
        return x, y

    # -- show frame <-> WGS84 ---------------------------------------------------------------
    def show_to_wgs84(self, xyz: np.ndarray):
        """xyz (..., 3) in show frame -> (lat_deg, lon_deg, rel_alt_m) arrays."""
        xyz = np.asarray(xyz, dtype=np.float64)
        east, north = self.show_to_en(xyz[..., 0], xyz[..., 1])
        rm, rn = self._radii()
        lat = self.lat + np.degrees(north / rm)
        lon = self.lon + np.degrees(east / rn)
        return lat, lon, xyz[..., 2].copy()

    def wgs84_to_show(self, lat, lon, rel_alt):
        lat = np.asarray(lat, dtype=np.float64)
        lon = np.asarray(lon, dtype=np.float64)
        rm, rn = self._radii()
        north = np.radians(lat - self.lat) * rm
        east = np.radians(lon - self.lon) * rn
        x, y = self.en_to_show(east, north)
        return np.stack([x, y, np.asarray(rel_alt, dtype=np.float64)], axis=-1)
