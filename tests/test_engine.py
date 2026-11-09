"""Engine tests.  Run:  python -m pytest tests  (or  python tests/test_engine.py)"""
import io
import math
import os
import struct
import sys
import zipfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

from langit.geo import Origin  # noqa: E402
from langit.show.compiler import compile_show, field_layout  # noqa: E402
from langit.show.export import export_package  # noqa: E402
from langit.show.formations import ALL_SHAPES, build_formation, min_nn_distance  # noqa: E402
from langit.show.mission import mission_items  # noqa: E402
from langit.show.spec import default_show, merged_settings  # noqa: E402
from langit.show.trajectory import CompiledShow, Keyframe, Segment  # noqa: E402
from langit.show.validator import validate  # noqa: E402


def test_formations_exact_count_and_spacing():
    s = merged_settings(default_show(600))
    for shape in ALL_SHAPES:
        item = {"shape": shape, "count": 0.5, "center": [0, 0, 60], "params": {}}
        if shape == "points":
            g = np.stack(np.meshgrid(np.arange(20), np.arange(20)), -1).reshape(-1, 2) * 3.0
            item["params"]["csv"] = "\n".join(f"{x},{y},0" for x, y in g)
        if shape == "image":
            continue        # needs an image; covered by the UI
        f = build_formation(item, 600, s)
        assert len(f.points) == 300, shape
        assert min_nn_distance(f.points) >= s["formation_spacing"] * 0.999, shape


def test_compile_and_validate_demo():
    sh = compile_show(default_show(400))
    r = validate(sh)
    assert r["ok"], [c for c in r["checks"] if c["status"] == "fail"]
    assert r["stats"]["min_separation_m"] is None or r["stats"]["min_separation_m"] >= 1.5


def test_validator_catches_head_on_collision():
    s = merged_settings(default_show(2))
    A = np.array([[0.0, 0, 50], [10.0, 0, 50]])
    B = A[::-1].copy()                                 # swap -> they meet in the middle
    on = np.ones(2, bool)
    kfs = [Keyframe(A, on, None, {"effect": "off"}, "a", 0), Keyframe(B, on, None, {"effect": "off"}, "b", 0)]
    segs = [Segment("transition", 0.0, 10.0, 0, 1, 0, "swap")]
    sh = CompiledShow("t", s, A * [1, 1, 0], kfs, segs, {0: 10.0}, ["b"])
    r = validate(sh)
    sep = [c for c in r["checks"] if c["id"] == "separation"][0]
    assert sep["status"] == "fail" and r["violations"], r


def test_geo_round_trip():
    o = Origin(-6.12, 106.84, 5.0, 37.0)
    p = np.array([[120.0, -45.0, 80.0], [-300.0, 250.0, 10.0]])
    lat, lon, alt = o.show_to_wgs84(p)
    back = o.wgs84_to_show(lat, lon, alt)
    assert np.abs(back - p).max() < 1e-3


def test_mission_and_export(tmp_path=None):
    spec = default_show(60, 0)
    sh = compile_show(spec)
    o = Origin.from_dict(sh.settings["origin"])
    items = mission_items(sh, 0, o)
    cmds = [it["command"] for it in items]
    assert cmds[1] == 22 and cmds[-1] == 21          # take-off ... land
    path = os.path.join(tmp_path or "/tmp", "lsw_test_export.zip")
    export_package(sh, spec, path)
    z = zipfile.ZipFile(path)
    names = z.namelist()
    assert "manifest.json" in names and "missions/drone_0060.waypoints" in names
    raw = z.read("lights/drone_0001.lights")
    magic, fps, ver, frames, idx, _ = struct.unpack("<4sHHIII", raw[:20])
    assert magic == b"LSL1" and len(raw) == 20 + 3 * frames and frames == math.ceil(sh.duration * fps) + 1
    np.load(io.BytesIO(z.read("keyframes.npz")))


def test_takeoff_pads_spacing():
    s = merged_settings(default_show(5000))
    pads, spare = field_layout(5000, 100, s)
    assert len(pads) == 5100 and spare.sum() == 100
    assert min_nn_distance(pads) >= math.sqrt(2) * s["min_separation"]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
