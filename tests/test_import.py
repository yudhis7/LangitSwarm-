"""Blender / Skybrush Studio importer tests.

    python tests/test_import.py            (synthetic shows; also the Skybrush viewer demo if given)
    python tests/test_import.py demo.skyc  (additionally import a real .skyc / show.json)
"""
import io
import json
import math
import os
import sys
import tempfile
import time
import zipfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))
TMP = tempfile.mkdtemp(prefix="langit_import_")
os.environ["LANGIT_HOME"] = TMP

import skybrush_fixture as fx  # noqa: E402
from langit.geo import Origin  # noqa: E402
from langit.show import importer as im  # noqa: E402
from langit.show.compiler import compile_show, field_layout_for  # noqa: E402
from langit.show.export import export_package  # noqa: E402
from langit.show.mission import mission_items  # noqa: E402
from langit.show.spec import default_show, merged_settings  # noqa: E402
from langit.show.validator import validate  # noqa: E402


def spec_for(path, drones, rotate=0.0, offset=None, settings=None):
    raw = im.load_raw(path)
    s = merged_settings({"settings": settings or {}})
    off = offset if offset is not None else im.centre_offset(raw, rotate, s["takeoff_center"])
    return {"name": "t", "drones": drones, "spares": 0, "settings": settings or {}, "timeline": [],
            "imported": {"file": path, "sha256": raw.sha256, "drones": drones, "rotate_deg": rotate,
                         "offset": off, "tolerance": 0.25, "format": raw.meta["format"]}}


def run_mission(items, origin, dt=0.05, t_start=0.0, t_max=400.0):
    """Ideal mission executor: constant-speed legs (horizontal and vertical speed set by
    DO_CHANGE_SPEED, as in tools/mavlink_emulator.py), exact timing, no acceleration.
    Measures the mission plan (path tolerance + timing), not autopilot dynamics.
    Returns sample times (k,) and positions (k,3) in the show frame."""
    pos = np.asarray(origin.wgs84_to_show(items[0]["lat"], items[0]["lon"], 0.0), float)
    pos[2] = 0.0
    vh, vu, vd = 5.0, 2.5, 2.5
    t = t_start
    legs = []                                   # (t0, t1, A, B, vh, vz)
    for it in items[1:]:
        c = it["command"]
        if c == 178:
            if int(it["p1"]) == 1:
                vh = it["p2"]
            elif int(it["p1"]) == 2:
                vu = it["p2"]
            else:
                vd = it["p2"]
            continue
        if c not in (16, 22, 21):
            continue
        tgt = np.asarray(origin.wgs84_to_show(it["lat"], it["lon"], it["alt"]), float)
        if c == 22:
            tgt = np.array([pos[0], pos[1], it["alt"]])
        if c == 21:
            tgt[2] = 0.0
        d = tgt - pos
        dh = math.hypot(d[0], d[1])
        vz = vu if d[2] > 0 else vd
        T = max(dh / vh, abs(d[2]) / vz)
        legs.append((t, t + T, pos.copy(), tgt.copy(), vh, vz))
        t += T
        pos = tgt
        if c == 16 and it["p1"] > 0:
            legs.append((t, t + it["p1"], pos.copy(), pos.copy(), vh, vz))
            t += it["p1"]
    times = np.arange(t_start, t + 1e-9, dt)
    P = np.zeros((len(times), 3))
    k = 0
    for j, tt in enumerate(times):
        while k < len(legs) - 1 and tt > legs[k][1]:
            k += 1
        t0, t1, A, B, vh_, vz_ = legs[k]
        tau = min(max(tt - t0, 0.0), t1 - t0)
        d = B - A
        dh = math.hypot(d[0], d[1])
        q = A.copy()
        if dh > 1e-9:
            q[:2] += d[:2] / dh * min(dh, vh_ * tau)
        if abs(d[2]) > 1e-9:
            q[2] += math.copysign(min(abs(d[2]), vz_ * tau), d[2])
        P[j] = q
    return times, P


def main(extra=None):
    ok = True

    def check(cond, msg):
        nonlocal ok
        print(("PASS " if cond else "FAIL ") + msg)
        ok &= bool(cond)

    # ---------------------------------------------------------------- synthetic .skyc
    drones = fx.build(64)
    skyc = os.path.join(TMP, "synthetic.skyc")
    fx.write_skyc(skyc, drones)
    t = time.perf_counter()
    raw = im.parse_file(skyc, sha="test")
    check(raw.n == 64, f"parsed {raw.n} drones from .skyc in {time.perf_counter() - t:.2f} s")
    check(len(raw.cc) == 64 * 4, f"cubic Bezier segments kept ({len(raw.cc)})")
    s = merged_settings({"settings": {}})
    sh = im.ImportedShow(raw, s, "synthetic")
    # positions vs independent reference (no transform)
    err = 0.0
    for tt in np.linspace(0, sh.duration, 377):
        P = sh.positions(tt)
        R = np.array([fx.ref_position(d, tt) for d in drones])
        err = max(err, float(np.abs(P - R).max()))
    check(err < 2e-3, f"trajectory matches the Skybrush Bezier definition (max error {err * 1000:.2f} mm)")
    # lights vs independent reference, avoiding the exact step instants
    cerr = 0.0
    for tt in np.linspace(0.013, sh.duration - 0.01, 501):
        C = sh.colors(tt) * 255.0
        R = np.array([fx.ref_color(d["lights"], tt) for d in drones])
        cerr = max(cerr, float(np.abs(C - R).max()))
    check(cerr < 1.0, f"LED bytecode decoded (SET/FADE/WAIT_UNTIL/LOOP), max colour error {cerr:.2f}/255")
    # phases / take-off offsets
    tko = np.array([d["takeoff"] - 1.0 for d in drones])
    check(np.allclose(sh.takeoff_offsets, tko, atol=1e-6), "staggered take-off times found per drone")
    check(abs(sh.ph["z_climb"].min() - 12.0) < 1e-6, "vertical take-off climb detected (to 12 m)")
    kinds = [x["kind"] for x in sh.timeline_summary()]
    labels = [x["label"] for x in sh.timeline_summary()]
    check(kinds[0] == "hold" and "takeoff" in kinds and kinds[-1] == "landing" and "Turn" in labels,
          f"scenes from take-off/landing phases and cues: {labels}")

    # ---------------------------------------------------------------- spec / compile / validate
    spec = spec_for(skyc, 64)
    show = compile_show(spec)
    check(show.imported and abs(show.start_positions[:, 0].mean()) < 1e-6,
          "compile_show() dispatches to the importer; pads centred on the take-off field")
    val = validate(show)
    for c in val["checks"]:
        print("     ", c["status"], c["name"], "-", c["detail"][:110])
    check(val["ok"], f"synthetic show validates (closest {val['stats']['min_separation_m']} m)")
    # rotation / offset are rigid: separation identical, extents move
    spec_r = spec_for(skyc, 64, rotate=90.0, offset=[10.0, 20.0])
    show_r = compile_show(spec_r)
    t5 = 40.0
    A, B = show.positions(t5), show_r.positions(t5)
    dA = np.linalg.norm(A[:, None] - A[None], axis=-1)
    dB = np.linalg.norm(B[:, None] - B[None], axis=-1)
    check(np.abs(dA - dB).max() < 1e-3, "rotation + offset keep all distances")
    v2 = validate(show_r)
    check(v2["stats"]["min_separation_m"] == val["stats"]["min_separation_m"],
          "separation result reused from the cache for a moved show")
    # deliberately unsafe: two drones swap positions through each other
    bad = fx.build(16)
    bad[0]["points"][4] = (bad[0]["points"][4][0], bad[1]["points"][4][1].copy(), [])
    bad_path = os.path.join(TMP, "bad.skyc")
    fx.write_skyc(bad_path, bad)
    vb = validate(compile_show(spec_for(bad_path, 16)))
    sep = [c for c in vb["checks"] if c["id"] == "separation"][0]
    check(sep["status"] == "fail" and vb["violations"], f"collision is caught: {sep['detail'][:90]}")

    # ---------------------------------------------------------------- live re-plan subset
    pads, spare = field_layout_for(spec, 64, 10, s)
    check(len(pads) == 74 and spare.sum() == 10 and np.allclose(pads[:64], show.start_positions),
          "field layout = imported pads + spare rows")
    keep = np.setdiff1d(np.arange(64), [5, 17, 40])
    sub = compile_show(spec, start_positions=pads[keep])
    check(sub.n == 61 and np.allclose(sub.positions(50.0), show.positions(50.0)[keep]),
          "re-plan with 3 missing drones flies the other 61 paths unchanged")
    vs = validate(sub)
    check(vs["ok"], "subset validates from the cached separation scan")

    # ---------------------------------------------------------------- missions
    origin = Origin.from_dict(s["origin"])
    worst = 0.0
    worst_t = 0.0
    nitems = []
    for i in range(0, 64, 7):
        items = mission_items(show, i, origin)
        nitems.append(len(items))
        T, P = run_mission(items, origin, t_start=float(show.takeoff_offsets[i]))
        ref = show.eval_positions(T, np.full(len(T), i))
        dev = np.linalg.norm(P - ref, axis=1)
        worst = max(worst, float(dev.max()))
        worst_t = max(worst_t, abs(T[-1] - float(show.ph["t_down"][i])))
        assert items[1]["command"] == 178 and items[2]["command"] == 22 and items[-1]["command"] == 21
    check(worst < 0.5, f"waypoint missions follow the path in time: max deviation {worst:.2f} m "
                       f"(constant-speed legs, {min(nitems)}-{max(nitems)} items)")
    check(worst_t < 3.0, f"mission ends within {worst_t:.1f} s of the planned touchdown")
    items_d = mission_items(show, 0, origin, "utc_delay", start_utc=1_800_000_000.0)
    check(any(it["command"] == 93 and it["p1"] == -1 for it in items_d), "utc_delay mode resyncs hovers with NAV_DELAY")
    s2 = dict(s, mission_max_items=12)
    show2 = im.ImportedShow(raw, s2, "tight")
    it2 = show2.mission_items(3, origin)
    check(len(it2) <= 12 or show2.mission_info.get("tol", {}).get(3, 0) > 0.25,
          f"path tolerance loosened to fit the mission item limit ({len(it2)} items)")

    # ---------------------------------------------------------------- export
    out = os.path.join(TMP, "export.zip")
    res = export_package(show, spec, out)
    with zipfile.ZipFile(out) as z:
        names = z.namelist()
        man = json.loads(z.read("manifest.json"))
        lf = z.read("lights/drone_0001.lights")
        csv = z.read("trajectories/drone_0001.csv").decode()
        wpl = z.read("missions/drone_0001.waypoints").decode()
        npz = np.load(io.BytesIO(z.read("imported_show.npz")))
    fps = int(s["light_fps"])
    check("imported_show.npz" in names and res["drones"] == 64 and man["imported"]["drones"] == 64
          and len(man["takeoff_s"]) == 64, "export package: manifest, npz, per-drone files")
    check(len(lf) == 20 + 3 * (int(math.ceil(show.duration * fps)) + 1), "light file frame count")
    rows = [r for r in csv.splitlines() if r and r[0].isdigit()]
    check(len(rows) == int(np.diff(show.pfirst)[0]) - 1 and wpl.startswith("QGC WPL 110"),
          f"trajectory CSV has the Bezier segments ({len(rows)} rows); QGC mission file")
    check(npz["points"].shape == show.pp.shape, "npz holds the transformed trajectories")

    # ---------------------------------------------------------------- CSV export import
    csvzip = os.path.join(TMP, "synthetic_csv.zip")
    fx.write_csv_zip(csvzip, drones, fps=4.0)
    rc = im.parse_file(csvzip, sha="csv")
    shc = im.ImportedShow(rc, s, "csv")
    e2 = 0.0
    c2 = 0.0
    for tt in np.linspace(0, min(shc.duration, sh.duration) - 0.5, 300):
        e2 = max(e2, float(np.abs(shc.positions(tt) - sh.positions(tt)).max()))
        c2 = max(c2, float(np.abs(shc.colors(tt) - sh.colors(tt)).max() * 255))
    check(rc.n == 64, f"CSV .zip import: {rc.n} drones, {len(rc.tk)} points after simplification "
                      f"(from {64 * int(sh.duration * 4)})")
    check(e2 < 0.6, f"CSV positions match the .skyc within {e2:.2f} m (4 fps samples, linear between)")
    check(c2 < 130, f"CSV colours follow the light program (max {c2:.0f}/255 at 4 fps steps)")

    # ---------------------------------------------------------------- plain / gzip show.json
    sj = os.path.join(TMP, "show.json")
    with open(sj, "w") as f:
        json.dump(fx.show_json(drones), f)
    rj = im.parse_file(sj, sha="json")
    check(rj.n == 64 and np.allclose(rj.pp, raw.pp), "plain show.json import")

    # ---------------------------------------------------------------- errors
    junk = os.path.join(TMP, "junk.skyc")
    with open(junk, "wb") as f:
        f.write(b"PK\x03\x04garbage")
    try:
        im.parse_file(junk)
        check(False, "garbage file rejected")
    except im.ShowImportError as e:
        check(True, f"garbage file rejected: {e}")

    # ---------------------------------------------------------------- larger show performance
    big = fx.build(1600, scale_time=2.0, extra_turns=3, expand=1.15)
    bp = os.path.join(TMP, "big.skyc")
    fx.write_skyc(bp, big)
    t = time.perf_counter()
    rb = im.load_raw(bp)
    tp = time.perf_counter() - t
    sb = compile_show(spec_for(bp, 1600))
    t = time.perf_counter()
    for k in range(20):
        sb.positions(30 + k)
        sb.colors_u8(30 + k)
    tf = (time.perf_counter() - t) / 20
    t = time.perf_counter()
    vb = validate(sb)
    tv = time.perf_counter() - t
    t = time.perf_counter()
    rb2 = im.load_raw(bp)            # memory cache
    im._RAW_CACHE.clear()
    rb3 = im.load_raw(bp)            # disk cache
    tc = time.perf_counter() - t
    check(rb3.n == 1600 and np.allclose(rb3.pp, rb.pp) and rb2 is rb, "parse cache (memory and disk)")
    print(f"     1600 drones, {sb.duration:.0f} s: parse {tp:.1f} s, frame (positions + colours) {tf * 1000:.1f} ms, "
          f"validate {tv:.1f} s ({'ok' if vb['ok'] else 'FAIL'}), reload from disk cache {tc:.2f} s")
    check(vb["ok"] and tf < 0.05, "1600-drone show validates and evaluates fast enough for 10 Hz preview")

    # ---------------------------------------------------------------- real Skybrush file(s)
    for p in extra or []:
        rr = im.parse_file(p, sha="")
        ss = im.ImportedShow(rr, merged_settings({"settings": {"max_speed_h": 10, "max_accel": 10}}), "real")
        vv = validate(ss)
        print(f"     {os.path.basename(p)}: {rr.n} drones, {rr.duration:.0f} s, notes: {rr.meta['notes']}")
        for c in vv["checks"]:
            print("      ", c["status"], c["name"], "-", c["detail"][:120])
        check(rr.n > 0, f"real Skybrush file {os.path.basename(p)} imported")

    print("ALL PASSED" if ok else "SOME CHECKS FAILED")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1:]) else 1)
