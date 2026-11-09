"""Headless rehearsal of an IMPORTED (Skybrush) show on the simulator, virtual clock.

Scenario A: full show; HOLD in the middle of the staggered take-off (some drones still
            on their pads), RESUME, finish.  Waiting drones must not lift during the hold.
Scenario B: LAND ALL during the take-off window: drones not yet airborne stay on the pads.
    python tests/sim_flow_import.py [drones]
"""
import asyncio
import os
import sys
import tempfile

import numpy as np
from scipy.spatial import cKDTree

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))
os.environ["LANGIT_HOME"] = tempfile.mkdtemp(prefix="langit_simimp_")

import langit.fleet.controller as C  # noqa: E402
import skybrush_fixture as fx  # noqa: E402
from langit.show import importer as im  # noqa: E402
from langit.show.spec import merged_settings  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 1_800_000_000.0

    def time(self):
        return self.t


clock = Clock()


def make_spec(n):
    path = os.path.join(os.environ["LANGIT_HOME"], "sim.skyc")
    fx.write_skyc(path, fx.build(n, scale_time=1.5, expand=1.3))
    raw = im.load_raw(path)
    s = merged_settings({"settings": {}})
    return {"name": "imported sim", "drones": n, "spares": 4, "settings": {}, "timeline": [],
            "imported": {"file": path, "sha256": raw.sha256, "drones": n, "rotate_deg": 0.0,
                         "offset": im.centre_offset(raw, 0.0, s["takeoff_center"]), "tolerance": 0.25,
                         "format": "skyc"}}


async def scenario(n, mode):
    C.time.time = clock.time
    spec = make_spec(n)
    ctl = C.ShowController(spec, os.path.join(os.environ["LANGIT_HOME"], "logs"))

    async def ex(fn):
        return fn()

    await ctl.connect("sim", drones=n, spares=4, inflight_fault_per_min=0.0, preflight_fault_rate=0.0, seed=3)
    b = ctl.backend
    stats = {"min": np.inf, "lifted_during_hold": 0}

    def sep():
        fl = ctl.flyers
        idx = fl[b.state.in_air[fl]]
        if len(idx) > 1:
            P = b.true_pos[idx]
            d, _ = cKDTree(P).query(P, k=2)
            stats["min"] = min(stats["min"], float(d[:, 1].min()))

    def run(sec, dt=0.1):
        for k in range(int(round(sec / dt))):
            clock.t += dt
            ctl.tick(clock.t, dt)
            if k % 5 == 0 and ctl.state in ("RUNNING", "HOLDING", "LANDING"):
                sep()

    run(1.0)
    await ctl.plan(ex)
    await ctl.upload()
    run(0.5)
    for k in ctl.checklist:
        ctl.checklist[k] = True
    ctl.wind = 3.0
    ctl.start(countdown=10)
    sh = ctl.live_show
    first, last = float(np.nanmin(sh.takeoff_offsets)), float(np.nanmax(sh.takeoff_offsets))
    print(f"[{mode}] {len(ctl.flyers)} flyers, take-offs T+{first:.1f} .. T+{last:.1f} s, show {sh.duration:.0f} s, "
          f"{len(spec['imported'])} import fields, waiting at start: {len(ctl.wait_idx)}")
    mid = 0.5 * (first + last)
    run(10.0 + mid)                                   # countdown + until half the field is up
    air0 = int(b.state.in_air[ctl.flyers].sum())
    waiting0 = len(ctl.wait_idx)
    ok = True
    if mode == "hold":
        ctl.hold_all()
        on_ground = ctl.flyers[~b.state.in_air[ctl.flyers]]
        run(8.0)
        stats["lifted_during_hold"] = int(b.state.in_air[on_ground].sum())
        held_ok = stats["lifted_during_hold"] == 0
        print(f"[hold] HOLD at T+{mid:.1f}: {air0} airborne, {waiting0} waiting; after 8 s hold "
              f"{stats['lifted_during_hold']} of the waiting drones lifted (must be 0) -> {'PASS' if held_ok else 'FAIL'}")
        ok &= held_ok
        ctl.resume()
        guard = 0
        while ctl.state not in ("FINISHED", "ABORTED") and guard < 4000:
            run(1.0)
            guard += 1
        flown = int(ctl.flown[ctl.flyers].sum())
        landed = int((~b.state.in_air[ctl.flyers]).sum())
        res = ctl.state == "FINISHED" and flown == len(ctl.flyers) and landed == len(ctl.flyers)
        print(f"[hold] final {ctl.state}: {flown}/{len(ctl.flyers)} flew, {landed} on the ground, "
              f"closest true distance {stats['min']:.2f} m -> {'PASS' if res and stats['min'] > 1.5 else 'FAIL'}")
        ok &= res and stats["min"] > 1.5
    else:
        ctl.land_all()
        never = ctl.flyers[~ctl.flown[ctl.flyers]]
        guard = 0
        while ctl.state not in ("FINISHED", "ABORTED") and guard < 600:
            run(1.0)
            guard += 1
        lifted = int(ctl.flown[never].sum())
        res = ctl.state == "FINISHED" and lifted == 0 and not b.state.in_air[ctl.flyers].any()
        print(f"[land] LAND ALL at T+{mid:.1f}: {air0} were airborne, {len(never)} still on pads; "
              f"{lifted} of those lifted afterwards (must be 0); final {ctl.state} -> {'PASS' if res else 'FAIL'}")
        ok &= res
    for e in [e for e in ctl.events if e["level"] in ("warn", "error")][-5:]:
        print("    ", e["level"], e["drone"], e["msg"])
    return ok


async def main(n=400):
    a = await scenario(n, "hold")
    b = await scenario(n, "land")
    print("ALL PASSED" if a and b else "SOME CHECKS FAILED")
    return a and b


if __name__ == "__main__":
    sys.exit(0 if asyncio.run(main(int(sys.argv[1]) if len(sys.argv) > 1 else 400)) else 1)
