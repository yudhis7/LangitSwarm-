"""End-to-end MAVLink test of an IMPORTED (Skybrush) show: LangitSwarm <-> emulated
drones over real UDP.  Checks the staggered take-off (each drone is started at its own
Skybrush take-off time), the waypoint missions and the landing.  Runs ~2 minutes.
    python tests/test_mavlink_import.py
"""
import asyncio
import os
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "tests"))
os.environ["LANGIT_HOME"] = tempfile.mkdtemp(prefix="langit_mavimp_")

import mavlink_emulator as emu  # noqa: E402
import skybrush_fixture as fx  # noqa: E402
from langit.fleet.controller import ShowController  # noqa: E402
from langit.show import importer as im  # noqa: E402
from langit.show.spec import merged_settings  # noqa: E402

N = 36
PORT = 14573


async def main():
    path = os.path.join(os.environ["LANGIT_HOME"], "mav.skyc")
    fx.write_skyc(path, fx.build(N))
    raw = im.load_raw(path)
    settings = {"min_ready_fraction": 1.0}
    s = merged_settings({"settings": settings})
    spec = {"name": "imported over MAVLink", "drones": N, "spares": 0, "settings": settings, "timeline": [],
            "imported": {"file": path, "sha256": raw.sha256, "drones": N, "rotate_deg": 0.0,
                         "offset": im.centre_offset(raw, 0.0, s["takeoff_center"]), "tolerance": 0.25,
                         "format": "skyc"}}
    ctl = ShowController(spec, os.path.join(os.environ["LANGIT_HOME"], "logs"))
    loop = asyncio.get_running_loop()

    async def ex(fn):
        return await loop.run_in_executor(None, fn)

    await ctl.connect("mavlink", drones=N, spares=0, endpoints=[f"udp:127.0.0.1:{PORT}"])
    emu_task = asyncio.create_task(emu.run(N, ("127.0.0.1", PORT), spec, same_sysid=True, duration=600))

    async def ticker():
        last = time.time()
        while True:
            await asyncio.sleep(0.1)
            now = time.time()
            ctl.tick(now, now - last)
            last = now

    tick_task = asyncio.create_task(ticker())
    await asyncio.sleep(3.0)
    st = ctl.backend.state
    assert st.m == N, f"discovered {st.m} of {N}"
    await ctl.plan(ex)
    sh = ctl.live_show
    print("planned:", ctl.state, "imported:", sh.imported, "duration", round(sh.duration, 1), "s",
          "| missions", sh.mission_info.get("max_items"), "items max")
    await ctl.upload()
    print("uploaded", int(st.uploaded[:st.m].sum()), "/", N)
    for k in ctl.checklist:
        ctl.checklist[k] = True
    ctl.wind = 3.0
    await asyncio.sleep(1.5)
    g = ctl.evaluate_gonogo()
    assert g["go"], g
    ctl.start(countdown=10)
    lift = np.full(N, np.nan)
    by_emu = {}
    max_dev = 0.0
    t_end = time.time() + sh.duration + 60
    while time.time() < t_end and ctl.state not in ("FINISHED", "ABORTED"):
        await asyncio.sleep(0.1)
        if ctl.t0 is None:
            continue
        ts = time.time() - ctl.t0
        for d in emu.RUNNING:
            if d.idx not in by_emu and d.pos[2] > 0.3:
                by_emu[d.idx] = ts
        if ctl.state == "RUNNING" and int(ts * 10) % 10 == 0:
            fl = ctl.flyers
            air = fl[st.in_air[fl]]
            if len(air):
                dev = sh.path_deviation(ts, st.slot[air], st.pos[air])
                max_dev = max(max_dev, float(dev.max()))
    # map emulator drones to show slots by pad
    planned = []
    actual = []
    for d in emu.RUNNING:
        if d.idx in by_emu:
            k = int(np.argmin(np.linalg.norm(sh.start_positions[:, :2] - d.home[:2], axis=1)))
            planned.append(sh.takeoff_offsets[k])
            actual.append(by_emu[d.idx])
    planned, actual = np.array(planned), np.array(actual)
    lag = actual - planned
    print(f"take-off: {len(actual)}/{N} drones lifted; planned spread {planned.min():.2f}..{planned.max():.2f} s; "
          f"lift-off lag vs plan {lag.min():.2f}..{lag.max():.2f} s (0.3 m threshold at the take-off climb speed)")
    final = ctl.state
    print("final state:", final, "| airborne:", int(st.in_air[:st.m].sum()),
          "| max distance from planned path:", round(max_dev, 2), "m")
    for e in ctl.events:
        if e["level"] != "info":
            print("  ", e["level"], e["drone"], e["msg"])
    tick_task.cancel()
    emu_task.cancel()
    await ctl.disconnect()
    jitter = float(np.abs(lag - np.median(lag)).max())
    print(f"lift-off timing spread around the median lag: +-{jitter:.2f} s (telemetry 10 Hz, polling 10 Hz)")
    ok = (final == "FINISHED" and len(actual) == N and jitter < 0.35 and lag.max() < 1.5 and max_dev < 2.0)
    print("MAVLINK IMPORT TEST PASSED" if ok else
          f"MAVLINK IMPORT TEST FAILED (state {final}, lifted {len(actual)}, jitter {jitter:.2f}, "
          f"lag {lag.max():.2f}, dev {max_dev:.2f})")
    return ok


if __name__ == "__main__":
    sys.exit(0 if asyncio.run(main()) else 1)
