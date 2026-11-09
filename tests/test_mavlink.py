"""End-to-end MAVLink test: LangitSwarm <-> emulated drones over real UDP sockets.

All emulated drones use system ID 1 (as many real fleets do) and are told apart by
their UDP source address.  Runs a short show in real time (~1 minute).
    python tests/test_mavlink.py
"""
import asyncio
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

import mavlink_emulator as emu  # noqa: E402
from langit.fleet.controller import ShowController  # noqa: E402
from langit.show.spec import default_show  # noqa: E402

N = 40
PORT = 14571


async def main():
    spec = default_show(N, 0)
    spec["settings"].update(takeoff_altitude=5.0, min_ready_fraction=1.0, battery_minutes=18)
    spec["timeline"] = [{"name": "Disk", "shape": "disk", "count": 1.0, "center": [0, 0, 18], "hold": 4,
                         "light": {"effect": "solid", "color": "#00ffaa"}}]
    ctl = ShowController(spec, "/tmp/langit_logs")
    loop = asyncio.get_running_loop()

    async def ex(fn):
        return await loop.run_in_executor(None, fn)

    await ctl.connect("mavlink", drones=N, spares=0, endpoints=[f"udp:127.0.0.1:{PORT}"])
    emu_task = asyncio.create_task(emu.run(N, ("127.0.0.1", PORT), spec, same_sysid=True, duration=400))

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
    print("discovered", st.m, "drones; rx msgs", ctl.backend.rx_count)
    assert st.m == N, "every emulated drone must be discovered"
    assert all(a == "ardupilot" for a in st.autopilot[:st.m])

    await ctl.plan(ex)
    print("planned:", ctl.state, "show duration", round(ctl.live_show.duration, 1), "s")
    t = time.time()
    await ctl.upload()
    print("uploaded in", round(time.time() - t, 2), "s:", int(st.uploaded[:st.m].sum()), "/", N)
    for k in ctl.checklist:
        ctl.checklist[k] = True
    ctl.wind = 3.0
    await asyncio.sleep(1.5)
    g = ctl.evaluate_gonogo()
    print("go/no-go:", g["go"], g["ready"], "/", g["flyers"], g["reasons"])
    assert g["go"]
    ctl.start(countdown=10)
    t_end = time.time() + ctl.live_show.duration + 40
    max_dev = 0.0
    while time.time() < t_end and ctl.state not in ("FINISHED", "ABORTED"):
        await asyncio.sleep(1.0)
        if ctl.state == "RUNNING":
            ts = time.time() - ctl.t0
            if int(ts) % 10 == 0:
                print(f"  t={ts:5.1f}s airborne={int(st.in_air[:st.m].sum())} modes={sorted(set(st.mode[:st.m]))}")
            dev = ctl.live_show.path_deviation(ts, st.slot[ctl.flyers], st.pos[ctl.flyers])
            max_dev = max(max_dev, float(dev.max()))
    final = ctl.state
    for d in emu.RUNNING:
        if d.armed:
            it = d.items[d.mission_seq] if 0 <= d.mission_seq < len(d.items) else None
            print("   stuck emu drone", d.idx, "seq", d.mission_seq, "/", len(d.items), "cmd", getattr(it, "command", None),
                  "pos", d.pos.round(2), "tgt", d._item_target(it).round(2) if it is not None else None,
                  "speed", round(d.speed_h, 2), round(d.speed_up, 2), round(d.speed_dn, 2), "hold_until", d.hold_until)
            break
    print("final state:", ctl.state, "| airborne:", int(st.in_air[:st.m].sum()),
          "| max distance from planned path:", round(max_dev, 2), "m")
    for e in ctl.events:
        if e["level"] != "info":
            print("  ", e["level"], e["drone"], e["msg"])
    tick_task.cancel()
    emu_task.cancel()
    await ctl.disconnect()
    assert final == "FINISHED", f"show did not finish (state {final})"
    print("MAVLINK TEST PASSED")


if __name__ == "__main__":
    asyncio.run(main())
