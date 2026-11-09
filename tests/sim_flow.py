"""Headless end-to-end rehearsal on the simulator with a virtual clock."""
import asyncio, sys, os
from time import perf_counter
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import langit.fleet.controller as C
from langit.show.spec import default_show

class Clock:
    def __init__(self): self.t = 1_800_000_000.0
    def time(self): return self.t
clock = Clock()

async def main(n=2000, faults=0.0015, hold_test=True):
    C.time.time = clock.time          # controller uses time.time()
    spec = default_show(n)
    ctl = C.ShowController(spec, "/tmp/langit_logs")
    async def ex(fn): return fn()
    await ctl.connect("sim", drones=n, spares=spec["spares"], inflight_fault_per_min=faults, preflight_fault_rate=0.01)
    from scipy.spatial import cKDTree
    safety = {"min": float("inf"), "close": 0, "checks": 0}
    def check_true_separation():
        b = ctl.backend
        fl = ctl.flyers
        if not len(fl):
            return
        ok = b.state.in_air[fl] & (b.mode[fl] != 7)          # every airborne drone except falling ones
        idx = fl[ok]
        if len(idx) < 2:
            return
        P = b.true_pos[idx]
        d, _ = cKDTree(P).query(P, k=2)
        safety["min"] = min(safety["min"], float(d[:, 1].min()))
        safety["close"] += int((d[:, 1] < 1.0).sum())
        safety["checks"] += 1
    def run(seconds, dt=0.1):
        for k in range(int(round(seconds / dt))):
            clock.t += dt; ctl.tick(clock.t, dt)
            if k % 5 == 0 and ctl.state in ("RUNNING", "HOLDING", "RETURNING", "LANDING"):
                check_true_separation()
    run(1.0)
    t = perf_counter(); await ctl.plan(ex); print("plan", round(perf_counter() - t, 1), "s; flyers", len(ctl.flyers), "state", ctl.state)
    await ctl.upload(); run(0.5)
    for k in ctl.checklist: ctl.checklist[k] = True
    ctl.wind = 4.0
    print("go/no-go", ctl.evaluate_gonogo())
    ctl.start(countdown=10)
    t = perf_counter()
    held = False
    while ctl.state not in ("FINISHED", "ABORTED") and clock.t - ctl.t0 < ctl.live_show.duration + 120:
        run(1.0)
        if hold_test and not held and ctl.state == "RUNNING" and clock.t - ctl.t0 > 120:
            held = True
            try:
                ctl.hold_all()
                run(5.0)
                ctl.resume()
                print("HOLD 5 s + RESUME done")
            except C.NeedsConfirm as e:
                print("HOLD refused (operator would be asked):", str(e)[:80], "... -> show continues")
    print("final state", ctl.state, "sim wall time", round(perf_counter() - t, 1), "s; show clock", round(clock.t - ctl.t0, 1))
    print("counts", ctl.status()["fleet"]["counts"], "taken out", len(ctl.taken_out))
    print(f"true-position separation over {safety['checks']} checks: closest {safety['min']:.2f} m, "
          f"drone-pairs under 1.0 m: {safety['close']}")
    lv = [e for e in ctl.events if e["level"] in ("warn", "error")]
    for e in lv[-12:]: print("  ", e["level"], e["drone"], e["msg"])
    # collision check on true positions is done during the run below
    ctl.safety = safety
    return ctl

if __name__ == "__main__":
    asyncio.run(main(int(sys.argv[1]) if len(sys.argv) > 1 else 2000))
