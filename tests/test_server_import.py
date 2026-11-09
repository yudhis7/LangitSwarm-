"""Server API test for the Blender / Skybrush import (aiohttp test client, no browser).
    python tests/test_server_import.py
"""
import asyncio
import os
import sys
import tempfile
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))
os.environ["LANGIT_HOME"] = tempfile.mkdtemp(prefix="langit_srv_")

from aiohttp.test_utils import TestClient, TestServer  # noqa: E402

import skybrush_fixture as fx  # noqa: E402
from langit.server import build_app  # noqa: E402


async def wait_design(c, timeout=120):
    for _ in range(int(timeout * 10)):
        st = await (await c.get("/api/design")).json()
        if st and (st.get("validation") or st.get("error")):
            return st
        await asyncio.sleep(0.1)
    raise TimeoutError("design not compiled")


async def compile_now(c):
    """POST /api/compile once any running job is done, then wait for it to start."""
    for _ in range(600):
        r = await c.post("/api/compile")
        if r.status == 200:
            break
        await asyncio.sleep(0.2)
    for _ in range(100):
        st = await (await c.get("/api/design")).json()
        if st is None or not st.get("validation"):
            break
        await asyncio.sleep(0.05)
        job = (await (await c.get("/api/info")).json())
        break


async def main():
    ok = True

    def check(cond, msg):
        nonlocal ok
        print(("PASS " if cond else "FAIL ") + msg)
        ok &= bool(cond)

    path = os.path.join(os.environ["LANGIT_HOME"], "studio_export.skyc")
    fx.write_skyc(path, fx.build(49))
    async with TestClient(TestServer(build_app())) as c:
        await asyncio.sleep(0.5)
        base = await (await c.get("/api/spec")).json()
        with open(path, "rb") as f:
            r = await c.post("/api/import?name=studio_export.skyc", data=f.read(),
                             headers={"Content-Type": "application/octet-stream"})
        j = await r.json()
        if r.status != 200:
            print("   error:", j)
        check(r.status == 200 and j["drones"] == 49, f"POST /api/import -> {r.status}, {j.get('drones')} drones")
        spec = j["spec"]
        stored = os.path.basename(spec["imported"]["file"])
        check(spec["timeline"] == [] and spec["imported"]["prev"]["timeline"] == base["timeline"],
              "spec switched to the import; the formation timeline is kept for 'Remove import'")
        d = await wait_design(c)
        check(d.get("imported") and d["validation"]["ok"], f"compiled + validated: {d['validation']['stats']}")
        check([s["kind"] for s in d["segments"]][:2] == ["hold", "takeoff"], "timeline scenes in the design summary")
        lay = await (await c.get("/api/layout")).json()
        check(len(lay["pads"]) == 49, "GET /api/layout returns the imported pads")
        # move + rotate via PUT /api/spec (as the UI does)
        spec["imported"]["rotate_deg"] = 90
        spec["imported"]["offset"] = [5.0, 30.0]
        spec["drones"] = 999                                   # must be ignored
        spec["imported"]["file"] = "/etc/passwd"               # must be confined to the imports folder
        r = await c.put("/api/spec", json=spec)
        check(r.status == 200, "PUT /api/spec with placement changes")
        cur = await (await c.get("/api/spec")).json()
        check(cur["drones"] == 49 and os.path.dirname(cur["imported"]["file"]).endswith("imports"),
              "drone count fixed by the import; file path confined to the imports folder")
        r = await c.put("/api/spec", json=dict(cur, imported=dict(cur["imported"], file=spec["imported"]["file"])))
        cur = await (await c.get("/api/spec")).json()
        cur["imported"]["file"] = stored
        await c.put("/api/spec", json=cur)
        await compile_now(c)
        d2 = await wait_design(c)
        if "validation" not in d2:
            print("   design:", d2)
        ext = d2["validation"]["stats"]["extent"]
        from langit.show import importer as im
        gc = im.grid_centre(im.load_raw(path))
        cy = (ext["y"][0] + ext["y"][1]) / 2
        check(d2["imported"]["rotate_deg"] == 90 and abs(cy - (gc[1] + 30.0)) < 1.0,
              f"re-compiled with rotation about the grid centre + offset (y centre {cy:.1f}, expected {gc[1] + 30:.1f})")
        r = await c.post("/api/import/center")
        cen = await r.json()
        check(r.status == 200 and cen["imported"]["offset"] != [5.0, 30.0], f"centre on field -> offset {cen['imported']['offset']}")
        await compile_now(c)
        await wait_design(c)
        r = await c.post("/api/export", json={"include": ["trajectories", "missions", "lights"]})
        ex = await r.json()
        with zipfile.ZipFile(ex["path"]) as z:
            n_m = len([n for n in z.namelist() if n.startswith("missions/")])
        check(r.status == 200 and n_m == 49, f"export package with {n_m} missions")
        # websocket preview frame
        ws = await c.ws_connect("/ws")
        await ws.receive()                                     # design message
        await ws.send_json({"type": "preview", "t": 30.0})
        for _ in range(5):
            m = await ws.receive()
            if m.type.name == "BINARY":
                break
        check(m.type.name == "BINARY" and len(m.data) == 16 + 49 * 12 + 49 * 3 + 49, "websocket preview frame")
        await ws.close()
        # bad file
        r = await c.post("/api/import?name=x.skyc", data=b"PK\x03\x04nope")
        check(r.status == 400, f"bad file rejected: {(await r.json())['error'][:60]}")
        r = await c.post("/api/import?name=x.exe", data=b"MZ")
        check(r.status == 400, "wrong extension rejected")
    # restart: autosave keeps the imported show
    async with TestClient(TestServer(build_app())) as c:
        await asyncio.sleep(0.3)
        sp = await (await c.get("/api/spec")).json()
        d = await wait_design(c)
        check(sp.get("imported") and d.get("imported") and d["drones"] == 49, "restart reloads the imported show")
        r = await c.post("/api/import/remove")
        sp = await r.json()
        check(r.status == 200 and not sp.get("imported") and len(sp["timeline"]) > 0,
              "remove import restores the formation timeline")
    print("ALL PASSED" if ok else "SOME CHECKS FAILED")
    return ok


if __name__ == "__main__":
    sys.exit(0 if asyncio.run(main()) else 1)
