"""Local web server: REST API + WebSocket for the operator UI.

Runs entirely on the operator laptop (127.0.0.1 by default).  The 3D view gets
compact binary frames:
    uint8 kind (1 = preview, 2 = live), 3 pad bytes, uint32 n, float64 t,
    float32[n*3] positions (show frame), uint8[n*3] colours, uint8[n] status
"""
from __future__ import annotations

import asyncio
import base64
import copy
import datetime as dt
import hashlib
import json
import os
import struct
import sys
import threading
import time
import traceback

import numpy as np
from aiohttp import WSMsgType, web

from . import APP_NAME, DISPLAY_NAME, __version__
from .fleet.controller import CHECKLIST, NeedsConfirm, OpError, ShowController
from .fleet.state import STATUS_NAMES
from .show.compiler import CompileError, compile_show, field_layout_for
from .show.export import export_package
from .show.importer import ShowImportError, centre_offset, load_raw
from .show.formations import ALL_SHAPES
from .show.lights import EFFECTS
from .show.spec import DEFAULT_SETTINGS, default_show, merged_settings
from .show.validator import validate


def resource_dir() -> str:
    base = getattr(sys, "_MEIPASS", None) or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, "web")


def home_dir() -> str:
    d = os.environ.get("LANGIT_HOME") or os.path.join(os.path.expanduser("~"), "LangitSwarm")
    for sub in ("shows", "exports", "logs", "imports"):
        os.makedirs(os.path.join(d, sub), exist_ok=True)
    return d


def pack_frame(kind: int, t: float, pos: np.ndarray, col: np.ndarray, status: np.ndarray | None) -> bytes:
    n = len(pos)
    if status is None:
        status = np.zeros(n, np.uint8)
    return (struct.pack("<B3xId", kind, n, float(t)) + np.ascontiguousarray(pos, "<f4").tobytes()
            + np.ascontiguousarray(col, np.uint8).tobytes() + np.ascontiguousarray(status, np.uint8).tobytes())


class App:
    def __init__(self):
        self.home = home_dir()
        os.environ.setdefault("LANGIT_HOME", self.home)
        self.autosave = os.path.join(self.home, "shows", "_autosave.json")
        self.spec = self._load_autosave()
        self.design = None
        self.design_val = None
        self.design_err = None
        self.job = {"name": "", "frac": 0.0, "msg": "", "running": False}
        self.ctl = ShowController(self.spec, os.path.join(self.home, "logs"))
        self.clients: set = set()
        self.loop = None
        self.compile_lock = asyncio.Lock()
        self.spec_gen = 0          # bumped whenever the whole show is replaced (import, load, new, remove)

    def _load_autosave(self):
        try:
            with open(self.autosave, encoding="utf-8") as f:
                spec = json.load(f)
            if spec.get("imported"):
                return self.sanitize_imported(spec)
            if spec.get("timeline"):
                return spec
        except (OSError, ValueError):
            pass
        return default_show(3000)

    def _save_autosave(self):
        tmp = self.autosave + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.spec, f)
        os.replace(tmp, self.autosave)

    def sanitize_imported(self, spec: dict) -> dict:
        """Imported show specs may only point into the imports folder; the drone count is
        fixed by the imported file."""
        imp = spec.get("imported")
        if not isinstance(imp, dict) or not imp.get("file"):
            raise ValueError("imported show without a file")
        imp["file"] = os.path.join(self.home, "imports", os.path.basename(str(imp["file"])))
        imp["rotate_deg"] = float(imp.get("rotate_deg", 0.0)) % 360.0
        off = imp.get("offset") or [0.0, 0.0]
        imp["offset"] = [float(off[0]), float(off[1])]
        imp["tolerance"] = min(5.0, max(0.05, float(imp.get("tolerance", 0.25))))
        spec["drones"] = int(imp.get("drones", spec.get("drones", 2)))
        spec["timeline"] = spec.get("timeline") or []
        return spec

    async def run_blocking(self, fn):
        return await self.loop.run_in_executor(None, fn)

    def set_job(self, name, frac, msg, running=True):
        self.job = {"name": name, "frac": float(frac), "msg": msg, "running": running}

    # ---- design ------------------------------------------------------------------------------------
    async def compile_design(self):
        async with self.compile_lock:
            gen = self.spec_gen
            spec = copy.deepcopy(self.spec)
            self.set_job("compile", 0.0, "Starting")
            self.design_err = None
            try:
                show = await self.run_blocking(lambda: compile_show(
                    spec, progress=lambda f, m: self.set_job("compile", 0.75 * f, m)))
                val = await self.run_blocking(lambda: validate(
                    show, progress=lambda f, m: self.set_job("compile", 0.75 + 0.25 * f, m)))
                if gen != self.spec_gen:            # the show was replaced while compiling: drop the result
                    self.set_job("compile", 1.0, "Superseded", running=False)
                    return
                self.design, self.design_val = show, val
                self.set_job("compile", 1.0, "Compiled", running=False)
            except Exception as e:  # noqa: BLE001
                if gen != self.spec_gen:
                    self.set_job("compile", 1.0, "Superseded", running=False)
                    return
                self.design_err = str(e)
                self.set_job("compile", 1.0, f"Error: {e}", running=False)
                if not isinstance(e, CompileError):
                    traceback.print_exc()
            await self.broadcast({"type": "design", "design": self.design_summary()})

    def design_summary(self):
        if self.design_err:
            return {"error": self.design_err}
        sh = self.design
        if sh is None:
            return None
        imported = None
        if getattr(sh, "imported", False):
            imp = self.spec.get("imported") or {}
            imported = dict(sh.summary(), original_name=imp.get("original_name"), rotate_deg=sh.rotate_deg,
                            offset=sh.offset.tolist(), tolerance=sh.tolerance)
        return {
            "imported": imported,
            "name": sh.name, "drones": sh.n, "duration": sh.duration,
            "segments": sh.timeline_summary(), "formations": sh.formation_names,
            "formation_start": {str(k): v for k, v in sh.formation_start.items()},
            "validation": self.design_val, "warnings": sh.warnings,
            "settings": {k: sh.settings[k] for k in ("geofence", "audience_y", "audience_buffer",
                                                     "takeoff_altitude", "max_altitude", "min_separation")},
        }

    async def broadcast(self, msg: dict):
        data = json.dumps(msg, default=_json_default)
        for ws in list(self.clients):
            try:
                await ws.send_str(data)
            except (ConnectionResetError, RuntimeError):
                self.clients.discard(ws)

    # ---- background loops ---------------------------------------------------------------------------
    async def tick_loop(self):
        last = time.time()
        while True:
            await asyncio.sleep(0.1)
            now = time.time()
            try:
                self.ctl.tick(now, now - last)
            except Exception:  # noqa: BLE001 - never let the safety loop die
                traceback.print_exc()
            last = now

    async def push_loop(self):
        k = 0
        last_seq = 0
        while True:
            await asyncio.sleep(0.1)
            k += 1
            if not self.clients:
                continue
            ctl = self.ctl
            try:
                if ctl.backend is not None:
                    pos, col, stt = ctl.frame()
                    t = (time.time() - ctl.t0) if ctl.t0 else 0.0
                    frame = pack_frame(2, t, pos, col, stt)
                    for ws in list(self.clients):
                        if getattr(ws, "_live", False):
                            await ws.send_bytes(frame)
                if k % 5 == 0:
                    st = ctl.status()
                    st["job"] = self.job
                    msg = {"type": "status", "status": st}
                    if ctl.backend is not None:
                        m = ctl.backend.state.m
                        msg["grid"] = base64.b64encode(ctl.backend.state.status[:m].tobytes()).decode()
                    new = [e for e in ctl.events if e["seq"] > last_seq]
                    if new:
                        last_seq = new[-1]["seq"]
                        msg["events"] = new[-60:]
                    await self.broadcast(msg)
            except Exception:  # noqa: BLE001
                traceback.print_exc()


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.bool_,)):
        return bool(o)
    return str(o)


def jresp(obj, status=200):
    return web.Response(text=json.dumps(obj, default=_json_default), status=status,
                        content_type="application/json")


# The page reports how far it got ("hello" = HTML loaded, "ready" = app running, "error"),
# so the desktop launcher can fall back to the browser if the window stays blank.
CLIENT_EVENTS = {"window_hello": threading.Event(), "window_ready": threading.Event(),
                 "browser_hello": threading.Event(), "browser_ready": threading.Event()}
CLIENT_LOG: list = []


def build_app() -> web.Application:
    A = App()
    app = web.Application(client_max_size=32 * 1024 * 1024)
    routes = web.RouteTableDef()

    @web.middleware
    async def errors(request, handler):
        try:
            return await handler(request)
        except NeedsConfirm as e:
            return jresp({"error": str(e), "confirm": True}, 409)
        except (OpError, CompileError, ValueError, KeyError) as e:
            return jresp({"error": str(e)}, 400)
        except web.HTTPException:
            raise
        except Exception as e:  # noqa: BLE001
            traceback.print_exc()
            return jresp({"error": f"internal error: {e}"}, 500)

    app.middlewares.append(errors)

    async def no_cache(request, response):
        # the desktop window keeps a persistent cache: always revalidate the UI files so an
        # updated version of the app is never shown with an old script
        if not request.path.startswith("/api/"):
            response.headers["Cache-Control"] = "no-cache"

    app.on_response_prepare.append(no_cache)

    @routes.post("/api/client/{what}")
    async def client_report(request):
        what = request.match_info["what"]
        try:
            body = await request.json()
        except Exception:  # noqa: BLE001
            body = {}
        shell = "window" if body.get("shell") == "window" else "browser"
        msg = str(body.get("msg", ""))[:2000]
        ua = request.headers.get("User-Agent", "")[:200]
        if what in ("hello", "ready"):
            ev = CLIENT_EVENTS.get(f"{shell}_{what}")
            if ev is not None and not ev.is_set():
                ev.set()
                print(f"[ui] {shell}: page {'loaded' if what == 'hello' else 'running'} ({ua})", flush=True)
        elif what == "error":
            CLIENT_LOG.append({"shell": shell, "msg": msg, "ua": ua})
            print(f"[ui] {shell} ERROR: {msg}", flush=True)
        return jresp({"ok": True})

    @routes.get("/")
    async def index(_):
        return web.FileResponse(os.path.join(resource_dir(), "index.html"))

    @routes.get("/api/info")
    async def info(_):
        return jresp({"app": APP_NAME, "name": DISPLAY_NAME, "version": __version__, "shapes": ALL_SHAPES, "effects": EFFECTS,
                      "defaults": DEFAULT_SETTINGS, "status_names": STATUS_NAMES, "checklist": CHECKLIST,
                      "home": A.home})

    @routes.get("/api/spec")
    async def get_spec(_):
        return jresp(A.spec)

    @routes.put("/api/spec")
    async def put_spec(request):
        if A.ctl.state not in ("DISCONNECTED", "CONNECTED", "FINISHED", "ABORTED"):
            raise OpError("the show is locked while a live plan is loaded - finish or disconnect first")
        spec = await request.json()
        if spec.get("imported"):
            spec = A.sanitize_imported(spec)
        elif not isinstance(spec.get("timeline"), list):
            raise ValueError("spec needs a timeline list")
        spec["drones"] = int(spec.get("drones", 100))
        if not 2 <= spec["drones"] <= 10000:
            raise ValueError("drones must be between 2 and 10000")
        A.spec = spec
        A.ctl.spec = spec
        A._save_autosave()
        return jresp({"ok": True})

    @routes.post("/api/spec/new")
    async def new_spec(request):
        body = await request.json() if request.can_read_body else {}
        A.spec = default_show(int(body.get("drones", 3000)))
        A.ctl.spec = A.spec
        A.spec_gen += 1
        A.design = None
        A._save_autosave()
        return jresp(A.spec)

    @routes.get("/api/layout")
    async def layout(_):
        s = merged_settings(A.spec)
        pads, spare = await A.run_blocking(lambda: field_layout_for(A.spec, int(A.spec["drones"]),
                                                                     int(A.spec.get("spares", 0)), s))
        return jresp({"pads": pads[:, :2].round(2).tolist(), "spare": spare.tolist(), "settings": s})

    @routes.post("/api/compile")
    async def compile_(_):
        if A.job["running"]:
            raise OpError("a compile is already running")
        A.set_job("compile", 0.0, "Queued")
        asyncio.create_task(A.compile_design())
        return jresp({"started": True})

    @routes.get("/api/design")
    async def design(_):
        return jresp(A.design_summary())

    @routes.post("/api/export")
    async def export(request):
        if A.design is None:
            raise OpError("compile the show first")
        body = await request.json() if request.can_read_body else {}
        include = tuple(body.get("include") or ("trajectories", "missions", "lights"))
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in A.design.name)[:40]
        fname = f"{safe}_{A.design.n}drones_{stamp}.zip"
        path = os.path.join(A.home, "exports", fname)
        A.set_job("export", 0, "Exporting")
        res = await A.run_blocking(lambda: export_package(
            A.design, A.spec, path, include=include, progress=lambda f, m: A.set_job("export", f, m)))
        A.set_job("export", 1, "Exported", running=False)
        res["url"] = f"/api/download/{fname}"
        return jresp(res)

    @routes.get("/api/download/{name}")
    async def download(request):
        name = os.path.basename(request.match_info["name"])
        path = os.path.join(A.home, "exports", name)
        if not os.path.isfile(path):
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers={"Content-Disposition": f'attachment; filename="{name}"'})

    @routes.get("/api/shows")
    async def list_shows(_):
        d = os.path.join(A.home, "shows")
        items = sorted((f[:-5] for f in os.listdir(d) if f.endswith(".json") and not f.startswith("_")))
        return jresp({"shows": items, "folder": d})

    @routes.post("/api/shows/save")
    async def save_show(request):
        body = await request.json()
        name = "".join(c if c.isalnum() or c in "-_ " else "_" for c in body.get("name", "show")).strip() or "show"
        path = os.path.join(A.home, "shows", name + ".json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(A.spec, f, indent=1)
        return jresp({"saved": path})

    @routes.post("/api/shows/load")
    async def load_show(request):
        body = await request.json()
        name = os.path.basename(body.get("name", ""))
        with open(os.path.join(A.home, "shows", name + ".json"), encoding="utf-8") as f:
            spec = json.load(f)
        if spec.get("imported"):
            spec = A.sanitize_imported(spec)
        A.spec = spec
        A.ctl.spec = spec
        A.spec_gen += 1
        A.design = None
        A._save_autosave()
        return jresp(spec)

    # ---- import from Blender / Skybrush Studio ------------------------------------------------------
    def _editable():
        if A.ctl.state not in ("DISCONNECTED", "CONNECTED", "FINISHED", "ABORTED"):
            raise OpError("the show is locked while a live plan is loaded - finish or disconnect first")
        if A.job["running"] and A.job["name"] in ("import", "export"):
            raise OpError(f"busy: {A.job['name']} is running")

    @routes.post("/api/import")
    async def import_show(request):
        """Body: the raw file (.skyc, show.json or CSV .zip); query ?name=<original file name>."""
        _editable()
        name = os.path.basename(request.query.get("name", "show.skyc")) or "show.skyc"
        ext = os.path.splitext(name)[1].lower()
        if ext not in (".skyc", ".zip", ".json", ".gz"):
            raise ValueError("import a Skybrush .skyc, a show.json or a Skybrush CSV export (.zip)")
        folder = os.path.join(A.home, "imports")
        os.makedirs(folder, exist_ok=True)
        tmp = os.path.join(folder, f"_upload_{int(time.time() * 1000)}{ext}")
        h = hashlib.sha256()
        size = 0
        A.set_job("import", 0.0, f"Receiving {name}")
        try:
            with open(tmp, "wb") as f:
                async for chunk in request.content.iter_chunked(1 << 20):
                    size += len(chunk)
                    if size > 2 * 1024 ** 3:
                        raise ValueError("file larger than 2 GB")
                    f.write(chunk)
                    h.update(chunk)
            if size == 0:
                raise ValueError("empty file")
            sha = h.hexdigest()
            final = os.path.join(folder, sha + ext)
            os.replace(tmp, final)
            raw = await A.run_blocking(lambda: load_raw(final, sha, progress=lambda fr, m: A.set_job("import", fr, m)))
        except Exception as e:
            if os.path.exists(tmp):
                os.remove(tmp)
            A.set_job("import", 1.0, f"Import failed: {e}", running=False)
            if isinstance(e, ShowImportError):
                raise ValueError(str(e)) from e
            raise
        old = A.spec
        prev = old["imported"].get("prev") if old.get("imported") else copy.deepcopy(old)
        settings = copy.deepcopy(old.get("settings") or {})
        settings["light_fps"] = 25
        s = merged_settings({"settings": settings})
        title = raw.meta.get("title") or os.path.splitext(name)[0]
        A.spec = {
            "name": title, "drones": raw.n, "spares": 0, "settings": settings, "timeline": [],
            "imported": {"file": final, "sha256": sha, "original_name": name, "format": raw.meta.get("format"),
                         "drones": raw.n, "duration": raw.duration, "rotate_deg": 0.0,
                         "offset": centre_offset(raw, 0.0, s["takeoff_center"]), "tolerance": 0.25,
                         "prev": prev},
        }
        A.ctl.spec = A.spec
        A.spec_gen += 1
        A.design = None
        A._save_autosave()
        A.set_job("import", 1.0, "Imported", running=False)
        asyncio.create_task(A.compile_design())
        return jresp({"spec": A.spec, "drones": raw.n, "duration": raw.duration, "notes": raw.meta.get("notes", [])})

    @routes.post("/api/import/center")
    async def import_center(_):
        _editable()
        imp = A.spec.get("imported")
        if not imp:
            raise OpError("no imported show")
        raw = await A.run_blocking(lambda: load_raw(imp["file"], imp.get("sha256")))
        imp["offset"] = centre_offset(raw, float(imp.get("rotate_deg", 0.0)), merged_settings(A.spec)["takeoff_center"])
        A._save_autosave()
        return jresp(A.spec)

    @routes.post("/api/import/remove")
    async def import_remove(_):
        _editable()
        imp = A.spec.get("imported")
        if not imp:
            raise OpError("no imported show")
        prev = imp.get("prev")
        A.spec = prev if prev and prev.get("timeline") else default_show(int(A.spec.get("drones", 300)))
        A.ctl.spec = A.spec
        A.spec_gen += 1
        A.design = None
        A._save_autosave()
        return jresp(A.spec)

    # ---- fleet & operations ------------------------------------------------------------------------
    @routes.post("/api/fleet/connect")
    async def connect(request):
        body = await request.json()
        kind = body.pop("kind", "sim")
        await A.ctl.connect(kind, **body)
        return jresp(A.ctl.status())

    @routes.post("/api/fleet/disconnect")
    async def disconnect(_):
        if A.ctl.state in ("COUNTDOWN", "RUNNING", "HOLDING", "RETURNING", "LANDING"):
            raise OpError("cannot disconnect during a show")
        await A.ctl.disconnect()
        return jresp(A.ctl.status())

    @routes.get("/api/fleet/drone/{i}")
    async def drone(request):
        if A.ctl.backend is None:
            raise OpError("no fleet connected")
        return jresp(A.ctl.drone_detail(int(request.match_info["i"])))

    @routes.post("/api/fleet/drone/{i}/{action}")
    async def drone_action(request):
        A.ctl.drone_action(int(request.match_info["i"]), request.match_info["action"])
        return jresp({"ok": True})

    @routes.post("/api/op/{action}")
    async def op(request):
        c = A.ctl
        a = request.match_info["action"]
        body = await request.json() if request.can_read_body else {}
        if c.backend is None and a not in ("checklist", "wind", "audience_action"):
            raise OpError("connect a fleet first")
        if a == "plan":
            await c.plan(A.run_blocking)
        elif a == "upload":
            await c.upload()
        elif a == "gonogo":
            c._require("UPLOADED")
            c.evaluate_gonogo()
        elif a == "start":
            c.start(float(body.get("countdown", 30)))
        elif a == "abort":
            c.abort_countdown()
        elif a == "hold":
            c.hold_all(force=bool(body.get("force")))
        elif a == "resume":
            c.resume()
        elif a == "return":
            await c.coordinated_return(A.run_blocking, force=bool(body.get("force")))
        elif a == "land_all":
            c.land_all()
        elif a == "kill_all":
            c.kill_all(body.get("confirm", ""))
        elif a == "checklist":
            if body.get("key") not in c.checklist:
                raise OpError("unknown checklist item")
            c.checklist[body["key"]] = bool(body.get("value"))
        elif a == "wind":
            v = body.get("value")
            c.wind = None if v in (None, "") else float(v)
        elif a == "audience_action":
            if body.get("value") not in ("land", "kill"):
                raise OpError("audience action must be land or kill")
            c.audience_action = body["value"]
        else:
            raise OpError(f"unknown action {a}")
        if c.backend is not None and c.state == "UPLOADED":
            c.evaluate_gonogo()
        return jresp(c.status())

    # ---- websocket ------------------------------------------------------------------------------------
    @routes.get("/ws")
    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=20, max_msg_size=4 * 1024 * 1024)
        await ws.prepare(request)
        ws._live = False
        A.clients.add(ws)
        try:
            await ws.send_str(json.dumps({"type": "design", "design": A.design_summary()}, default=_json_default))
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    m = json.loads(msg.data)
                except ValueError:
                    continue
                if m.get("type") == "view":
                    ws._live = m.get("mode") == "live"
                elif m.get("type") == "preview" and A.design is not None:
                    t = float(m.get("t", 0.0))
                    sh = A.design
                    await ws.send_bytes(pack_frame(1, t, sh.positions(t), sh.colors_u8(t), None))
        finally:
            A.clients.discard(ws)
        return ws

    app.add_routes(routes)
    app.router.add_static("/", resource_dir(), show_index=False)

    async def on_start(_app):
        A.loop = asyncio.get_running_loop()
        A.ctl.spec = A.spec
        _app["tasks"] = [asyncio.create_task(A.tick_loop()), asyncio.create_task(A.push_loop())]
        asyncio.create_task(A.compile_design())

    async def on_stop(_app):
        for t in _app.get("tasks", []):
            t.cancel()
        await A.ctl.disconnect()

    app.on_startup.append(on_start)
    app.on_cleanup.append(on_stop)
    app["A"] = A
    return app
