"""MAVLink fleet link for PX4 and ArduPilot drones over UDP.

Scaling to thousands of drones
  * MAVLink system IDs are one byte (1-254), so a single network cannot address
    5,000 drones by system ID alone.  Drones are identified by
    (channel, source IP:port, system ID).  Use several UDP channels/subnets
    (e.g. 20 x 250 drones) or give every drone its own IP - both work.
  * One asyncio socket per channel; pymavlink parses ~180k messages/s on one core,
    enough for 5,000 drones at the telemetry rates requested below.
  * Mission uploads run in parallel with a concurrency limit, with timeouts and
    retries per item.

Autopilot setup expected on every drone (see README for details)
  ArduPilot: AUTO_OPTIONS = 3 (arm + take off in AUTO), FS_GCS_ENABLE = 0 (drone
  finishes its show if the link drops), GPS RTK base corrections, SCR_ENABLE = 1
  for the LED player script.   PX4: COM_RCL_EXCEPT includes Mission, NAV_DLL_ACT = 0.
"""
from __future__ import annotations

import asyncio
import math
import time

import numpy as np
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav2

from ..geo import Origin
from ..show.mission import mission_items
from .state import FleetBackend

CMD_SET_MODE = 176
CMD_ARM = 400
CMD_MISSION_START = 300
CMD_CHANGE_SPEED = 178
CMD_REPOSITION = 192
CMD_SET_MSG_INTERVAL = 511
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

APM_MODES = {"AUTO": 3, "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9, "BRAKE": 17}
PX4_MODES = {"AUTO": (4, 4), "GUIDED": (4, 3), "LOITER": (4, 3), "RTL": (4, 5), "LAND": (4, 6),
             "BRAKE": (4, 3)}
TELEMETRY_RATES = {33: 2.0, 1: 1.0, 24: 1.0, 245: 1.0, 147: 0.5}   # msg id -> Hz


def parse_endpoint(ep: str):
    """'udp:0.0.0.0:14550' / 'udpin:...' / ':14550' -> (host, port)."""
    ep = ep.strip()
    for pre in ("udpin:", "udp:"):
        if ep.startswith(pre):
            ep = ep[len(pre):]
    host, _, port = ep.rpartition(":")
    return (host or "0.0.0.0"), int(port)


class _Proto(asyncio.DatagramProtocol):
    def __init__(self, fleet: "MavlinkFleet", ch: int):
        self.fleet, self.ch, self.transport = fleet, ch, None

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        self.fleet._rx(self.ch, data, addr)


class MavlinkFleet(FleetBackend):
    kind = "mavlink"

    def __init__(self, endpoints: list[str], settings: dict, capacity: int = 6000,
                 upload_concurrency: int = 64):
        super().__init__(capacity)
        self.endpoints = endpoints
        self.s = settings
        self.origin = Origin.from_dict(settings["origin"])
        self.keys: dict[tuple, int] = {}
        self.addr: list[tuple] = []           # fleet index -> (channel, (ip, port), sysid)
        self.parsers: dict[tuple, mav2.MAVLink] = {}
        self.protos: list[_Proto] = []
        self.tx = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
        self.mission_q: dict[int, asyncio.Queue] = {}
        self.acks: dict[tuple, int] = {}
        self.sem = asyncio.Semaphore(upload_concurrency)
        self.routes: dict[int, dict] = {}    # guided routes: idx -> {"wps": [...], "speed": v, "land": bool}
        self._hb_task = None
        self.loop = None
        self.rx_count = 0

    # ---- lifecycle ----------------------------------------------------------------------------
    async def start(self):
        self.loop = asyncio.get_running_loop()
        for ch, ep in enumerate(self.endpoints):
            host, port = parse_endpoint(ep)
            _, proto = await self.loop.create_datagram_endpoint(lambda ch=ch: _Proto(self, ch),
                                                                 local_addr=(host, port))
            self.protos.append(proto)
            self.on_event("info", f"MAVLink listening on udp {host}:{port} (channel {ch + 1})")
        self._hb_task = asyncio.create_task(self._heartbeat_loop())

    async def stop(self):
        if self._hb_task:
            self._hb_task.cancel()
        for p in self.protos:
            if p.transport:
                p.transport.close()
        self.protos.clear()

    async def _heartbeat_loop(self):
        hb = self.tx.heartbeat_encode(mav2.MAV_TYPE_GCS, mav2.MAV_AUTOPILOT_INVALID, 0, 0, mav2.MAV_STATE_ACTIVE)
        while True:
            buf = hb.pack(self.tx)
            for i in range(self.state.m):
                self._sendbuf(i, buf)
            await asyncio.sleep(1.0)

    # ---- receive ------------------------------------------------------------------------------
    def _index(self, ch, addr, sysid, autopilot="") -> int:
        key = (ch, addr, sysid)
        i = self.keys.get(key)
        if i is None:
            i = self.state.add(f"C{ch + 1}-{addr[0]}-{sysid}", autopilot)
            self.keys[key] = i
            self.addr.append(key)
            self._request_rates(i)
            self.on_event("info", f"drone discovered ({autopilot or 'unknown autopilot'})", i)
        return i

    def _rx(self, ch, data, addr):
        pk = (ch, addr)
        parser = self.parsers.get(pk)
        if parser is None:
            parser = mav2.MAVLink(None)
            parser.robust_parsing = True
            self.parsers[pk] = parser
        try:
            msgs = parser.parse_buffer(data) or []
        except Exception:   # noqa: BLE001 - corrupt packet, ignore
            return
        now = time.time()
        st = self.state
        for msg in msgs:
            t = msg.get_type()
            if t == "BAD_DATA":
                continue
            self.rx_count += 1
            sysid, comp = msg.get_srcSystem(), msg.get_srcComponent()
            if t == "HEARTBEAT":
                if msg.type == mav2.MAV_TYPE_GCS or comp != 1:
                    continue
                ap = "px4" if msg.autopilot == mav2.MAV_AUTOPILOT_PX4 else (
                    "ardupilot" if msg.autopilot == mav2.MAV_AUTOPILOT_ARDUPILOTMEGA else "other")
                i = self._index(ch, addr, sysid, ap)
                st.autopilot[i] = ap
                st.armed[i] = bool(msg.base_mode & mav2.MAV_MODE_FLAG_SAFETY_ARMED)
                try:
                    st.mode[i] = mavutil.mode_string_v10(msg)
                except Exception:   # noqa: BLE001
                    st.mode[i] = str(msg.custom_mode)
                st.last_seen[i] = now
                continue
            i = self.keys.get((ch, addr, sysid))
            if i is None:
                continue
            st.last_seen[i] = now
            if t == "GLOBAL_POSITION_INT":
                p = self.origin.wgs84_to_show(msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0)
                st.pos[i] = p
                vx, vy = self.origin.en_to_show(msg.vy / 100.0, msg.vx / 100.0)
                st.vel[i] = (vx, vy, -msg.vz / 100.0)
            elif t == "SYS_STATUS":
                if msg.battery_remaining >= 0:
                    st.batt[i] = msg.battery_remaining
                st.volt[i] = msg.voltage_battery / 1000.0
                en = msg.onboard_control_sensors_enabled & msg.onboard_control_sensors_present
                st.sensors_ok[i] = (en & ~msg.onboard_control_sensors_health) == 0
            elif t == "GPS_RAW_INT":
                st.fix[i] = msg.fix_type
                st.sats[i] = msg.satellites_visible if msg.satellites_visible != 255 else 0
            elif t == "EXTENDED_SYS_STATE":
                st.in_air[i] = msg.landed_state in (2, 3, 4)
            elif t == "BATTERY_STATUS":
                if msg.battery_remaining >= 0:
                    st.batt[i] = msg.battery_remaining
            elif t in ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"):
                q = self.mission_q.get(i)
                if q is not None:
                    q.put_nowait(msg)
            elif t == "COMMAND_ACK":
                self.acks[(i, msg.command)] = msg.result
                if msg.result not in (0, 5):          # not ACCEPTED / IN_PROGRESS
                    self.on_event("warn", f"command {msg.command} rejected (result {msg.result})", i)
            elif t == "STATUSTEXT":
                lvl = "error" if msg.severity <= 3 else ("warn" if msg.severity == 4 else "info")
                if msg.severity <= 4:
                    self.on_event(lvl, f"autopilot: {msg.text}", i)

    # ---- send ---------------------------------------------------------------------------------
    def _sendbuf(self, i: int, buf: bytes):
        ch, addr, _ = self.addr[i]
        tr = self.protos[ch].transport
        if tr is not None:
            tr.sendto(buf, addr)

    def _send(self, i: int, msg):
        self._sendbuf(i, msg.pack(self.tx))

    def _sys(self, i):
        return self.addr[i][2]

    def _cmd(self, i, command, p=(0, 0, 0, 0, 0, 0, 0), repeat=1):
        msg = self.tx.command_long_encode(self._sys(i), 1, command, 0, *[float(v) for v in p])
        buf = msg.pack(self.tx)
        self._sendbuf(i, buf)
        for k in range(1, repeat):
            self.loop.call_later(0.15 * k, self._sendbuf, i, buf)

    def _request_rates(self, i):
        for mid, hz in TELEMETRY_RATES.items():
            self._cmd(i, CMD_SET_MSG_INTERVAL, (mid, 1e6 / hz, 0, 0, 0, 0, 0))

    def _set_mode(self, i, name, repeat=2):
        if self.state.autopilot[i] == "px4":
            main, sub = PX4_MODES[name]
            self._cmd(i, CMD_SET_MODE, (1, main, sub, 0, 0, 0, 0), repeat)
        else:
            self._cmd(i, CMD_SET_MODE, (1, APM_MODES[name], 0, 0, 0, 0, 0), repeat)

    def _reposition(self, i, target_xyz, speed):
        lat, lon, alt = self.origin.show_to_wgs84(np.asarray(target_xyz, float))
        self._cmd(i, CMD_CHANGE_SPEED, (1, float(speed), -1, 0, 0, 0, 0))
        msg = self.tx.command_int_encode(self._sys(i), 1, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, CMD_REPOSITION,
                                         0, 0, float(speed), 1.0, 0, float("nan"),
                                         int(float(lat) * 1e7), int(float(lon) * 1e7), float(alt))
        self._send(i, msg)

    def set_param(self, i, name: str, value: float):
        msg = self.tx.param_set_encode(self._sys(i), 1, name.encode(), float(value),
                                       mav2.MAV_PARAM_TYPE_REAL32)
        self._send(i, msg)

    # ---- mission upload ------------------------------------------------------------------------
    async def _upload_one(self, i: int, items: list, retries: int = 6, timeout: float = 1.2) -> bool:
        if self.state.autopilot[i] == "px4":       # PX4 has no home item at seq 0
            items = items[1:]
        sysid = self._sys(i)
        q: asyncio.Queue = asyncio.Queue()
        self.mission_q[i] = q
        try:
            count = self.tx.mission_count_encode(sysid, 1, len(items), 0)
            self._send(i, count)
            tries = 0
            while True:
                try:
                    msg = await asyncio.wait_for(q.get(), timeout)
                except asyncio.TimeoutError:
                    tries += 1
                    if tries > retries:
                        return False
                    self._send(i, count)
                    continue
                t = msg.get_type()
                if t == "MISSION_ACK":
                    return msg.type == 0
                seq = msg.seq
                if seq >= len(items):
                    continue
                it = items[seq]
                item = self.tx.mission_item_int_encode(
                    sysid, 1, seq, it["frame"], it["command"], 0, 1, it["p1"], it["p2"], it["p3"], it["p4"],
                    int(round(it["lat"] * 1e7)), int(round(it["lon"] * 1e7)), it["alt"], 0)
                self._send(i, item)
                self.state.upload_progress[i] = (seq + 1) / len(items)
        finally:
            self.mission_q.pop(i, None)

    async def upload_show(self, show, idx, slots, start_utc=None, progress=None):
        st = self.state
        idx = np.asarray(idx, int)
        st.uploaded[: st.m] = False
        sync = self.s.get("mission_sync", "hold_time")
        done = 0
        ok_count = 0

        async def one(i, slot):
            nonlocal done, ok_count
            async with self.sem:
                sync_i = sync if st.autopilot[i] != "px4" else "hold_time"
                items = mission_items(show, int(slot), self.origin, sync_i, start_utc)
                ok = await self._upload_one(int(i), items)
                st.uploaded[i] = ok
                done += 1
                ok_count += int(ok)
                if not ok:
                    self.on_event("warn", "mission upload failed", int(i))
                if progress and done % 25 == 0:
                    progress(done / len(idx), f"Uploaded {done}/{len(idx)}")

        await asyncio.gather(*(one(i, s) for i, s in zip(idx, slots)))
        if progress:
            progress(1.0, f"Uploaded {ok_count}/{len(idx)}")
        return ok_count

    # ---- commands -------------------------------------------------------------------------------
    def arm(self, idx):
        for i in np.asarray(idx, int):
            self._cmd(int(i), CMD_ARM, (1, 0, 0, 0, 0, 0, 0), repeat=2)

    def disarm(self, idx):
        for i in np.asarray(idx, int):
            self._cmd(int(i), CMD_ARM, (0, 0, 0, 0, 0, 0, 0), repeat=2)

    def kill(self, idx):
        self._drop_pending(idx)
        for i in np.asarray(idx, int):
            self._cmd(int(i), CMD_ARM, (0, 21196, 0, 0, 0, 0, 0), repeat=3)

    def start_show(self, idx, t0, offsets=None):
        """Schedule: mode AUTO at t0-4 s, then arm + MISSION_START at t0 (wall clock).
        offsets (s, one per drone) stagger the take-offs of imported shows: drone k is
        started at t0 + offsets[k] (inf = never, it stays on its pad)."""
        loop = self.loop
        now = time.time()
        sod = t0 % 86400.0
        idx = [int(i) for i in np.asarray(idx, int)]
        offs = [0.0] * len(idx) if offsets is None else [float(o) for o in offsets]
        self._t0 = float(t0)
        self._pending = {i: o for i, o in zip(idx, offs) if math.isfinite(o)}
        self._paused = {}

        def prep():
            for i in idx:
                if self.state.autopilot[i] == "ardupilot":
                    self.set_param(i, "SCR_USER1", sod)    # LED player sync (seconds of day UTC)
                self._set_mode(i, "AUTO")

        self._start_handles = [loop.call_later(max(0.0, t0 - 4.0 - now), prep)]
        self._schedule(list(self._pending), now)

    def _schedule(self, drones, now):
        groups: dict = {}
        for i in drones:
            groups.setdefault(round(self._pending[i], 1), []).append(i)
        for o, grp in groups.items():
            self._start_handles.append(self.loop.call_later(max(0.0, self._t0 + o - now), self._go, grp))

    def _go(self, grp):
        for i in grp:
            if self._pending.pop(i, None) is None:      # cancelled, held or already started
                continue
            self._cmd(i, CMD_ARM, (1, 0, 0, 0, 0, 0, 0), repeat=2)
            self._cmd(i, CMD_MISSION_START, (0, 0, 0, 0, 0, 0, 0), repeat=2)

    def _drop_pending(self, idx) -> set:
        """Remove drones from the take-off schedule; returns those that were still waiting."""
        out = set()
        for i in np.asarray(idx, int):
            i = int(i)
            if getattr(self, "_pending", {}).pop(i, None) is not None or getattr(self, "_paused", {}).pop(i, None) is not None:
                out.add(i)
        return out

    def cancel_start(self, idx):
        self._drop_pending(idx)
        if not getattr(self, "_pending", None):
            for h in getattr(self, "_start_handles", []):
                h.cancel()
            self._start_handles = []
        self.disarm(idx)

    def hold(self, idx):
        pend = getattr(self, "_pending", {})
        for i in np.asarray(idx, int):
            i = int(i)
            if i in pend:                                   # not airborne yet: keep it on the pad
                self._paused[i] = pend.pop(i)
                continue
            self.routes.pop(i, None)
            self._set_mode(i, "BRAKE", repeat=3)

    def resume(self, idx, t_shift):
        self._t0 = getattr(self, "_t0", time.time()) + float(t_shift)
        back = []
        for i in np.asarray(idx, int):
            i = int(i)
            if i in getattr(self, "_paused", {}):
                self._pending[i] = self._paused.pop(i)
                back.append(i)
                continue
            self._set_mode(i, "AUTO", repeat=3)
        if back:
            self._schedule(back, time.time())

    def land(self, idx):
        waiting = self._drop_pending(idx)
        for i in np.asarray(idx, int):
            if int(i) in waiting:
                continue
            self.routes.pop(int(i), None)
            self._set_mode(int(i), "LAND", repeat=3)

    def goto(self, idx, targets, speeds, then_land=False):
        for i, tgt, v in zip(np.asarray(idx, int), np.asarray(targets, float), np.asarray(speeds, float)):
            i = int(i)
            self.routes[i] = {"wps": [], "target": tgt, "speed": max(0.3, float(v)), "land": then_land}
            self._set_mode(i, "GUIDED")
            self._reposition(i, tgt, max(0.3, float(v)))

    def exit_path(self, i, waypoints, speed):
        wps = [np.asarray(w, float) for w in waypoints]
        if not wps:
            return
        first = wps.pop(0)
        self.routes[int(i)] = {"wps": wps, "target": first, "speed": speed, "land": True}
        self._set_mode(int(i), "GUIDED", repeat=3)
        self._reposition(int(i), first, speed)

    def tick(self, now, dt):
        st = self.state
        for i, r in list(self.routes.items()):
            if np.isnan(st.pos[i, 0]):
                continue
            if np.linalg.norm(st.pos[i] - r["target"]) < 1.0:
                if r["wps"]:
                    r["target"] = r["wps"].pop(0)
                    self._reposition(i, r["target"], r["speed"])
                else:
                    del self.routes[i]
                    if r["land"]:
                        self._set_mode(i, "LAND", repeat=3)

