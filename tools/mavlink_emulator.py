"""MAVLink drone emulator - many lightweight virtual ArduPilot-like drones over UDP.

Use it to rehearse the real MAVLink path of LangitSwarm without hardware:

    python tools/mavlink_emulator.py --drones 100 --gcs 127.0.0.1:14550
    (then in LangitSwarm: Fleet & Show -> Real drones - MAVLink UDP -> udp:0.0.0.0:14550)

Every emulated drone has its own UDP socket (own port) and, like a real fleet,
they may all use system ID 1.  Implements: heartbeat + telemetry, SET_MESSAGE_INTERVAL,
DO_SET_MODE, arm/disarm/force-disarm, MISSION_START, DO_CHANGE_SPEED, DO_REPOSITION,
PARAM_SET, the mission upload protocol and a simple mission executor
(TAKEOFF, WAYPOINT with hold, CHANGE_SPEED, NAV_DELAY relative or until UTC, LAND).
This is a protocol test tool, not a flight-dynamics simulator - use ArduPilot SITL
for that.
"""
from __future__ import annotations

import argparse
import asyncio
import datetime as dt
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pymavlink.dialects.v20 import ardupilotmega as mav2  # noqa: E402

from langit.geo import Origin  # noqa: E402
from langit.show.compiler import field_layout_for  # noqa: E402
from langit.show.spec import default_show, merged_settings  # noqa: E402

RUNNING: list = []     # emulated drones of the current run (for tests)
MODES = {3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND", 17: "BRAKE", 0: "STABILIZE"}


class Drone(asyncio.DatagramProtocol):
    def __init__(self, idx, sysid, gcs, origin: Origin, pad_xyz):
        self.idx, self.sysid, self.gcs, self.origin = idx, sysid, gcs, origin
        self.mav = mav2.MAVLink(None, srcSystem=sysid, srcComponent=1)
        self.parser = mav2.MAVLink(None)
        self.parser.robust_parsing = True
        self.pos = np.array(pad_xyz, float) + np.r_[np.random.normal(0, 0.05, 2), 0]
        self.home = self.pos.copy()
        self.armed = False
        self.mode = 0
        self.items: list = []
        self.expect = 0
        self.mission_seq = -1
        self.started = False
        self.speed_h, self.speed_up, self.speed_dn = 5.0, 2.5, 2.5
        self.hold_until = 0.0
        self.target = None
        self.batt = 100.0
        self.transport = None
        self.rates = {}

    def connection_made(self, transport):
        self.transport = transport

    def send(self, msg):
        if self.transport:
            self.transport.sendto(msg.pack(self.mav), self.gcs)

    def ack(self, cmd, result=0):
        self.send(self.mav.command_ack_encode(cmd, result))

    def datagram_received(self, data, addr):
        for m in self.parser.parse_buffer(data) or []:
            t = m.get_type()
            if t == "COMMAND_LONG" and m.target_system in (0, self.sysid):
                self.on_cmd(m.command, [m.param1, m.param2, m.param3, m.param4, m.param5, m.param6, m.param7])
            elif t == "COMMAND_INT" and m.target_system in (0, self.sysid):
                if m.command == 192:     # DO_REPOSITION
                    p = self.origin.wgs84_to_show(m.x / 1e7, m.y / 1e7, m.z)
                    self.target = np.asarray(p, float)
                    if m.param1 and m.param1 > 0:
                        self.speed_h = m.param1
                    self.mode = 4
                    self.ack(192)
            elif t == "MISSION_COUNT" and m.target_system == self.sysid:
                self.items = [None] * m.count
                self.expect = 0
                self.send(self.mav.mission_request_int_encode(255, 190, 0, 0))
            elif t == "MISSION_ITEM_INT" and m.target_system == self.sysid:
                if m.seq < len(self.items):
                    self.items[m.seq] = m
                    if m.seq + 1 < len(self.items):
                        self.send(self.mav.mission_request_int_encode(255, 190, m.seq + 1, 0))
                    else:
                        self.send(self.mav.mission_ack_encode(255, 190, 0, 0))
            elif t == "PARAM_SET" and m.target_system == self.sysid:
                pid = m.param_id.encode() if isinstance(m.param_id, str) else m.param_id
                self.send(self.mav.param_value_encode(pid, m.param_value, m.param_type, 1, 0))

    def on_cmd(self, cmd, p):
        if cmd == 511:
            self.rates[int(p[0])] = p[1]
            self.ack(cmd)
        elif cmd == 176:
            self.mode = int(p[1])
            self.ack(cmd)
            if self.mode == 4 and self.target is None:
                self.target = self.pos.copy()
        elif cmd == 400:
            if p[0] >= 1:
                if not self.items and self.mode == 3:
                    return self.ack(cmd, 4)
                self.armed = True
                if self.mode == 3:          # AUTO_OPTIONS=3 behaviour: arming in AUTO starts the mission
                    self._begin()
            else:
                if p[1] == 21196 or self.pos[2] < 0.2:
                    self.armed = False
                    self.started = False
            self.ack(cmd)
        elif cmd == 300:
            if self.armed and self.mode == 3:
                self._begin()
            self.ack(cmd)
        elif cmd == 178:
            if int(p[0]) in (0, 1):
                self.speed_h = max(0.1, p[1])
            elif int(p[0]) == 2:
                self.speed_up = max(0.1, p[1])
            elif int(p[0]) == 3:
                self.speed_dn = max(0.1, p[1])
            self.ack(cmd)
        else:
            self.ack(cmd, 3)

    def _begin(self):
        if not self.started:
            self.started = True
            self.mission_seq = 1      # skip home
            self.hold_until = 0.0
            self.target = None

    def _item_target(self, it):
        return np.asarray(self.origin.wgs84_to_show(it.x / 1e7, it.y / 1e7, it.z), float)

    def step(self, dt_):
        now = time.time()
        if self.armed and self.mode == 3 and self.started and 0 < self.mission_seq < len(self.items):
            it = self.items[self.mission_seq]
            c = it.command
            if c == 178:
                if int(it.param1) in (0, 1):
                    self.speed_h = max(0.1, it.param2)
                elif int(it.param1) == 2:
                    self.speed_up = max(0.1, it.param2)
                else:
                    self.speed_dn = max(0.1, it.param2)
                self.mission_seq += 1
            elif c == 93:
                if it.param1 >= 0:                       # relative delay (seconds)
                    if self.hold_until == 0.0:
                        self.hold_until = now + it.param1
                    if now >= self.hold_until:
                        self.hold_until = 0.0
                        self.mission_seq += 1
                else:                                    # until UTC time of day
                    d = dt.datetime.now(dt.timezone.utc)
                    sod = d.hour * 3600 + d.minute * 60 + d.second + d.microsecond / 1e6
                    if sod >= it.param2 * 3600 + it.param3 * 60 + it.param4:
                        self.mission_seq += 1
            elif c in (16, 22, 21):
                tgt = self._item_target(it)
                if c == 22:
                    tgt = np.r_[self.pos[:2], it.z]
                if c == 21:
                    tgt = np.r_[tgt[:2], 0.0]
                if self._move(tgt, dt_):
                    if c == 21:
                        self.pos[2] = 0.0
                        self.armed = False
                        self.started = False
                        self.mission_seq = len(self.items)
                    elif self.hold_until == 0.0:
                        self.hold_until = now + it.param1
                    if self.hold_until and now >= self.hold_until:
                        self.hold_until = 0.0
                        self.mission_seq += 1
            else:
                self.mission_seq += 1
        elif self.armed and self.mode == 4 and self.target is not None:
            self._move(self.target, dt_)
        elif self.mode == 9 and self.pos[2] > 0:
            self.pos[2] = max(0.0, self.pos[2] - 1.5 * dt_)
            if self.pos[2] == 0.0:
                self.armed = False
        if self.pos[2] > 0.1:
            self.batt -= dt_ * 100 / (18 * 60)

    def _move(self, tgt, dt_):
        d = tgt - self.pos
        dh = np.hypot(d[0], d[1])
        if dh > 1e-6:
            step = min(dh, self.speed_h * dt_)
            self.pos[:2] += d[:2] / dh * step
        if abs(d[2]) > 1e-6:
            v = self.speed_up if d[2] > 0 else self.speed_dn
            self.pos[2] += np.sign(d[2]) * min(abs(d[2]), v * dt_)
        return np.linalg.norm(tgt - self.pos) < 0.3

    def telemetry(self, k):
        base = mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mav2.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
        if k % 5 == 0:
            self.send(self.mav.heartbeat_encode(mav2.MAV_TYPE_QUADROTOR, mav2.MAV_AUTOPILOT_ARDUPILOTMEGA, base,
                                                self.mode, mav2.MAV_STATE_ACTIVE))
            sensors = 0x3FFF
            self.send(self.mav.sys_status_encode(sensors, sensors, sensors, 200, int(14000 + 26 * self.batt), -1,
                                                 int(self.batt), 0, 0, 0, 0, 0, 0))
            lat, lon, _ = self.origin.show_to_wgs84(self.pos)
            self.send(self.mav.gps_raw_int_encode(int(time.time() * 1e6), 6, int(lat * 1e7), int(lon * 1e7),
                                                  0, 80, 120, 0, 0, 24))
            self.send(self.mav.extended_sys_state_encode(0, 2 if self.pos[2] > 0.2 else 1))
        if k % 2 == 0:
            lat, lon, alt = self.origin.show_to_wgs84(self.pos)
            self.send(self.mav.global_position_int_encode(int(time.time() * 1000) & 0xFFFFFFFF, int(lat * 1e7),
                                                          int(lon * 1e7), int((self.origin.alt_amsl + alt) * 1000),
                                                          int(alt * 1000), 0, 0, 0, 0))


async def run(n: int, gcs: tuple, spec: dict, same_sysid: bool = True, duration: float | None = None, rate=10.0):
    s = merged_settings(spec)
    origin = Origin.from_dict(s["origin"])
    pads, _ = field_layout_for(spec, int(spec["drones"]), int(spec.get("spares", 0)), s)
    loop = asyncio.get_running_loop()
    drones = []
    for i in range(min(n, len(pads))):
        d = Drone(i, 1 if same_sysid else (i % 250) + 1, gcs, origin, pads[i])
        await loop.create_datagram_endpoint(lambda d=d: d, local_addr=("0.0.0.0", 0))
        drones.append(d)
    print(f"{len(drones)} emulated drones streaming to {gcs[0]}:{gcs[1]}", flush=True)
    RUNNING.extend(drones)
    t_end = time.time() + duration if duration else None
    k = 0
    dt_ = 1.0 / rate
    while t_end is None or time.time() < t_end:
        for d in drones:
            d.step(dt_)
            d.telemetry(k)
        k += 1
        await asyncio.sleep(dt_)
    return drones


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drones", type=int, default=50)
    ap.add_argument("--gcs", default="127.0.0.1:14550")
    ap.add_argument("--spec", help="show JSON (pads come from its field layout, or the pads of an imported "
                                   "Skybrush show); default demo show")
    ap.add_argument("--unique-sysid", action="store_true", help="give drones system IDs 1..250 instead of all 1")
    a = ap.parse_args()
    host, port = a.gcs.rsplit(":", 1)
    spec = json.load(open(a.spec, encoding="utf-8")) if a.spec else default_show(a.drones, 0)
    spec.setdefault("drones", a.drones)
    asyncio.run(run(a.drones, (host, int(port)), spec, same_sysid=not a.unique_sysid))


if __name__ == "__main__":
    main()
