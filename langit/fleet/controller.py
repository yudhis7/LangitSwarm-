"""Show controller: operations state machine, live safety monitors and
emergency procedures for the whole fleet.

Normal flow
  CONNECTED -> plan (map drones to pads, choose flyers + spares, re-compile the show
  from the drones that are really there, validate) -> PLANNED -> upload -> UPLOADED
  -> go/no-go (per-drone checks + operator checklist + wind) -> COUNTDOWN -> RUNNING
  -> FINISHED

Live monitors (10 Hz)
  * deviation from the planned position (warn / take drone out of the show)
  * geofence and audience-zone breach (land / terminate, configurable)
  * critical battery, link loss, GPS degradation
Emergency actions
  * HOLD all (pause in place) and RESUME (show clock shifts for every drone)
  * COORDINATED RETURN: collision-free return of every airborne drone to the pads
    (spread out if needed, then CAPT assignment to the landing layer, then land)
  * LAND ALL in place, KILL (motor stop), per-drone land / kill / take-out
"""
from __future__ import annotations

import math
import os
import time
from collections import deque

import numpy as np
from scipy.spatial import cKDTree

from ..show.assignment import assign_rect
from ..geo import Origin
from ..show.compiler import compile_show, field_layout_for
from ..show.spec import merged_settings
from ..show.validator import validate
from .blackbox import BlackBox
from .preflight import check_drones, choose_flyers, map_to_pads
from .state import (ARMED, CONNECTED, ERROR, F_AUDIENCE, F_CRASH, F_CRIT_BATT, F_DEVIATION, F_EXIT, F_GEOFENCE,
                    F_LINK, FAILSAFE, FLYING, LANDED, OFFLINE, READY, ROLE_EXCLUDED, ROLE_FLYER, ROLE_SPARE, SPARE,
                    STATUS_NAMES, WARNING, flag_text)

CHECKLIST = [
    ("permit", "Flight permit and airspace approval valid for this time and place"),
    ("site", "Take-off field and audience zone secured, marshals in position"),
    ("weather", "Wind, rain and visibility within limits"),
    ("rtk", "RTK base station running, corrections reaching the drones"),
    ("radio", "Spectrum check done, no interference on show frequencies"),
    ("pic", "Pilot in command and safety officer at the emergency controls"),
]

ACTIVE_STATES = ("COUNTDOWN", "RUNNING", "HOLDING", "RETURNING", "LANDING")


class OpError(Exception):
    pass


class NeedsConfirm(OpError):
    """The action is possible but has a specific risk the operator must accept."""


class ShowController:
    def __init__(self, spec: dict, log_root: str):
        self.spec = spec
        self.log_root = log_root
        self.backend = None
        self.state = "DISCONNECTED"
        self.events: deque = deque(maxlen=500)
        self.seq = 0
        self.live_show = None
        self.live_validation = None
        self.flyers = np.zeros(0, int)
        self.pads = None
        self.is_spare_pad = None
        self.offsets = None
        self.t0 = None
        self.hold_started = None
        self.taken_out: set = set()
        self.flown = None
        self.dev_since = None
        self.rtk_since = None
        self.hold_missed = np.zeros(0, int)
        self.stuck: dict = {}
        self.checklist = {k: False for k, _ in CHECKLIST}
        self.wind = None
        self.progress = {"task": "", "frac": 0.0, "msg": ""}
        self.gonogo = None
        self.ret = None
        self.blackbox = None
        self._last_bb = 0.0
        self._busy = False
        self.audience_action = "land"
        self.wait_idx = np.zeros(0, int)      # imported shows: flyers still waiting for their take-off time
        self.wait_t = np.zeros(0)
        self.held_waiting = np.zeros(0, int)
        self.held_idx = np.zeros(0, int)

    # ---- helpers ------------------------------------------------------------------------------
    @property
    def s(self) -> dict:
        return merged_settings(self.spec)

    def log(self, level: str, msg: str, drone: int | None = None):
        self.seq += 1
        did = None
        if drone is not None and self.backend is not None and 0 <= drone < self.backend.state.m:
            did = self.backend.state.ids[drone]
        rec = {"seq": self.seq, "t": time.time(), "level": level, "msg": msg, "drone": did, "idx": drone}
        self.events.append(rec)
        if self.blackbox:
            self.blackbox.event(rec)

    def _set_progress(self, task, frac, msg):
        self.progress = {"task": task, "frac": float(frac), "msg": msg}

    def _require(self, *states):
        if self.state not in states:
            raise OpError(f"not possible in state {self.state} (needs {' / '.join(states)})")

    def _airborne(self, include_out=False):
        st = self.backend.state
        idx = self.flyers
        idx = idx[st.in_air[idx]]
        if not include_out and self.taken_out:
            idx = np.array([i for i in idx if i not in self.taken_out], dtype=int)
        return idx

    @property
    def imported(self) -> bool:
        return bool(self.spec.get("imported"))

    def _waiting(self):
        """Flyers of an imported show that have not taken off yet (staggered take-off)."""
        return self.wait_idx

    def _cancel_waiting(self, why: str):
        w = self.wait_idx
        if len(w):
            self.backend.cancel_start(w)
            self.log("warn", f"{len(w)} drones had not taken off yet and stay on their pads ({why})")
        self.wait_idx = np.zeros(0, int)
        self.wait_t = np.zeros(0)
        self.held_waiting = np.zeros(0, int)

    # ---- connection ---------------------------------------------------------------------------------
    async def connect(self, kind: str, **kw):
        if self.state in ACTIVE_STATES:
            raise OpError("cannot change the fleet connection while a show is active")
        await self.disconnect()
        s = self.s
        n = int(kw.get("drones") or self.spec.get("drones", 100))
        spares = int(kw.get("spares") if kw.get("spares") is not None else self.spec.get("spares", 0))
        self.pads, self.is_spare_pad = field_layout_for(self.spec, n, spares, s)
        if kind == "sim":
            from .sim import SimFleet
            self.backend = SimFleet(self.pads, s, preflight_fault_rate=float(kw.get("preflight_fault_rate", 0.01)),
                                    inflight_fault_per_min=float(kw.get("inflight_fault_per_min", 0.0004)),
                                    seed=kw.get("seed", 7))
        elif kind == "mavlink":
            from .mavlink_link import MavlinkFleet
            eps = kw.get("endpoints") or ["udp:0.0.0.0:14550"]
            self.backend = MavlinkFleet(eps, s, capacity=max(6000, int((n + spares) * 1.2)),
                                        upload_concurrency=int(kw.get("upload_concurrency", 64)))
        else:
            raise OpError(f"unknown fleet type {kind}")
        self.backend.on_event = self.log
        await self.backend.start()
        cap = self.backend.state.capacity
        self.flown = np.zeros(cap, bool)
        self.dev_since = np.full(cap, np.inf)
        self.rtk_since = np.full(cap, np.inf)
        self.state = "CONNECTED"
        self.live_show = None
        self.taken_out.clear()
        self.log("info", f"connected to {'simulated' if kind == 'sim' else 'MAVLink'} fleet "
                         f"({n} show pads + {spares} spare pads)")

    async def disconnect(self):
        if self.backend is not None:
            await self.backend.stop()
            self.backend = None
            self.log("info", "fleet disconnected")
        self.state = "DISCONNECTED"
        self.flyers = np.zeros(0, int)

    # ---- planning ----------------------------------------------------------------------------------
    async def plan(self, loop_exec):
        self._require("CONNECTED", "PLANNED", "UPLOADED", "FINISHED", "ABORTED")
        st = self.backend.state
        if self.state in ("FINISHED", "ABORTED"):           # next flight with the same fleet
            if st.in_air[: st.m].any():
                raise OpError("drones are still airborne")
            self.flown[:] = False
            self.taken_out.clear()
            self.stuck.clear()
            self.t0 = self.hold_started = self.ret = self.gonogo = None
            st.flags[: st.m] = 0
            if self.blackbox:
                self.blackbox.close()
                self.blackbox = None
        s = self.s
        now = time.time()
        self.offsets = map_to_pads(st, self.pads, float(s["max_slot_offset"]), now)
        ready, flags = check_drones(st, s, now, self.offsets, require_mission=False)
        st.flags[: st.m] = flags
        n_needed = int(self.spec.get("drones", 0))
        chosen = choose_flyers(st, ready, self.is_spare_pad, n_needed, self.pads, use_spares=not self.imported)
        need = int(math.ceil(n_needed * float(s["min_ready_fraction"])))
        n_bad = int((~ready).sum())
        if len(chosen) < need:
            self.log("error", f"NO-GO: only {len(chosen)} ready drones, need at least {need} "
                              f"({n_bad} drones failing checks)")
            raise OpError(f"only {len(chosen)} drones are ready; the show needs at least {need}")
        if len(chosen) < n_needed:
            if self.imported:
                self.log("warn", f"flying with {len(chosen)} of {n_needed} drones - the imported paths of pads "
                                 f"without a ready drone are left out (spares cannot stand in)")
            else:
                self.log("warn", f"flying with {len(chosen)} of {n_needed} drones - formations are re-sampled")
        start = self.pads[st.pad[chosen]]
        self.state = "PLANNING"
        self._set_progress("plan", 0.0, "Re-compiling show from the drones on the field")
        try:
            show = await loop_exec(lambda: compile_show(
                self.spec, start_positions=start,
                progress=lambda f, m: self._set_progress("plan", 0.8 * f, m)))
            val = await loop_exec(lambda: validate(
                show, progress=lambda f, m: self._set_progress("plan", 0.8 + 0.2 * f, m)))
            if getattr(show, "imported", False) and self.backend.kind == "mavlink":
                info = await loop_exec(lambda: show.prepare_missions(
                    Origin.from_dict(s["origin"]), s.get("mission_sync", "hold_time"), None,
                    progress=lambda f, m: self._set_progress("plan", 0.95 + 0.05 * f, m)))
                msg = (f"missions built: up to {info['max_items']} items per drone "
                       f"(limit {int(s.get('mission_max_items', 700))})")
                if info.get("loosened"):
                    msg += f"; path tolerance loosened to {info['max_tol']} m for {info['loosened']} drones"
                self.log("warn" if info.get("loosened") else "info", msg)
        except Exception as e:
            self.state = "CONNECTED"
            self._set_progress("", 0, "")
            raise OpError(f"planning failed: {e}") from e
        self._set_progress("", 1.0, "")
        self.live_show, self.live_validation = show, val
        st.role[: st.m] = ROLE_EXCLUDED
        st.role[np.nonzero(ready)[0]] = ROLE_SPARE
        st.role[chosen] = ROLE_FLYER
        st.slot[: st.m] = -1
        st.slot[chosen] = np.arange(len(chosen))
        self.flyers = chosen
        if not val["ok"]:
            self.state = "CONNECTED"
            bad = [c["name"] for c in val["checks"] if c["status"] == "fail"]
            self.log("error", f"live plan failed validation: {', '.join(bad)}")
            raise OpError("the re-planned show failed validation: " + ", ".join(bad))
        self.state = "PLANNED"
        self.log("info", f"plan ready: {len(chosen)} drones fly, {int(ready.sum()) - len(chosen)} ready "
                         f"spares stand by, show {show.duration / 60:.1f} min")

    async def upload(self):
        self._require("PLANNED", "UPLOADED")
        s = self.s
        start_utc = None
        if s.get("mission_sync") == "utc_delay":
            start_utc = getattr(self, "scheduled_start", None)
            if not start_utc:
                raise OpError("mission sync 'utc_delay' needs a scheduled start time before upload")
        self.state = "UPLOADING"
        st = self.backend.state
        slots = st.slot[self.flyers]
        self._set_progress("upload", 0.0, "Uploading")
        try:
            n_ok = await self.backend.upload_show(self.live_show, self.flyers, slots, start_utc,
                                                  progress=lambda f, m: self._set_progress("upload", f, m))
        except Exception as e:
            self.state = "PLANNED"
            raise OpError(f"upload failed: {e}") from e
        self._set_progress("", 1.0, "")
        need = int(math.ceil(len(self.flyers) * float(s["min_ready_fraction"])))
        if n_ok < need:
            self.state = "PLANNED"
            self.log("error", f"upload: only {n_ok}/{len(self.flyers)} drones accepted the show")
            raise OpError(f"only {n_ok} of {len(self.flyers)} drones accepted the show")
        self.state = "UPLOADED"
        self.log("info", f"show uploaded to {n_ok}/{len(self.flyers)} drones")
        self.evaluate_gonogo()

    def evaluate_gonogo(self) -> dict:
        st = self.backend.state
        s = self.s
        now = time.time()
        self.offsets = map_to_pads(st, self.pads, float(s["max_slot_offset"]), now)
        ready, flags = check_drones(st, s, now, self.offsets, require_mission=True)
        st.flags[: st.m] = flags
        fl = self.flyers
        # a flyer must still be on the pad it was planned for
        same_pad = np.ones(len(fl), bool)
        if self.live_show is not None and len(fl):
            planned = self.live_show.start_positions[st.slot[fl]]
            same_pad = np.hypot(*(st.pos[fl, :2] - planned[:, :2]).T) <= float(s["max_slot_offset"])
        fl_ready = ready[fl] & same_pad if len(fl) else np.zeros(0, bool)
        need = int(math.ceil(int(self.spec.get("drones", 0)) * float(s["min_ready_fraction"])))
        reasons: dict = {}
        for i in fl[~fl_ready] if len(fl) else []:
            txt = flag_text(int(st.flags[i])) or "moved off its planned pad"
            reasons[txt] = reasons.get(txt, 0) + 1
        wind_ok = self.wind is not None and float(self.wind) <= float(s["max_wind"])
        check_ok = all(self.checklist.values())
        drones_ok = int(fl_ready.sum()) >= need
        self.gonogo = {
            "go": bool(drones_ok and check_ok and wind_ok and self.state in ("UPLOADED",)),
            "ready": int(fl_ready.sum()), "flyers": int(len(fl)), "required": need,
            "reasons": reasons, "checklist_ok": check_ok, "wind_ok": wind_ok,
            "wind": self.wind, "max_wind": float(s["max_wind"]),
        }
        self._ready_flyers = fl[fl_ready] if len(fl) else fl
        return self.gonogo

    # ---- show execution --------------------------------------------------------------------------
    def start(self, countdown: float = 30.0):
        self._require("UPLOADED")
        g = self.evaluate_gonogo()
        if not g["go"]:
            why = []
            if g["ready"] < g["required"]:
                why.append(f"only {g['ready']}/{g['required']} drones ready")
            if not g["checklist_ok"]:
                why.append("operator checklist incomplete")
            if not g["wind_ok"]:
                why.append("wind not entered or above limit")
            raise OpError("NO-GO: " + "; ".join(why))
        countdown = max(10.0, float(countdown))
        self.t0 = time.time() + countdown
        fly = self._ready_flyers
        not_ready = np.setdiff1d(self.flyers, fly)
        if len(not_ready):
            self.log("warn", f"{len(not_ready)} planned drones failed final checks and stay on the ground")
            self.backend.state.role[not_ready] = ROLE_EXCLUDED
        self.flyers = fly
        self.blackbox = BlackBox(self.log_root, self.spec.get("name", "show"))
        self.blackbox.write_json("show.json", self.spec)
        self.blackbox.write_json("gonogo.json", g)
        if getattr(self.live_show, "imported", False):
            offs = self.live_show.takeoff_offsets[self.backend.state.slot[fly]]
            self.backend.start_show(fly, self.t0, offsets=offs)
            later = np.isfinite(offs) & (offs > 0.5)
            self.wait_idx, self.wait_t = fly[later], offs[later]
            if later.any():
                self.log("info", f"staggered take-off: {int(later.sum())} drones lift off between "
                                 f"T+{offs[later].min():.0f} s and T+{offs[later].max():.0f} s")
        else:
            self.backend.start_show(fly, self.t0)
            self.wait_idx, self.wait_t = np.zeros(0, int), np.zeros(0)
        self.held_waiting = np.zeros(0, int)
        self.state = "COUNTDOWN"
        self.taken_out.clear()
        self.log("info", f"countdown started: {len(fly)} drones, take-off in {countdown:.0f} s")

    def abort_countdown(self):
        self._require("COUNTDOWN")
        self.backend.cancel_start(self.flyers)
        self.wait_idx, self.wait_t, self.held_waiting = np.zeros(0, int), np.zeros(0), np.zeros(0, int)
        self.state = "UPLOADED"
        self.t0 = None
        self.log("warn", "countdown aborted by operator - drones disarmed")

    HOLD_MODES = ("HOLD", "BRAKE", "LOITER", "AUTO.LOITER", "POSHOLD", "GUIDED")

    def _unpaused(self):
        """Airborne show drones that did not pause (no link at HOLD time, or still not in a
        hold mode 2 s after it).  They keep flying the show on the original clock."""
        st = self.backend.state
        now = time.time()
        idx = self._airborne()
        if len(idx) == 0 or self.hold_started is None:
            return np.zeros(0, int)
        link = (now - st.last_seen[idx]) < 3.0
        mode_ok = np.array([st.mode[i] in self.HOLD_MODES for i in idx], bool)
        missed = np.isin(idx, self.hold_missed)
        late = ~mode_ok & (now - self.hold_started > 2.0)
        return idx[missed | (link & late) | ~link & ~mode_ok]

    def _no_link_airborne(self):
        st = self.backend.state
        idx = self._airborne()
        return idx[(time.time() - st.last_seen[idx]) >= 3.0] if len(idx) else idx

    def hold_all(self, force: bool = False):
        self._require("RUNNING")
        idx = self._airborne()
        now = time.time()
        no_link = self._no_link_airborne()
        if len(no_link) and not force:
            raise NeedsConfirm(f"{len(no_link)} airborne drones have no link and cannot pause. They will keep "
                               f"flying the show while the others stop, so their paths may cross. "
                               f"Keeping the show running is usually safer.")
        waiting = self._waiting()
        self.backend.hold(np.concatenate([idx, waiting]).astype(int))
        self.held_waiting = waiting.copy()
        self.held_idx = idx.copy()
        self.hold_started = now
        self.hold_missed = no_link
        self.state = "HOLDING"
        self.log("warn", f"HOLD: {len(idx)} drones pausing in place"
                         + (f", {len(waiting)} waiting drones keep their take-off on hold" if len(waiting) else ""))
        if len(no_link):
            self.log("error", f"{len(no_link)} drones have no link and cannot pause - they continue their show; "
                              f"RESUME will be blocked while they fly (use COORDINATED RETURN)")

    def resume(self):
        self._require("HOLDING")
        bad = self._unpaused()
        if len(bad):
            raise OpError(f"{len(bad)} airborne drones did not pause and are still flying the show on the "
                          f"original clock; resuming would put them out of sync with the rest. "
                          f"Use COORDINATED RETURN, or wait until they are on the ground")
        shift = time.time() - self.hold_started
        # resume every drone that was paused (a drone that had just lifted may read as
        # "on the ground" now) plus anything airborne, plus drones whose take-off was held
        held = getattr(self, "held_idx", np.zeros(0, int))
        held = np.array([i for i in held if i not in self.taken_out], dtype=int)
        idx = np.union1d(self._airborne(), held).astype(int)
        self.backend.resume(np.concatenate([idx, self.held_waiting]).astype(int), shift)
        self.held_idx = np.zeros(0, int)
        self.held_waiting = np.zeros(0, int)
        self.t0 += shift
        self.state = "RUNNING"
        self.log("info", f"show resumed after {shift:.1f} s hold")

    def land_all(self):
        self._require(*ACTIVE_STATES)
        if self.state == "COUNTDOWN":
            return self.abort_countdown()
        idx = self._airborne(include_out=True)
        self._cancel_waiting("land all")
        self.backend.land(idx)
        self.state = "LANDING"
        self.ret = None
        self.log("error", f"LAND ALL in place: {len(idx)} drones descending (vertical, no coordination)")

    def kill_all(self, confirm: str):
        if confirm != "KILL":
            raise OpError("type KILL to confirm motor stop of all drones")
        st = self.backend.state
        self._cancel_waiting("kill all")
        idx = self.flyers[st.armed[self.flyers] | st.in_air[self.flyers]]
        self.backend.kill(idx)
        self.state = "ABORTED"
        self.log("error", f"KILL ALL: motors stopped on {len(idx)} drones")

    def drone_action(self, i: int, action: str):
        st = self.backend.state
        if not (0 <= i < st.m):
            raise OpError("unknown drone")
        if i in set(self.wait_idx.tolist()):
            keep = self.wait_idx != i
            self.wait_idx, self.wait_t = self.wait_idx[keep], self.wait_t[keep]
            self.held_waiting = self.held_waiting[self.held_waiting != i]
            self.backend.cancel_start([i])
        if action == "land":
            self.backend.land([i])
            self.taken_out.add(i)
            st.flags[i] |= F_EXIT
            self.log("warn", "operator: land", i)
        elif action == "kill":
            self.backend.kill([i])
            self.taken_out.add(i)
            self.log("error", "operator: KILL (motor stop)", i)
        elif action == "exit":
            self.safe_exit(i, "operator request")
        else:
            raise OpError(f"unknown action {action}")

    # ---- coordinated return -----------------------------------------------------------------------
    async def coordinated_return(self, loop_exec, force: bool = False):
        self._require("RUNNING", "HOLDING")
        st = self.backend.state
        s = self.s
        loose = self._no_link_airborne()
        if len(loose) and not force:
            raise NeedsConfirm(f"{len(loose)} airborne drones have no link and will keep flying their show "
                               f"during the return; returning drones may cross their paths.")
        idx = self._airborne()
        if len(idx) == 0 and not len(self._waiting()):
            raise OpError("no airborne drones")
        self._cancel_waiting("coordinated return")
        if len(idx) == 0:
            self.state = "LANDING"
            return
        self.backend.hold(idx)                       # stop first, then plan
        P = st.pos[idx].copy()
        D = float(s["min_separation"])
        need = math.sqrt(2) * D * 1.02
        d, _ = cKDTree(P).query(P, k=2) if len(P) > 1 else (np.array([[0, np.inf]]), None)
        dmin = float(d[:, 1].min())
        targets1 = None
        if dmin < need:
            f = min(need / max(dmin, 0.1), 2.0)
            c = np.array([P[:, 0].mean(), P[:, 1].min(), max(P[:, 2].mean(), float(s["min_altitude"]))])
            targets1 = c + (P - c) * f            # all pair distances grow; y only increases
            g = s["geofence"]
            targets1[:, 0] = np.clip(targets1[:, 0], g["x"][0] + 2, g["x"][1] - 2)
            targets1[:, 1] = np.clip(targets1[:, 1], g["y"][0] + 2, g["y"][1] - 2)
            targets1[:, 2] = np.clip(targets1[:, 2], float(s["min_altitude"]), float(s["max_altitude"]))
        src = targets1 if targets1 is not None else P
        layer = self.live_show.start_positions.copy()
        layer[:, 2] = float(s["takeoff_altitude"]) + 2.0
        self._set_progress("return", 0.2, "Planning coordinated return")
        slot = await loop_exec(lambda: assign_rect(src, layer))
        targets2 = layer[slot]
        self._set_progress("", 1.0, "")

        def timing(a, b):
            dd = b - a
            dh = np.hypot(dd[:, 0], dd[:, 1])
            T = max(float(dh.max(initial=0)) / float(s["max_speed_h"]),
                    float(np.clip(dd[:, 2], 0, None).max(initial=0)) / float(s["max_speed_up"]),
                    float(np.clip(-dd[:, 2], 0, None).max(initial=0)) / float(s["max_speed_down"]), 2.0) * 1.2
            return T, np.linalg.norm(dd, axis=1) / T

        self.ret = {"idx": idx, "t1": targets1, "t2": targets2, "phase": 0, "since": time.time()}
        if len(loose):
            self.log("error", f"{len(loose)} drones without link keep flying their show during the return - "
                              f"watch them closely")
        if targets1 is not None:
            T1, v1 = timing(P, targets1)
            self.backend.goto(idx, targets1, v1)
            self.ret.update(phase=1, T=T1)
            self.log("warn", f"RETURN phase 1: spreading {len(idx)} drones to a safe spacing ({T1:.0f} s)")
        else:
            T2, v2 = timing(P, targets2)
            self.backend.goto(idx, targets2, v2, then_land=True)
            self.ret.update(phase=2, T=T2)
            self.log("warn", f"RETURN: {len(idx)} drones flying to the landing layer ({T2:.0f} s), then land")
        self.state = "RETURNING"

    def _return_tick(self, now):
        r = self.ret
        if r is None:
            return
        if r["phase"] == 1 and now - r["since"] > r["T"] + 2.0:
            st = self.backend.state
            idx = np.array([i for i in r["idx"] if st.in_air[i] and i not in self.taken_out], dtype=int)
            P = st.pos[idx]
            sel = np.isin(r["idx"], idx)
            tg = r["t2"][sel]
            dd = tg - P
            dh = np.hypot(dd[:, 0], dd[:, 1])
            s = self.s
            T = max(float(dh.max(initial=0)) / float(s["max_speed_h"]),
                    float(np.clip(dd[:, 2], 0, None).max(initial=0)) / float(s["max_speed_up"]),
                    float(np.clip(-dd[:, 2], 0, None).max(initial=0)) / float(s["max_speed_down"]), 2.0) * 1.2
            self.backend.goto(idx, tg, np.linalg.norm(dd, axis=1) / T, then_land=True)
            r.update(phase=2, T=T, since=now)
            self.log("warn", f"RETURN phase 2: flying to the landing layer ({T:.0f} s), then land")

    # ---- single-drone safe exit ---------------------------------------------------------------------
    def plan_exit(self, i: int, now: float):
        """Best-effort escape route for one drone, checked in space AND time: every point of
        a candidate path (sampled every 0.5 m) is compared with where every other drone will
        be at the exact moment the exiting drone gets there.  The landing spot is kept off
        the pad field so landing drones cannot come down on it.
        Returns waypoints (last one 3 m above the ground) or None."""
        st = self.backend.state
        s = self.s
        D = float(s["min_separation"])
        clear = D + 0.3                                 # margin for tracking / GPS error of the others
        v = 2.0
        p = st.pos[i].copy()
        others = np.array([j for j in self.flyers[st.in_air[self.flyers]]
                           if j != i and not st.flags[j] & F_CRASH], dtype=int)
        low_z = 3.0
        g = s["geofence"]
        pads = self.live_show.start_positions if self.live_show is not None else self.pads
        px0, py0 = pads[:, :2].min(0) - 3.0
        px1, py1 = pads[:, :2].max(0) + 3.0

        def off_pads(e):
            """Move a point (at its altitude) to the nearest place outside the pad field
            (never toward the audience)."""
            if not (px0 <= e[0] <= px1 and py0 <= e[1] <= py1):
                return None
            opts = [np.array([e[0], py1 + 2.0, e[2]]),
                    np.array([px0 - 2.0, e[1], e[2]]), np.array([px1 + 2.0, e[1], e[2]])]
            opts = [o for o in opts if g["x"][0] + 2 <= o[0] <= g["x"][1] - 2 and o[1] <= g["y"][1] - 2]
            return min(opts, key=lambda o: float(np.linalg.norm(o - e))) if opts else None

        running = self.state == "RUNNING" and self.live_show is not None and self.t0 is not None
        ts = now - self.t0 if running else 0.0
        slots = st.slot[others]
        static = np.nan_to_num(st.pos[others], nan=-1e6)
        cache: dict = {}

        def others_at(tau):
            if not running:
                return static
            k = round(tau * 20) / 20.0                       # 50 ms resolution
            if k not in cache:
                cache[k] = self.live_show.positions(ts + k)[slots]
            return cache[k]

        def free(wps) -> bool:
            if len(others) == 0:
                return True
            a, tau = p, 0.0
            for w in wps:
                L = float(np.linalg.norm(w - a))
                n = max(2, int(L / 0.5) + 1)
                for f in np.linspace(0, 1, n)[1:]:
                    q = a + f * (w - a)
                    O = others_at(tau + f * L / v)
                    if float(np.min(np.sum((O - q) ** 2, axis=1))) < clear * clear:
                        return False
                a, tau = w, tau + L / v
            return True

        def route(*pts):
            wps = [np.asarray(x, float) for x in pts]
            last = wps[-1]
            out = off_pads(last)
            if out is not None:
                wps.append(out)
                last = out
            wps.append(np.array([last[0], last[1], low_z]))
            return wps

        cloud = st.pos[others] if len(others) else p[None, :]
        ymax = float(np.nanmax(cloud[:, 1]))
        xmin, xmax = float(np.nanmin(cloud[:, 0])), float(np.nanmax(cloud[:, 0]))
        yb = lambda y: min(y, g["y"][1] - 3.0)   # noqa: E731
        cands = []
        near = cloud[np.linalg.norm(cloud - p, axis=1) < 40.0]
        c = near.mean(0) if len(near) else p
        rdir = p - c
        rdir[1] = max(rdir[1], 0.0)                               # never toward the audience
        if np.linalg.norm(rdir) > 1e-3:                           # out of a 3D shell
            rdir /= np.linalg.norm(rdir)
            reach = float(np.linalg.norm(near - c, axis=1).max()) if len(near) else 10.0
            e = p + rdir * (reach - float(np.linalg.norm(p - c)) + 8.0)
            e[0] = np.clip(e[0], g["x"][0] + 3.0, g["x"][1] - 3.0)
            e[1] = np.clip(e[1], p[1], g["y"][1] - 3.0)
            e[2] = np.clip(e[2], low_z + 2.0, float(s["max_altitude"]))
            cands.append(route(e))
        for e in (np.array([p[0], yb(p[1] + 6.0), p[2]]), np.array([p[0], yb(p[1] + 12.0), p[2]]),
                  np.array([p[0], yb(ymax + 8.0), p[2]]),
                  np.array([max(xmin - 8.0, g["x"][0] + 3.0), p[1], p[2]]),
                  np.array([min(xmax + 8.0, g["x"][1] - 3.0), p[1], p[2]])):
            cands.append(route(e))
        up = np.array([p[0], p[1], min(p[2] + 8.0, float(s["max_altitude"]))])
        cands.append(route(up, np.array([p[0], yb(ymax + 8.0), up[2]])))
        cands.insert(0, route(np.array([p[0], p[1], max(low_z, p[2])])))   # nothing in the way: off the pads, down
        for wps in cands:
            if free(wps):
                return wps
        return None

    def safe_exit(self, i: int, reason: str):
        if i in self.taken_out:
            return
        self.taken_out.add(i)
        st = self.backend.state
        st.flags[i] |= F_EXIT
        wps = self.plan_exit(i, time.time())
        if wps is None:
            # holding in place would make it an obstacle: let it keep flying its show path
            # (it is still under control) and look for a gap again every second
            self.stuck[i] = time.time()
            self.log("error", f"must leave the show ({reason}) but no clear escape route yet - it keeps "
                              f"flying its show path; re-checking every second", i)
            return
        self.backend.exit_path(i, wps, speed=2.0)
        how = "descending" if len(wps) == 1 else "leaving the formation, then descending"
        self.log("warn", f"taken out of show ({reason}); {how}", i)

    # ---- periodic update ---------------------------------------------------------------------------------
    def tick(self, now: float, dt: float):
        b = self.backend
        if b is None:
            return
        b.tick(now, dt)
        st = b.state
        m = st.m
        s = self.s
        link = (now - st.last_seen[:m]) < 3.0
        air = st.in_air[:m]
        self.flown[:m] |= air

        if self.state == "COUNTDOWN" and now >= self.t0:
            self.state = "RUNNING"
            self.log("info", "SHOW STARTED")

        if self.state in ("RUNNING", "HOLDING", "RETURNING", "LANDING") and len(self.flyers):
            self._monitor(now, s, link)
            if self.state == "RETURNING":
                self._return_tick(now)
            fl = self.flyers
            t_show = now - self.t0 if self.t0 else 0
            if len(self.wait_idx) and self.state == "RUNNING":
                gone = self.flown[self.wait_idx] | np.isin(self.wait_idx, list(self.taken_out))
                late = ~gone & (t_show > self.wait_t + 20.0)
                for i in self.wait_idx[late]:
                    self.log("error", "did not take off at its take-off time - stays on its pad", int(i))
                    self.taken_out.add(int(i))
                if late.any():
                    self.backend.cancel_start(self.wait_idx[late])
                keep = ~(gone | late)
                self.wait_idx, self.wait_t = self.wait_idx[keep], self.wait_t[keep]
            done_time = (self.state != "RUNNING" or t_show > self.live_show.duration + 3.0) and not len(self.wait_idx)
            air_fl = fl[st.in_air[fl]]
            linked = (now - st.last_seen[air_fl]) < 3.0
            still = air_fl[linked]                     # drones we can still see in the air
            if done_time and not len(still) and (now - (self.t0 or now)) > 5:
                self.state = "FINISHED"
                left = int((~linked).sum())
                if left:
                    self.log("error", f"show finished - {left} drones without telemetry may still be airborne; "
                                      f"find them visually before anyone enters the field")
                else:
                    self.log("info", "all drones on the ground - show finished")
                if self.blackbox:
                    self.blackbox.write_json("events_summary.json", list(self.events)[-200:])

        # status codes
        status = np.full(m, OFFLINE, np.uint8)
        status[link] = CONNECTED
        role = st.role[:m]
        status[link & (role == ROLE_SPARE)] = SPARE
        flags = st.flags[:m]
        if self.state in ("PLANNED", "UPLOADING", "UPLOADED"):
            status[link & (role == ROLE_FLYER) & ((flags & ~np.uint32(0x80)) == 0)] = READY
        if self.state == "UPLOADED":
            status[link & (role == ROLE_FLYER) & (flags == 0)] = READY
        status[st.armed[:m] & ~air] = ARMED
        status[air] = FLYING
        status[air & ((flags & (F_DEVIATION | F_GEOFENCE | F_CRIT_BATT | F_LINK)) != 0)] = WARNING
        out = np.zeros(m, bool)
        if self.taken_out:
            out[list(self.taken_out)] = True
        status[out & air] = FAILSAFE
        status[self.flown[:m] & ~air & link] = LANDED
        status[~link & air] = WARNING
        crashed = np.array([md == "CRASHED" for md in st.mode[:m]], bool) if b.kind == "sim" else np.zeros(m, bool)
        status[crashed] = ERROR
        st.status[:m] = status

        if self.blackbox and now - self._last_bb >= 1.0 and self.state in ACTIVE_STATES:
            self._last_bb = now
            self.blackbox.telemetry(now, st.pos[:m], st.batt[:m], st.status[:m], st.flags[:m])

    def _monitor(self, now, s, link):
        st = self.backend.state
        fl = self.flyers
        air = st.in_air[fl]
        inair = fl[air]
        if len(inair) == 0:
            return
        P = st.pos[inair]
        lk = link[inair]
        # drones waiting for an escape route: re-plan every 2 s
        for i, last in list(self.stuck.items()):
            if not st.in_air[i]:
                self.stuck.pop(i, None)
            elif now - last >= 1.0:
                wps = self.plan_exit(i, now)
                if wps is None:
                    self.stuck[i] = now
                else:
                    self.stuck.pop(i, None)
                    self.backend.exit_path(i, wps, speed=2.0)
                    self.log("warn", "escape route found - leaving the formation", i)
        # falling drone (motor / power failure): nothing to command, just record it
        falling = lk & (st.vel[inair, 2] < -6.0)
        for i in inair[falling]:
            if not st.flags[i] & F_CRASH:
                st.flags[i] |= F_CRASH
                self.taken_out.add(int(i))
                self.log("error", "drone falling (motor or power failure)", int(i))
        # link loss
        for i in inair[~lk]:
            if not st.flags[i] & F_LINK:
                st.flags[i] |= F_LINK
                self.log("warn", "telemetry lost - drone continues autonomously", int(i))
        st.flags[inair[lk]] &= ~np.uint32(F_LINK)
        # deviation from plan (running only)
        if self.state == "RUNNING":
            t_show = now - self.t0
            dev = self.live_show.path_deviation(t_show, st.slot[inair], P)
            warn = lk & (dev > float(s["deviation_warn"]))
            st.flags[inair[warn]] |= F_DEVIATION
            st.flags[inair[~warn]] &= ~np.uint32(F_DEVIATION)
            bad = lk & (dev > float(s["deviation_abort"]))
            since = self.dev_since
            since[inair[~bad]] = np.inf
            newly = inair[bad & np.isinf(since[inair])]
            since[newly] = now
            for k in np.nonzero(bad)[0]:
                i = int(inair[k])
                if now - since[i] >= 1.0 and i not in self.taken_out:
                    self.safe_exit(i, f"{dev[k]:.1f} m off track")
        # RTK / GPS degradation: 2.2 m formations need centimetre positioning
        if self.state == "RUNNING":
            need_fix = 5 if s.get("require_rtk", True) else 3
            lost = lk & (st.fix[inair] < need_fix)
            since = self.rtk_since
            since[inair[~lost]] = np.inf
            since[inair[lost & np.isinf(since[inair])]] = now
            for i in inair[lost]:
                if now - since[i] >= 2.0 and int(i) not in self.taken_out:
                    self.safe_exit(int(i), f"GPS fix degraded (fix type {int(st.fix[i])})")
        # battery
        crit = lk & (st.batt[inair] < float(s["critical_battery"]))
        for i in inair[crit]:
            if not st.flags[i] & F_CRIT_BATT:
                st.flags[i] |= F_CRIT_BATT
                if self.state == "RUNNING":
                    self.safe_exit(int(i), f"battery critical ({st.batt[i]:.0f}%)")
        # geofence & audience
        g = s["geofence"]
        m_ = 1.0
        outside = lk & ((P[:, 0] < g["x"][0] - m_) | (P[:, 0] > g["x"][1] + m_) | (P[:, 1] < g["y"][0] - m_) |
                        (P[:, 1] > g["y"][1] + m_) | (P[:, 2] > g["z"][1] + m_))
        aud = lk & (P[:, 1] < float(s["audience_y"]) + 0.5 * float(s["audience_buffer"]))
        for k in np.nonzero(outside | aud)[0]:
            i = int(inair[k])
            if aud[k] and not st.flags[i] & F_AUDIENCE:
                st.flags[i] |= F_AUDIENCE
                if self.audience_action == "kill":
                    self.backend.kill([i])
                    self.taken_out.add(i)
                    self.log("error", "AUDIENCE ZONE breach - motors stopped", i)
                else:
                    self.backend.land([i])
                    self.taken_out.add(i)
                    self.log("error", "AUDIENCE ZONE breach - landing immediately", i)
            elif outside[k] and not st.flags[i] & F_GEOFENCE:
                st.flags[i] |= F_GEOFENCE
                self.backend.land([i])
                self.taken_out.add(i)
                self.log("error", "geofence breach - landing", i)

    # ---- UI payloads ---------------------------------------------------------------------------------------
    def status(self) -> dict:
        b = self.backend
        now = time.time()
        out = {"state": self.state, "fleet": None, "progress": self.progress, "checklist": self.checklist,
               "checklist_items": CHECKLIST, "wind": self.wind, "gonogo": self.gonogo,
               "audience_action": self.audience_action}
        if b is None:
            return out
        st = b.state
        m = st.m
        counts = np.bincount(st.status[:m], minlength=len(STATUS_NAMES))
        out["fleet"] = {
            "kind": b.kind, "known": int(m), "flyers": int(len(self.flyers)),
            "counts": {STATUS_NAMES[k]: int(counts[k]) for k in range(len(STATUS_NAMES))},
            "taken_out": len(self.taken_out),
            "min_batt": float(st.batt[self.flyers].min()) if len(self.flyers) else None,
        }
        if self.live_show is not None:
            out["show"] = {"duration": self.live_show.duration, "drones": self.live_show.n,
                           "validation_ok": bool(self.live_validation and self.live_validation["ok"])}
        if self.t0 is not None:
            out["t_show"] = now - self.t0 if self.state != "HOLDING" else self.hold_started - self.t0
            out["countdown"] = max(0.0, self.t0 - now) if self.state == "COUNTDOWN" else 0
        if self.ret:
            out["return_phase"] = self.ret["phase"]
        return out

    def drone_detail(self, i: int) -> dict:
        st = self.backend.state
        if not (0 <= i < st.m):
            raise OpError("unknown drone")
        p = st.pos[i]
        return {
            "idx": i, "id": st.ids[i], "autopilot": st.autopilot[i], "mode": st.mode[i],
            "status": STATUS_NAMES[int(st.status[i])], "flags": flag_text(int(st.flags[i])),
            "pos": [None if np.isnan(v) else round(float(v), 2) for v in p],
            "battery": round(float(st.batt[i]), 1), "voltage": round(float(st.volt[i]), 2),
            "gps_fix": int(st.fix[i]), "sats": int(st.sats[i]), "armed": bool(st.armed[i]),
            "in_air": bool(st.in_air[i]), "pad": int(st.pad[i]), "slot": int(st.slot[i]),
            "role": ["unknown", "flyer", "spare", "excluded"][int(st.role[i])],
            "uploaded": bool(st.uploaded[i]), "link_age": round(float(time.time() - st.last_seen[i]), 1),
            "pad_offset": None if self.offsets is None or i >= len(self.offsets) or np.isinf(self.offsets[i])
            else round(float(self.offsets[i]), 2),
        }

    def frame(self):
        """positions (m,3) float32, colours (m,3) uint8, status (m,) uint8 for the 3D view."""
        st = self.backend.state
        m = st.m
        pos = np.nan_to_num(st.pos[:m], nan=0.0).astype(np.float32)
        return pos, st.color[:m].copy(), st.status[:m].copy()


def default_log_root() -> str:
    base = os.environ.get("LANGIT_HOME") or os.path.join(os.path.expanduser("~"), "LangitSwarm")
    return os.path.join(base, "logs")
