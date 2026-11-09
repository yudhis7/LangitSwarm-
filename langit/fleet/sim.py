"""Simulated fleet: thousands of virtual drones with realistic faults.

Each virtual drone behaves like a show drone running its trajectory onboard
against a shared clock.  Fault injection lets operators rehearse procedures:
  * pre-flight faults: low battery, no RTK fix, sensor fault, drone off its pad,
    drone that never connects
  * in-flight faults: battery sag, GPS degradation (drift), motor failure, link loss
"""
from __future__ import annotations

import asyncio
import math

import numpy as np

from .state import FleetBackend

M_IDLE, M_ARMED, M_SHOW, M_HOLD, M_GOTO, M_LAND, M_LANDED, M_CRASH, M_EXIT = range(9)
MODE_NAMES = ["STANDBY", "ARMED", "SHOW", "HOLD", "GUIDED", "LAND", "LANDED", "CRASHED", "EXIT"]


class SimFleet(FleetBackend):
    kind = "sim"

    def __init__(self, pads: np.ndarray, settings: dict, preflight_fault_rate: float = 0.01,
                 inflight_fault_per_min: float = 0.0004, seed: int | None = 7, gps_noise: float = 0.08):
        m = len(pads)
        super().__init__(m)
        self.s = settings
        self.rng = np.random.default_rng(seed)
        st = self.state
        for i in range(m):
            st.add(f"SIM-{i + 1:04d}", "sim")
        st.mode = ["STANDBY"] * m
        self.pads = pads.copy()
        rng = self.rng
        jitter = rng.normal(0, 0.06, (m, 3))
        jitter[:, 2] = 0
        self.true_pos = pads + jitter
        self.vel = np.zeros((m, 3))
        self.mode = np.full(m, M_IDLE, np.int8)
        st.batt[:m] = rng.uniform(97.0, 100.0, m)
        st.volt[:m] = 16.6
        st.fix[:m] = 6
        st.sats[:m] = rng.integers(18, 30, m)
        self.drain = rng.uniform(0.92, 1.12, m)
        self.online = np.ones(m, bool)
        self.gps_bad = np.zeros(m, bool)
        self.drift = np.zeros((m, 3))
        self.gps_noise = gps_noise
        self.inflight_rate = inflight_fault_per_min
        # pre-flight faults
        k = int(round(m * preflight_fault_rate))
        bad = rng.choice(m, size=k, replace=False) if k else np.array([], int)
        for j, i in enumerate(bad):
            kind = j % 5
            if kind == 0:
                st.batt[i] = rng.uniform(55, 88)
            elif kind == 1:
                st.fix[i] = int(rng.choice([3, 4, 5]))
            elif kind == 2:
                st.sensors_ok[i] = False
            elif kind == 3:
                self.true_pos[i, :2] += rng.normal(0, 1.5, 2)
            else:
                self.online[i] = False
        self.preflight_faults = bad
        # show execution
        self.show = None
        self.slots = np.full(m, -1, np.int64)
        self.t0 = np.full(m, math.inf)
        self.target = self.true_pos.copy()
        self.speed = np.full(m, 3.0)
        self.then_land = np.zeros(m, bool)
        self.wp: dict[int, list] = {}
        self.fail_after = np.full(m, math.inf)      # failure time (show clock) per drone
        self.fail_kind = np.zeros(m, np.int8)
        self._now = 0.0

    # ------------------------------------------------------------------------------------------
    async def upload_show(self, show, idx, slots, start_utc=None, progress=None):
        st = self.state
        idx = np.asarray(idx, int)
        self.show = show
        self.slots[:] = -1
        st.uploaded[: st.m] = False
        order = np.argsort(idx)
        idx, slots = idx[order], np.asarray(slots, int)[order]
        for b in range(0, len(idx), 250):
            sel = idx[b:b + 250]
            ok = sel[self.online[sel]]
            self.slots[ok] = slots[b:b + 250][self.online[sel]]
            st.uploaded[ok] = True
            st.upload_progress[sel] = 1.0
            if progress:
                progress(min(1.0, (b + 250) / max(1, len(idx))), f"Uploaded {min(len(idx), b + 250)}/{len(idx)}")
            await asyncio.sleep(0.03)
        # schedule random in-flight faults for this flight
        n = len(idx)
        p = 1.0 - math.exp(-self.inflight_rate * show.duration / 60.0)
        hit = idx[self.rng.random(n) < p]
        self.fail_after[:] = math.inf
        if len(hit):
            self.fail_after[hit] = self.rng.uniform(0.15, 0.9, len(hit)) * show.duration
            self.fail_kind[hit] = self.rng.integers(0, 4, len(hit))
        return int(st.uploaded[idx].sum())

    def arm(self, idx):
        idx = self._alive(idx)
        idx = idx[~self.state.in_air[idx]]
        self.state.armed[idx] = True
        self.mode[idx] = M_ARMED

    def disarm(self, idx):
        idx = self._alive(idx)
        idx = idx[~self.state.in_air[idx]]
        self.state.armed[idx] = False
        self.mode[idx] = M_IDLE

    def kill(self, idx):
        idx = self._alive(idx)
        air = idx[self.state.in_air[idx]]
        self.mode[air] = M_CRASH
        gnd = idx[~self.state.in_air[idx]]
        self.mode[gnd] = M_IDLE
        self.state.armed[idx] = False

    def start_show(self, idx, t0, offsets=None):
        """Arm now and wait on the pad until t0 (like an onboard show player).  Staggered
        take-offs of imported shows need no offsets here: their trajectories keep each
        drone on its pad until its take-off time."""
        idx = self._alive(idx)
        idx = idx[(self.slots[idx] >= 0) & self.state.uploaded[idx] & ~self.state.in_air[idx]]
        self.state.armed[idx] = True
        self.t0[idx] = t0
        self.mode[idx] = M_SHOW

    def cancel_start(self, idx):
        idx = self._alive(idx)
        idx = idx[~self.state.in_air[idx]]
        self.mode[idx] = M_IDLE
        self.t0[idx] = math.inf
        self.state.armed[idx] = False

    def hold(self, idx):
        idx = self._alive(idx)
        idx = idx[np.isin(self.mode[idx], (M_SHOW, M_GOTO, M_EXIT))]
        self.target[idx] = self.true_pos[idx]
        self.mode[idx] = M_HOLD

    def resume(self, idx, t_shift):
        idx = self._alive(idx)
        idx = idx[(self.mode[idx] == M_HOLD) & (self.slots[idx] >= 0)]
        self.t0[idx] += t_shift
        self.mode[idx] = M_SHOW

    def land(self, idx):
        idx = self._alive(idx)
        gnd = idx[~self.state.in_air[idx] & np.isin(self.mode[idx], (M_SHOW, M_HOLD))]
        self.mode[gnd] = M_IDLE                    # waiting for a staggered take-off: stay on the pad
        self.state.armed[gnd] = False
        idx = idx[self.state.in_air[idx] & (self.mode[idx] != M_CRASH)]
        self.mode[idx] = M_LAND

    def goto(self, idx, targets, speeds, then_land=False):
        idx = np.asarray(idx, int)
        targets = np.asarray(targets, float)
        speeds = np.broadcast_to(np.asarray(speeds, float), (len(idx),))
        ok = self.online[idx] & self.state.in_air[idx] & np.isin(self.mode[idx], (M_SHOW, M_HOLD, M_GOTO, M_EXIT))
        sel = idx[ok]
        self.target[sel] = targets[ok]
        self.speed[sel] = np.maximum(0.2, speeds[ok])
        self.then_land[sel] = then_land
        self.mode[sel] = M_GOTO

    def exit_path(self, i, waypoints, speed):
        if not self.online[i] or self.mode[i] in (M_CRASH, M_LANDED, M_IDLE) or not self.state.in_air[i]:
            return
        self.wp[i] = [np.asarray(w, float) for w in waypoints]
        self.speed[i] = speed
        self.target[i] = self.wp[i].pop(0)
        self.mode[i] = M_EXIT

    def _alive(self, idx):
        """Only drones with a working link receive commands."""
        idx = np.asarray(idx, dtype=int)
        return idx[self.online[idx]]

    # ------------------------------------------------------------------------------------------
    def _move_toward(self, sel, dt):
        if len(sel) == 0:
            return np.zeros(0, bool)
        d = self.target[sel] - self.true_pos[sel]
        dist = np.linalg.norm(d, axis=1)
        step = np.minimum(dist, self.speed[sel] * dt)
        with np.errstate(invalid="ignore", divide="ignore"):
            u = np.where(dist[:, None] > 1e-9, d / dist[:, None], 0.0)
        self.true_pos[sel] += u * step[:, None]
        return dist - step < 0.3

    def tick(self, now, dt):
        self._now = now
        st = self.state
        m = st.m
        rng = self.rng
        mode = self.mode
        prev = self.true_pos.copy()
        colors = np.zeros((m, 3), np.uint8)

        # --- show execution ---------------------------------------------------------------------
        show_sel = np.nonzero(mode == M_SHOW)[0]
        if len(show_sel) and self.show is not None:
            # drones share t0 except after individual hold/resume: group by t0
            for t0 in np.unique(self.t0[show_sel]):
                grp = show_sel[self.t0[show_sel] == t0]
                t = now - t0
                if t < 0:
                    continue
                if t >= self.show.duration:
                    P = self.show.final_positions[self.slots[grp]]
                    self.true_pos[grp] = P
                    mode[grp] = M_LANDED
                    continue
                P = self.show.positions_for(t, self.slots[grp])
                gust = rng.normal(0, 0.05, (len(grp), 3))
                gust[P[:, 2] <= 0.05] = 0.0            # still on the pad (staggered take-off)
                self.true_pos[grp] += (P - self.true_pos[grp]) * min(1.0, 9.0 * dt) + gust
                self.true_pos[grp, 2] = np.maximum(self.true_pos[grp, 2], 0.0)
                pad = (P[:, 2] <= 0.05) & (self.true_pos[grp, 2] <= 0.05)
                self.true_pos[grp[pad]] = P[pad]
                colors[grp] = self.show.colors_u8(t)[self.slots[grp]]
                # injected in-flight faults
                due = grp[(t >= self.fail_after[grp])]
                for i in due:
                    self._fault(int(i))
                    self.fail_after[i] = math.inf
        # --- hold ------------------------------------------------------------------------------------
        sel = np.nonzero(mode == M_HOLD)[0]
        if len(sel):
            jit = rng.normal(0, 0.03, (len(sel), 3))
            jit[self.target[sel, 2] <= 0.05] = 0.0      # held before take-off: stays on the pad
            self.true_pos[sel] += (self.target[sel] - self.true_pos[sel]) * min(1.0, 5.0 * dt) + jit
            self.true_pos[sel, 2] = np.maximum(self.true_pos[sel, 2], 0.0)
            colors[sel] = (90, 90, 90)
        # --- goto / exit ------------------------------------------------------------------------------
        sel = np.nonzero(mode == M_GOTO)[0]
        if len(sel):
            done = self._move_toward(sel, dt)
            land_now = sel[done & self.then_land[sel]]
            mode[land_now] = M_LAND
            colors[sel] = (60, 60, 160)
        sel = np.nonzero(mode == M_EXIT)[0]
        for i in sel:
            if self._move_toward(np.array([i]), dt)[0]:
                if self.wp.get(i):
                    self.target[i] = self.wp[i].pop(0)
                else:
                    mode[i] = M_LAND
        colors[sel] = (255, 140, 0)
        # --- land ----------------------------------------------------------------------------------------
        sel = np.nonzero(mode == M_LAND)[0]
        if len(sel):
            self.true_pos[sel, 2] -= 1.5 * dt
            down = sel[self.true_pos[sel, 2] <= 0.0]
            self.true_pos[down, 2] = 0.0
            mode[down] = M_LANDED
            colors[sel] = (255, 140, 0)
        # --- crash ---------------------------------------------------------------------------------------
        sel = np.nonzero(mode == M_CRASH)[0]
        if len(sel):
            self.vel[sel, 2] -= 9.81 * dt
            self.true_pos[sel] += self.vel[sel] * dt
            hit = sel[self.true_pos[sel, 2] <= 0]
            self.true_pos[hit, 2] = 0.0
            self.vel[hit] = 0.0
        # --- landed: disarm --------------------------------------------------------------------------
        landed = mode == M_LANDED
        st.armed[:m][landed] = False

        # --- telemetry ---------------------------------------------------------------------------------
        v = (self.true_pos - prev) / max(dt, 1e-3)
        flying_modes = np.isin(mode, (M_SHOW, M_HOLD, M_GOTO, M_LAND, M_EXIT, M_CRASH))
        air = flying_modes & (self.true_pos[:, 2] > 0.05)
        air |= (mode == M_SHOW) & (self.true_pos[:, 2] > 0.01)
        st.in_air[:m] = air
        drain = 100.0 / (float(self.s.get("battery_minutes", 18.0)) * 60.0)
        st.batt[:m] -= (air * drain * self.drain * dt).astype(np.float32)
        st.batt[:m] = np.maximum(st.batt[:m], 0)
        st.volt[:m] = 14.0 + 2.6 * st.batt[:m] / 100.0
        nb = int(self.gps_bad.sum())
        if nb:   # degraded GPS: the drone physically wanders by a bounded, slowly changing error
            old = self.drift[self.gps_bad].copy()
            new = old + np.c_[rng.normal(0, 0.5, (nb, 2)) * dt, np.zeros(nb)]
            norm = np.linalg.norm(new, axis=1, keepdims=True)
            new *= np.minimum(1.0, 4.0 / np.maximum(norm, 1e-9))
            self.drift[self.gps_bad] = new
            self.true_pos[self.gps_bad] += new - old
        noise = rng.normal(0, self.gps_noise, (m, 3))
        noise[~air] *= 0.3
        rep = self.true_pos + noise
        on = self.online
        st.pos[:m][on] = rep[on]
        st.vel[:m][on] = v[on]
        st.last_seen[:m][on] = now
        st.color[:m] = colors
        st.armed[:m] |= air
        st.armed[:m][mode == M_CRASH] = False
        for i in np.nonzero(on)[0]:
            st.mode[i] = MODE_NAMES[mode[i]]

    def _fault(self, i: int):
        k = int(self.fail_kind[i])
        st = self.state
        if k == 0:
            st.batt[i] = min(st.batt[i], 17.0)
            self.drain[i] *= 6.0
            self.on_event("warn", "battery sag", i)
        elif k == 1:
            self.gps_bad[i] = True
            st.fix[i] = 3
            self.drift[i] = 0.0
            self.on_event("warn", "GPS degraded (RTK lost)", i)
        elif k == 2:
            self.mode[i] = M_CRASH
            self.vel[i] = self.vel[i] * 0.5
            self.on_event("error", "motor failure", i)
        else:
            self.online[i] = False
            self.on_event("warn", "telemetry link lost (drone continues its show)", i)
