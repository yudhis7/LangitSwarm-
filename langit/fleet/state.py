"""Fleet telemetry state (struct-of-arrays so 5,000 drones stay cheap) and the
backend interface shared by the simulator and the MAVLink link."""
from __future__ import annotations

import numpy as np

# drone status codes (UI colours are keyed on these)
OFFLINE, CONNECTED, READY, ARMED, FLYING, WARNING, FAILSAFE, LANDED, SPARE, ERROR = range(10)
STATUS_NAMES = ["offline", "connected", "ready", "armed", "flying", "warning", "failsafe",
                "landed", "spare", "error"]

# flag bits
F_LINK = 1 << 0
F_LOW_BATT = 1 << 1
F_NO_RTK = 1 << 2
F_OFF_PAD = 1 << 3
F_SENSORS = 1 << 4
F_DEVIATION = 1 << 5
F_GEOFENCE = 1 << 6
F_NO_MISSION = 1 << 7
F_CRIT_BATT = 1 << 8
F_CRASH = 1 << 9
F_EXIT = 1 << 10
F_AUDIENCE = 1 << 11
FLAG_NAMES = {F_LINK: "link lost", F_LOW_BATT: "battery low", F_NO_RTK: "no RTK fix", F_OFF_PAD: "off pad",
              F_SENSORS: "sensor fault", F_DEVIATION: "off track", F_GEOFENCE: "geofence",
              F_NO_MISSION: "no mission", F_CRIT_BATT: "battery critical", F_CRASH: "crashed",
              F_EXIT: "taken out of show", F_AUDIENCE: "audience zone"}

ROLE_UNKNOWN, ROLE_FLYER, ROLE_SPARE, ROLE_EXCLUDED = range(4)


def flag_text(flags: int) -> str:
    return ", ".join(name for bit, name in FLAG_NAMES.items() if flags & bit)


class FleetState:
    def __init__(self, capacity: int):
        self.capacity = capacity
        self.m = 0
        c = capacity
        self.ids: list[str] = []
        self.autopilot: list[str] = []
        self.mode: list[str] = []
        self.pos = np.full((c, 3), np.nan)
        self.vel = np.zeros((c, 3))
        self.batt = np.zeros(c, np.float32)
        self.volt = np.zeros(c, np.float32)
        self.fix = np.zeros(c, np.uint8)
        self.sats = np.zeros(c, np.uint8)
        self.armed = np.zeros(c, bool)
        self.in_air = np.zeros(c, bool)
        self.sensors_ok = np.ones(c, bool)
        self.last_seen = np.full(c, -1e9)
        self.uploaded = np.zeros(c, bool)
        self.upload_progress = np.zeros(c, np.float32)
        self.pad = np.full(c, -1, np.int32)        # pad index on the field
        self.slot = np.full(c, -1, np.int32)       # drone index in the live show
        self.role = np.zeros(c, np.uint8)
        self.status = np.zeros(c, np.uint8)
        self.flags = np.zeros(c, np.uint32)
        self.color = np.zeros((c, 3), np.uint8)

    def add(self, ident: str, autopilot: str = "") -> int:
        if self.m >= self.capacity:
            raise RuntimeError("fleet capacity reached")
        i = self.m
        self.m += 1
        self.ids.append(ident)
        self.autopilot.append(autopilot)
        self.mode.append("")
        return i

    def view(self, name: str):
        return getattr(self, name)[: self.m]


class FleetBackend:
    """Common interface. All index arguments are arrays of fleet indices."""
    kind = "base"

    def __init__(self, capacity: int):
        self.state = FleetState(capacity)
        self.on_event = lambda level, msg, drone=None: None

    async def start(self):
        pass

    async def stop(self):
        pass

    def tick(self, now: float, dt: float):
        pass

    async def upload_show(self, show, idx, slots, start_utc=None, progress=None):
        raise NotImplementedError

    def arm(self, idx):
        raise NotImplementedError

    def disarm(self, idx):
        raise NotImplementedError

    def kill(self, idx):
        raise NotImplementedError

    def start_show(self, idx, t0: float, offsets=None):
        """Start the uploaded show at wall-clock t0; offsets (s, per drone) delay each
        drone's take-off (imported shows with staggered take-offs)."""
        raise NotImplementedError

    def cancel_start(self, idx):
        raise NotImplementedError

    def hold(self, idx):
        raise NotImplementedError

    def resume(self, idx, t_shift: float):
        raise NotImplementedError

    def land(self, idx):
        raise NotImplementedError

    def goto(self, idx, targets, speeds, then_land: bool = False):
        raise NotImplementedError

    def exit_path(self, idx: int, waypoints, speed: float):
        raise NotImplementedError
