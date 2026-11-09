"""Flight recorder: events as JSON lines, telemetry as a compact binary stream.

telemetry.bin record (little endian):
    float64 unix time, uint32 drone count m, float32[m*3] positions (show frame),
    uint8[m] battery %, uint8[m] status code, uint32[m] flag bits
Read it back with ``read_telemetry(path)``.
"""
from __future__ import annotations

import datetime as dt
import json
import os
import struct

import numpy as np


class BlackBox:
    def __init__(self, root: str, name: str):
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in name)[:40]
        self.dir = os.path.join(root, f"{stamp}_{safe}")
        os.makedirs(self.dir, exist_ok=True)
        self._ev = open(os.path.join(self.dir, "events.jsonl"), "a", encoding="utf-8")
        self._tm = open(os.path.join(self.dir, "telemetry.bin"), "ab")

    def event(self, rec: dict):
        self._ev.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._ev.flush()

    def telemetry(self, t: float, pos: np.ndarray, batt: np.ndarray, status: np.ndarray, flags: np.ndarray):
        m = len(status)
        self._tm.write(struct.pack("<dI", t, m))
        self._tm.write(np.nan_to_num(pos, nan=0.0).astype("<f4").tobytes())
        self._tm.write(np.clip(batt, 0, 255).astype(np.uint8).tobytes())
        self._tm.write(status.astype(np.uint8).tobytes())
        self._tm.write(flags.astype("<u4").tobytes())

    def write_json(self, name: str, obj):
        with open(os.path.join(self.dir, name), "w", encoding="utf-8") as f:
            json.dump(obj, f, indent=1, default=str)

    def close(self):
        for f in (self._ev, self._tm):
            try:
                f.close()
            except OSError:
                pass


def read_telemetry(path: str):
    out = []
    with open(path, "rb") as f:
        while True:
            hdr = f.read(12)
            if len(hdr) < 12:
                break
            t, m = struct.unpack("<dI", hdr)
            pos = np.frombuffer(f.read(m * 12), "<f4").reshape(m, 3)
            batt = np.frombuffer(f.read(m), np.uint8)
            status = np.frombuffer(f.read(m), np.uint8)
            flags = np.frombuffer(f.read(m * 4), "<u4")
            out.append((t, pos, batt, status, flags))
    return out
