"""Swarm goal assignment.

We use the CAPT result (Turpin, Michael & Kumar, "Concurrent assignment and
planning of trajectories for multiple robots", IJRR 2014):

    If drones move on straight lines, all starting and arriving at the same time
    with the same time profile, and the goal assignment minimises the SUM OF
    SQUARED distances, then no two drones come closer than D / sqrt(2), where D
    is the smallest spacing among the start positions and among the goal positions.

So with formation spacing >= sqrt(2) * min_separation the transition is
collision free by construction.  The validator still re-checks every pair.

Solvers, fastest first for these geometric problems:
  * POT network simplex (exact optimal transport)  ~1 s @ 2,000, ~12 s @ 5,000
  * lap (Jonker-Volgenant)
  * SciPy linear_sum_assignment
Transitions are independent of each other (the assignment between two point
SETS does not depend on which drone sits where), so the compiler solves them
in parallel and composes the permutations.

A z-scale (downwash factor) k can be applied: the guarantee then holds for the
ellipsoidal metric sqrt(dx^2 + dy^2 + (dz/k)^2).
"""
from __future__ import annotations

import hashlib
import warnings
from collections import OrderedDict

import numpy as np
from scipy.spatial.distance import cdist

try:
    import ot as _ot
except Exception:            # pragma: no cover - optional dependency
    _ot = None
try:
    import lap as _lap
except Exception:            # pragma: no cover
    _lap = None

_CACHE: "OrderedDict[bytes, np.ndarray]" = OrderedDict()
_CACHE_MAX = 128


def set_key(a: np.ndarray, b: np.ndarray, zscale: float) -> bytes:
    h = hashlib.blake2b(digest_size=20)
    h.update(np.ascontiguousarray(a, dtype=np.float64).tobytes())
    h.update(np.ascontiguousarray(b, dtype=np.float64).tobytes())
    h.update(np.float64(zscale).tobytes())
    return h.digest()


def cache_get(key: bytes):
    if key in _CACHE:
        _CACHE.move_to_end(key)
        return _CACHE[key].copy()
    return None


def cache_put(key: bytes, perm: np.ndarray):
    _CACHE[key] = np.asarray(perm).copy()
    while len(_CACHE) > _CACHE_MAX:
        _CACHE.popitem(last=False)


def solve(start: np.ndarray, goal: np.ndarray, zscale: float = 1.0) -> np.ndarray:
    """Exact min sum-of-squared-distance assignment. perm[i] = goal index for start i."""
    n = len(start)
    if len(goal) != n:
        raise ValueError("start and goal must have the same number of points")
    A = np.array(start, dtype=np.float64)
    B = np.array(goal, dtype=np.float64)
    if zscale != 1.0:
        A[:, 2] /= zscale
        B[:, 2] /= zscale
    # Optimal assignment is invariant to translating either set; centring improves conditioning.
    A -= A.mean(0)
    B -= B.mean(0)
    C = cdist(A, B, "sqeuclidean")
    if _ot is not None:
        w = np.ones(n)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            G = _ot.emd(w, w, C, numItermax=10 ** 9)
        perm = np.asarray(G.argmax(axis=1), dtype=np.int64)
        del G
        if len(np.unique(perm)) == n:
            return perm
    if _lap is not None:
        _, x, _ = _lap.lapjv(C, extend_cost=False)
        return np.asarray(x, dtype=np.int64)
    from scipy.optimize import linear_sum_assignment   # pragma: no cover
    _, perm = linear_sum_assignment(C)
    return perm.astype(np.int64)


def assign(start: np.ndarray, goal: np.ndarray, zscale: float = 1.0) -> np.ndarray:
    key = set_key(start, goal, zscale)
    hit = cache_get(key)
    if hit is not None:
        return hit
    perm = solve(start, goal, zscale)
    cache_put(key, perm)
    return perm


def assign_rect(drones: np.ndarray, slots: np.ndarray) -> np.ndarray:
    """Assign each drone to one of len(slots) >= len(drones) slots (min total squared
    distance).  Returns slot index per drone."""
    C = cdist(drones, slots, "sqeuclidean")
    if _lap is not None:
        _, x, _ = _lap.lapjv(C, extend_cost=True)
        return np.asarray(x, dtype=np.int64)
    from scipy.optimize import linear_sum_assignment   # pragma: no cover
    r, c = linear_sum_assignment(C)
    out = np.full(len(drones), -1, dtype=np.int64)
    out[r] = c
    return out
