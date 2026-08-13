from __future__ import print_function
import math


class DetectionStabilityDiagnostics(object):
    """Diagnostic-only frame-to-frame candidate continuity monitor.

    This module never changes perception or tracker output. It observes plain
    candidate dictionaries after ROI/scoring and before the production tracker,
    so the same logic can later be ported to Qt/C++ without CARLA dependencies.
    """

    def __init__(self, match_distance=3.5, max_missed_frames=2,
                 fragmentation_distance=2.0):
        self.match_distance = float(match_distance)
        self.max_missed_frames = max(0, int(max_missed_frames))
        self.fragmentation_distance = float(fragmentation_distance)
        self._states = {}
        self._next_id = 1
        self.last_stats = self._empty_stats()

    def _empty_stats(self):
        return {
            "candidates": 0,
            "persistent": 0,
            "new": 0,
            "one_frame_lost": 0,
            "reassociated": 0,
            "fragmented": 0,
            "lost": 0,
            "mean_jump": 0.0,
            "max_jump": 0.0,
            "mean_extent_delta": 0.0,
        }

    @staticmethod
    def _distance(a, b):
        return math.hypot(float(a.get("x", 0.0)) - float(b.get("x", 0.0)),
                          float(a.get("y", 0.0)) - float(b.get("y", 0.0)))

    @staticmethod
    def _extent_delta(a, b):
        ea = list(a.get("extent", [0.0, 0.0, 0.0]))
        eb = list(b.get("extent", [0.0, 0.0, 0.0]))
        while len(ea) < 3:
            ea.append(0.0)
        while len(eb) < 3:
            eb.append(0.0)
        denom = max(0.5, sum(abs(float(x)) for x in ea[:3]))
        return sum(abs(float(ea[i]) - float(eb[i])) for i in range(3)) / denom

    def update(self, candidates):
        current = [dict(x) for x in (candidates or [])]
        stats = self._empty_stats()
        stats["candidates"] = len(current)

        # Greedy nearest-neighbour continuity matching. This is diagnostic only.
        pairs = []
        for ci, c in enumerate(current):
            for sid, state in self._states.items():
                d = self._distance(c, state["candidate"])
                if d <= self.match_distance:
                    pairs.append((d, ci, sid))
        pairs.sort(key=lambda x: x[0])

        assigned_current = set()
        assigned_state = set()
        matches = []
        for d, ci, sid in pairs:
            if ci in assigned_current or sid in assigned_state:
                continue
            assigned_current.add(ci)
            assigned_state.add(sid)
            matches.append((d, ci, sid))

        jumps = []
        extent_deltas = []
        for d, ci, sid in matches:
            state = self._states[sid]
            c = current[ci]
            stats["persistent"] += 1
            if int(state.get("misses", 0)) > 0:
                stats["reassociated"] += 1
            jumps.append(float(d))
            extent_deltas.append(self._extent_delta(state["candidate"], c))
            state["candidate"] = c
            state["hits"] = int(state.get("hits", 1)) + 1
            state["misses"] = 0

        # Detect likely cluster splitting: more than one current candidate falls
        # near a single previous candidate. Count only the extra pieces.
        for sid, state in self._states.items():
            close = 0
            for c in current:
                if self._distance(c, state["candidate"]) <= self.fragmentation_distance:
                    close += 1
            if close > 1:
                stats["fragmented"] += close - 1

        for ci, c in enumerate(current):
            if ci in assigned_current:
                continue
            sid = "cand_%04d" % self._next_id
            self._next_id += 1
            self._states[sid] = {"candidate": c, "hits": 1, "misses": 0}
            stats["new"] += 1

        stale = []
        for sid, state in list(self._states.items()):
            if sid in assigned_state:
                continue
            # Newly created states belong to this frame and must not be aged yet.
            if int(state.get("hits", 1)) == 1 and int(state.get("misses", 0)) == 0 and \
                    any(state["candidate"] is c for c in current):
                continue
            state["misses"] = int(state.get("misses", 0)) + 1
            if state["misses"] == 1:
                stats["lost"] += 1
                if int(state.get("hits", 1)) == 1:
                    stats["one_frame_lost"] += 1
            if state["misses"] > self.max_missed_frames:
                stale.append(sid)
        for sid in stale:
            self._states.pop(sid, None)

        if jumps:
            stats["mean_jump"] = sum(jumps) / len(jumps)
            stats["max_jump"] = max(jumps)
        if extent_deltas:
            stats["mean_extent_delta"] = sum(extent_deltas) / len(extent_deltas)

        self.last_stats = stats
        return dict(stats)
