from __future__ import print_function

import math
import statistics


class NearRadarTrackInitiator(object):
    """Build temporally confirmed near-field candidates from radar returns.

    The implementation consumes only Cartesian radar returns and scalar
    configuration.  It intentionally has no CARLA dependency so the same
    boundary can later accept MR76 target data in the real-device adapter.
    """

    def __init__(self, config=None):
        c = config or {}
        self.enabled = bool(c.get("radar_initiation_enabled", False))
        self.shadow_mode = bool(c.get("radar_initiation_shadow_mode", True))
        self.min_range = float(c.get("radar_initiation_min_range", 2.0))
        self.max_range = float(c.get("radar_initiation_max_range", 30.0))
        self.cluster_radius = float(c.get("radar_initiation_cluster_radius", 1.5))
        self.cluster_z_gate = float(c.get("radar_initiation_cluster_z_gate", 1.5))
        self.min_points = max(1, int(c.get("radar_initiation_min_points", 2)))
        self.required_frames = max(1, int(c.get("radar_initiation_required_frames", 2)))
        self.match_gate = float(c.get("radar_initiation_match_gate", 2.5))
        self.ttl = float(c.get("radar_initiation_ttl", 0.6))
        self.min_abs_speed = float(c.get("radar_initiation_min_abs_speed", 0.6))
        self.dedupe_distance = float(c.get("radar_initiation_dedupe_distance", 3.0))
        self.max_candidates = max(1, int(c.get("radar_initiation_max_candidates", 24)))
        self.default_extent = list(c.get(
            "radar_initiation_default_extent", [4.5, 1.8, 1.6]))
        self._pending = []
        self._last_frame = None
        self.last_shadow_candidates = []
        self.last_stats = self._empty_stats()

    def _empty_stats(self):
        return {"world_points": 0, "range_points": 0, "clusters": 0,
                "point_rejected": 0, "pending": 0, "confirmed": 0,
                "moving_confirmed": 0, "static_rejected": 0,
                "dedupe_rejected": 0, "roi_rejected": 0, "emitted": 0,
                "shadow_mode": self.shadow_mode, "new_frame": False}

    def _clusters(self, points):
        remaining = set(range(len(points)))
        groups = []
        while remaining:
            seed = remaining.pop()
            group = [seed]
            queue = [seed]
            while queue:
                current = queue.pop()
                p = points[current]
                joined = []
                for index in remaining:
                    q = points[index]
                    if abs(float(p["z"]) - float(q["z"])) > self.cluster_z_gate:
                        continue
                    if math.hypot(float(p["x"]) - float(q["x"]),
                                  float(p["y"]) - float(q["y"])) <= self.cluster_radius:
                        joined.append(index)
                for index in joined:
                    remaining.remove(index)
                    group.append(index)
                    queue.append(index)
            if len(group) >= self.min_points:
                groups.append([points[index] for index in group])
        return groups

    def _candidate(self, points):
        velocities = [float(p.get("velocity", 0.0)) for p in points]
        velocity = float(statistics.median(velocities))
        use = min(points, key=lambda p: abs(float(p.get("velocity", 0.0)) - velocity))
        x = float(statistics.median([float(p["x"]) for p in points]))
        y = float(statistics.median([float(p["y"]) for p in points]))
        z = float(statistics.median([float(p["z"]) for p in points]))
        return {"x": x, "y": y, "z": z,
                "extent": list(self.default_extent),
                "confidence": .82, "sources": ["radar"],
                "candidate_score": .82, "candidate_score_bypass": True,
                "radar_radial_velocity": velocity,
                "radar_los_x": float(use.get("los_x", 0.0)),
                "radar_los_y": float(use.get("los_y", 0.0)),
                "radar_hits": len(points), "radar_initiated": True,
                "sensor_range": float(use.get("sensor_range", math.hypot(x, y)))}

    def _near_existing(self, candidate, existing):
        return any(math.hypot(float(candidate["x"]) - float(item["x"]),
                              float(candidate["y"]) - float(item["y"])) <=
                   self.dedupe_distance for item in (existing or []))

    def _expire(self, now):
        self._pending = [item for item in self._pending
                         if now - float(item.get("last_time", now)) <= self.ttl]

    def update(self, world_points, existing, now, frame_id=None, validator=None):
        stats = self._empty_stats()
        stats["world_points"] = len(world_points or [])
        if not self.enabled:
            self.last_shadow_candidates = []
            self.last_stats = stats
            return []
        token = frame_id if frame_id is not None else now
        if token == self._last_frame:
            stats["pending"] = len(self._pending)
            self.last_stats = stats
            return []
        self._last_frame = token
        stats["new_frame"] = True
        self._expire(now)
        ranged = [p for p in (world_points or [])
                  if self.min_range <= float(p.get("sensor_range", 0.0)) <= self.max_range]
        stats["range_points"] = len(ranged)
        groups = self._clusters(ranged)
        stats["clusters"] = len(groups)
        stats["point_rejected"] = max(0, len(ranged) - sum(len(g) for g in groups))
        confirmed = []
        existing_pending = len(self._pending)
        used_pending = set()
        for group in groups:
            candidate = self._candidate(group)
            best = None
            for index, old in enumerate(self._pending[:existing_pending]):
                if index in used_pending:
                    continue
                distance = math.hypot(float(candidate["x"]) - float(old["x"]),
                                      float(candidate["y"]) - float(old["y"]))
                if distance <= self.match_gate and (best is None or distance < best[0]):
                    best = (distance, index, old)
            if best is None:
                entry = dict(candidate)
                entry.update({"hits": 1, "last_time": now, "last_frame": token})
                self._pending.append(entry)
                continue
            unused_distance, unused_index, old = best
            used_pending.add(unused_index)
            old.update(candidate)
            old["hits"] = int(old.get("hits", 1)) + 1
            old["last_time"] = now
            old["last_frame"] = token
            if int(old["hits"]) < self.required_frames:
                continue
            item = dict(candidate)
            item["radar_initiation_frames"] = int(old["hits"])
            confirmed.append(item)
        stats["pending"] = len(self._pending)
        stats["confirmed"] = len(confirmed)
        self.last_shadow_candidates = [dict(item) for item in confirmed]
        emitted = []
        for item in confirmed:
            if abs(float(item.get("radar_radial_velocity", 0.0))) < self.min_abs_speed:
                stats["static_rejected"] += 1
                continue
            stats["moving_confirmed"] += 1
            if self._near_existing(item, existing):
                stats["dedupe_rejected"] += 1
                continue
            if validator is not None:
                try:
                    result = validator(item)
                except Exception:
                    result = False
                if not bool(result):
                    stats["roi_rejected"] += 1
                    continue
            emitted.append(item)
            if len(emitted) >= self.max_candidates:
                break
        stats["emitted"] = len(emitted)
        self.last_stats = stats
        return [] if self.shadow_mode else emitted
