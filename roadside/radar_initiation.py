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
        self.single_point_enabled = bool(c.get(
            "radar_initiation_single_point_enabled", False))
        self.single_point_min_abs_speed = float(c.get(
            "radar_initiation_single_point_min_abs_speed", .20))
        self.single_point_required_frames = max(2, int(c.get(
            "radar_initiation_single_point_required_frames", 3)))
        self.single_point_ttl = float(c.get(
            "radar_initiation_single_point_ttl", 1.0))
        self.seed_bridge_shadow_enabled = bool(c.get(
            "radar_initiation_seed_bridge_shadow_enabled", False))
        self.seed_bridge_required_frames = sorted(set(
            max(2, int(value)) for value in c.get(
                "radar_initiation_seed_bridge_required_frames", [2, 3])))
        self.seed_bridge_match_gates = sorted(set(
            float(value) for value in c.get(
                "radar_initiation_seed_bridge_match_gates", [2.5, 4.0, 6.0])))
        self.seed_to_component_shadow_enabled = bool(c.get(
            "radar_initiation_seed_to_component_shadow_enabled", False))
        self.seed_to_component_match_gates = sorted(set(
            float(value) for value in c.get(
                "radar_initiation_seed_to_component_match_gates", [2.5, 4.0])))
        self.required_frames = max(1, int(c.get("radar_initiation_required_frames", 2)))
        self.match_gate = float(c.get("radar_initiation_match_gate", 2.5))
        self.ttl = float(c.get("radar_initiation_ttl", 0.6))
        self.min_abs_speed = float(c.get("radar_initiation_min_abs_speed", 0.6))
        self.dedupe_distance = float(c.get("radar_initiation_dedupe_distance", 3.0))
        self.max_candidates = max(1, int(c.get("radar_initiation_max_candidates", 24)))
        self.speed_shadow_thresholds = [float(value) for value in c.get(
            "radar_initiation_speed_shadow_thresholds", [.10, .20, .40, .60])]
        self.single_speed_shadow_thresholds = [float(value) for value in c.get(
            "radar_initiation_single_speed_shadow_thresholds",
            [.05, .10, .20, .40, .60])]
        self.default_extent = list(c.get(
            "radar_initiation_default_extent", [4.5, 1.8, 1.6]))
        self._pending = []
        self._seed_bridge_pending = []
        self._seed_bridge_gate_pending = dict(
            ("%.1f" % gate, []) for gate in self.seed_bridge_match_gates
            if abs(gate - self.match_gate) > 1e-6)
        self._seed_to_component_pending = dict(
            ("%.1f" % gate, []) for gate in self.seed_to_component_match_gates)
        self._last_frame = None
        self.last_shadow_candidates = []
        self.last_seed_bridge_shadow_candidates = {}
        self.seed_bridge_stats = self._empty_seed_bridge_stats()
        self.seed_to_component_stats = self._empty_seed_to_component_stats()
        self.cumulative_stats = self._empty_cumulative_stats()
        self.last_stats = self._empty_stats()

    def _empty_seed_bridge_stats(self):
        return {"frames": 0, "seeds": 0, "matches": 0,
                "below_speed_matches": 0, "expired": 0,
                "expired_hits": {"1": 0, "2": 0, "3+": 0},
                "rules": dict(
                    (str(frames), {"confirmed": 0, "dedupe_rejected": 0,
                                   "roi_rejected": 0, "would_emit": 0})
                    for frames in self.seed_bridge_required_frames),
                "gate_ablation": dict(
                    ("%.1f" % gate, self._empty_seed_bridge_gate_stats())
                    for gate in self.seed_bridge_match_gates
                    if abs(gate - self.match_gate) > 1e-6)}

    def _empty_seed_bridge_gate_stats(self):
        return {"seeds": 0, "matches": 0, "below_speed_matches": 0,
                "expired": 0,
                "expired_hits": {"1": 0, "2": 0, "3+": 0},
                "rules": dict(
                    (str(frames), {"confirmed": 0, "dedupe_rejected": 0,
                                   "roi_rejected": 0, "would_emit": 0})
                    for frames in self.seed_bridge_required_frames)}

    def _empty_seed_to_component_stats(self):
        return dict(
            ("%.1f" % gate,
             {"seeds": 0, "matches": 0, "moving_matches": 0,
              "matched_points": 0, "expired": 0,
              "dedupe_rejected": 0, "roi_rejected": 0, "would_emit": 0})
            for gate in self.seed_to_component_match_gates)

    def _empty_cumulative_stats(self):
        return {"frames": 0, "range_points": 0, "components": 0,
                "single_point_components": 0,
                "mixed_moving_components": 0,
                "moving_points_in_multi_components": 0,
                "single_point_candidates": 0,
                "single_point_started": 0, "single_point_matched": 0,
                "single_point_below_speed_near_pending": 0,
                "single_point_expired": 0,
                "single_point_confirmed": 0,
                "single_point_emitted": 0,
                "single_point_speed_counts": dict(
                    ("%.2f" % threshold, 0)
                    for threshold in self.single_speed_shadow_thresholds),
                "single_point_expired_hits": {"1": 0, "2": 0, "3+": 0}}

    def _empty_stats(self):
        return {"world_points": 0, "range_points": 0, "components": 0,
                "clusters": 0, "single_point_candidates": 0,
                "single_point_components": 0,
                "mixed_moving_components": 0,
                "moving_points_in_multi_components": 0,
                "single_point_started": 0, "single_point_matched": 0,
                "single_point_below_speed_near_pending": 0,
                "single_point_expired": 0,
                "single_point_expired_hits": {"1": 0, "2": 0, "3+": 0},
                "single_point_speed_counts": {},
                "point_rejected": 0, "pending": 0, "confirmed": 0,
                "single_point_confirmed": 0,
                "moving_confirmed": 0, "static_rejected": 0,
                "dedupe_rejected": 0, "roi_rejected": 0, "emitted": 0,
                "single_point_emitted": 0,
                "confirmed_abs_speed_p50": None,
                "confirmed_abs_speed_max": None,
                "speed_shadow_counts": {},
                "shadow_mode": self.shadow_mode, "new_frame": False}

    def _attach_cumulative(self, stats):
        stats["cumulative"] = dict(self.cumulative_stats)
        stats["cumulative"]["single_point_speed_counts"] = dict(
            self.cumulative_stats["single_point_speed_counts"])
        stats["cumulative"]["single_point_expired_hits"] = dict(
            self.cumulative_stats["single_point_expired_hits"])
        bridge = dict(self.seed_bridge_stats)
        bridge["expired_hits"] = dict(self.seed_bridge_stats["expired_hits"])
        bridge["rules"] = dict(
            (key, dict(value))
            for key, value in self.seed_bridge_stats["rules"].items())
        bridge["gate_ablation"] = {}
        for gate, value in self.seed_bridge_stats["gate_ablation"].items():
            copied = dict(value)
            copied["expired_hits"] = dict(value["expired_hits"])
            copied["rules"] = dict(
                (key, dict(rule)) for key, rule in value["rules"].items())
            bridge["gate_ablation"][gate] = copied
        stats["seed_bridge_shadow"] = bridge
        stats["seed_to_component_shadow"] = dict(
            (key, dict(value))
            for key, value in self.seed_to_component_stats.items())

    def _accumulate(self, stats):
        scalar_keys = ("range_points", "components", "single_point_components",
                       "mixed_moving_components",
                       "moving_points_in_multi_components",
                       "single_point_candidates", "single_point_started",
                       "single_point_matched",
                       "single_point_below_speed_near_pending",
                       "single_point_expired", "single_point_confirmed",
                       "single_point_emitted")
        self.cumulative_stats["frames"] += 1
        for key in scalar_keys:
            self.cumulative_stats[key] += int(stats.get(key, 0))
        for key, value in stats.get("single_point_speed_counts", {}).items():
            self.cumulative_stats["single_point_speed_counts"][key] = (
                self.cumulative_stats["single_point_speed_counts"].get(key, 0) +
                int(value))
        for key, value in stats.get("single_point_expired_hits", {}).items():
            self.cumulative_stats["single_point_expired_hits"][key] += int(value)
        self._attach_cumulative(stats)

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
            groups.append([points[index] for index in group])
        return groups

    def _candidate(self, points, mode="cluster"):
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
                "radar_initiation_mode": mode,
                "sensor_range": float(use.get("sensor_range", math.hypot(x, y)))}

    def _update_seed_bridge_shadow(self, singleton_groups, existing, now,
                                   token, validator):
        self.last_seed_bridge_shadow_candidates = dict(
            (str(frames), []) for frames in self.seed_bridge_required_frames)
        if not self.seed_bridge_shadow_enabled:
            return
        self.seed_bridge_stats["frames"] += 1
        for gate in self.seed_bridge_match_gates:
            if abs(gate - self.match_gate) <= 1e-6:
                continue
            self._update_seed_bridge_gate_shadow(
                "%.1f" % gate, gate, singleton_groups, existing, now,
                token, validator)
        kept = []
        for old in self._seed_bridge_pending:
            if now - float(old.get("last_time", now)) <= self.single_point_ttl:
                kept.append(old)
                continue
            self.seed_bridge_stats["expired"] += 1
            hits = int(old.get("hits", 1))
            self.seed_bridge_stats["expired_hits"][
                "3+" if hits >= 3 else str(hits)] += 1
        self._seed_bridge_pending = kept
        existing_pending = len(self._seed_bridge_pending)
        used_pending = set()
        for group in singleton_groups:
            candidate = self._candidate(group, mode="single_seed_bridge_shadow")
            speed = abs(float(candidate.get("radar_radial_velocity", 0.0)))
            best = None
            for index, old in enumerate(
                    self._seed_bridge_pending[:existing_pending]):
                if index in used_pending:
                    continue
                distance = math.hypot(float(candidate["x"]) - float(old["x"]),
                                      float(candidate["y"]) - float(old["y"]))
                if distance <= self.match_gate and (best is None or
                                                     distance < best[0]):
                    best = (distance, index, old)
            if best is None:
                if speed < self.single_point_min_abs_speed:
                    continue
                entry = dict(candidate)
                entry.update({"hits": 1, "last_time": now,
                              "last_frame": token,
                              "seed_speed": speed,
                              "confirmed_rules": []})
                self._seed_bridge_pending.append(entry)
                self.seed_bridge_stats["seeds"] += 1
                continue
            unused_distance, index, old = best
            used_pending.add(index)
            self.seed_bridge_stats["matches"] += 1
            if speed < self.single_point_min_abs_speed:
                self.seed_bridge_stats["below_speed_matches"] += 1
            seed_speed = float(old.get("seed_speed", speed))
            confirmed_rules = list(old.get("confirmed_rules", []))
            old.update(candidate)
            old["hits"] = int(old.get("hits", 1)) + 1
            old["last_time"] = now
            old["last_frame"] = token
            old["seed_speed"] = seed_speed
            old["confirmed_rules"] = confirmed_rules
            for required in self.seed_bridge_required_frames:
                rule = str(required)
                if int(old["hits"]) < required or rule in confirmed_rules:
                    continue
                confirmed_rules.append(rule)
                item = dict(candidate)
                item["radar_initiation_frames"] = int(old["hits"])
                item["radar_seed_abs_speed"] = seed_speed
                item["radar_radial_velocity"] = float(
                    old.get("radar_radial_velocity", 0.0))
                self.seed_bridge_stats["rules"][rule]["confirmed"] += 1
                if self._near_existing(item, existing):
                    self.seed_bridge_stats["rules"][rule]["dedupe_rejected"] += 1
                    continue
                if validator is not None:
                    try:
                        valid = bool(validator(item))
                    except Exception:
                        valid = False
                    if not valid:
                        self.seed_bridge_stats["rules"][rule]["roi_rejected"] += 1
                        continue
                self.seed_bridge_stats["rules"][rule]["would_emit"] += 1
                self.last_seed_bridge_shadow_candidates[rule].append(item)

    def _update_seed_bridge_gate_shadow(self, gate_key, gate, singleton_groups,
                                        existing, now, token, validator):
        pending = self._seed_bridge_gate_pending[gate_key]
        stats = self.seed_bridge_stats["gate_ablation"][gate_key]
        kept = []
        for old in pending:
            if now - float(old.get("last_time", now)) <= self.single_point_ttl:
                kept.append(old)
                continue
            stats["expired"] += 1
            hits = int(old.get("hits", 1))
            stats["expired_hits"]["3+" if hits >= 3 else str(hits)] += 1
        pending = kept
        existing_pending = len(pending)
        used_pending = set()
        for group in singleton_groups:
            candidate = self._candidate(group, mode="single_seed_bridge_shadow")
            speed = abs(float(candidate.get("radar_radial_velocity", 0.0)))
            best = None
            for index, old in enumerate(pending[:existing_pending]):
                if index in used_pending:
                    continue
                distance = math.hypot(float(candidate["x"]) - float(old["x"]),
                                      float(candidate["y"]) - float(old["y"]))
                if distance <= gate and (best is None or distance < best[0]):
                    best = (distance, index, old)
            if best is None:
                if speed < self.single_point_min_abs_speed:
                    continue
                entry = dict(candidate)
                entry.update({"hits": 1, "last_time": now,
                              "last_frame": token, "seed_speed": speed,
                              "confirmed_rules": []})
                pending.append(entry)
                stats["seeds"] += 1
                continue
            unused_distance, index, old = best
            used_pending.add(index)
            stats["matches"] += 1
            if speed < self.single_point_min_abs_speed:
                stats["below_speed_matches"] += 1
            seed_speed = float(old.get("seed_speed", speed))
            confirmed_rules = list(old.get("confirmed_rules", []))
            old.update(candidate)
            old.update({"hits": int(old.get("hits", 1)) + 1,
                        "last_time": now, "last_frame": token,
                        "seed_speed": seed_speed,
                        "confirmed_rules": confirmed_rules})
            for required in self.seed_bridge_required_frames:
                rule = str(required)
                if int(old["hits"]) < required or rule in confirmed_rules:
                    continue
                confirmed_rules.append(rule)
                item = dict(candidate)
                item["radar_initiation_frames"] = int(old["hits"])
                item["radar_seed_abs_speed"] = seed_speed
                value = stats["rules"][rule]
                value["confirmed"] += 1
                if self._near_existing(item, existing):
                    value["dedupe_rejected"] += 1
                    continue
                if validator is not None:
                    try:
                        valid = bool(validator(item))
                    except Exception:
                        valid = False
                    if not valid:
                        value["roi_rejected"] += 1
                        continue
                value["would_emit"] += 1
                variant = "g%s_f%s" % (gate_key, rule)
                self.last_seed_bridge_shadow_candidates.setdefault(
                    variant, []).append(item)
        self._seed_bridge_gate_pending[gate_key] = pending

    def _update_seed_to_component_shadow(self, singleton_groups,
                                         component_groups, existing, now,
                                         token, validator):
        """Profile singleton-to-cluster morphology changes without emitting.

        A moving singleton is the only allowed seed.  On a later frame, any
        multi-return component may terminate that seed.  This deliberately
        measures representation changes separately from the singleton-only
        bridge and never contributes candidates to the production output.
        """
        if not self.seed_to_component_shadow_enabled:
            return
        for gate in self.seed_to_component_match_gates:
            key = "%.1f" % gate
            stats = self.seed_to_component_stats[key]
            pending = []
            for old in self._seed_to_component_pending[key]:
                if now - float(old.get("last_time", now)) <= self.single_point_ttl:
                    pending.append(old)
                else:
                    stats["expired"] += 1
            existing_pending = len(pending)
            used_pending = set()
            matched_pending = set()
            for group in component_groups:
                candidate = self._candidate(
                    group, mode="single_seed_to_component_shadow")
                best = None
                for index, old in enumerate(pending[:existing_pending]):
                    if index in used_pending:
                        continue
                    distance = math.hypot(
                        float(candidate["x"]) - float(old["x"]),
                        float(candidate["y"]) - float(old["y"]))
                    if distance <= gate and (best is None or distance < best[0]):
                        best = (distance, index, old)
                if best is None:
                    continue
                unused_distance, index, old = best
                used_pending.add(index)
                matched_pending.add(index)
                stats["matches"] += 1
                stats["matched_points"] += len(group)
                if abs(float(candidate.get("radar_radial_velocity", 0.0))) >= \
                        self.single_point_min_abs_speed:
                    stats["moving_matches"] += 1
                item = dict(candidate)
                item["radar_initiation_frames"] = 2
                item["radar_seed_abs_speed"] = float(old.get("seed_speed", 0.0))
                if self._near_existing(item, existing):
                    stats["dedupe_rejected"] += 1
                    continue
                if validator is not None:
                    try:
                        valid = bool(validator(item))
                    except Exception:
                        valid = False
                    if not valid:
                        stats["roi_rejected"] += 1
                        continue
                stats["would_emit"] += 1
                variant = "morph_g%s" % key
                self.last_seed_bridge_shadow_candidates.setdefault(
                    variant, []).append(item)
            pending = [old for index, old in enumerate(pending)
                       if index not in matched_pending]
            for group in singleton_groups:
                candidate = self._candidate(
                    group, mode="single_seed_to_component_shadow")
                speed = abs(float(candidate.get("radar_radial_velocity", 0.0)))
                if speed < self.single_point_min_abs_speed:
                    continue
                if any(math.hypot(
                        float(candidate["x"]) - float(old["x"]),
                        float(candidate["y"]) - float(old["y"])) <= gate
                       for old in pending):
                    continue
                entry = dict(candidate)
                entry.update({"last_time": now, "last_frame": token,
                              "seed_speed": speed})
                pending.append(entry)
                stats["seeds"] += 1
            self._seed_to_component_pending[key] = pending

    def _near_existing(self, candidate, existing):
        return any(math.hypot(float(candidate["x"]) - float(item["x"]),
                              float(candidate["y"]) - float(item["y"])) <=
                   self.dedupe_distance for item in (existing or []))

    def _expire(self, now):
        kept = []
        expired = []
        for item in self._pending:
            if now - float(item.get("last_time", now)) <= float(
                    item.get("ttl", self.ttl)):
                kept.append(item)
            else:
                expired.append(item)
        self._pending = kept
        return expired

    def update(self, world_points, existing, now, frame_id=None, validator=None):
        stats = self._empty_stats()
        stats["world_points"] = len(world_points or [])
        if not self.enabled:
            self.last_shadow_candidates = []
            self._attach_cumulative(stats)
            self.last_stats = stats
            return []
        token = frame_id if frame_id is not None else now
        if token == self._last_frame:
            self.last_seed_bridge_shadow_candidates = dict(
                (str(frames), [])
                for frames in self.seed_bridge_required_frames)
            stats["pending"] = len(self._pending)
            self._attach_cumulative(stats)
            self.last_stats = stats
            return []
        self._last_frame = token
        stats["new_frame"] = True
        expired = self._expire(now)
        expired_single = [item for item in expired
                          if item.get("radar_initiation_mode") == "single_moving"]
        stats["single_point_expired"] = len(expired_single)
        for item in expired_single:
            hits = int(item.get("hits", 1))
            key = "3+" if hits >= 3 else str(hits)
            stats["single_point_expired_hits"][key] += 1
        ranged = [p for p in (world_points or [])
                  if self.min_range <= float(p.get("sensor_range", 0.0)) <= self.max_range]
        stats["range_points"] = len(ranged)
        components = self._clusters(ranged)
        stats["components"] = len(components)
        groups = []
        singleton_speeds = []
        singleton_groups = []
        for group in components:
            if len(group) >= self.min_points:
                moving_points = sum(
                    1 for point in group
                    if abs(float(point.get("velocity", 0.0))) >=
                    self.single_point_min_abs_speed)
                if moving_points:
                    stats["mixed_moving_components"] += 1
                    stats["moving_points_in_multi_components"] += moving_points
                groups.append((group, "cluster"))
            elif len(group) == 1:
                singleton_groups.append(group)
                stats["single_point_components"] += 1
                speed = abs(float(group[0].get("velocity", 0.0)))
                singleton_speeds.append(speed)
                if self.single_point_enabled and speed >= self.single_point_min_abs_speed:
                    groups.append((group, "single_moving"))
                    stats["single_point_candidates"] += 1
                elif any(
                        old.get("radar_initiation_mode") == "single_moving" and
                        math.hypot(float(group[0]["x"]) - float(old["x"]),
                                   float(group[0]["y"]) - float(old["y"])) <=
                        self.match_gate for old in self._pending):
                    stats["single_point_below_speed_near_pending"] += 1
        stats["single_point_speed_counts"] = dict(
            ("%.2f" % threshold,
             sum(1 for speed in singleton_speeds if speed >= threshold))
            for threshold in self.single_speed_shadow_thresholds)
        self._update_seed_bridge_shadow(
            singleton_groups, existing, now, token, validator)
        self._update_seed_to_component_shadow(
            singleton_groups,
            [group for group in components if len(group) >= self.min_points],
            existing, now, token, validator)
        stats["clusters"] = len(groups)
        stats["point_rejected"] = max(
            0, len(ranged) - sum(len(group) for group, unused_mode in groups))
        confirmed = []
        existing_pending = len(self._pending)
        used_pending = set()
        for group, mode in groups:
            candidate = self._candidate(group, mode=mode)
            best = None
            for index, old in enumerate(self._pending[:existing_pending]):
                if index in used_pending:
                    continue
                if old.get("radar_initiation_mode", "cluster") != mode:
                    continue
                distance = math.hypot(float(candidate["x"]) - float(old["x"]),
                                      float(candidate["y"]) - float(old["y"]))
                if distance <= self.match_gate and (best is None or distance < best[0]):
                    best = (distance, index, old)
            if best is None:
                entry = dict(candidate)
                entry.update({"hits": 1, "last_time": now, "last_frame": token,
                              "ttl": (self.single_point_ttl
                                      if mode == "single_moving" else self.ttl)})
                self._pending.append(entry)
                if mode == "single_moving":
                    stats["single_point_started"] += 1
                continue
            unused_distance, unused_index, old = best
            used_pending.add(unused_index)
            if mode == "single_moving":
                stats["single_point_matched"] += 1
            old.update(candidate)
            old["hits"] = int(old.get("hits", 1)) + 1
            old["last_time"] = now
            old["last_frame"] = token
            required = (self.single_point_required_frames
                        if mode == "single_moving" else self.required_frames)
            if int(old["hits"]) < required:
                continue
            item = dict(candidate)
            item["radar_initiation_frames"] = int(old["hits"])
            item["radar_initiation_required_frames"] = required
            confirmed.append(item)
        stats["pending"] = len(self._pending)
        stats["confirmed"] = len(confirmed)
        stats["single_point_confirmed"] = sum(
            1 for item in confirmed
            if item.get("radar_initiation_mode") == "single_moving")
        confirmed_speeds = [abs(float(item.get("radar_radial_velocity", 0.0)))
                            for item in confirmed]
        if confirmed_speeds:
            stats["confirmed_abs_speed_p50"] = float(
                statistics.median(confirmed_speeds))
            stats["confirmed_abs_speed_max"] = max(confirmed_speeds)
        stats["speed_shadow_counts"] = dict(
            ("%.2f" % threshold,
             sum(1 for speed in confirmed_speeds if speed >= threshold))
            for threshold in self.speed_shadow_thresholds)
        self.last_shadow_candidates = [dict(item) for item in confirmed]
        emitted = []
        for item in confirmed:
            min_speed = (self.single_point_min_abs_speed
                         if item.get("radar_initiation_mode") == "single_moving"
                         else self.min_abs_speed)
            if abs(float(item.get("radar_radial_velocity", 0.0))) < min_speed:
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
        stats["single_point_emitted"] = sum(
            1 for item in emitted
            if item.get("radar_initiation_mode") == "single_moving")
        self._accumulate(stats)
        self.last_stats = stats
        return [] if self.shadow_mode else emitted
