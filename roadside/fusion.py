from __future__ import print_function
import math
import time
import statistics
from collections import defaultdict

from .models import DetectedObject, ObjectList
from .perception import voxel_cluster_lidar, adaptive_voxel_cluster_lidar, merge_lidar_clusters
from .sparse_geometry_rescue import track_guided_sparse_rescue
from .tracking import NearestTracker


class PersistentStaticFilter(object):
    def __init__(self, calibration_seconds=6., cell_size=1., occupancy_ratio=.45,
                 moving_radar_speed=1.2, neighbor_radius_cells=2, **kwargs):
        self.calibration_seconds = float(calibration_seconds)
        self.cell_size = float(cell_size)
        self.occupancy_ratio = float(occupancy_ratio)
        self.moving_radar_speed = float(moving_radar_speed)
        self.neighbor_radius_cells = int(neighbor_radius_cells)
        self.started_at = None
        self.frames = 0
        self.counts = defaultdict(int)
        self.static_cells = set()
        self.ready = False

    def _key(self, x, y):
        return int(math.floor(x / self.cell_size)), int(math.floor(y / self.cell_size))

    def _near_static(self, k):
        x, y = k
        r = max(0, self.neighbor_radius_cells)
        return any((x + a, y + b) in self.static_cells
                   for a in range(-r, r + 1) for b in range(-r, r + 1))

    def update_and_filter(self, c, now):
        if self.started_at is None:
            self.started_at = now
        if not self.ready:
            self.frames += 1
            for k in set(self._key(x["x"], x["y"]) for x in c):
                self.counts[k] += 1
            if now - self.started_at >= self.calibration_seconds:
                th = max(2, int(math.ceil(self.frames * self.occupancy_ratio)))
                self.static_cells = set(k for k, n in self.counts.items() if n >= th)
                self.ready = True
            return []
        return [x for x in c if not (
            self._near_static(self._key(x["x"], x["y"])) and not (
                x.get("radar_radial_velocity") is not None and
                abs(x["radar_radial_velocity"]) >= self.moving_radar_speed))]

    def remaining_seconds(self, now):
        if self.ready:
            return 0.
        if self.started_at is None:
            return self.calibration_seconds
        return max(0., self.calibration_seconds - (now - self.started_at))


class SimpleFusion(object):
    def __init__(self, station_id, config):
        self.station_id = station_id
        self.config = config
        self.tracker = NearestTracker(
            config.get("track_match_distance", 4.), config.get("track_max_age", 1.5),
            config.get("track_max_speed", 20.), config.get("velocity_alpha", .25),
            config.get("extent_alpha", .25), config.get("extent_shrink_alpha", .05),
            config.get("extent_lock_hits", 5), config.get("radar_velocity_alpha", .35),
            config.get("velocity_window", 5), config.get("position_alpha", .45),
            config.get("stationary_speed", .35), config.get("track_coast_frames", 0),
            config.get("track_coast_confidence_decay", .88),
            config.get("track_adaptive_coast_enabled", False),
            config.get("track_coast_young_frames", 1),
            config.get("track_coast_stable_frames", 3),
            config.get("track_coast_far_frames", 4),
            config.get("track_coast_stable_hits", 3),
            config.get("track_coast_far_range", 50.0),
            config.get("track_coast_low_score", 0.55),
            config.get("track_coast_edge_ratio", 0.90))
        self.tracker.configure_quality(config)
        self.background = PersistentStaticFilter(**config)
        self.world_transform = None
        self.radar_matrix = None
        self.radar_origin = None
        self.ground_reference_z = None
        self.candidate_validator = None
        self.last_stats = {}
        self.last_geometry_world = []
        self.last_roi_candidates = []
        self.last_scored_candidates = []
        self.last_dynamic_candidates = []
        self.last_tracked_candidates = []
        self.last_roi_rejections = []
        self.last_score_rejections = []
        self.last_recovery_quality_candidates = []
        self.last_recovery_quality_rejections = []
        self.last_far_admission_candidates = []
        self.last_far_admission_rejections = []
        self._far_admission_pending = []
        self._far_admission_last_frame = None
        self._far_admission_sequence = 0

    def set_world_transform(self, t):
        if t is None:
            self.world_transform = None
            return
        self.world_transform = {"x": float(t.location.x), "y": float(t.location.y),
                                "z": float(t.location.z),
                                "yaw": math.radians(float(t.rotation.yaw))}

    def set_ground_reference(self, z):
        self.ground_reference_z = None if z is None else float(z)

    def set_radar_transform(self, t):
        if t is None:
            self.radar_matrix = None
            self.radar_origin = None
            return
        try:
            self.radar_matrix = t.get_matrix()
        except Exception:
            self.radar_matrix = None
        self.radar_origin = (float(t.location.x), float(t.location.y), float(t.location.z))

    def set_candidate_validator(self, v):
        self.candidate_validator = v

    def _to_world(self, x, y, z):
        if self.world_transform is None:
            return x, y, z
        t = self.world_transform
        c = math.cos(t["yaw"])
        s = math.sin(t["yaw"])
        return t["x"] + c * x - s * y, t["y"] + s * x + c * y, t["z"] + z

    def _sensor_range(self, x, y):
        if self.world_transform is None:
            return math.hypot(float(x), float(y))
        return math.hypot(float(x) - self.world_transform["x"],
                          float(y) - self.world_transform["y"])

    def _remove_ground_points(self, points):
        if points is None:
            return None, 0
        total = len(points)
        if total == 0:
            return points, 0
        if not self.config.get("ground_removal_enabled", True) or \
                self.ground_reference_z is None or self.world_transform is None:
            return points, 0
        clearance = float(self.config.get("ground_clearance", 0.30))
        sensor_z = float(self.world_transform["z"])
        cut_local = float(self.ground_reference_z) + clearance - sensor_z
        kept = [p for p in points if float(p[2]) > cut_local]
        return kept, total - len(kept)

    def _radar_point_to_world(self, d):
        if self.radar_matrix is None:
            return None
        m = self.radar_matrix
        x, y, z = float(d["x"]), float(d["y"]), float(d["z"])
        wx = m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3]
        wy = m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3]
        wz = m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3]
        ox, oy, oz = self.radar_origin
        dx, dy, dz = wx - ox, wy - oy, wz - oz
        n = math.sqrt(dx * dx + dy * dy + dz * dz)
        if n < 1e-3:
            return None
        return {"x": wx, "y": wy, "z": wz, "velocity": float(d.get("velocity", 0.0)),
                "los_x": dx / n, "los_y": dy / n, "los_z": dz / n}

    def _associate_radar_world(self, clusters, radar_detections):
        points = []
        for d in radar_detections or []:
            p = self._radar_point_to_world(d)
            if p is not None:
                points.append(p)
        max_d = float(self.config.get("radar_match_distance", 4.0))
        max_z = float(self.config.get("radar_match_z", 2.5))
        min_hits = int(self.config.get("radar_min_hits", 1))
        out = []
        matched = 0
        for c in clusters:
            near = []
            nearest_xy = None
            nearest_3d = None
            for p in points:
                dxy = math.hypot(p["x"] - c["x"], p["y"] - c["y"])
                d3 = math.sqrt(dxy * dxy + (p["z"] - c["z"]) ** 2)
                if nearest_xy is None or dxy < nearest_xy:
                    nearest_xy = dxy
                if nearest_3d is None or d3 < nearest_3d:
                    nearest_3d = d3
                if abs(p["z"] - c["z"]) > max_z:
                    continue
                if dxy <= max_d:
                    near.append((dxy, p))
            item = dict(c)
            item["radar_nearest_xy"] = nearest_xy
            item["radar_nearest_3d"] = nearest_3d
            item["radar_hits"] = 0
            if len(near) >= min_hits:
                near.sort(key=lambda x: x[0])
                use = [p for _, p in near[:max(1, min(8, len(near)))]]
                item["radar_radial_velocity"] = float(statistics.median([p["velocity"] for p in use]))
                item["radar_los_x"] = sum(p["los_x"] for p in use) / len(use)
                item["radar_los_y"] = sum(p["los_y"] for p in use) / len(use)
                item["radar_hits"] = len(near)
                matched += 1
            out.append(item)
        return out, len(points), matched

    def _looks_like_pole(self, e):
        ex, ey, ez = [float(v) for v in e]
        hl, hs = max(ex, ey), min(ex, ey)
        c = self.config
        return hs < c.get("pole_short_max", .75) and \
            hl < c.get("pole_long_max", 2.5) and ez > c.get("pole_height_min", 1.5)

    def _validate_candidate(self, wx, wy, wz, e, candidate=None):
        if not self.candidate_validator:
            return True, "ok", {}
        try:
            try:
                result = self.candidate_validator(wx, wy, wz, e, candidate)
            except TypeError:
                # Keep compatibility with real-device/map adapters that still
                # implement the original four-argument validator contract.
                result = self.candidate_validator(wx, wy, wz, e)
            if isinstance(result, tuple):
                ok = bool(result[0])
                reason = result[1] if len(result) > 1 else ("ok" if ok else "rejected")
                details = result[2] if len(result) > 2 else {}
                return ok, reason, details
            return bool(result), ("ok" if result else "rejected"), {}
        except Exception as exc:
            return False, "validator_error", {"error": str(exc)}

    def _cluster(self, clean_points, c):
        if c.get("range_adaptive_clustering", False):
            return adaptive_voxel_cluster_lidar(
                clean_points, c.get("range_bands", []), c.get("lidar_min_z", -7.5),
                c.get("lidar_max_z", 2.), c.get("max_range", 80.),
                c.get("vehicle_max_length", 8.), c.get("vehicle_max_width", 4.),
                c.get("vehicle_max_height", 4.), c.get("max_objects", 120))
        return voxel_cluster_lidar(
            clean_points, c.get("voxel_size", .8), c.get("cluster_min_points", 6),
            c.get("lidar_min_z", -7.5), c.get("lidar_max_z", 2.),
            c.get("max_range", 70.), c.get("vehicle_min_length", .6),
            c.get("vehicle_max_length", 8.), c.get("vehicle_min_width", .4),
            c.get("vehicle_max_width", 4.), c.get("vehicle_min_height", .25),
            c.get("vehicle_max_height", 4.), c.get("max_objects", 80))

    def _candidate_score(self, item):
        e = [float(v) for v in item.get("extent", [0, 0, 0])]
        hl, hs, h = max(e[0], e[1]), min(e[0], e[1]), e[2]
        points = int(item.get("point_count", 0))
        votes = int(item.get("scale_votes", 1))
        details = item.get("roi_details", {}) or {}
        score = 0.0
        if 1.2 <= hl <= 7.5:
            score += .22
        elif .7 <= hl <= 8.0:
            score += .10
        if .55 <= hs <= 3.4:
            score += .20
        elif .30 <= hs <= 3.8:
            score += .08
        if .45 <= h <= 3.5:
            score += .20
        elif .20 <= h <= 3.8:
            score += .08
        if points >= 8:
            score += .16
        elif points >= 4:
            score += .11
        elif points >= 2:
            score += .05
        score += .17 if votes >= 2 else .04
        if item.get("sparse_rescued", False):
            score += float(self.config.get("sparse_geometry_rescue_score_bonus", 0.08))
        lateral = details.get("lateral")
        allowed = details.get("allowed_lateral")
        if lateral is not None and allowed not in (None, 0):
            ratio = float(lateral) / max(.01, float(allowed))
            if ratio <= .65:
                score += .05
            elif ratio <= .85:
                score += .03
        return min(1.0, score)

    def _score_candidates(self, items):
        c = self.config
        if not c.get("candidate_scoring_enabled", False):
            return [dict(x) for x in items], []
        min_range = float(c.get("candidate_scoring_min_range", 50.0))
        far_relaxed = float(c.get("candidate_scoring_far_relaxed_range", 65.0))
        threshold = float(c.get("candidate_scoring_threshold_far", .46))
        threshold_long = float(c.get("candidate_scoring_threshold_far_long", .42))
        kept, rejected = [], []
        for src in items:
            item = dict(src)
            rng = self._sensor_range(item["x"], item["y"])
            item["sensor_range"] = rng
            if rng < min_range:
                item["candidate_score"] = 1.0
                item["candidate_score_threshold"] = 0.0
                item["candidate_score_bypass"] = True
                kept.append(item)
                continue
            score = self._candidate_score(item)
            use_threshold = threshold_long if rng >= far_relaxed else threshold
            item["candidate_score"] = score
            item["candidate_score_threshold"] = use_threshold
            item["candidate_score_bypass"] = False
            if score >= use_threshold:
                kept.append(item)
            else:
                r = dict(item)
                r["reason"] = "candidate_score"
                rejected.append(r)
        return kept, rejected

    def _mark_recovery_track_support(self, item, previous_tracks):
        """Annotate recovery evidence from stable perception tracks only."""
        out = dict(item)
        out["recovery_track_supported"] = False
        if not out.get("far_geometry_recovered", False):
            return out
        gate = float(self.config.get("far_recovery_quality_track_gate", 3.5))
        min_hits = int(self.config.get("far_recovery_quality_track_min_hits", 3))
        min_quality = float(self.config.get("far_recovery_quality_track_min_quality", .47))
        best = None
        for track in previous_tracks or []:
            if int(track.get("track_hits", 0)) < min_hits:
                continue
            if float(track.get("track_quality", 0.0)) < min_quality:
                continue
            d = math.hypot(float(track.get("x", 0.0)) - float(out["x"]),
                           float(track.get("y", 0.0)) - float(out["y"]))
            if d <= gate and (best is None or d < best[0]):
                best = (d, track.get("id"))
        if best is not None:
            out["recovery_track_supported"] = True
            out["recovery_support_track_distance"] = best[0]
            out["recovery_support_track_id"] = best[1]
        return out

    def _gate_recovery_candidates(self, items):
        """Block weak recovery candidates before they can create tracks."""
        if not self.config.get("far_recovery_quality_gate_enabled", True):
            return [dict(x) for x in items], []
        min_current = int(self.config.get(
            "far_recovery_quality_min_current_points_without_support", 4))
        kept, rejected = [], []
        for src in items or []:
            item = dict(src)
            if not item.get("far_geometry_recovered", False):
                kept.append(item)
                continue
            evidence = []
            if item.get("far_geometry_temporal_supported", False):
                evidence.append("temporal")
            if item.get("radar_radial_velocity") is not None:
                evidence.append("radar")
            if item.get("recovery_track_supported", False):
                evidence.append("stable_track")
            current_points = int(item.get("current_point_count", 0))
            if current_points >= min_current:
                evidence.append("current_points")
            item["recovery_quality_evidence"] = evidence
            if evidence:
                item["recovery_quality_gate"] = "pass"
                kept.append(item)
            else:
                item["recovery_quality_gate"] = "reject"
                item["reason"] = "recovery_quality"
                rejected.append(item)
        return kept, rejected

    def _far_admission_track_support(self, item, previous_tracks):
        gate = float(self.config.get("far_track_admission_track_gate", 3.5))
        min_hits = int(self.config.get("far_track_admission_track_min_hits", 2))
        best = None
        for track in previous_tracks or []:
            admitted = bool(track.get("far_track_admission_confirmed", False))
            if not admitted and int(track.get("track_hits", 0)) < min_hits:
                continue
            d = math.hypot(float(track.get("x", 0.0)) - float(item["x"]),
                           float(track.get("y", 0.0)) - float(item["y"]))
            if d <= gate and (best is None or d < best):
                best = d
        return best

    def _gate_far_new_tracks(self, items, previous_tracks, now, frame_id=None):
        """Require repeat evidence before a far pure-LiDAR candidate reaches Tracker."""
        if not self.config.get("far_track_admission_enabled", True):
            return [dict(x) for x in items], [], {
                "pending": 0, "held": 0, "confirmed": 0, "expired": 0,
                "sensor_bypass": 0, "strong_bypass": 0, "track_bypass": 0}
        min_range = float(self.config.get("far_track_admission_min_range", 50.0))
        required = max(2, int(self.config.get("far_track_admission_required_frames", 2)))
        match_gate = float(self.config.get("far_track_admission_match_gate", 2.5))
        ttl = float(self.config.get("far_track_admission_ttl", 0.5))
        strong_points = int(self.config.get("far_track_admission_strong_min_points", 10))
        strong_score = float(self.config.get("far_track_admission_strong_min_score", .72))

        if frame_id is None:
            self._far_admission_sequence += 1
            token = self._far_admission_sequence
        else:
            token = frame_id
        new_frame = token != self._far_admission_last_frame
        if new_frame:
            self._far_admission_last_frame = token

        expired = 0
        pending = []
        for p in self._far_admission_pending:
            if now - float(p.get("last_time", now)) <= ttl:
                pending.append(p)
            elif new_frame:
                expired += 1
        self._far_admission_pending = pending

        kept, rejected, used = [], [], set()
        stats = {"pending": 0, "held": 0, "confirmed": 0,
                 "expired": expired, "sensor_bypass": 0,
                 "strong_bypass": 0, "track_bypass": 0}
        for src in items or []:
            item = dict(src)
            if self._sensor_range(item["x"], item["y"]) < min_range:
                kept.append(item)
                continue
            if item.get("radar_radial_velocity") is not None:
                item["far_track_admission_confirmed"] = True
                item["far_track_admission_reason"] = "sensor"
                stats["sensor_bypass"] += 1
                kept.append(item)
                continue
            track_distance = self._far_admission_track_support(item, previous_tracks)
            if track_distance is not None:
                item["far_track_admission_confirmed"] = True
                item["far_track_admission_reason"] = "existing_track"
                item["far_track_admission_track_distance"] = track_distance
                stats["track_bypass"] += 1
                kept.append(item)
                continue
            points = int(item.get("current_point_count", item.get("point_count", 0)) or 0)
            score = float(item.get("candidate_score", 0.0) or 0.0)
            if points >= strong_points and score >= strong_score:
                item["far_track_admission_confirmed"] = True
                item["far_track_admission_reason"] = "strong"
                stats["strong_bypass"] += 1
                kept.append(item)
                continue

            best = None
            for index, old in enumerate(self._far_admission_pending):
                if index in used:
                    continue
                d = math.hypot(float(old["x"]) - float(item["x"]),
                               float(old["y"]) - float(item["y"]))
                if d <= match_gate and (best is None or d < best[0]):
                    best = (d, index, old)
            if best is None:
                entry = {"x": float(item["x"]), "y": float(item["y"]),
                         "z": float(item.get("z", 0.0)), "hits": 1,
                         "last_time": now, "last_frame": token}
                self._far_admission_pending.append(entry)
                used.add(len(self._far_admission_pending) - 1)
                hits = 1
            else:
                _, index, entry = best
                used.add(index)
                if new_frame and entry.get("last_frame") != token:
                    entry["hits"] = int(entry.get("hits", 1)) + 1
                    entry["x"] = float(item["x"])
                    entry["y"] = float(item["y"])
                    entry["z"] = float(item.get("z", 0.0))
                    entry["last_time"] = now
                    entry["last_frame"] = token
                hits = int(entry.get("hits", 1))
            if hits >= required:
                item["far_track_admission_confirmed"] = True
                item["far_track_admission_reason"] = "repeat"
                item["far_track_admission_hits"] = hits
                stats["confirmed"] += 1
                kept.append(item)
                if best is not None:
                    self._far_admission_pending[best[1]]["confirmed"] = True
            else:
                item["far_track_admission_confirmed"] = False
                item["far_track_admission_reason"] = "pending"
                item["far_track_admission_hits"] = hits
                item["reason"] = "far_track_admission"
                stats["held"] += 1
                rejected.append(item)
        self._far_admission_pending = [p for p in self._far_admission_pending
                                       if not p.get("confirmed", False)]
        stats["pending"] = len(self._far_admission_pending)
        return kept, rejected, stats

    def _refresh_quality_stats(self):
        qs = self.tracker.quality_stats()
        self.last_stats["track_quality_active"] = qs["active"]
        self.last_stats["track_quality_high"] = qs["high"]
        self.last_stats["track_quality_medium"] = qs["medium"]
        self.last_stats["track_quality_low"] = qs["low"]
        self.last_stats["track_quality_avg"] = qs["avg"]

    def apply_camera_confirmations(self, pairs, timestamp=None):
        ids = []
        for p in pairs or []:
            try:
                idx = int(p.get("lidar_index", -1))
                if 0 <= idx < len(self.last_tracked_candidates):
                    ids.append(self.last_tracked_candidates[idx].get("id"))
            except Exception:
                pass
        ids = [x for x in ids if x]
        self.tracker.apply_sensor_confirmations(ids, sensor="camera", timestamp=timestamp)
        self.last_tracked_candidates = self.tracker.refresh_output_quality(
            self.last_tracked_candidates, timestamp=timestamp)
        self._refresh_quality_stats()

    def fuse(self, lidar_points, radar_detections, timestamp=None, frame_id=None):
        now = time.time() if timestamp is None else float(timestamp)
        c = self.config
        previous_tracks = [dict(x) for x in self.last_tracked_candidates]
        clean_points, ground_removed = self._remove_ground_points(lidar_points)
        raw = self._cluster(clean_points, c)
        filtered = [x for x in raw if not self._looks_like_pole(x.get("extent", [0, 0, 0]))]
        clusters = merge_lidar_clusters(
            filtered, c.get("cluster_merge_gap", 1.4),
            c.get("merged_vehicle_max_length", 14.), c.get("merged_vehicle_max_width", 4.2),
            c.get("merged_vehicle_max_height", 4.2), c.get("near_merge_range", 30.0),
            c.get("near_merge_gap", .65), c.get("near_merged_vehicle_max_length", 7.5),
            c.get("near_merged_vehicle_max_width", 3.2)) \
            if c.get("cluster_merge_enabled", True) else filtered
        sparse_rescues = track_guided_sparse_rescue(
            clean_points, previous_tracks, self.world_transform, clusters, c)
        if sparse_rescues:
            clusters = list(clusters) + list(sparse_rescues)

        world_clusters, accepted, roi_rejections = [], [], []
        roi_rescued = 0
        for i in clusters:
            wx, wy, wz = self._to_world(i["x"], i["y"], i["z"])
            e = i.get("extent", [0, 0, 0])
            item = {"x": wx, "y": wy, "z": wz, "confidence": .72,
                    "sources": ["lidar"], "point_count": i.get("point_count", 0),
                    "extent": e, "cluster_mode": i.get("cluster_mode", "3d"),
                    "scale_votes": int(i.get("scale_votes", 1)),
                    "scale_modes": list(i.get("scale_modes", [i.get("cluster_mode", "3d")])),
                    "sparse_rescued": bool(i.get("sparse_rescued", False)),
                    "rescue_track_id": i.get("rescue_track_id"),
                    "rescue_track_hits": int(i.get("rescue_track_hits", 0))}
            # V0.6.12.5: keep far-geometry evidence through the ROI boundary.
            # The adaptive corridor consumes perception metadata only; CARLA
            # actors and evaluation truth are never exposed to it.
            for key in ("far_geometry_built", "far_geometry_quality_v2",
                        "far_geometry_temporal_supported", "current_point_count",
                        "temporal_point_count", "oriented_yaw",
                        "oriented_extent", "axis_aligned_extent",
                        "far_geometry_recovered", "recovery_fragment_count"):
                if key in i:
                    item[key] = i.get(key)
            item = self._mark_recovery_track_support(item, previous_tracks)
            world_clusters.append(dict(item))
            ok, reason, details = self._validate_candidate(wx, wy, wz, e, item)
            if not ok:
                rej = dict(item)
                rej["reason"] = reason
                rej["details"] = details
                roi_rejections.append(rej)
                continue
            item["roi_reason"] = reason
            item["roi_details"] = details
            if details.get("geometry_rescued", False):
                roi_rescued += 1
            accepted.append(item)

        self.last_geometry_world = [dict(x) for x in world_clusters]
        self.last_roi_rejections = roi_rejections
        self.last_roi_candidates = [dict(x) for x in accepted]
        scored, score_rejections = self._score_candidates(accepted)
        self.last_scored_candidates = [dict(x) for x in scored]
        self.last_score_rejections = [dict(x) for x in score_rejections]

        assoc, radar_world_count, radar_matched = self._associate_radar_world(scored, radar_detections)
        quality_pass, quality_rejections = self._gate_recovery_candidates(assoc)
        self.last_recovery_quality_candidates = [dict(x) for x in quality_pass
                                                 if x.get("far_geometry_recovered", False)]
        self.last_recovery_quality_rejections = [dict(x) for x in quality_rejections]
        roi = []
        for item in quality_pass:
            if item.get("radar_radial_velocity") is not None:
                item["confidence"] = .90
                item["sources"] = ["lidar", "radar"]
            roi.append(item)

        if c.get("background_filter_enabled", False):
            dyn = self.background.update_and_filter(roi, now)
            background_ready = self.background.ready
            background_remaining = self.background.remaining_seconds(now)
            background_cells = len(self.background.static_cells)
        else:
            dyn = [dict(x) for x in roi]
            background_ready = True
            background_remaining = 0.0
            background_cells = 0

        admitted, admission_rejections, admission_stats = self._gate_far_new_tracks(
            dyn, previous_tracks, now, frame_id=frame_id)
        self.last_far_admission_candidates = [dict(x) for x in admitted]
        self.last_far_admission_rejections = [dict(x) for x in admission_rejections]
        self.last_dynamic_candidates = [dict(x) for x in admitted]
        tracked = self.tracker.update(admitted, now)
        self.last_tracked_candidates = [dict(x) for x in tracked]
        objs = [DetectedObject(i["id"], i["x"], i["y"], i["z"], vx=i["vx"], vy=i["vy"],
                               object_type="unknown", confidence=i["confidence"],
                               sources=i["sources"]) for i in tracked]
        nearest = [x.get("radar_nearest_xy") for x in roi if x.get("radar_nearest_xy") is not None]
        reasons = defaultdict(int)
        for r in roi_rejections:
            reasons[r.get("reason", "rejected")] += 1
        score_values = [float(x.get("candidate_score", 1.0)) for x in scored
                        if not x.get("candidate_score_bypass", False)]
        sparse_roi = sum(1 for x in accepted if x.get("sparse_rescued", False))
        sparse_score = sum(1 for x in scored if x.get("sparse_rescued", False))
        sparse_dynamic = sum(1 for x in admitted if x.get("sparse_rescued", False))
        ts = dict(getattr(self.tracker, "last_stats", {}) or {})
        self.last_stats = {
            "lidar_points": 0 if lidar_points is None else len(lidar_points),
            "ground_removed_points": ground_removed,
            "lidar_points_after_ground": 0 if clean_points is None else len(clean_points),
            "raw_lidar_clusters": len(raw), "geometry_clusters": len(filtered),
            "lidar_clusters": len(clusters), "world_geometry_candidates": len(world_clusters),
            "sparse_rescue_candidates": len(sparse_rescues),
            "sparse_rescue_roi": sparse_roi, "sparse_rescue_score": sparse_score,
            "sparse_rescue_dynamic": sparse_dynamic,
            "roi_candidates": len(accepted), "roi_rejected": len(roi_rejections),
            "roi_rescued": roi_rescued, "roi_rejection_reasons": dict(reasons),
            "scored_candidates": len(scored), "score_rejected": len(score_rejections),
            "candidate_scoring_enabled": bool(c.get("candidate_scoring_enabled", False)),
            "candidate_score_avg": (sum(score_values) / len(score_values) if score_values else None),
            "recovery_quality_pass": len(self.last_recovery_quality_candidates),
            "recovery_quality_rejected": len(self.last_recovery_quality_rejections),
            "background_candidates": len(admitted),
            "background_pre_admission_candidates": len(dyn),
            "background_rejected": max(0, len(roi) - len(dyn)),
            "background_ready": background_ready, "background_remaining": background_remaining,
            "background_cells": background_cells,
            "background_filter_enabled": bool(c.get("background_filter_enabled", False)),
            "far_admission_pending": int(admission_stats.get("pending", 0)),
            "far_admission_held": int(admission_stats.get("held", 0)),
            "far_admission_confirmed": int(admission_stats.get("confirmed", 0)),
            "far_admission_expired": int(admission_stats.get("expired", 0)),
            "far_admission_sensor_bypass": int(admission_stats.get("sensor_bypass", 0)),
            "far_admission_strong_bypass": int(admission_stats.get("strong_bypass", 0)),
            "far_admission_track_bypass": int(admission_stats.get("track_bypass", 0)),
            "range_adaptive_clustering": bool(c.get("range_adaptive_clustering", False)),
            "radar_detections": 0 if not radar_detections else len(radar_detections),
            "radar_world_points": radar_world_count, "radar_matched_objects": radar_matched,
            "radar_nearest_min": min(nearest) if nearest else None,
            "tracked_objects": len(objs), "track_new": int(ts.get("new", 0)),
            "track_update": int(ts.get("update", 0)), "track_coast": int(ts.get("coast", 0)),
            "track_drop": int(ts.get("drop", 0)), "track_suppress": int(ts.get("suppress", 0)),
            "track_low_hit_keep": int(ts.get("low_hit_keep", 0)),
            "track_low_new_drop": int(ts.get("low_new_drop", 0)),
            "track_quality_active": int(ts.get("quality_active", 0)),
            "track_quality_high": int(ts.get("quality_high", 0)),
            "track_quality_medium": int(ts.get("quality_medium", 0)),
            "track_quality_low": int(ts.get("quality_low", 0)),
            "track_quality_avg": float(ts.get("quality_avg", 0.0))
        }
        return ObjectList(self.station_id, objs, timestamp=now)
