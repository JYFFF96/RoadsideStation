from __future__ import print_function
import math
import time
import statistics
from collections import defaultdict

from .models import DetectedObject, ObjectList
from .perception import voxel_cluster_lidar, adaptive_voxel_cluster_lidar, merge_lidar_clusters
from .sparse_geometry_rescue import track_guided_sparse_rescue
from .road_object_geometry_recovery import RoadObjectGeometryRecovery
from .radar_initiation import NearRadarTrackInitiator
from .tracking import NearestTracker
from .selected_delayed_risk import selected_delayed_risk_gate_passes


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
        self.background = PersistentStaticFilter(
            calibration_seconds=config.get("background_calibration_seconds",6.0),
            cell_size=config.get("background_cell_size",1.0),
            occupancy_ratio=config.get("background_occupancy_ratio",.45),
            moving_radar_speed=config.get("background_moving_radar_speed",1.2),
            neighbor_radius_cells=config.get("background_neighbor_radius_cells",2))
        self.road_object_recovery = RoadObjectGeometryRecovery()
        self.radar_initiator = NearRadarTrackInitiator(config)
        self.world_transform = None
        self.radar_matrix = None
        self.radar_origin = None
        self.ground_reference_z = None
        self.candidate_validator = None
        self.last_stats = {}
        self.last_geometry_world = []
        self.last_road_object_recovery_candidates = []
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
        self.last_far_admission_expired_candidates = []
        self._far_admission_pending = []
        self._far_admission_last_frame = None
        self._far_admission_sequence = 0
        self.last_selected_track_admission_candidates = []
        self.last_selected_track_admission_rejections = []
        self.last_selected_track_admission_expired_candidates = []
        self._selected_track_admission_pending = []
        self._selected_track_admission_last_frame = None
        self._selected_track_admission_sequence = 0
        self._selected_track_admission_pending_sequence = 0
        self._selected_delayed_reappearance_pending = {}
        self.last_selected_delayed_reappearance_candidates = {}
        self.last_selected_delayed_reappearance_stats = {}
        self.last_selected_delayed_risk_shadow_candidates = []
        self.last_radar_initiation_candidates = []
        self.last_radar_seed_bridge_shadow_candidates = {}

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

    def road_object_recovery_diagnostics_world(self):
        """Transform the latest scalar recovery diagnostics for evaluation only."""
        raw=[]
        for point in self.road_object_recovery.last_input_points:
            x,y,z=self._to_world(float(point[0]),float(point[1]),float(point[2]))
            raw.append({"x":x,"y":y,"z":z})
        stages={}
        for name,items in (self.road_object_recovery.last_stage_outputs or {}).items():
            world=[]
            for item in items or []:
                candidate=dict(item);x,y,z=self._to_world(item["x"],item["y"],item["z"])
                candidate["sensor_range"]=math.hypot(float(item["x"]),float(item["y"]))
                candidate.update({"x":x,"y":y,"z":z});world.append(candidate)
            stages[name]=world
        return {"input_points":raw,"stages":stages}

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
                "los_x": dx / n, "los_y": dy / n, "los_z": dz / n,
                "sensor_range": math.hypot(dx, dy), "sensor_range_3d": n}

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
        return out, points, matched

    def _validate_radar_initiation_candidate(self, item):
        ok, unused_reason, unused_details = self._validate_candidate(
            item["x"], item["y"], item["z"], item.get("extent", [0, 0, 0]), item)
        return ok

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
        compact = (.18 <= hl <= float(self.config.get("multiclass_compact_max_length", 1.20)) and
                   .08 <= hs <= float(self.config.get("multiclass_compact_max_width", 1.00)) and
                   .30 <= h <= float(self.config.get("multiclass_compact_max_height", 2.40)))
        if self.config.get("multiclass_compact_geometry_enabled", True) and compact:
            score += float(self.config.get("multiclass_compact_score_bonus", 0.14))
            item["multiclass_compact_geometry"] = True
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
            if (item.get("road_object_selected_enforced", False) and
                    (c.get("road_object_selected_admission_score_shadow", False) or
                     c.get("road_object_selected_admission_score_enforcing", False))):
                # V0.6.12.8.2.2.20: expose the normal sensor-only geometry
                # score below the generic 50m scoring boundary. This is
                # diagnostic metadata only; the production decision below is
                # deliberately unchanged.
                item["selected_admission_shadow_score"] = self._candidate_score(item)
                item["selected_admission_shadow_bypass"] = rng < min_range
                selected_threshold=float(c.get("road_object_selected_admission_score_threshold",.20))
                selected_enforcing=bool(c.get("road_object_selected_admission_score_enforcing",False))
                item["selected_admission_score_threshold"]=selected_threshold
                item["selected_admission_score_enforcing"]=selected_enforcing
                if (selected_enforcing and
                        item["selected_admission_shadow_score"]<selected_threshold):
                    item["candidate_score"]=item["selected_admission_shadow_score"]
                    item["candidate_score_threshold"]=selected_threshold
                    item["candidate_score_bypass"]=False
                    item["reason"]="selected_admission_score"
                    rejected.append(item)
                    continue
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
            self.last_far_admission_expired_candidates = []
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

        expired_items = []
        pending = []
        for p in self._far_admission_pending:
            age = max(0.0, now - float(p.get("last_time", now)))
            if not new_frame or age <= ttl:
                pending.append(p)
            else:
                item = dict(p)
                item["far_track_admission_reason"] = "expired"
                item["far_track_admission_time_gap"] = age
                expired_items.append(item)
        self._far_admission_pending = pending
        self.last_far_admission_expired_candidates = expired_items

        kept, rejected, used = [], [], set()
        stats = {"pending": 0, "held": 0, "confirmed": 0,
                 "expired": len(expired_items), "sensor_bypass": 0,
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
                entry = dict(item)
                entry.update({"x": float(item["x"]), "y": float(item["y"]),
                              "z": float(item.get("z", 0.0)), "hits": 1,
                              "last_time": now, "last_frame": token})
                self._far_admission_pending.append(entry)
                used.add(len(self._far_admission_pending) - 1)
                hits = 1
            else:
                match_distance, index, entry = best
                used.add(index)
                previous_time = float(entry.get("last_time", now))
                previous_frame = entry.get("last_frame")
                item["far_track_admission_match_distance"] = float(match_distance)
                item["far_track_admission_time_gap"] = max(0.0, now - previous_time)
                try:
                    frame_gap = float(token) - float(previous_frame)
                    if frame_gap >= 0.0:
                        item["far_track_admission_frame_gap"] = frame_gap
                except (TypeError, ValueError):
                    pass
                if new_frame and entry.get("last_frame") != token:
                    next_hits = int(entry.get("hits", 1)) + 1
                    entry.update(item)
                    entry["hits"] = next_hits
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

    def _far_admission_tracker_candidates(self, dynamic_items, admitted_items):
        """Select Tracker input while keeping admission diagnostics observer-only."""
        shadow = (self.config.get("far_track_admission_enabled", True) and
                  self.config.get("far_track_admission_shadow_mode", False))
        source = dynamic_items if shadow else admitted_items
        return [dict(x) for x in source]

    def _selected_track_support(self, item, previous_tracks):
        gate = float(self.config.get("selected_track_admission_track_gate",
                                     self.config.get("track_match_distance", 4.0)))
        best = None
        for track in previous_tracks or []:
            # Shadow-held Selected measurements still create real Tracker state.
            # Such state cannot be used as counterfactual existing-track proof;
            # require at least one normal (non-Selected) geometry update.
            if int(track.get("track_non_selected_hits", 0)) <= 0:
                continue
            d = math.hypot(float(track.get("x", 0.0)) - float(item["x"]),
                           float(track.get("y", 0.0)) - float(item["y"]))
            if d <= gate and (best is None or d < best):
                best = d
        return best

    def _gate_selected_new_tracks(self, items, previous_tracks, now, frame_id=None):
        """Profile repeat evidence for Selected candidates that would start tracks.

        Default Shadow mode computes sensor-only decisions while preserving the
        original Tracker input.
        """
        if not self.config.get("selected_track_admission_enabled", False):
            self.last_selected_track_admission_expired_candidates = []
            return [dict(x) for x in items], [], {
                "pending": 0, "held": 0, "confirmed": 0, "expired": 0,
                "sensor_bypass": 0, "track_bypass": 0}
        required = max(2, int(self.config.get(
            "selected_track_admission_required_frames", 2)))
        match_gate = float(self.config.get(
            "selected_track_admission_match_gate", 2.5))
        ttl = float(self.config.get("selected_track_admission_ttl", 0.5))
        if frame_id is None:
            self._selected_track_admission_sequence += 1
            token = self._selected_track_admission_sequence
        else:
            token = frame_id
        new_frame = token != self._selected_track_admission_last_frame
        if new_frame:
            self._selected_track_admission_last_frame = token
        self._profile_selected_delayed_reappearance(
            items, previous_tracks, now, token, new_frame)

        pending, expired = [], []
        for old in self._selected_track_admission_pending:
            age = max(0.0, now - float(old.get("last_time", now)))
            if not new_frame or age <= ttl:
                pending.append(old)
            else:
                item = dict(old)
                item["selected_track_admission_reason"] = "expired"
                item["selected_track_admission_time_gap"] = age
                expired.append(item)
        self._selected_track_admission_pending = pending
        self.last_selected_track_admission_expired_candidates = expired

        admitted, rejected, used = [], [], set()
        stats = {"pending": 0, "held": 0, "confirmed": 0,
                 "expired": len(expired), "sensor_bypass": 0,
                 "track_bypass": 0}
        for src in items or []:
            item = dict(src)
            if not item.get("road_object_selected_enforced", False):
                admitted.append(item)
                continue
            if item.get("radar_radial_velocity") is not None:
                item["selected_track_admission_reason"] = "sensor"
                item["selected_track_admission_would_pass"] = True
                stats["sensor_bypass"] += 1
                admitted.append(item)
                continue
            best = None
            for index, old in enumerate(self._selected_track_admission_pending):
                if index in used:
                    continue
                d = math.hypot(float(old["x"]) - float(item["x"]),
                               float(old["y"]) - float(item["y"]))
                if d <= match_gate and (best is None or d < best[0]):
                    best = (d, index, old)
            if best is None:
                track_distance = self._selected_track_support(item, previous_tracks)
                if track_distance is not None:
                    item["selected_track_admission_reason"] = "existing_track"
                    item["selected_track_admission_track_distance"] = track_distance
                    item["selected_track_admission_would_pass"] = True
                    stats["track_bypass"] += 1
                    admitted.append(item)
                    continue
                self._selected_track_admission_pending_sequence += 1
                pending_id = self._selected_track_admission_pending_sequence
                item["selected_track_admission_pending_id"] = pending_id
                entry = dict(item)
                entry.update({"x": float(item["x"]), "y": float(item["y"]),
                              "z": float(item.get("z", 0.0)), "hits": 1,
                              "last_time": now, "last_frame": token})
                self._selected_track_admission_pending.append(entry)
                used.add(len(self._selected_track_admission_pending) - 1)
                hits = 1
            else:
                distance, index, entry = best
                used.add(index)
                item["selected_track_admission_pending_id"] = entry.get(
                    "selected_track_admission_pending_id")
                previous_time = float(entry.get("last_time", now))
                previous_frame = entry.get("last_frame")
                item["selected_track_admission_match_distance"] = distance
                item["selected_track_admission_time_gap"] = max(0.0, now - previous_time)
                try:
                    gap = float(token) - float(previous_frame)
                    if gap >= 0.0:
                        item["selected_track_admission_frame_gap"] = gap
                except (TypeError, ValueError):
                    pass
                if new_frame and previous_frame != token:
                    entry.update(item)
                    entry["hits"] = int(entry.get("hits", 1)) + 1
                    entry["last_time"] = now
                    entry["last_frame"] = token
                hits = int(entry.get("hits", 1))
            item["selected_track_admission_hits"] = hits
            if hits >= required:
                item["selected_track_admission_reason"] = "repeat"
                item["selected_track_admission_would_pass"] = True
                stats["confirmed"] += 1
                admitted.append(item)
                if best is not None:
                    self._selected_track_admission_pending[best[1]]["confirmed"] = True
            else:
                item["selected_track_admission_reason"] = "pending"
                item["selected_track_admission_would_pass"] = False
                item["reason"] = "selected_track_admission"
                stats["held"] += 1
                rejected.append(item)
        self._selected_track_admission_pending = [
            p for p in self._selected_track_admission_pending
            if not p.get("confirmed", False)]
        stats["pending"] = len(self._selected_track_admission_pending)
        return admitted, rejected, stats

    def _selected_delayed_reappearance_rules(self):
        rules = []
        for index, item in enumerate(self.config.get(
                "selected_track_admission_delayed_reappearance_ablations", []) or []):
            if not isinstance(item, dict):continue
            rules.append({"name":str(item.get("name", "rule_%d" % index)),
                          "ttl":float(item.get("ttl", 1.0)),
                          "match_gate":float(item.get("match_gate", 2.5))})
        return rules

    def _profile_selected_delayed_reappearance(self, items, previous_tracks,
                                                now, token, new_frame):
        """Run longer LiDAR reappearance windows in parallel, without filtering."""
        outputs = {};stats = {}
        self.last_selected_delayed_risk_shadow_candidates = []
        rules = self._selected_delayed_reappearance_rules()
        if (not self.config.get(
                "selected_track_admission_delayed_reappearance_shadow", False)
                or not rules or not new_frame):
            self.last_selected_delayed_reappearance_candidates = outputs
            self.last_selected_delayed_reappearance_stats = stats
            return
        eligible = []
        for source in items or []:
            if not source.get("road_object_selected_enforced", False):continue
            if source.get("radar_radial_velocity") is not None:continue
            if self._selected_track_support(source, previous_tracks) is not None:continue
            eligible.append(dict(source))
        for rule in rules:
            name = rule["name"];ttl = max(0.0, rule["ttl"])
            gate = max(0.0, rule["match_gate"])
            active = [];expired = 0
            for old in self._selected_delayed_reappearance_pending.get(name, []):
                if max(0.0, now - float(old.get("last_time", now))) <= ttl:
                    active.append(old)
                else:expired += 1
            confirmed = [];used = set()
            for source in eligible:
                best = None
                for index, old in enumerate(active):
                    if index in used:continue
                    distance = math.hypot(float(old["x"]) - float(source["x"]),
                                          float(old["y"]) - float(source["y"]))
                    if distance <= gate and (best is None or distance < best[0]):
                        best = (distance, index, old)
                if best is None:
                    entry = dict(source)
                    entry.update({"x":float(source["x"]), "y":float(source["y"]),
                                  "last_time":now, "last_frame":token})
                    active.append(entry);used.add(len(active) - 1)
                    continue
                distance, index, old = best;used.add(index)
                event = dict(source)
                event["selected_delayed_reappearance_rule"] = name
                event["selected_delayed_reappearance_ttl"] = ttl
                event["selected_delayed_reappearance_match_gate"] = gate
                event["selected_delayed_reappearance_match_distance"] = distance
                event["selected_delayed_reappearance_time_gap"] = max(
                    0.0, now - float(old.get("last_time", now)))
                event["selected_delayed_reappearance_origin"] = dict(old)
                confirmed.append(event);old["confirmed"] = True
            active = [item for item in active if not item.get("confirmed", False)]
            self._selected_delayed_reappearance_pending[name] = active
            outputs[name] = confirmed
            stats[name] = {"eligible":len(eligible), "confirmed":len(confirmed),
                           "expired":expired, "pending":len(active),
                           "ttl":ttl, "match_gate":gate}
        selected_name = str(self.config.get(
            "selected_delayed_reappearance_selected_rule", ""))
        selected_gate = self.config.get(
            "selected_delayed_reappearance_selected_risk_gate", {}) or {}
        if selected_name in outputs and isinstance(selected_gate, dict):
            selected = []
            for source in outputs[selected_name]:
                item = dict(source)
                keep = selected_delayed_risk_gate_passes(item, selected_gate)
                item["selected_delayed_risk_shadow_would_keep"] = bool(keep)
                selected.append(item)
            kept = sum(1 for item in selected
                       if item["selected_delayed_risk_shadow_would_keep"])
            stats[selected_name]["selected_risk_shadow"] = True
            stats[selected_name]["would_keep"] = kept
            stats[selected_name]["would_reject"] = len(selected) - kept
            stats[selected_name]["risk_gate"] = dict(selected_gate)
            self.last_selected_delayed_risk_shadow_candidates = selected
        self.last_selected_delayed_reappearance_candidates = outputs
        self.last_selected_delayed_reappearance_stats = stats

    def _selected_admission_tracker_candidates(self, original, admitted):
        shadow = (self.config.get("selected_track_admission_enabled", False) and
                  self.config.get("selected_track_admission_shadow_mode", True))
        return [dict(x) for x in (original if shadow else admitted)]

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

    def fuse(self, lidar_points, radar_detections, timestamp=None, frame_id=None,
             radar_frame_id=None):
        now = time.time() if timestamp is None else float(timestamp)
        c = self.config
        previous_tracks = [dict(x) for x in self.last_tracked_candidates]
        clean_points, ground_removed = self._remove_ground_points(lidar_points)
        raw = self._cluster(clean_points, c)
        filtered = ([x for x in raw if not self._looks_like_pole(x.get("extent", [0, 0, 0]))]
                    if c.get("pole_filter_enabled", False) else [dict(x) for x in raw])
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
        ground_cut_local=None
        if self.ground_reference_z is not None and self.world_transform is not None:
            ground_cut_local=float(self.ground_reference_z)+float(c.get("ground_clearance",.30))-float(self.world_transform["z"])
        road_object_rescues=self.road_object_recovery.update(
            lidar_points,clusters,ground_cut_local,c,frame_id=frame_id)
        road_object_shadow=bool(c.get("road_object_recovery_shadow_mode",True))
        road_object_world=[]
        for item in road_object_rescues:
            wx,wy,wz=self._to_world(item["x"],item["y"],item["z"])
            candidate=dict(item);candidate.update({"x":wx,"y":wy,"z":wz,
                                                   "confidence":.72,"sources":["lidar"]})
            road_object_world.append(candidate)
        self.last_road_object_recovery_candidates=[dict(x) for x in road_object_world]
        if road_object_rescues and not road_object_shadow:
            clusters=list(clusters)+list(road_object_rescues)

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
                        "far_geometry_recovered", "recovery_fragment_count",
                        "road_object_recovered", "road_object_temporal_hits",
                        "road_object_selected_enforced", "road_object_selected_policy",
                        "adaptive_hybrid_source", "adaptive_hybrid_gate_keep",
                        "adaptive_hybrid_temporal_rescue",
                        "adaptive_hybrid_geometry_gate_keep"):
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

        assoc, radar_world_points, radar_matched = self._associate_radar_world(
            scored, radar_detections)
        radar_world_count = len(radar_world_points)
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

        if background_ready:
            radar_initiated = self.radar_initiator.update(
                radar_world_points, list(roi) + list(previous_tracks), now,
                frame_id=radar_frame_id,
                validator=self._validate_radar_initiation_candidate)
        else:
            radar_initiated = []
        self.last_radar_initiation_candidates = [dict(x) for x in radar_initiated]
        self.last_radar_seed_bridge_shadow_candidates = dict(
            (key, [dict(item) for item in value])
            for key, value in
            self.radar_initiator.last_seed_bridge_shadow_candidates.items())
        if radar_initiated:
            dyn = list(dyn) + list(radar_initiated)

        admitted, admission_rejections, admission_stats = self._gate_far_new_tracks(
            dyn, previous_tracks, now, frame_id=frame_id)
        tracker_candidates = self._far_admission_tracker_candidates(dyn, admitted)
        admission_shadow = bool(c.get("far_track_admission_enabled", True) and
                                c.get("far_track_admission_shadow_mode", False))
        self.last_far_admission_candidates = [dict(x) for x in admitted]
        self.last_far_admission_rejections = [dict(x) for x in admission_rejections]
        selected_admitted, selected_rejections, selected_admission_stats = \
            self._gate_selected_new_tracks(
                tracker_candidates, previous_tracks, now, frame_id=frame_id)
        selected_admission_shadow = bool(
            c.get("selected_track_admission_enabled", False) and
            c.get("selected_track_admission_shadow_mode", True))
        self.last_selected_track_admission_candidates = [
            dict(x) for x in selected_admitted
            if x.get("road_object_selected_enforced", False)]
        self.last_selected_track_admission_rejections = [
            dict(x) for x in selected_rejections]
        tracker_candidates = self._selected_admission_tracker_candidates(
            tracker_candidates, selected_admitted)
        self.last_dynamic_candidates = [dict(x) for x in tracker_candidates]
        tracked = self.tracker.update(tracker_candidates, now)
        self.last_tracked_candidates = [dict(x) for x in tracked]
        objs = [DetectedObject(i["id"], i["x"], i["y"], i["z"], vx=i["vx"], vy=i["vy"],
                               object_type="unknown_obstacle", confidence=i["confidence"],
                               sources=i["sources"]) for i in tracked]
        nearest = [x.get("radar_nearest_xy") for x in roi if x.get("radar_nearest_xy") is not None]
        reasons = defaultdict(int)
        for r in roi_rejections:
            reasons[r.get("reason", "rejected")] += 1
        score_values = [float(x.get("candidate_score", 1.0)) for x in scored
                        if not x.get("candidate_score_bypass", False)]
        sparse_roi = sum(1 for x in accepted if x.get("sparse_rescued", False))
        sparse_score = sum(1 for x in scored if x.get("sparse_rescued", False))
        sparse_dynamic = sum(1 for x in tracker_candidates if x.get("sparse_rescued", False))
        road_stats=dict(self.road_object_recovery.last_stats or {})
        radar_init_stats=dict(self.radar_initiator.last_stats or {})
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
            "road_object_recovery_input": int(road_stats.get("input_points",0)),
            "road_object_recovery_components": int(road_stats.get("components",0)),
            "road_object_recovery_shape_pass": int(road_stats.get("shape_pass",0)),
            "road_object_recovery_pending": int(road_stats.get("pending",0)),
            "road_object_recovery_temporal_pass": int(road_stats.get("temporal_pass",0)),
            "road_object_recovery_dedupe": int(road_stats.get("dedupe",0)),
            "road_object_recovery_cap_reject": int(road_stats.get("cap_reject",0)),
            "road_object_recovery_built": int(road_stats.get("built",0)),
            "road_object_recovery_balanced_built": int(road_stats.get("balanced_built",0)),
            "road_object_recovery_balanced_bands": dict(road_stats.get("balanced_band_counts",{}) or {}),
            "road_object_recovery_adaptive_history_frames": int(road_stats.get("adaptive_history_frames",0)),
            "road_object_recovery_adaptive_points": int(road_stats.get("adaptive_points",0)),
            "road_object_recovery_adaptive_components": int(road_stats.get("adaptive_components",0)),
            "road_object_recovery_adaptive_shape_pass": int(road_stats.get("adaptive_shape_pass",0)),
            "road_object_recovery_adaptive_temporal_pass": int(road_stats.get("adaptive_temporal_pass",0)),
            "road_object_recovery_adaptive_dedupe": int(road_stats.get("adaptive_dedupe",0)),
            "road_object_recovery_adaptive_built": int(road_stats.get("adaptive_built",0)),
            "road_object_recovery_adaptive_bands": dict(road_stats.get("adaptive_band_counts",{}) or {}),
            "road_object_recovery_adaptive_ranked_built": int(road_stats.get("adaptive_ranked_built",0)),
            "road_object_recovery_adaptive_ranked_bands": dict(road_stats.get("adaptive_ranked_band_counts",{}) or {}),
            "road_object_recovery_adaptive_stratified_built": int(road_stats.get("adaptive_stratified_built",0)),
            "road_object_recovery_adaptive_stratified_bands": dict(road_stats.get("adaptive_stratified_band_counts",{}) or {}),
            "road_object_recovery_adaptive_stratified_heights": dict(road_stats.get("adaptive_stratified_height_counts",{}) or {}),
            "road_object_recovery_adaptive_hybrid_built": int(road_stats.get("adaptive_hybrid_built",0)),
            "road_object_recovery_adaptive_hybrid_bands": dict(road_stats.get("adaptive_hybrid_band_counts",{}) or {}),
            "road_object_recovery_adaptive_hybrid_sources": dict(road_stats.get("adaptive_hybrid_source_counts",{}) or {}),
            "road_object_recovery_adaptive_hybrid_gate_kept": int(road_stats.get("adaptive_hybrid_gate_kept",0)),
            "road_object_recovery_adaptive_hybrid_gate_rejected": int(road_stats.get("adaptive_hybrid_gate_rejected",0)),
            "road_object_recovery_adaptive_hybrid_gate_reasons": dict(road_stats.get("adaptive_hybrid_gate_reasons",{}) or {}),
            "road_object_recovery_adaptive_hybrid_rescue_kept": int(road_stats.get("adaptive_hybrid_rescue_kept",0)),
            "road_object_recovery_adaptive_hybrid_rescued": int(road_stats.get("adaptive_hybrid_rescued",0)),
            "road_object_recovery_adaptive_hybrid_rescue_sources": dict(road_stats.get("adaptive_hybrid_rescue_sources",{}) or {}),
            "road_object_recovery_adaptive_hybrid_geometry_gate_kept": int(road_stats.get("adaptive_hybrid_geometry_gate_kept",0)),
            "road_object_recovery_adaptive_hybrid_geometry_gate_rejected": int(road_stats.get("adaptive_hybrid_geometry_gate_rejected",0)),
            "road_object_recovery_adaptive_hybrid_geometry_gate_reasons": dict(road_stats.get("adaptive_hybrid_geometry_gate_reasons",{}) or {}),
            "road_object_recovery_selected_output_built": int(road_stats.get("selected_output_built",0)),
            "road_object_recovery_selected_output_policy": str(road_stats.get("selected_output_policy","disabled")),
            "road_object_recovery_selected_output_enforcing": bool(road_stats.get("selected_output_enforcing",False)),
            "road_object_recovery_active_output_built": int(road_stats.get("active_output_built",0)),
            "road_object_recovery_active_output_policy": str(road_stats.get("active_output_policy","baseline")),
            "road_object_recovery_shadow_mode": road_object_shadow,
            "roi_candidates": len(accepted), "roi_rejected": len(roi_rejections),
            "roi_rescued": roi_rescued, "roi_rejection_reasons": dict(reasons),
            "scored_candidates": len(scored), "score_rejected": len(score_rejections),
            "selected_admission_score_enforcing": bool(c.get("road_object_selected_admission_score_enforcing",False)),
            "selected_admission_score_threshold": float(c.get("road_object_selected_admission_score_threshold",.20)),
            "selected_admission_score_kept": sum(1 for x in scored if x.get("road_object_selected_enforced",False)),
            "selected_admission_score_rejected": sum(1 for x in score_rejections if x.get("reason")=="selected_admission_score"),
            "candidate_scoring_enabled": bool(c.get("candidate_scoring_enabled", False)),
            "candidate_score_avg": (sum(score_values) / len(score_values) if score_values else None),
            "recovery_quality_pass": len(self.last_recovery_quality_candidates),
            "recovery_quality_rejected": len(self.last_recovery_quality_rejections),
            "background_candidates": len(tracker_candidates),
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
            "far_admission_shadow_mode": admission_shadow,
            "far_admission_tracker_input": len(tracker_candidates),
            "selected_track_admission_pending": int(selected_admission_stats.get("pending", 0)),
            "selected_track_admission_held": int(selected_admission_stats.get("held", 0)),
            "selected_track_admission_confirmed": int(selected_admission_stats.get("confirmed", 0)),
            "selected_track_admission_expired": int(selected_admission_stats.get("expired", 0)),
            "selected_track_admission_sensor_bypass": int(selected_admission_stats.get("sensor_bypass", 0)),
            "selected_track_admission_track_bypass": int(selected_admission_stats.get("track_bypass", 0)),
            "selected_track_admission_shadow_mode": selected_admission_shadow,
            "selected_track_admission_tracker_input": len(tracker_candidates),
            "range_adaptive_clustering": bool(c.get("range_adaptive_clustering", False)),
            "radar_detections": 0 if not radar_detections else len(radar_detections),
            "radar_world_points": radar_world_count, "radar_matched_objects": radar_matched,
            "radar_initiation_enabled": bool(self.radar_initiator.enabled),
            "radar_initiation_shadow_mode": bool(self.radar_initiator.shadow_mode),
            "radar_initiation_range_points": int(radar_init_stats.get("range_points",0)),
            "radar_initiation_components": int(radar_init_stats.get("components",0)),
            "radar_initiation_clusters": int(radar_init_stats.get("clusters",0)),
            "radar_initiation_single_point_candidates": int(
                radar_init_stats.get("single_point_candidates",0)),
            "radar_initiation_pending": int(radar_init_stats.get("pending",0)),
            "radar_initiation_confirmed": int(radar_init_stats.get("confirmed",0)),
            "radar_initiation_single_point_confirmed": int(
                radar_init_stats.get("single_point_confirmed",0)),
            "radar_initiation_moving": int(radar_init_stats.get("moving_confirmed",0)),
            "radar_initiation_static_rejected": int(radar_init_stats.get("static_rejected",0)),
            "radar_initiation_dedupe_rejected": int(radar_init_stats.get("dedupe_rejected",0)),
            "radar_initiation_roi_rejected": int(radar_init_stats.get("roi_rejected",0)),
            "radar_initiation_emitted": int(radar_init_stats.get("emitted",0)),
            "radar_initiation_single_point_emitted": int(
                radar_init_stats.get("single_point_emitted",0)),
            "radar_initiation_cumulative": dict(
                radar_init_stats.get("cumulative",{}) or {}),
            "radar_seed_bridge_shadow": dict(
                radar_init_stats.get("seed_bridge_shadow",{}) or {}),
            "radar_seed_to_component_shadow": dict(
                radar_init_stats.get("seed_to_component_shadow",{}) or {}),
            "radar_initiation_speed_p50": radar_init_stats.get("confirmed_abs_speed_p50"),
            "radar_initiation_speed_max": radar_init_stats.get("confirmed_abs_speed_max"),
            "radar_initiation_speed_shadow_counts": dict(
                radar_init_stats.get("speed_shadow_counts",{}) or {}),
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
