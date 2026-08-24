from __future__ import print_function
import math
import time
from collections import deque


class NearestTracker(object):
    """Portable nearest-neighbour tracker with quality-aware persistence.

    The quality model deliberately uses only scalar detection/track evidence so
    the same policy can later be moved to Qt/C++ without CARLA dependencies.
    """

    def __init__(self, max_distance=4.0, max_age=1.5, max_speed=20.0,
                 velocity_alpha=.35, extent_alpha=.25, extent_shrink_alpha=.05,
                 extent_lock_hits=5, radar_velocity_alpha=.35, velocity_window=5,
                 position_alpha=.45, stationary_speed=.35, coast_frames=0,
                 coast_confidence_decay=.88, adaptive_coast_enabled=False,
                 coast_young_frames=1, coast_stable_frames=3, coast_far_frames=4,
                 coast_stable_hits=3, coast_far_range=50.0, coast_low_score=0.55,
                 coast_edge_ratio=0.90):
        self.max_distance = float(max_distance)
        self.max_age = float(max_age)
        self.max_speed = float(max_speed)
        self.velocity_alpha = float(velocity_alpha)
        self.extent_alpha = float(extent_alpha)
        self.extent_shrink_alpha = float(extent_shrink_alpha)
        self.extent_lock_hits = int(extent_lock_hits)
        self.radar_velocity_alpha = float(radar_velocity_alpha)
        self.velocity_window = max(2, int(velocity_window))
        self.position_alpha = float(position_alpha)
        self.stationary_speed = float(stationary_speed)
        self.coast_frames = max(0, int(coast_frames))
        self.coast_confidence_decay = float(coast_confidence_decay)
        self.adaptive_coast_enabled = bool(adaptive_coast_enabled)
        self.coast_young_frames = max(0, int(coast_young_frames))
        self.coast_stable_frames = max(0, int(coast_stable_frames))
        self.coast_far_frames = max(0, int(coast_far_frames))
        self.coast_stable_hits = max(1, int(coast_stable_hits))
        self.coast_far_range = float(coast_far_range)
        self.coast_low_score = float(coast_low_score)
        self.coast_edge_ratio = float(coast_edge_ratio)

        self.quality_enabled = True
        self.quality_high = .72
        self.quality_medium = .50
        self.quality_medium_coast_frames = 2
        self.quality_low_coast_frames = 0
        self.quality_camera_bonus = .12
        self.quality_radar_bonus = .08
        self.quality_sensor_memory = 1.5
        self.quality_coast_penalty = .10
        self.quality_low_min_hits_for_coast = 3

        self._tracks = {}
        self._camera_tombstones = {}
        self.camera_tombstone_shadow_ttl = 5.0
        self._next_id = 1
        self.last_stats = {"new": 0, "update": 0, "coast": 0,
                           "drop": 0, "suppress": 0,
                           "low_hit_keep": 0, "low_new_drop": 0}

    def configure_quality(self, config):
        c = config or {}
        self.quality_enabled = bool(c.get("track_quality_enabled", True))
        self.quality_high = float(c.get("track_quality_high", .72))
        self.quality_medium = float(c.get("track_quality_medium", .50))
        self.quality_medium_coast_frames = max(0, int(c.get("track_quality_medium_coast_frames", 2)))
        self.quality_low_coast_frames = max(0, int(c.get("track_quality_low_coast_frames", 0)))
        self.quality_camera_bonus = float(c.get("track_quality_camera_bonus", .12))
        self.quality_radar_bonus = float(c.get("track_quality_radar_bonus", .08))
        self.quality_sensor_memory = max(.1, float(c.get("track_quality_sensor_memory", 1.5)))
        self.quality_coast_penalty = max(0.0, float(c.get("track_quality_coast_penalty", .10)))
        self.quality_low_min_hits_for_coast = max(1, int(c.get("track_quality_low_min_hits_for_coast", 3)))
        self.camera_tombstone_shadow_ttl = max(.1, float(c.get(
            "camera_tombstone_shadow_ttl", 5.0)))

    def _remember_camera_tombstone(self, tid, track, now):
        if not track or not track.get("camera_ground_origin",False):return
        self._camera_tombstones[str(tid)]={
            "x":float(track.get("x",0.0)),"y":float(track.get("y",0.0)),
            "vx":float(track.get("vx",0.0)),"vy":float(track.get("vy",0.0)),
            "last_seen":float(track.get("timestamp",now)),
            "expired_at":float(now)}

    def _prune_camera_tombstones(self, now):
        stale=[tid for tid,item in self._camera_tombstones.items()
               if now-float(item.get("expired_at",now))>
               self.camera_tombstone_shadow_ttl]
        for tid in stale:self._camera_tombstones.pop(tid,None)

    def _nearest_camera_tombstones(self, det, now):
        best={"frozen":None,"predicted":None}
        for tid,item in self._camera_tombstones.items():
            gap=max(0.0,now-float(item.get("last_seen",now)))
            points={
                "frozen":(float(item["x"]),float(item["y"])),
                "predicted":(float(item["x"])+float(item["vx"])*gap,
                             float(item["y"])+float(item["vy"])*gap)}
            for mode,(px,py) in points.items():
                distance=math.hypot(float(det["x"])-px,float(det["y"])-py)
                if best[mode] is None or distance<best[mode][0]:
                    best[mode]=(distance,tid,gap)
        return best

    def _clamp_velocity(self, vx, vy):
        s = math.hypot(vx, vy)
        if s <= self.max_speed or s < 1e-6:
            return vx, vy
        k = self.max_speed / s
        return vx * k, vy * k

    def _smooth_extent(self, old, cur, hits):
        prev = old.get("extent", cur)
        out = []
        for i in range(3):
            grow = float(cur[i]) >= float(prev[i])
            a = self.extent_alpha if grow or hits < self.extent_lock_hits else self.extent_shrink_alpha
            out.append((1.0 - a) * float(prev[i]) + a * float(cur[i]))
        return out

    def _history_velocity(self, hist):
        if len(hist) < 2:
            return 0.0, 0.0
        first = hist[0]
        last = hist[-1]
        dt = max(1e-3, last[0] - first[0])
        return self._clamp_velocity((last[1] - first[1]) / dt,
                                    (last[2] - first[2]) / dt)

    def _apply_radar_radial(self, vx, vy, det):
        rv = det.get("radar_radial_velocity")
        lx = det.get("radar_los_x")
        ly = det.get("radar_los_y")
        if rv is None or lx is None or ly is None:
            return vx, vy
        norm = math.hypot(float(lx), float(ly))
        if norm < 1e-6:
            return vx, vy
        lx = float(lx) / norm
        ly = float(ly) / norm
        measured = float(rv)
        pred = vx * lx + vy * ly
        err = measured - pred
        a = self.radar_velocity_alpha
        return self._clamp_velocity(vx + a * err * lx, vy + a * err * ly)

    def _legacy_evidence(self, t):
        d = t.get("last_det", {}) or {}
        score = float(d.get("candidate_score", 1.0))
        bypass = bool(d.get("candidate_score_bypass", False))
        details = d.get("roi_details", {}) or {}
        lateral = details.get("lateral")
        allowed = details.get("allowed_lateral")
        edge_ratio = 0.0
        if lateral is not None and allowed not in (None, 0):
            edge_ratio = float(lateral) / max(.01, float(allowed))
        return (score, bypass, edge_ratio,
                bool(details.get("geometry_rescued", False)),
                float(d.get("sensor_range", 0.0)))

    def _stability_update(self, old, extent, vx, vy):
        if old is None:
            return .5, .5
        prev_e = old.get("extent", extent)
        denom = max(.5, sum(abs(float(v)) for v in prev_e))
        extent_delta = sum(abs(float(extent[i]) - float(prev_e[i])) for i in range(3)) / denom
        size_now = max(0.0, 1.0 - min(1.0, extent_delta))
        dv = math.hypot(float(vx) - float(old.get("vx", vx)),
                        float(vy) - float(old.get("vy", vy)))
        vel_now = max(0.0, 1.0 - min(1.0, dv / 4.0))
        size_prev = float(old.get("size_stability", .5))
        vel_prev = float(old.get("velocity_stability", .5))
        return .7 * size_prev + .3 * size_now, .7 * vel_prev + .3 * vel_now

    def _sensor_recent(self, t, key, now):
        ts = t.get(key)
        return ts is not None and now - float(ts) <= self.quality_sensor_memory

    def _quality_value(self, t, now=None):
        if now is None:
            now = time.time()
        score, bypass, edge_ratio, rescued, rng = self._legacy_evidence(t)
        detection = .30 * (1.0 if bypass else max(0.0, min(1.0, score)))
        temporal = .25 * min(1.0, float(t.get("hits", 1)) / 5.0)
        sensor = 0.0
        if self._sensor_recent(t, "last_camera_time", now):
            sensor += self.quality_camera_bonus
        if self._sensor_recent(t, "last_radar_time", now):
            sensor += self.quality_radar_bonus
        sensor = min(.20, sensor)
        stability = .075 * max(0.0, min(1.0, float(t.get("size_stability", .5))))
        stability += .075 * max(0.0, min(1.0, float(t.get("velocity_stability", .5))))
        if rng <= 30.0:
            distance = .10
        elif rng <= 50.0:
            distance = .07
        else:
            distance = .03
        penalty = self.quality_coast_penalty * int(t.get("misses", 0))
        if edge_ratio >= self.coast_edge_ratio:
            penalty += .08
        if rescued:
            penalty += .05
        return max(0.0, min(1.0, detection + temporal + sensor + stability + distance - penalty))

    def _quality_band(self, q):
        if q >= self.quality_high:
            return "high"
        if q >= self.quality_medium:
            return "medium"
        return "low"

    def _base_allowed_coast(self, t):
        if not self.adaptive_coast_enabled:
            return self.coast_frames
        hits = int(t.get("hits", 1))
        score, bypass, edge_ratio, rescued, rng = self._legacy_evidence(t)
        if hits < 2:
            return 0
        allowed = self.coast_young_frames if hits < self.coast_stable_hits else self.coast_stable_frames
        if hits >= self.coast_stable_hits and rng >= self.coast_far_range:
            allowed = self.coast_far_frames
        if (not bypass and score < self.coast_low_score) or edge_ratio >= self.coast_edge_ratio or rescued:
            allowed = min(allowed, 1)
        return max(0, int(allowed))

    def _allowed_coast(self, t, now=None):
        allowed = self._base_allowed_coast(t)
        if not self.quality_enabled:
            return allowed
        q = self._quality_value(t, now)
        band = self._quality_band(q)
        if band == "high":
            return allowed
        if band == "medium":
            return min(allowed, self.quality_medium_coast_frames)
        if int(t.get("hits", 1)) < self.quality_low_min_hits_for_coast:
            return 0
        return min(allowed, self.quality_low_coast_frames)

    def _sensor_string(self, t, now=None):
        if now is None:
            now = time.time()
        s = "L" if int(t.get("lidar_hits", 0)) > 0 else ""
        if self._sensor_recent(t, "last_radar_time", now):
            s += "R"
        if self._sensor_recent(t, "last_camera_time", now):
            s += "C"
        return s or "-"

    def _decorate_quality(self, item, t, now):
        q = self._quality_value(t, now)
        item["track_quality"] = q
        item["track_quality_band"] = self._quality_band(q)
        item["track_sensors"] = self._sensor_string(t, now)
        item["track_lidar_hits"] = int(t.get("lidar_hits", 0))
        item["track_radar_hits"] = int(t.get("radar_confirmations", 0))
        item["track_camera_hits"] = int(t.get("camera_confirmations", 0))
        item["track_camera_ground_origin"] = bool(
            t.get("camera_ground_origin", False))
        item["track_camera_ground_current"] = bool(
            item.get("camera_ground_tracker_enforced", False))
        item["track_camera_ground_enforced_hits"] = int(
            t.get("camera_ground_enforced_hits", 0))
        return item

    def _coast_item(self, tid, t, now):
        dt = max(0.0, now - float(t.get("timestamp", now)))
        item = dict(t.get("last_det", {}))
        allowed = self._allowed_coast(t, now)
        item.update({
            "id": tid,
            "x": float(t["x"]) + float(t["vx"]) * dt,
            "y": float(t["y"]) + float(t["vy"]) * dt,
            "z": float(t["z"]),
            "raw_x": float(t["x"]), "raw_y": float(t["y"]), "raw_z": float(t["z"]),
            "raw_vx": float(t["vx"]), "raw_vy": float(t["vy"]),
            "vx": float(t["vx"]), "vy": float(t["vy"]),
            "extent": list(t.get("extent", [0, 0, 0])),
            "track_hits": int(t.get("hits", 1)),
            "track_state": "coast",
            "coast_frames": int(t.get("misses", 0)),
            "coast_allowed": allowed,
            "track_selected_enforced_current": False,
            "track_selected_enforced_ever": int(t.get("selected_enforced_hits", 0)) > 0,
            "track_selected_enforced_hits": int(t.get("selected_enforced_hits", 0)),
            "track_selected_enforced_origin": bool(t.get("selected_enforced_origin", False)),
            "track_non_selected_hits": int(t.get("non_selected_hits", 0))
        })
        item["confidence"] = float(item.get("confidence", .72)) * (
            self.coast_confidence_decay ** max(1, int(t.get("misses", 1))))
        item["sources"] = list(item.get("sources", ["lidar"]))
        return self._decorate_quality(item, t, now)

    def apply_sensor_confirmations(self, track_ids, sensor="camera", timestamp=None):
        now = time.time() if timestamp is None else float(timestamp)
        key_time = "last_camera_time" if sensor == "camera" else "last_radar_time"
        key_count = "camera_confirmations" if sensor == "camera" else "radar_confirmations"
        for tid in set(track_ids or []):
            t = self._tracks.get(tid)
            if t is None:
                continue
            t[key_time] = now
            t[key_count] = int(t.get(key_count, 0)) + 1

    def refresh_output_quality(self, items, timestamp=None):
        now = time.time() if timestamp is None else float(timestamp)
        out = []
        for src in items or []:
            item = dict(src)
            t = self._tracks.get(item.get("id"))
            if t is not None:
                self._decorate_quality(item, t, now)
                item["coast_allowed"] = self._allowed_coast(t, now)
            out.append(item)
        return out

    def quality_stats(self, timestamp=None):
        now = time.time() if timestamp is None else float(timestamp)
        values = [self._quality_value(t, now) for t in self._tracks.values()]
        high = sum(1 for q in values if q >= self.quality_high)
        medium = sum(1 for q in values if self.quality_medium <= q < self.quality_high)
        low = sum(1 for q in values if q < self.quality_medium)
        return {"active": len(values), "high": high, "medium": medium, "low": low,
                "avg": (sum(values) / len(values) if values else 0.0)}

    def update(self, detections, timestamp=None):
        now = time.time() if timestamp is None else float(timestamp)
        self._prune_camera_tombstones(now)
        unmatched = set(self._tracks)
        results = []
        pending = []
        association_meta = {}
        stats = {"new": 0, "update": 0, "coast": 0, "drop": 0, "suppress": 0,
                 "low_hit_keep": 0, "low_new_drop": 0}

        for di, det in enumerate(detections):
            nearest = None
            nearest_camera = None
            nearest_camera_id = None
            candidate_count = 0
            camera_candidate_count = 0
            for tid in unmatched:
                t = self._tracks[tid]
                dt = max(1e-3, now - t["timestamp"])
                px = t["x"] + t["vx"] * dt
                py = t["y"] + t["vy"] * dt
                d = math.hypot(det["x"] - px, det["y"] - py)
                if nearest is None or d < nearest:nearest = d
                camera_origin = bool(t.get("camera_ground_origin", False))
                if camera_origin and (nearest_camera is None or d < nearest_camera):
                    nearest_camera = d
                    nearest_camera_id = tid
                if d < self.max_distance:
                    pending.append((d, di, tid))
                    candidate_count += 1
                    if camera_origin:camera_candidate_count += 1
            association_meta[di] = {
                "nearest": nearest,"nearest_camera": nearest_camera,
                "nearest_camera_id":nearest_camera_id,
                "candidates": candidate_count,
                "camera_candidates": camera_candidate_count}
            tombstones=self._nearest_camera_tombstones(det,now)
            for mode,value in tombstones.items():
                association_meta[di]["tombstone_"+mode]=value
        pending.sort()
        assigned_det = {}
        assigned_distance = {}
        assigned_track = set()
        for d, di, tid in pending:
            if di not in assigned_det and tid not in assigned_track:
                assigned_det[di] = tid
                assigned_distance[di] = d
                assigned_track.add(tid)

        for di, det in enumerate(detections):
            tid = assigned_det.get(di)
            old = self._tracks.get(tid) if tid else None
            raw_x = float(det["x"])
            raw_y = float(det["y"])
            raw_z = float(det.get("z", 0))
            if old is None:
                tid = "vehicle_%03d" % self._next_id
                self._next_id += 1
                x, y, z = raw_x, raw_y, raw_z
                vx = vy = raw_vx = raw_vy = 0.0
                extent = list(det.get("extent", [0, 0, 0]))
                hits = 1
                hist = deque(maxlen=self.velocity_window)
                hist.append((now, raw_x, raw_y))
                size_stability = velocity_stability = .5
                stats["new"] += 1
            else:
                pa = self.position_alpha
                x = (1 - pa) * old["x"] + pa * raw_x
                y = (1 - pa) * old["y"] + pa * raw_y
                z = (1 - pa) * old["z"] + pa * raw_z
                hist = deque(old.get("history", []), maxlen=self.velocity_window)
                hist.append((now, raw_x, raw_y))
                raw_vx, raw_vy = self._history_velocity(hist)
                a = self.velocity_alpha
                vx = (1 - a) * old["vx"] + a * raw_vx
                vy = (1 - a) * old["vy"] + a * raw_vy
                vx, vy = self._clamp_velocity(vx, vy)
                hits = int(old.get("hits", 1)) + 1
                extent = self._smooth_extent(old, list(det.get("extent", old.get("extent", [0, 0, 0]))), hits)
                size_stability, velocity_stability = self._stability_update(old, extent, vx, vy)
                stats["update"] += 1

            pre_radar_vx, pre_radar_vy = vx, vy
            vx, vy = self._apply_radar_radial(vx, vy, det)
            if det.get("radar_radial_velocity") is None and \
                    math.hypot(raw_vx, raw_vy) < self.stationary_speed and \
                    math.hypot(vx, vy) < self.stationary_speed * 1.5:
                vx = vy = 0.0

            last_det = dict(det)
            last_det.pop("id", None)
            selected_current = bool(det.get("road_object_selected_enforced", False))
            detection_sources = list(det.get("sources", ["lidar"]))
            has_lidar = "lidar" in detection_sources
            has_camera = "camera" in detection_sources
            camera_ground_current = bool(det.get(
                "camera_ground_tracker_enforced", False))
            selected_hits = int(old.get("selected_enforced_hits", 0) if old else 0)
            non_selected_hits = int(old.get("non_selected_hits", 0) if old else 0)
            if selected_current:
                selected_hits += 1
            else:
                non_selected_hits += 1
            track = {
                "x": x, "y": y, "z": z, "vx": vx, "vy": vy,
                "extent": extent, "hits": hits, "timestamp": now,
                "history": hist, "misses": 0, "last_det": last_det,
                "size_stability": size_stability,
                "velocity_stability": velocity_stability,
                "lidar_hits": int(old.get("lidar_hits", 0) if old else 0) + (1 if has_lidar else 0),
                "radar_confirmations": int(old.get("radar_confirmations", 0) if old else 0),
                "camera_confirmations": int(old.get("camera_confirmations", 0) if old else 0) + (1 if has_camera else 0),
                "last_radar_time": old.get("last_radar_time") if old else None,
                "last_camera_time": old.get("last_camera_time") if old else None,
                "camera_ground_origin": bool(
                    old.get("camera_ground_origin", False)
                    if old else camera_ground_current),
                "camera_ground_enforced_hits": int(
                    old.get("camera_ground_enforced_hits", 0) if old else 0) +
                    (1 if camera_ground_current else 0),
                "selected_enforced_hits": selected_hits,
                "non_selected_hits": non_selected_hits,
                "selected_enforced_origin": bool(
                    old.get("selected_enforced_origin", False) if old else selected_current),
            }
            if det.get("radar_radial_velocity") is not None:
                track["last_radar_time"] = now
                track["radar_confirmations"] += 1
            if has_camera:
                track["last_camera_time"] = now
            self._tracks[tid] = track

            state = "new" if old is None else "confirmed"
            item = dict(det)
            item.update({
                "id": tid, "x": x, "y": y, "z": z,
                "raw_x": raw_x, "raw_y": raw_y, "raw_z": raw_z,
                "raw_vx": raw_vx, "raw_vy": raw_vy,
                "track_vx_before_radar": pre_radar_vx,
                "track_vy_before_radar": pre_radar_vy,
                "vx": vx, "vy": vy, "extent": extent,
                "track_hits": hits, "track_state": state,
                "coast_frames": 0, "coast_allowed": self._allowed_coast(track, now),
                "track_selected_enforced_current": selected_current,
                "track_selected_enforced_ever": selected_hits > 0,
                "track_selected_enforced_hits": selected_hits,
                "track_selected_enforced_origin": track["selected_enforced_origin"],
                "track_non_selected_hits": non_selected_hits
            })
            meta = association_meta.get(di, {})
            item["track_association_distance"] = assigned_distance.get(di)
            item["track_association_candidate_count"] = int(
                meta.get("candidates", 0))
            item["track_association_camera_origin_candidate_count"] = int(
                meta.get("camera_candidates", 0))
            item["track_association_nearest_distance"] = meta.get("nearest")
            item["track_association_nearest_camera_origin_distance"] = \
                meta.get("nearest_camera")
            item["track_association_nearest_camera_origin_id"] = \
                meta.get("nearest_camera_id")
            item["track_association_nearest_camera_origin_claimed"] = bool(
                meta.get("nearest_camera_id") in assigned_track)
            for mode in ("frozen","predicted"):
                tombstone=meta.get("tombstone_"+mode)
                item["track_camera_tombstone_%s_id"%mode] = (
                    tombstone[1] if tombstone is not None else None)
                item["track_camera_tombstone_%s_distance"%mode] = (
                    tombstone[0] if tombstone is not None else None)
                item["track_camera_tombstone_%s_gap"%mode] = (
                    tombstone[2] if tombstone is not None else None)
            if old is None:
                if meta.get("nearest") is None:
                    reason = "no_active_tracks"
                elif int(meta.get("candidates", 0)) == 0:
                    reason = "outside_gate"
                else:
                    reason = "assignment_conflict"
                item["track_association_birth_reason"] = reason
            self._decorate_quality(item, track, now)
            results.append(item)

        for tid in list(unmatched - assigned_track):
            t = self._tracks.get(tid)
            if t is None:
                continue
            t["misses"] = int(t.get("misses", 0)) + 1
            q = self._quality_value(t, now)
            band = self._quality_band(q)
            hits = int(t.get("hits", 1))
            allowed = self._allowed_coast(t, now)
            in_age = now - float(t.get("timestamp", now)) <= self.max_age
            if in_age and t["misses"] <= allowed:
                results.append(self._coast_item(tid, t, now))
                stats["coast"] += 1
                if band == "low" and hits >= self.quality_low_min_hits_for_coast:
                    stats["low_hit_keep"] += 1
            elif in_age:
                stats["suppress"] += 1
                if band == "low" and hits < self.quality_low_min_hits_for_coast:
                    stats["low_new_drop"] += 1

        stale = [tid for tid, t in self._tracks.items()
                 if now - t["timestamp"] > self.max_age]
        for tid in stale:
            self._remember_camera_tombstone(tid,self._tracks.get(tid),now)
            del self._tracks[tid]
            stats["drop"] += 1

        qs = self.quality_stats(now)
        stats.update({"quality_active": qs["active"], "quality_high": qs["high"],
                      "quality_medium": qs["medium"], "quality_low": qs["low"],
                      "quality_avg": qs["avg"]})
        self.last_stats = stats
        return results
