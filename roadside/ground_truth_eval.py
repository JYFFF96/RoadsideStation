from __future__ import print_function

import math


class GroundTruthEvaluator(object):
    def __init__(self, world, center_provider, config=None):
        self.world = world
        self.center_provider = center_provider
        self.config = config or {}
        self.radius = float(self.config.get("radius", 80.0))
        self.match_distance = float(self.config.get("match_distance", 4.0))
        self.include_roles = set(self.config.get("include_roles", ["autopilot", "roadside_autopilot", "rsu_local_autopilot"]))

    @staticmethod
    def _distance2d(a, b):
        return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))

    def _center(self):
        c = self.center_provider()
        return c

    def truth_vehicles(self):
        center = self._center()
        out = []
        if center is None:
            return out
        for actor in self.world.get_actors().filter("vehicle.*"):
            try:
                role = actor.attributes.get("role_name", "")
                if self.include_roles and role not in self.include_roles:
                    continue
                loc = actor.get_location()
                if self._distance2d(loc, center) > self.radius:
                    continue
                vel = actor.get_velocity()
                extent = actor.bounding_box.extent
                out.append({
                    "actor_id": int(actor.id),
                    "type_id": actor.type_id,
                    "role": role,
                    "x": float(loc.x),
                    "y": float(loc.y),
                    "z": float(loc.z),
                    "vx": float(vel.x),
                    "vy": float(vel.y),
                    "speed": math.hypot(float(vel.x), float(vel.y)),
                    "size": [float(extent.x) * 2.0, float(extent.y) * 2.0, float(extent.z) * 2.0],
                })
            except Exception:
                continue
        return out

    def _match(self, truth, detected):
        candidates = []
        for ti, gt in enumerate(truth):
            for di, det in enumerate(detected or []):
                dx = float(det.get("x", 0.0)) - gt["x"]
                dy = float(det.get("y", 0.0)) - gt["y"]
                d = math.hypot(dx, dy)
                if d <= self.match_distance:
                    candidates.append((d, ti, di))
        candidates.sort(key=lambda x: x[0])
        used_t = set()
        used_d = set()
        pairs = []
        for d, ti, di in candidates:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            pairs.append((ti, di, d))
        return pairs

    def evaluate(self, detected_tracks, camera_objects=None, camera_pairs=None, radar_matched=0):
        truth = self.truth_vehicles()
        detected = list(detected_tracks or [])
        pairs = self._match(truth, detected)
        truth_n = len(truth)
        detected_n = len(detected)
        matched_n = len(pairs)
        false_pos = max(0, detected_n - matched_n)
        missed = max(0, truth_n - matched_n)
        recall = (float(matched_n) / truth_n) if truth_n else None
        precision = (float(matched_n) / detected_n) if detected_n else None
        pos_errors = [p[2] for p in pairs]
        mean_pos_error = (sum(pos_errors) / len(pos_errors)) if pos_errors else None
        max_pos_error = max(pos_errors) if pos_errors else None
        return {
            "truth": truth_n,
            "detected": detected_n,
            "matched": matched_n,
            "missed": missed,
            "false_positive": false_pos,
            "recall": recall,
            "precision": precision,
            "mean_position_error": mean_pos_error,
            "max_position_error": max_pos_error,
            "camera_visible": len(camera_objects or []),
            "camera_lidar_matched": len(camera_pairs or []),
            "radar_matched": int(radar_matched),
            "truth_objects": truth,
            "pairs": pairs,
        }
