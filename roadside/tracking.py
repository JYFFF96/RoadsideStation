from __future__ import print_function

import math
import time


class NearestTracker(object):
    """Nearest-neighbor tracker with V0.2.1 velocity stabilization."""

    def __init__(self, max_distance=4.0, max_age=1.5,
                 max_speed=20.0, velocity_alpha=0.35):
        self.max_distance = float(max_distance)
        self.max_age = float(max_age)
        self.max_speed = float(max_speed)
        self.velocity_alpha = float(velocity_alpha)
        self._tracks = {}
        self._next_id = 1

    def _clamp_velocity(self, vx, vy):
        speed = math.hypot(vx, vy)
        if speed <= self.max_speed or speed < 1e-6:
            return vx, vy
        scale = self.max_speed / speed
        return vx * scale, vy * scale

    def update(self, detections, timestamp=None):
        now = time.time() if timestamp is None else float(timestamp)
        unmatched_tracks = set(self._tracks.keys())
        results = []

        for det in detections:
            best_id = None
            best_dist = self.max_distance
            for track_id in list(unmatched_tracks):
                track = self._tracks[track_id]
                dt = max(1e-3, now - track["timestamp"])
                pred_x = track["x"] + track["vx"] * dt
                pred_y = track["y"] + track["vy"] * dt
                dist = math.hypot(det["x"] - pred_x, det["y"] - pred_y)
                if dist < best_dist:
                    best_dist = dist
                    best_id = track_id

            if best_id is None:
                best_id = "vehicle_%03d" % self._next_id
                self._next_id += 1
                vx = float(det.get("vx", 0.0))
                vy = float(det.get("vy", 0.0))
            else:
                old = self._tracks[best_id]
                dt = max(1e-3, now - old["timestamp"])
                raw_vx = (float(det["x"]) - old["x"]) / dt
                raw_vy = (float(det["y"]) - old["y"]) / dt
                raw_vx, raw_vy = self._clamp_velocity(raw_vx, raw_vy)

                radar_speed = det.get("radar_speed")
                if radar_speed is not None:
                    speed_xy = math.hypot(raw_vx, raw_vy)
                    radar_abs = min(abs(float(radar_speed)), self.max_speed)
                    if speed_xy > 0.2:
                        scale = radar_abs / speed_xy
                        raw_vx *= scale
                        raw_vy *= scale

                alpha = self.velocity_alpha
                vx = (1.0 - alpha) * old["vx"] + alpha * raw_vx
                vy = (1.0 - alpha) * old["vy"] + alpha * raw_vy
                vx, vy = self._clamp_velocity(vx, vy)
                unmatched_tracks.discard(best_id)

            self._tracks[best_id] = {
                "x": float(det["x"]), "y": float(det["y"]),
                "vx": vx, "vy": vy, "timestamp": now,
            }
            item = dict(det)
            item["id"] = best_id
            item["vx"] = vx
            item["vy"] = vy
            results.append(item)

        stale = [track_id for track_id, track in self._tracks.items()
                 if now - track["timestamp"] > self.max_age]
        for track_id in stale:
            del self._tracks[track_id]

        return results
