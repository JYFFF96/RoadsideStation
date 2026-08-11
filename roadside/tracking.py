from __future__ import print_function

import math
import time


class NearestTracker(object):
    """Small nearest-neighbor tracker for stable V0.2 object IDs."""

    def __init__(self, max_distance=4.0, max_age=1.5):
        self.max_distance = float(max_distance)
        self.max_age = float(max_age)
        self._tracks = {}
        self._next_id = 1

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
                vx = (float(det["x"]) - old["x"]) / dt
                vy = (float(det["y"]) - old["y"]) / dt
                # Prefer radar radial speed when available, but preserve
                # position-derived direction for the V0.2 tracker.
                radar_speed = det.get("radar_speed")
                if radar_speed is not None:
                    speed_xy = math.hypot(vx, vy)
                    if speed_xy > 0.2:
                        scale = abs(float(radar_speed)) / speed_xy
                        vx *= scale
                        vy *= scale
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
