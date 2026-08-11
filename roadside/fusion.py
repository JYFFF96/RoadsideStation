from __future__ import print_function

import math
import time

from .models import DetectedObject, ObjectList
from .perception import voxel_cluster_lidar, associate_radar
from .tracking import NearestTracker


class SimpleFusion(object):
    """RoadsideStation V0.2.1 perception and fusion pipeline."""

    def __init__(self, station_id, config):
        self.station_id = station_id
        self.config = config
        self.tracker = NearestTracker(
            max_distance=config.get("track_match_distance", 4.0),
            max_age=config.get("track_max_age", 1.5),
            max_speed=config.get("track_max_speed", 20.0),
            velocity_alpha=config.get("velocity_alpha", 0.35))
        self.world_transform = None
        self.candidate_validator = None
        self.last_stats = {
            "lidar_points": 0,
            "lidar_clusters": 0,
            "roi_candidates": 0,
            "radar_detections": 0,
            "tracked_objects": 0,
        }

    def set_world_transform(self, transform):
        if transform is None:
            self.world_transform = None
            return
        self.world_transform = {
            "x": float(transform.location.x),
            "y": float(transform.location.y),
            "z": float(transform.location.z),
            "yaw": math.radians(float(transform.rotation.yaw)),
        }

    def set_candidate_validator(self, validator):
        self.candidate_validator = validator

    def _to_world(self, x, y, z):
        if self.world_transform is None:
            return float(x), float(y), float(z)
        t = self.world_transform
        c = math.cos(t["yaw"])
        s = math.sin(t["yaw"])
        wx = t["x"] + c * float(x) - s * float(y)
        wy = t["y"] + s * float(x) + c * float(y)
        wz = t["z"] + float(z)
        return wx, wy, wz

    def fuse(self, lidar_points, radar_detections, timestamp=None):
        now = time.time() if timestamp is None else float(timestamp)
        cfg = self.config
        clusters = voxel_cluster_lidar(
            lidar_points,
            voxel_size=cfg.get("voxel_size", 0.8),
            min_points=cfg.get("cluster_min_points", 6),
            min_z=cfg.get("lidar_min_z", -7.5),
            max_z=cfg.get("lidar_max_z", 2.0),
            max_range=cfg.get("max_range", 70.0),
            min_length=cfg.get("vehicle_min_length", 0.6),
            max_length=cfg.get("vehicle_max_length", 8.0),
            min_width=cfg.get("vehicle_min_width", 0.4),
            max_width=cfg.get("vehicle_max_width", 4.0),
            min_height=cfg.get("vehicle_min_height", 0.25),
            max_height=cfg.get("vehicle_max_height", 4.0),
            max_objects=cfg.get("max_objects", 80))

        associated = associate_radar(
            clusters,
            radar_detections,
            max_distance=cfg.get("radar_match_distance", 3.0))

        candidates = []
        for item in associated:
            radar = item.get("radar")
            confidence = 0.72
            sources = ["lidar"]
            radar_speed = None
            if radar is not None:
                confidence = 0.90
                sources.append("radar")
                radar_speed = float(radar.get("velocity", 0.0))

            wx, wy, wz = self._to_world(item["x"], item["y"], item["z"])
            extent = item.get("extent", [0.0, 0.0, 0.0])
            if self.candidate_validator is not None:
                try:
                    if not self.candidate_validator(wx, wy, wz, extent):
                        continue
                except Exception:
                    continue

            candidates.append({
                "x": wx, "y": wy, "z": wz,
                "radar_speed": radar_speed,
                "confidence": confidence,
                "sources": sources,
                "point_count": item.get("point_count", 0),
                "extent": extent,
            })

        tracked = self.tracker.update(candidates, now)
        objects = []
        for item in tracked:
            objects.append(DetectedObject(
                item["id"], item["x"], item["y"], item["z"],
                vx=item.get("vx", 0.0), vy=item.get("vy", 0.0),
                object_type="vehicle", confidence=item["confidence"],
                sources=item["sources"]))

        self.last_stats = {
            "lidar_points": 0 if lidar_points is None else len(lidar_points),
            "lidar_clusters": len(clusters),
            "roi_candidates": len(candidates),
            "radar_detections": 0 if not radar_detections else len(radar_detections),
            "tracked_objects": len(objects),
        }
        return ObjectList(self.station_id, objects, timestamp=now)
