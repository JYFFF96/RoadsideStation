from __future__ import print_function

import time

from .models import DetectedObject, ObjectList
from .perception import voxel_cluster_lidar, associate_radar
from .tracking import NearestTracker


class SimpleFusion(object):
    """RoadsideStation V0.2 perception and fusion pipeline.

    V0.2 performs LiDAR clustering first, associates radar returns to each
    cluster, then maintains stable IDs with a nearest-neighbor tracker.
    Camera association is intentionally left for the next step.
    """

    def __init__(self, station_id, config):
        self.station_id = station_id
        self.config = config
        self.tracker = NearestTracker(
            max_distance=config.get("track_match_distance", 4.0),
            max_age=config.get("track_max_age", 1.5))
        self.last_stats = {
            "lidar_points": 0,
            "lidar_clusters": 0,
            "radar_detections": 0,
            "tracked_objects": 0,
        }

    def fuse(self, lidar_points, radar_detections, timestamp=None):
        now = time.time() if timestamp is None else float(timestamp)
        cfg = self.config
        clusters = voxel_cluster_lidar(
            lidar_points,
            voxel_size=cfg.get("voxel_size", 0.8),
            min_points=cfg.get("cluster_min_points", 6),
            min_z=cfg.get("lidar_min_z", -7.5),
            max_z=cfg.get("lidar_max_z", 2.0),
            max_range=cfg.get("max_range", 70.0))

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

            candidates.append({
                "x": item["x"], "y": item["y"], "z": item["z"],
                "radar_speed": radar_speed,
                "confidence": confidence,
                "sources": sources,
                "point_count": item.get("point_count", 0),
                "extent": item.get("extent", [0.0, 0.0, 0.0]),
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
            "radar_detections": 0 if not radar_detections else len(radar_detections),
            "tracked_objects": len(objects),
        }
        return ObjectList(self.station_id, objects, timestamp=now)
