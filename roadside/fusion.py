from __future__ import print_function

import math
import time
from collections import defaultdict

from .models import DetectedObject, ObjectList
from .perception import voxel_cluster_lidar, associate_radar
from .tracking import NearestTracker


class StaticBackgroundFilter(object):
    """Learn persistent roadside clusters during a short empty-scene warmup."""

    def __init__(self, calibration_seconds=6.0, cell_size=1.5,
                 occupancy_ratio=0.60, moving_radar_speed=0.8,
                 neighbor_radius_cells=1):
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
        s = self.cell_size
        return (int(math.floor(float(x) / s)),
                int(math.floor(float(y) / s)))

    def _is_near_static_cell(self, key):
        radius = max(0, self.neighbor_radius_cells)
        kx, ky = key
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if (kx + dx, ky + dy) in self.static_cells:
                    return True
        return False

    def update_and_filter(self, candidates, now):
        if self.started_at is None:
            self.started_at = float(now)

        if not self.ready:
            self.frames += 1
            seen = set()
            for item in candidates:
                seen.add(self._key(item["x"], item["y"]))
            for key in seen:
                self.counts[key] += 1

            if float(now) - self.started_at >= self.calibration_seconds:
                threshold = max(1, int(math.ceil(self.frames * self.occupancy_ratio)))
                self.static_cells = set(
                    key for key, count in self.counts.items()
                    if count >= threshold)
                self.ready = True
            # Never publish detections during the empty-scene calibration frame,
            # including the frame in which READY first becomes true.
            return []

        output = []
        for item in candidates:
            key = self._key(item["x"], item["y"])
            radar_speed = item.get("radar_speed")
            moving = (radar_speed is not None and
                      abs(float(radar_speed)) >= self.moving_radar_speed)
            # LiDAR centroids jitter by a few tenths of a metre frame-to-frame.
            # Check neighboring background cells so a static object sitting near
            # a cell boundary does not repeatedly leak into ObjectList.
            if self._is_near_static_cell(key) and not moving:
                continue
            output.append(item)
        return output

    def remaining_seconds(self, now):
        if self.ready:
            return 0.0
        if self.started_at is None:
            return self.calibration_seconds
        return max(0.0, self.calibration_seconds - (float(now) - self.started_at))


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
        self.background = StaticBackgroundFilter(
            calibration_seconds=config.get("background_calibration_seconds", 6.0),
            cell_size=config.get("background_cell_size", 1.5),
            occupancy_ratio=config.get("background_occupancy_ratio", 0.60),
            moving_radar_speed=config.get("background_moving_radar_speed", 0.8),
            neighbor_radius_cells=config.get("background_neighbor_radius_cells", 1))
        self.world_transform = None
        self.candidate_validator = None
        self.last_stats = {
            "lidar_points": 0,
            "lidar_clusters": 0,
            "roi_candidates": 0,
            "background_candidates": 0,
            "background_ready": False,
            "background_remaining": 0.0,
            "background_cells": 0,
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

        roi_candidates = []
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

            roi_candidates.append({
                "x": wx, "y": wy, "z": wz,
                "radar_speed": radar_speed,
                "confidence": confidence,
                "sources": sources,
                "point_count": item.get("point_count", 0),
                "extent": extent,
            })

        candidates = self.background.update_and_filter(roi_candidates, now)
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
            "roi_candidates": len(roi_candidates),
            "background_candidates": len(candidates),
            "background_ready": self.background.ready,
            "background_remaining": self.background.remaining_seconds(now),
            "background_cells": len(self.background.static_cells),
            "radar_detections": 0 if not radar_detections else len(radar_detections),
            "tracked_objects": len(objects),
        }
        return ObjectList(self.station_id, objects, timestamp=now)
