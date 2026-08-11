from __future__ import print_function

import math

from .models import DetectedObject, ObjectList


class SimpleFusion(object):
    """V0.1 fusion scaffold.

    Radar detections become candidate objects. Nearby LiDAR points are used to
    strengthen confidence. This is deliberately simple and will be replaced by
    clustering, calibration-aware association and tracking in later versions.
    """

    def __init__(self, station_id, max_match_distance=3.0):
        self.station_id = station_id
        self.max_match_distance = float(max_match_distance)
        self._next_id = 1

    def fuse(self, lidar_points, radar_detections):
        objects = []
        if radar_detections:
            for detection in radar_detections:
                x = detection["x"]
                y = detection["y"]
                z = detection["z"]
                confidence = 0.55
                if self._has_lidar_support(x, y, z, lidar_points):
                    confidence = 0.85
                speed = detection.get("velocity", 0.0)
                objects.append(DetectedObject(
                    self._new_id(), x, y, z, vx=speed,
                    object_type="vehicle", confidence=confidence,
                    sources=["radar", "lidar"] if confidence > 0.8 else ["radar"]
                ))
        return ObjectList(self.station_id, objects)

    def _has_lidar_support(self, x, y, z, points):
        if points is None or len(points) == 0:
            return False
        radius2 = self.max_match_distance * self.max_match_distance
        for point in points[::max(1, len(points) // 3000)]:
            dx = float(point[0]) - x
            dy = float(point[1]) - y
            dz = float(point[2]) - z
            if dx * dx + dy * dy + dz * dz <= radius2:
                return True
        return False

    def _new_id(self):
        value = "obj_%06d" % self._next_id
        self._next_id += 1
        return value
