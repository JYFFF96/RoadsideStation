from __future__ import print_function

import math
import threading

import numpy as np


class SensorCache(object):
    def __init__(self):
        self._lock = threading.Lock()
        self.camera = None
        self.lidar = None
        self.radar = None

    def set_camera(self, frame, image, timestamp=None):
        with self._lock:
            self.camera = (frame, image, timestamp)

    def set_lidar(self, frame, points, timestamp=None):
        with self._lock:
            self.lidar = (frame, points, timestamp)

    def set_radar(self, frame, detections, timestamp=None):
        with self._lock:
            self.radar = (frame, detections, timestamp)

    def snapshot(self):
        with self._lock:
            return self.camera, self.lidar, self.radar


def image_to_bgra(image):
    data = np.frombuffer(image.raw_data, dtype=np.uint8)
    return data.reshape((image.height, image.width, 4))


def lidar_to_xyz(measurement):
    data = np.frombuffer(measurement.raw_data, dtype=np.float32)
    return data.reshape((-1, 4))[:, :3].copy()


def radar_to_cartesian(measurement):
    result = []
    for detection in measurement:
        depth = float(detection.depth)
        azimuth = float(detection.azimuth)
        altitude = float(detection.altitude)
        x = depth * math.cos(altitude) * math.cos(azimuth)
        y = depth * math.cos(altitude) * math.sin(azimuth)
        z = depth * math.sin(altitude)
        result.append({
            "x": x, "y": y, "z": z,
            "velocity": float(detection.velocity),
            "azimuth": azimuth,
            "altitude": altitude,
        })
    return result
