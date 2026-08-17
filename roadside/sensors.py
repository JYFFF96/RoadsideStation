from __future__ import print_function

import math
import threading
from collections import deque

import numpy as np


class SensorCache(object):
    def __init__(self, history_size=8):
        self._lock = threading.Lock()
        self.camera = None
        self.lidar = None
        self.radar = None
        self._camera_history = deque(maxlen=max(2, int(history_size)))
        self._lidar_history = deque(maxlen=max(2, int(history_size)))
        self._radar_history = deque(maxlen=max(2, int(history_size)))

    def set_camera(self, frame, image, timestamp=None):
        with self._lock:
            self.camera = (frame, image, timestamp)
            self._camera_history.append(self.camera)

    def set_lidar(self, frame, points, timestamp=None):
        with self._lock:
            self.lidar = (frame, points, timestamp)
            self._lidar_history.append(self.lidar)

    def set_radar(self, frame, detections, timestamp=None):
        with self._lock:
            self.radar = (frame, detections, timestamp)
            self._radar_history.append(self.radar)

    def snapshot(self):
        with self._lock:
            return self.camera, self.lidar, self.radar

    def snapshot_aligned(self):
        """Return the newest exact Camera/LiDAR frame pair from bounded history.

        Radar is selected by nearest timestamp because it may run at a different
        cadence. If no common Camera/LiDAR frame has arrived yet, retain the
        legacy latest-snapshot behavior.
        """
        with self._lock:
            cameras = dict((item[0], item) for item in self._camera_history)
            lidars = dict((item[0], item) for item in self._lidar_history)
            common = set(cameras).intersection(lidars)
            if not common:
                return self.camera, self.lidar, self.radar
            frame = max(common)
            camera = cameras[frame]
            lidar = lidars[frame]
            radar = self.radar
            if lidar[2] is not None:
                timed_radar = [item for item in self._radar_history
                               if item[2] is not None]
                if timed_radar:
                    radar = min(timed_radar,
                                key=lambda item: abs(float(item[2])-float(lidar[2])))
            return camera, lidar, radar


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
