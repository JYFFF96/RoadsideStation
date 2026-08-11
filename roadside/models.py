from __future__ import print_function

import time


class DetectedObject(object):
    def __init__(self, object_id, x, y, z=0.0, vx=0.0, vy=0.0,
                 object_type="unknown", confidence=1.0, sources=None):
        self.object_id = str(object_id)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.vx = float(vx)
        self.vy = float(vy)
        self.object_type = object_type
        self.confidence = float(confidence)
        self.sources = sources or []

    def to_dict(self):
        return {
            "id": self.object_id,
            "type": self.object_type,
            "position": {"x": self.x, "y": self.y, "z": self.z},
            "velocity": {"x": self.vx, "y": self.vy},
            "confidence": self.confidence,
            "sources": list(self.sources),
        }


class ObjectList(object):
    def __init__(self, station_id, objects=None, timestamp=None):
        self.station_id = station_id
        self.objects = objects or []
        self.timestamp = timestamp if timestamp is not None else time.time()

    def to_dict(self):
        return {
            "stationId": self.station_id,
            "timestamp": self.timestamp,
            "objectCount": len(self.objects),
            "objects": [obj.to_dict() for obj in self.objects],
        }
