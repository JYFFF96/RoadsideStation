from __future__ import print_function

import time


class FusedObject(object):
    def __init__(self, object_id, object_type="unknown", x=0.0, y=0.0, z=0.0,
                 vx=0.0, vy=0.0, size=None, confidence=0.0, sources=None,
                 camera=None, radar_speed=None):
        self.object_id = str(object_id)
        self.object_type = str(object_type)
        self.x = float(x); self.y = float(y); self.z = float(z)
        self.vx = float(vx); self.vy = float(vy)
        self.size = [float(v) for v in (size or [0.0, 0.0, 0.0])]
        self.confidence = float(confidence)
        self.sources = list(sources or [])
        self.camera = camera
        self.radar_speed = None if radar_speed is None else float(radar_speed)

    def to_dict(self):
        out = {
            "id": self.object_id,
            "type": self.object_type,
            "position": {"x": self.x, "y": self.y, "z": self.z},
            "velocity": {"x": self.vx, "y": self.vy},
            "size": {"length": self.size[0], "width": self.size[1], "height": self.size[2]},
            "confidence": self.confidence,
            "sources": list(self.sources),
        }
        if self.radar_speed is not None:
            out["radarSpeed"] = self.radar_speed
        if self.camera is not None:
            out["camera"] = dict(self.camera)
        return out


class FusedObjectList(object):
    def __init__(self, station_id, objects=None, timestamp=None):
        self.station_id = str(station_id)
        self.objects = objects or []
        self.timestamp = time.time() if timestamp is None else float(timestamp)

    def to_dict(self):
        return {
            "msgType": "FusedObjectList",
            "version": "V0.4.5",
            "stationId": self.station_id,
            "timestamp": self.timestamp,
            "objectCount": len(self.objects),
            "objects": [o.to_dict() for o in self.objects],
        }


def build_fused_object_list(station_id, tracked_candidates, timestamp=None):
    """Build the stable fusion boundary from LiDAR/Radar tracked candidates.

    Camera metadata can be attached later without changing downstream RSM/event APIs.
    """
    objects = []
    for item in tracked_candidates or []:
        extent = item.get("extent", [0.0, 0.0, 0.0])
        sources = list(item.get("sources", []))
        objects.append(FusedObject(
            item.get("id", "unknown"),
            object_type=item.get("object_type", "unknown"),
            x=item.get("x", 0.0), y=item.get("y", 0.0), z=item.get("z", 0.0),
            vx=item.get("vx", 0.0), vy=item.get("vy", 0.0),
            size=extent,
            confidence=item.get("confidence", 0.0),
            sources=sources,
            radar_speed=item.get("radar_speed")))
    return FusedObjectList(station_id, objects, timestamp)
