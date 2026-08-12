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
            out["radarRadialVelocity"] = self.radar_speed
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


def build_fused_object_list(station_id, tracked_candidates, timestamp=None,
                            camera_objects=None, associations=None):
    """Build FusedObjectList and optionally attach CameraObject associations."""
    camera_objects = camera_objects or []
    associations = associations or []
    camera_by_track = {}
    for pair in associations:
        li = int(pair.get("lidar_index", -1)); ci = int(pair.get("camera_index", -1))
        if li < 0 or li >= len(tracked_candidates or []) or ci < 0 or ci >= len(camera_objects):
            continue
        track_id = (tracked_candidates or [])[li].get("id")
        if track_id is not None:
            camera_by_track[str(track_id)] = (camera_objects[ci], pair)

    objects = []
    for item in tracked_candidates or []:
        extent = item.get("extent", [0.0, 0.0, 0.0])
        # Normalize horizontal dimensions for the public fusion boundary.
        ex, ey, ez = [float(v) for v in extent]
        size = [max(ex, ey), min(ex, ey), ez]
        sources = list(item.get("sources", []))
        object_type = item.get("object_type", "unknown")
        confidence = float(item.get("confidence", 0.0))
        camera_meta = None
        matched = camera_by_track.get(str(item.get("id", "")))
        if matched is not None:
            cam, pair = matched
            object_type = cam.class_name
            if "camera" not in sources:
                sources.append("camera")
            confidence = max(confidence, float(cam.confidence))
            camera_meta = {
                "cameraId": cam.camera_id,
                "bbox": list(cam.bbox),
                "confidence": float(cam.confidence),
                "className": cam.class_name,
                "association": {
                    "iou": float(pair.get("iou", 0.0)),
                    "centerDistancePx": float(pair.get("center_distance", 0.0)),
                },
            }
        radar_velocity = item.get("radar_radial_velocity")
        objects.append(FusedObject(
            item.get("id", "unknown"), object_type=object_type,
            x=item.get("x", 0.0), y=item.get("y", 0.0), z=item.get("z", 0.0),
            vx=item.get("vx", 0.0), vy=item.get("vy", 0.0), size=size,
            confidence=confidence, sources=sources, camera=camera_meta,
            radar_speed=radar_velocity))
    return FusedObjectList(station_id, objects, timestamp)
