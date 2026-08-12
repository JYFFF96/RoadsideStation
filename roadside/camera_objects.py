from __future__ import print_function

import time


class CameraObject(object):
    def __init__(self, camera_id, class_name, confidence, bbox, center=None, class_id=None):
        self.camera_id = str(camera_id)
        self.class_name = str(class_name)
        self.class_id = class_id
        self.confidence = float(confidence)
        self.bbox = [int(v) for v in bbox]
        if center is None:
            x1, y1, x2, y2 = self.bbox
            center = [(x1 + x2) * 0.5, (y1 + y2) * 0.5]
        self.center = [float(center[0]), float(center[1])]

    def to_dict(self):
        return {
            "cameraId": self.camera_id,
            "classId": self.class_id,
            "className": self.class_name,
            "confidence": self.confidence,
            "bbox": list(self.bbox),
            "center": list(self.center),
        }


class CameraObjectList(object):
    def __init__(self, camera_id, objects=None, timestamp=None, frame_id=None):
        self.camera_id = str(camera_id)
        self.objects = objects or []
        self.timestamp = time.time() if timestamp is None else float(timestamp)
        self.frame_id = frame_id

    @classmethod
    def from_detections(cls, camera_id, detections, timestamp=None, frame_id=None):
        objects = [CameraObject(camera_id,
                                d.get("class_name", "unknown"),
                                d.get("confidence", 0.0),
                                d.get("bbox", [0, 0, 0, 0]),
                                d.get("center"),
                                d.get("class_id"))
                   for d in (detections or [])]
        return cls(camera_id, objects, timestamp, frame_id)

    def to_dict(self):
        return {
            "cameraId": self.camera_id,
            "timestamp": self.timestamp,
            "frameId": self.frame_id,
            "objectCount": len(self.objects),
            "objects": [o.to_dict() for o in self.objects],
        }
