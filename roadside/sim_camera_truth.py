from __future__ import print_function

from .camera_objects import CameraObjectList
from .object_taxonomy import carla_actor_class, iter_carla_road_actors


def _project_actor_bbox(actor, projector, width, height):
    pixels = []
    try:
        vertices = actor.bounding_box.get_world_vertices(actor.get_transform())
    except Exception:
        return None
    for v in vertices:
        p = projector.project(v.x, v.y, v.z)
        if p is not None:
            pixels.append((int(p["u"]), int(p["v"])))
    if len(pixels) < 2:
        return None
    xs = [p[0] for p in pixels]; ys = [p[1] for p in pixels]
    x1 = max(0, min(xs)); y1 = max(0, min(ys))
    x2 = min(int(width) - 1, max(xs)); y2 = min(int(height) - 1, max(ys))
    if x2 <= x1 or y2 <= y1 or (x2 - x1) < 8 or (y2 - y1) < 8:
        return None
    return [x1, y1, x2, y2]


def make_truth_camera_objects(world, projector, camera_id, width, height, frame_id=None,
                              timestamp=None, obstacle_patterns=None):
    """Simulation-only CameraObjectList built from CARLA actor bounding boxes.

    This is a temporary adapter to validate fusion while the production RTSP
    detector backend is still being selected. Downstream fusion sees the same
    CameraObjectList interface it will receive from the real detector.
    """
    detections = []
    for actor in iter_carla_road_actors(world, obstacle_patterns):
        rect = _project_actor_bbox(actor, projector, width, height)
        if rect is None:
            continue
        x1, y1, x2, y2 = rect
        detections.append({
            "class_id": None,
            "class_name": carla_actor_class(actor),
            "confidence": 1.0,
            "bbox": rect,
            "center": [(x1 + x2) * 0.5, (y1 + y2) * 0.5],
        })
    return CameraObjectList.from_detections(camera_id, detections, timestamp=timestamp, frame_id=frame_id)
