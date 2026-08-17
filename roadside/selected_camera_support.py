from __future__ import print_function

import math

from .camera_lidar_association import associate_camera_to_lidar
from .lidar_projection import project_lidar_tracks


def _camera_value(camera_object, name, default=None):
    if hasattr(camera_object, name):
        return getattr(camera_object, name)
    if isinstance(camera_object, dict):
        aliases = {"class_name": "className", "confidence": "confidence"}
        return camera_object.get(name, camera_object.get(aliases.get(name), default))
    return default


def _box(camera_object):
    if hasattr(camera_object, "bbox"):
        return camera_object.bbox
    return camera_object.get("bbox", [0, 0, 0, 0])


def _center(box):
    return ((float(box[0])+float(box[2]))*.5,
            (float(box[1])+float(box[3]))*.5)


def _iou(a, b):
    x1=max(float(a[0]),float(b[0]));y1=max(float(a[1]),float(b[1]))
    x2=min(float(a[2]),float(b[2]));y2=min(float(a[3]),float(b[3]))
    inter=max(0.0,x2-x1)*max(0.0,y2-y1)
    aa=max(0.0,float(a[2])-float(a[0]))*max(0.0,float(a[3])-float(a[1]))
    bb=max(0.0,float(b[2])-float(b[0]))*max(0.0,float(b[3])-float(b[1]))
    den=aa+bb-inter
    return 0.0 if den<=0.0 else inter/den


def annotate_selected_camera_support(candidates, projector, camera_objects,
                                     width, height, camera_source="none",
                                     min_iou=0.05,
                                     max_center_distance=120.0):
    """Attach camera-support diagnostics to copies of Selected HOLD candidates.

    The annotations are evaluator-only. They are never returned to Fusion or
    Tracker as admission evidence, so CARLA's camera-truth adapter cannot alter
    perception output. The same fields work with a real detector camera source.
    """
    annotated = []
    for candidate in candidates or []:
        item = dict(candidate)
        item["selected_track_admission_camera_visible"] = False
        item["selected_track_admission_camera_supported"] = False
        item["selected_track_admission_camera_source"] = str(camera_source)
        annotated.append(item)
    stats = {"held": len(annotated), "visible": 0, "supported": 0,
             "source": str(camera_source)}
    if projector is None or not annotated:
        return annotated, stats

    projected = project_lidar_tracks(projector, annotated, width, height)
    for item in projected:
        source_index = int(item["source_index"])
        annotated[source_index]["selected_track_admission_camera_visible"] = True
    stats["visible"] = len(projected)

    # Diagnostic-only nearest box, including boxes outside the support gate.
    for projected_item in projected:
        lidar_box=projected_item["bbox"];lidar_center=_center(lidar_box)
        nearest=None
        for camera_object in camera_objects or []:
            camera_box=_box(camera_object);camera_center=_center(camera_box)
            distance=math.hypot(camera_center[0]-lidar_center[0],
                                camera_center[1]-lidar_center[1])
            if nearest is None or distance<nearest[0]:
                nearest=(distance,camera_object,camera_box)
        if nearest is not None:
            source_index=int(projected_item["source_index"]);item=annotated[source_index]
            item["selected_track_admission_camera_nearest_distance"]=float(nearest[0])
            item["selected_track_admission_camera_nearest_iou"]=float(_iou(nearest[2],lidar_box))
            item["selected_track_admission_camera_nearest_class"]=str(
                _camera_value(nearest[1],"class_name","unknown"))
            confidence=_camera_value(nearest[1],"confidence")
            if confidence is not None:
                item["selected_track_admission_camera_nearest_confidence"]=float(confidence)

    matches = associate_camera_to_lidar(
        camera_objects, projected, min_iou=min_iou,
        max_center_distance=max_center_distance)
    for pair in matches:
        projected_item = projected[int(pair["lidar_index"])]
        source_index = int(projected_item["source_index"])
        camera_object = camera_objects[int(pair["camera_index"])]
        item = annotated[source_index]
        item["selected_track_admission_camera_supported"] = True
        item["selected_track_admission_camera_iou"] = float(pair["iou"])
        item["selected_track_admission_camera_center_distance"] = float(
            pair["center_distance"])
        item["selected_track_admission_camera_class"] = str(
            _camera_value(camera_object, "class_name", "unknown"))
        confidence = _camera_value(camera_object, "confidence")
        if confidence is not None:
            item["selected_track_admission_camera_confidence"] = float(confidence)
    stats["supported"] = len(matches)
    return annotated, stats


def selected_camera_rescue_passes(candidate, min_iou=0.05,
                                  max_center_distance=45.0,
                                  allowed_classes=None):
    """Return whether an annotated HOLD has strong detector-side support."""
    if not candidate.get("selected_track_admission_camera_supported", False):
        return False
    allowed = set(str(x).lower() for x in
                  (allowed_classes or ["person", "pedestrian"]))
    camera_class = str(candidate.get(
        "selected_track_admission_camera_class", "unknown")).lower()
    if camera_class not in allowed:
        return False
    try:iou = float(candidate.get("selected_track_admission_camera_iou", 0.0))
    except (TypeError, ValueError):iou = 0.0
    try:distance = float(candidate.get(
        "selected_track_admission_camera_center_distance", float("inf")))
    except (TypeError, ValueError):distance = float("inf")
    return iou >= float(min_iou) or distance <= float(max_center_distance)
