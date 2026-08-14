from __future__ import print_function

from .camera_lidar_association import associate_camera_to_lidar
from .lidar_projection import project_lidar_tracks


def _camera_value(camera_object, name, default=None):
    if hasattr(camera_object, name):
        return getattr(camera_object, name)
    if isinstance(camera_object, dict):
        aliases = {"class_name": "className", "confidence": "confidence"}
        return camera_object.get(name, camera_object.get(aliases.get(name), default))
    return default


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
