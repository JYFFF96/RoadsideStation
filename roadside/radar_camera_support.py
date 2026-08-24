from __future__ import print_function

from .camera_lidar_association import associate_camera_to_lidar
from .lidar_projection import project_lidar_tracks_with_diagnostics


def _camera_value(camera_object, name, default=None):
    if hasattr(camera_object, name):
        return getattr(camera_object, name)
    if isinstance(camera_object, dict):
        aliases = {"class_name": "className"}
        return camera_object.get(name, camera_object.get(aliases.get(name), default))
    return default


def annotate_radar_camera_support(candidates, camera_views,
                                  camera_source="none", min_iou=.05,
                                  max_center_distance=120.0):
    """Associate eligible radar singletons with either camera, in Shadow.

    camera_views entries contain camera_id, projector, camera_objects, width and
    height. Returned entries are copies; neither radar initiation nor tracking
    can consume the annotations.
    """
    annotated=[]
    for candidate in candidates or []:
        item=dict(candidate)
        item.update({"radar_camera_visible":False,
                     "radar_camera_supported":False,
                     "radar_camera_source":str(camera_source)})
        annotated.append(item)
    for view in camera_views or []:
        projector=view.get("projector")
        if projector is None or not annotated:
            continue
        projected,rejections=project_lidar_tracks_with_diagnostics(
            projector,annotated,int(view.get("width",0)),int(view.get("height",0)))
        for projected_item in projected:
            index=int(projected_item["source_index"])
            annotated[index]["radar_camera_visible"]=True
        pairs=associate_camera_to_lidar(
            view.get("camera_objects",[]),projected,min_iou=min_iou,
            max_center_distance=max_center_distance)
        for pair in pairs:
            projected_item=projected[int(pair["lidar_index"])]
            index=int(projected_item["source_index"])
            camera_object=view.get("camera_objects",[])[int(pair["camera_index"])]
            item=annotated[index]
            distance=float(pair["center_distance"])
            previous=item.get("radar_camera_center_distance")
            if item.get("radar_camera_supported",False) and previous is not None \
                    and float(previous)<=distance:
                continue
            item.update({
                "radar_camera_supported":True,
                "radar_camera_id":str(view.get("camera_id","unknown")),
                "radar_camera_iou":float(pair["iou"]),
                "radar_camera_center_distance":distance,
                "radar_camera_class":str(_camera_value(
                    camera_object,"class_name","unknown")),
                "radar_camera_confidence":float(_camera_value(
                    camera_object,"confidence",0.0) or 0.0)})
    return annotated
