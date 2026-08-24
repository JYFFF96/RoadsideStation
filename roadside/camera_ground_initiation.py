from __future__ import print_function

import math
import numpy as np


def _value(obj, name, default=None):
    if hasattr(obj, name):
        return getattr(obj, name)
    if isinstance(obj, dict):
        aliases={"class_name":"className"}
        return obj.get(name,obj.get(aliases.get(name),default))
    return default


def _extent(class_name):
    name=str(class_name).lower()
    if name in ("person","pedestrian"):
        return [.6,.6,1.7]
    if name in ("bicycle","motorcycle"):
        return [1.8,.8,1.5]
    if name in ("car","truck","bus","vehicle"):
        return [4.5,1.8,1.6]
    return [1.0,1.0,1.0]


def camera_box_ground_point(projector, bbox, ground_z):
    """Back-project the box bottom centre to a horizontal world plane."""
    if projector is None or bbox is None or len(bbox)<4:
        return None
    u=(float(bbox[0])+float(bbox[2]))*.5;v=float(bbox[3])
    try:
        focal=float(projector.K[0,0]);cx=float(projector.K[0,2]);cy=float(projector.K[1,2])
        matrix=np.asarray(projector.transform.get_matrix(),dtype=np.float64)
    except Exception:
        return None
    sensor_ray=np.array([1.0,(u-cx)/focal,-(v-cy)/focal],dtype=np.float64)
    world_ray=np.dot(matrix[:3,:3],sensor_ray);origin=matrix[:3,3]
    if float(world_ray[2])>=-1e-6:
        return None
    scale=(float(ground_z)-float(origin[2]))/float(world_ray[2])
    if scale<=0.0:
        return None
    point=origin+world_ray*scale
    return float(point[0]),float(point[1]),float(point[2])


class CameraGroundInitiationShadow(object):
    """Build evaluation-only near candidates from generic 2D detections."""
    def __init__(self, config=None):
        c=config or {}
        self.enabled=bool(c.get("camera_ground_initiation_shadow_enabled",False))
        self.min_range=float(c.get("camera_ground_initiation_min_range",2.0))
        self.max_range=float(c.get("camera_ground_initiation_max_range",30.0))
        self.min_confidence=float(c.get("camera_ground_initiation_min_confidence",.25))
        self.dedupe_distance=float(c.get("camera_ground_initiation_dedupe_distance",3.0))
        self.cross_camera_distance=float(c.get("camera_ground_initiation_cross_camera_distance",2.0))
        self.allowed_classes=set(str(x).lower() for x in c.get(
            "camera_ground_initiation_allowed_classes",
            ["person","pedestrian","bicycle","motorcycle","car","truck","bus"]))
        self._last_token=None
        self.stats={"frames":0,"detections":0,"class_rejected":0,
                    "confidence_rejected":0,"projection_rejected":0,
                    "range_rejected":0,"cross_camera_deduped":0,
                    "lidar_deduped":0,"roi_rejected":0,"would_emit":0,
                    "classes":{}}

    @staticmethod
    def _near(item, others, distance):
        return any(math.hypot(float(item["x"])-float(old["x"]),
                              float(item["y"])-float(old["y"]))<=distance
                   for old in (others or []))

    def update(self, camera_views, existing, ground_z, validator=None,
               frame_token=None):
        if not self.enabled:return []
        if frame_token is not None and frame_token==self._last_token:return []
        self._last_token=frame_token;self.stats["frames"]+=1;candidates=[]
        for view in camera_views or []:
            projector=view.get("projector")
            try:matrix=np.asarray(projector.transform.get_matrix(),dtype=np.float64)
            except Exception:matrix=None
            if matrix is None:continue
            origin=matrix[:3,3]
            for obj in view.get("camera_objects",[]) or []:
                self.stats["detections"]+=1
                class_name=str(_value(obj,"class_name","unknown")).lower()
                if class_name not in self.allowed_classes:
                    self.stats["class_rejected"]+=1;continue
                confidence=float(_value(obj,"confidence",0.0) or 0.0)
                if confidence<self.min_confidence:
                    self.stats["confidence_rejected"]+=1;continue
                point=camera_box_ground_point(projector,_value(obj,"bbox"),ground_z)
                if point is None:
                    self.stats["projection_rejected"]+=1;continue
                sensor_range=math.hypot(point[0]-float(origin[0]),
                                        point[1]-float(origin[1]))
                if sensor_range<self.min_range or sensor_range>self.max_range:
                    self.stats["range_rejected"]+=1;continue
                extent=_extent(class_name)
                item={"x":point[0],"y":point[1],
                      "z":float(ground_z)+extent[2]*.5,"extent":extent,
                      "confidence":confidence,"sources":["camera"],
                      "object_type":("person" if class_name=="pedestrian" else class_name),
                      "camera_ground_initiated":True,
                      "camera_id":str(view.get("camera_id","unknown")),
                      "camera_source":str(view.get("camera_source","none")),
                      "sensor_range":sensor_range}
                if self._near(item,candidates,self.cross_camera_distance):
                    self.stats["cross_camera_deduped"]+=1;continue
                if self._near(item,existing,self.dedupe_distance):
                    self.stats["lidar_deduped"]+=1;continue
                if validator is not None:
                    try:valid=bool(validator(item))
                    except Exception:valid=False
                    if not valid:
                        self.stats["roi_rejected"]+=1;continue
                candidates.append(item);self.stats["would_emit"]+=1
                self.stats["classes"][class_name]=self.stats["classes"].get(class_name,0)+1
        return candidates

    def report(self):
        result=dict(self.stats);result["classes"]=dict(self.stats["classes"])
        return result
