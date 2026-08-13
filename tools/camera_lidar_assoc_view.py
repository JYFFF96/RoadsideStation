from __future__ import print_function

import argparse
import itertools
import time
import yaml
import cv2

from roadside.carla_station import CarlaRoadsideStation
from roadside.fusion import SimpleFusion
from roadside.camera_fusion import CameraProjector
from roadside.camera_objects import CameraObjectList
from roadside.camera_lidar_association import associate_camera_to_lidar


def load_config(path):
    with open(path, "r") as fp:
        return yaml.safe_load(fp)


def lidar_box_corners(center, extent):
    x, y, z = center
    ex, ey, ez = [max(0.3, float(v)) for v in extent]
    hx = max(0.5, ex * 0.6); hy = max(0.5, ey * 0.6); hz = max(0.5, ez * 0.6)
    return [(x + sx * hx, y + sy * hy, z + sz * hz)
            for sx, sy, sz in itertools.product((-1.0, 1.0), repeat=3)]


def projected_rect(projector, points, width, height):
    pixels = []
    for x, y, z in points:
        p = projector.project(x, y, z)
        if p is not None:
            pixels.append((int(p["u"]), int(p["v"])))
    if len(pixels) < 2:
        return None
    xs = [p[0] for p in pixels]; ys = [p[1] for p in pixels]
    x1 = max(0, min(xs)); y1 = max(0, min(ys))
    x2 = min(width - 1, max(xs)); y2 = min(height - 1, max(ys))
    if x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def carla_class(actor):
    tid = actor.type_id.lower()
    if "bus" in tid:
        return "bus"
    if "truck" in tid or "carlacola" in tid or "firetruck" in tid:
        return "truck"
    if tid.startswith("vehicle."):
        return "car"
    return "unknown"


def make_truth_camera_objects(world, projector, camera_id, width, height, frame_id):
    """Simulation-only camera detections used to validate the association layer.

    This deliberately does not participate in production fusion. It stands in
    for a future neural-network detector until the runtime model backend is selected.
    """
    detections = []
    for actor in world.get_actors().filter("vehicle.*"):
        try:
            vertices = actor.bounding_box.get_world_vertices(actor.get_transform())
            rect = projected_rect(projector,
                                  [(v.x, v.y, v.z) for v in vertices],
                                  width, height)
        except Exception:
            rect = None
        if rect is None:
            continue
        x1, y1, x2, y2 = rect
        # Ignore almost invisible boxes at image borders.
        if (x2 - x1) < 8 or (y2 - y1) < 8:
            continue
        detections.append({
            "class_id": None,
            "class_name": carla_class(actor),
            "confidence": 1.0,
            "bbox": rect,
            "center": [(x1 + x2) * 0.5, (y1 + y2) * 0.5],
        })
    return CameraObjectList.from_detections(camera_id, detections, frame_id=frame_id)


def main():
    ap = argparse.ArgumentParser(description="V0.4.2 CameraObjectList <-> LiDAR Track association validation")
    ap.add_argument("--config", default="config/roadside.yaml")
    args = ap.parse_args(); cfg = load_config(args.config)
    sid = cfg["station"]["id"]
    camera_id = cfg.get("camera", {}).get("id", "CAM_01")
    assoc_cfg = cfg.get("camera_lidar_association", {})

    station = CarlaRoadsideStation(cfg)
    fusion = SimpleFusion(sid, cfg["fusion"])
    station.start(); fusion.set_world_transform(station.lidar_transform); fusion.set_candidate_validator(station.is_driving_roi)
    cc = cfg["camera"]; width = int(cc.get("width", 1280)); height = int(cc.get("height", 720)); fov = float(cc.get("fov", 90))
    projector = CameraProjector(width, height, fov, station.camera_transform)

    print("V0.4.2 association validation started.")
    print("Cyan = simulated CameraObjectList; green = LiDAR track; magenta = matched pair.")
    print("CARLA actor boxes are used ONLY as temporary camera-detector truth for association validation.")
    try:
        while True:
            camera, lidar, radar = station.cache.snapshot()
            if camera is None or lidar is None:
                time.sleep(.02); continue
            frame_id, bgra = camera
            ol = fusion.fuse(lidar[1], radar[1] if radar else None,
                             frame_id=lidar[0])
            view = bgra[:, :, :3].copy()

            cam_list = make_truth_camera_objects(station.world, projector, camera_id, width, height, frame_id)
            lidar_tracks = []
            by_id = {c.get("id"): c for c in fusion.last_tracked_candidates}
            for obj in ol.objects:
                cand = by_id.get(obj.object_id)
                if cand is None:
                    continue
                rect = projected_rect(projector,
                                      lidar_box_corners((cand["x"], cand["y"], cand["z"]), cand.get("extent", [2,1,1])),
                                      width, height)
                if rect is None:
                    continue
                lidar_tracks.append({"id": obj.object_id, "bbox": rect, "object": obj})

            pairs = associate_camera_to_lidar(
                cam_list.objects, lidar_tracks,
                min_iou=assoc_cfg.get("min_iou", 0.05),
                max_center_distance=assoc_cfg.get("max_center_distance", 120.0))

            matched_cam = set(); matched_lidar = set()
            for pair in pairs:
                ci = pair["camera_index"]; li = pair["lidar_index"]
                matched_cam.add(ci); matched_lidar.add(li)
                cam = cam_list.objects[ci]; track = lidar_tracks[li]
                x1,y1,x2,y2 = track["bbox"]
                cv2.rectangle(view, (x1,y1), (x2,y2), (255,0,255), 3)
                label = "%s <= %s IoU=%.2f d=%.0fpx" % (track["id"], cam.class_name, pair["iou"], pair["center_distance"])
                cv2.putText(view, label, (x1,max(20,y1-8)), cv2.FONT_HERSHEY_SIMPLEX,.45,(255,0,255),1,cv2.LINE_AA)

            for i, cam in enumerate(cam_list.objects):
                if i in matched_cam: continue
                x1,y1,x2,y2 = cam.bbox
                cv2.rectangle(view,(x1,y1),(x2,y2),(255,255,0),1)
                cv2.putText(view,"CAM %s"%cam.class_name,(x1,max(20,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,0),1,cv2.LINE_AA)
            for i, track in enumerate(lidar_tracks):
                if i in matched_lidar: continue
                x1,y1,x2,y2 = track["bbox"]
                cv2.rectangle(view,(x1,y1),(x2,y2),(0,255,0),1)
                cv2.putText(view,"LIDAR %s"%track["id"],(x1,max(20,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,0),1,cv2.LINE_AA)

            title = "V0.4.2 Camera/LiDAR Association | cam=%d lidar=%d matched=%d" % (len(cam_list.objects), len(lidar_tracks), len(pairs))
            cv2.putText(view,title,(20,30),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2,cv2.LINE_AA)
            cv2.imshow("RoadsideStation V0.4.2 Association",view)
            if cv2.waitKey(1)&0xff in (27,ord("q")): break
            time.sleep(.01)
    finally:
        station.stop(); cv2.destroyAllWindows(); print("V0.4.2 association viewer stopped cleanly.")


if __name__ == "__main__":
    main()
