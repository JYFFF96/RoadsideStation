from __future__ import print_function

import argparse
import itertools
import time
import yaml
import cv2
import numpy as np

from roadside.carla_station import CarlaRoadsideStation
from roadside.fusion import SimpleFusion
from roadside.camera_fusion import CameraProjector


def load_config(path):
    with open(path, "r") as fp:
        return yaml.safe_load(fp)


def box_corners(center, extent):
    """Axis-aligned world-space box around a fused LiDAR candidate.

    The current cluster extent is LiDAR-axis-aligned. For V0.3.4 validation we
    use it as an approximate 3D object box and pad it slightly so the projected
    2D rectangle is easy to inspect visually.
    """
    x, y, z = center
    ex, ey, ez = [max(0.3, float(v)) for v in extent]
    hx = max(0.5, ex * 0.6)
    hy = max(0.5, ey * 0.6)
    hz = max(0.5, ez * 0.6)
    return [(x + sx * hx, y + sy * hy, z + sz * hz)
            for sx, sy, sz in itertools.product((-1.0, 1.0), repeat=3)]


def projected_rect(projector, center, extent, width, height):
    pixels = []
    for p3 in box_corners(center, extent):
        p = projector.project(p3[0], p3[1], p3[2])
        if p is not None:
            pixels.append((int(p["u"]), int(p["v"])))
    if len(pixels) < 2:
        p = projector.project(center[0], center[1], center[2])
        if p is None:
            return None
        u, v = int(p["u"]), int(p["v"])
        return (max(0, u - 25), max(0, v - 20), min(width - 1, u + 25), min(height - 1, v + 20))
    xs = [p[0] for p in pixels]; ys = [p[1] for p in pixels]
    return (max(0, min(xs)), max(0, min(ys)), min(width - 1, max(xs)), min(height - 1, max(ys)))


def nearest_candidate(obj, candidates, max_distance=3.5):
    best = None; best_d2 = float(max_distance) ** 2
    for c in candidates:
        dx = float(c["x"]) - obj.x; dy = float(c["y"]) - obj.y; dz = float(c["z"]) - obj.z
        d2 = dx*dx + dy*dy + dz*dz
        if d2 < best_d2:
            best_d2 = d2; best = c
    return best


def main():
    ap = argparse.ArgumentParser(description="V0.3.4 LiDAR 3D cluster boxes projected onto RGB")
    ap.add_argument("--config", default="config/roadside.yaml")
    args = ap.parse_args(); cfg = load_config(args.config)
    sid = cfg["station"]["id"]
    station = CarlaRoadsideStation(cfg); fusion = SimpleFusion(sid, cfg["fusion"])
    station.start(); fusion.set_world_transform(station.lidar_transform); fusion.set_candidate_validator(station.is_driving_roi)
    cc = cfg["camera"]; width = int(cc.get("width", 1280)); height = int(cc.get("height", 720)); fov = float(cc.get("fov", 90))
    projector = CameraProjector(width, height, fov, station.camera_transform)
    print("V0.3.4 started. Empty scene until BG READY, then start traffic.")
    print("Green rectangle = projected LiDAR cluster extent; red cross = tracked center.")
    print("Press Q or ESC in the image window to exit.")
    try:
        while True:
            camera, lidar, radar = station.cache.snapshot()
            if camera is None or lidar is None:
                time.sleep(.02); continue
            ol = fusion.fuse(lidar[1], radar[1] if radar else None)
            bgra = camera[1]; view = bgra[:, :, :3].copy(); drawn = 0
            candidates = fusion.last_dynamic_candidates
            for obj in ol.objects:
                cand = nearest_candidate(obj, candidates)
                if cand is None: continue
                p = projector.project(obj.x, obj.y, obj.z)
                rect = projected_rect(projector, (cand["x"], cand["y"], cand["z"]), cand.get("extent", [2, 1, 1]), width, height)
                if p is None or rect is None: continue
                x1, y1, x2, y2 = rect; u, v = int(p["u"]), int(p["v"]); drawn += 1
                cv2.rectangle(view, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.drawMarker(view, (u,v), (0,0,255), cv2.MARKER_CROSS, 16, 2)
                speed = (obj.vx*obj.vx + obj.vy*obj.vy) ** 0.5
                label = "%s %.1fm/s %s" % (obj.object_id, speed, "+".join(obj.sources))
                cv2.putText(view, label, (x1, max(20,y1-8)), cv2.FONT_HERSHEY_SIMPLEX, .45, (0,255,0), 1, cv2.LINE_AA)
            s = fusion.last_stats
            bg = "READY/%d" % s["background_cells"] if s["background_ready"] else "LEARNING %.1fs" % s["background_remaining"]
            cv2.putText(view, "V0.3.4 projected LiDAR boxes | tracks=%d boxes=%d BG=%s" % (len(ol.objects),drawn,bg), (20,30), cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2,cv2.LINE_AA)
            cv2.imshow("RoadsideStation V0.3.4 LiDAR Box -> RGB", view)
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord('q')): break
            time.sleep(.01)
    finally:
        station.stop(); cv2.destroyAllWindows(); print("V0.3.4 viewer stopped cleanly.")


if __name__ == "__main__":
    main()
