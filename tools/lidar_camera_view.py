from __future__ import print_function

import argparse
import time
import yaml
import cv2

from roadside.carla_station import CarlaRoadsideStation
from roadside.fusion import SimpleFusion
from roadside.camera_fusion import CameraProjector


def load_config(path):
    with open(path, "r") as fp:
        return yaml.safe_load(fp)


def main():
    ap = argparse.ArgumentParser(description="V0.3.3 RoadsideStation LiDAR tracks on RGB camera")
    ap.add_argument("--config", default="config/roadside.yaml")
    args = ap.parse_args()

    cfg = load_config(args.config)
    sid = cfg["station"]["id"]
    station = CarlaRoadsideStation(cfg)
    fusion = SimpleFusion(sid, cfg["fusion"])
    station.start()
    fusion.set_world_transform(station.lidar_transform)
    fusion.set_candidate_validator(station.is_driving_roi)

    cc = cfg["camera"]
    width = int(cc.get("width", 1280)); height = int(cc.get("height", 720)); fov = float(cc.get("fov", 90))
    projector = CameraProjector(width, height, fov, station.camera_transform)
    print("V0.3.3 started. Keep scene empty until BG is READY; then start traffic.")
    print("Press Q or ESC in the image window to exit.")

    try:
        while True:
            camera, lidar, radar = station.cache.snapshot()
            if camera is None or lidar is None:
                time.sleep(.02); continue
            ol = fusion.fuse(lidar[1], radar[1] if radar else None)

            # SensorCache already stores the camera image as a BGRA numpy array
            # via roadside.sensors.image_to_bgra(). Do not try to read raw_data
            # again here.
            bgra = camera[1]
            if bgra is None or getattr(bgra, "ndim", 0) != 3 or bgra.shape[2] < 3:
                time.sleep(.02); continue
            view = bgra[:, :, :3].copy()

            visible = 0
            for obj in ol.objects:
                p = projector.project(obj.x, obj.y, obj.z)
                if p is None: continue
                visible += 1; u = int(p["u"]); v = int(p["v"]); d = p["depth"]
                speed = (obj.vx * obj.vx + obj.vy * obj.vy) ** 0.5
                cv2.circle(view, (u, v), 9, (0, 0, 255), 2)
                cv2.drawMarker(view, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
                label = "%s %.1fm %.1fm/s" % (obj.object_id, d, speed)
                cv2.putText(view, label, (max(0, u-75), max(20, v-15)), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 0, 255), 1, cv2.LINE_AA)
            s = fusion.last_stats
            bg = "READY/%d" % s["background_cells"] if s["background_ready"] else "LEARNING %.1fs" % s["background_remaining"]
            text = "V0.3.3 LiDAR Tracks | tracks=%d visible=%d BG=%s" % (len(ol.objects), visible, bg)
            cv2.putText(view, text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("RoadsideStation LiDAR Track -> RGB", view)
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord("q")): break
            time.sleep(.01)
    finally:
        station.stop(); cv2.destroyAllWindows(); print("V0.3.3 viewer stopped cleanly.")


if __name__ == "__main__":
    main()
