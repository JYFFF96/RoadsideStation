from __future__ import print_function

import argparse
import time
import yaml
import cv2

from roadside.carla_station import CarlaRoadsideStation
from roadside.camera_detector import YoloV5OnnxDetector


def load_config(path):
    with open(path, "r") as fp:
        return yaml.safe_load(fp)


def main():
    ap = argparse.ArgumentParser(description="V0.4 RGB camera object detection")
    ap.add_argument("--config", default="config/roadside.yaml")
    args = ap.parse_args()
    cfg = load_config(args.config)
    det_cfg = cfg.get("camera_detection", {})
    detector = YoloV5OnnxDetector(
        det_cfg.get("model", "models/yolov5n.onnx"),
        input_size=det_cfg.get("input_size", 640),
        confidence=det_cfg.get("confidence", 0.35),
        nms=det_cfg.get("nms", 0.45))

    station = CarlaRoadsideStation(cfg)
    station.start()
    print("RoadsideStation V0.4 camera detector started.")
    print("Press Q or ESC in the image window to exit.")
    try:
        last_frame = None
        while True:
            camera, _, _ = station.cache.snapshot()
            if camera is None:
                time.sleep(.02)
                continue
            frame_id, bgra = camera
            if frame_id == last_frame:
                time.sleep(.005)
                continue
            last_frame = frame_id
            bgr = bgra[:, :, :3].copy()
            detections = detector.detect(bgr)
            for det in detections:
                x1, y1, x2, y2 = det["bbox"]
                label = "%s %.2f" % (det["class_name"], det["confidence"])
                cv2.rectangle(bgr, (x1, y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(bgr, label, (x1, max(20, y1 - 7)),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(bgr, "V0.4 Camera Detection | detections=%d" % len(detections),
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("RoadsideStation V0.4 Camera Detection", bgr)
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord("q")):
                break
    finally:
        station.stop()
        cv2.destroyAllWindows()
        print("V0.4 camera detector stopped cleanly.")


if __name__ == "__main__":
    main()
