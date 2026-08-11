from __future__ import print_function

import os
import cv2
import numpy as np


COCO_CLASSES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light",
    "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
    "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
    "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch",
    "potted plant","bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone",
    "microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush"
]

ROAD_CLASSES = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}


class YoloV5OnnxDetector(object):
    """YOLOv5 ONNX detector using OpenCV DNN only.

    This keeps the RoadsideStation Python 3.7 environment lightweight: no
    ultralytics/torch runtime is needed on the target machine after export.
    """
    def __init__(self, model_path, input_size=640, confidence=0.35, nms=0.45):
        if not os.path.exists(model_path):
            raise IOError("Camera model not found: %s" % model_path)
        self.model_path = model_path
        self.input_size = int(input_size)
        self.confidence = float(confidence)
        self.nms = float(nms)
        self.net = cv2.dnn.readNetFromONNX(model_path)

    def detect(self, bgr):
        h, w = bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(bgr, 1.0 / 255.0,
                                     (self.input_size, self.input_size),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        out = self.net.forward()
        if isinstance(out, (list, tuple)):
            out = out[0]
        pred = np.asarray(out)
        if pred.ndim == 3:
            pred = pred[0]
        if pred.ndim != 2 or pred.shape[1] < 6:
            return []

        sx = float(w) / float(self.input_size)
        sy = float(h) / float(self.input_size)
        boxes, scores, class_ids = [], [], []
        for row in pred:
            obj = float(row[4])
            if obj < self.confidence:
                continue
            class_scores = row[5:]
            cid = int(np.argmax(class_scores))
            if cid not in ROAD_CLASSES:
                continue
            score = obj * float(class_scores[cid])
            if score < self.confidence:
                continue
            cx, cy, bw, bh = [float(v) for v in row[:4]]
            x = int((cx - bw * 0.5) * sx)
            y = int((cy - bh * 0.5) * sy)
            ww = int(bw * sx)
            hh = int(bh * sy)
            boxes.append([x, y, ww, hh])
            scores.append(score)
            class_ids.append(cid)

        if not boxes:
            return []
        keep = cv2.dnn.NMSBoxes(boxes, scores, self.confidence, self.nms)
        if len(keep) == 0:
            return []
        keep = np.asarray(keep).reshape(-1)
        result = []
        for idx in keep:
            x, y, ww, hh = boxes[int(idx)]
            x1 = max(0, x); y1 = max(0, y)
            x2 = min(w - 1, x + ww); y2 = min(h - 1, y + hh)
            result.append({
                "class_id": class_ids[int(idx)],
                "class_name": ROAD_CLASSES[class_ids[int(idx)]],
                "confidence": float(scores[int(idx)]),
                "bbox": [x1, y1, x2, y2],
                "center": [(x1 + x2) * 0.5, (y1 + y2) * 0.5],
            })
        return result
