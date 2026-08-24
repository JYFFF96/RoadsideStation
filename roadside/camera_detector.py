from __future__ import print_function

import os
import time
import cv2
import numpy as np

ROAD_CLASSES = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}


class CameraDetector(object):
    """Detector interface used by fusion code.

    Implementations must return a list of dictionaries with:
      class_id, class_name, confidence, bbox[x1,y1,x2,y2], center[u,v]
    The fusion layer therefore does not depend on YOLO, TensorRT or vendor SDKs.
    """
    name = "base"

    def detect(self, bgr):
        raise NotImplementedError

    def report(self):
        return {"name":self.name,"runtime":"none","frames":0,"detections":0,
                "latency_ms_avg":0.0,"latency_ms_max":0.0,"classes":{}}


class NullCameraDetector(CameraDetector):
    name = "null"

    def detect(self, bgr):
        return []


class YoloV5OnnxDetector(CameraDetector):
    name = "yolov5_onnx_auto"

    def __init__(self, model_path, input_size=640, confidence=0.35, nms=0.45):
        if not os.path.exists(model_path):
            raise IOError("Camera model not found: %s" % model_path)
        self.model_path = model_path
        self.input_size = int(input_size)
        self.confidence = float(confidence)
        self.nms = float(nms)
        self.net = None
        self.session = None
        self.input_name = None
        self.runtime = None
        self.stats={"frames":0,"detections":0,"latency_ms_total":0.0,
                    "latency_ms_max":0.0,"classes":{}}
        opencv_error = None
        try:
            self.net = cv2.dnn.readNetFromONNX(model_path)
            self.runtime = "opencv"
        except Exception as exc:
            opencv_error = exc
        if self.net is None:
            try:
                import onnxruntime as ort
                self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
                self.input_name = self.session.get_inputs()[0].name
                self.runtime = "onnxruntime"
            except Exception as exc:
                raise RuntimeError(
                    "OpenCV DNN rejected this ONNX model (%s). ONNX Runtime fallback also failed (%s). "
                    "For Python 3.7 install it with: python3.7 -m pip install onnxruntime==1.14.1"
                    % (opencv_error, exc)
                )
        self.name = "yolov5_onnx_%s" % self.runtime

    def _record(self, result, started):
        elapsed=max(0.0,(time.time()-started)*1000.0);self.stats["frames"]+=1
        self.stats["detections"]+=len(result or [])
        self.stats["latency_ms_total"]+=elapsed
        self.stats["latency_ms_max"]=max(self.stats["latency_ms_max"],elapsed)
        for item in result or []:
            name=str(item.get("class_name","unknown"));classes=self.stats["classes"]
            classes[name]=classes.get(name,0)+1
        return result

    def report(self):
        item=dict(self.stats);item["classes"]=dict(self.stats["classes"])
        item["latency_ms_avg"]=(self.stats["latency_ms_total"]/
                                self.stats["frames"] if self.stats["frames"] else 0.0)
        item["runtime"]=self.runtime;item["name"]=self.name
        return item

    def detect(self, bgr):
        started=time.time()
        h, w = bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(bgr, 1.0 / 255.0, (self.input_size, self.input_size), swapRB=True, crop=False)
        if self.runtime == "opencv":
            self.net.setInput(blob)
            out = self.net.forward()
        else:
            out = self.session.run(None, {self.input_name: blob})
        if isinstance(out, (list, tuple)): out = out[0]
        pred = np.asarray(out)
        if pred.ndim == 3: pred = pred[0]
        if pred.ndim != 2 or pred.shape[1] < 6:return self._record([],started)
        sx = float(w) / self.input_size; sy = float(h) / self.input_size
        boxes=[]; scores=[]; class_ids=[]
        for row in pred:
            obj=float(row[4])
            if obj < self.confidence: continue
            cs=row[5:]; cid=int(np.argmax(cs))
            if cid not in ROAD_CLASSES: continue
            score=obj*float(cs[cid])
            if score < self.confidence: continue
            cx,cy,bw,bh=[float(v) for v in row[:4]]
            boxes.append([int((cx-bw*.5)*sx),int((cy-bh*.5)*sy),int(bw*sx),int(bh*sy)])
            scores.append(score); class_ids.append(cid)
        if not boxes:return self._record([],started)
        keep=cv2.dnn.NMSBoxes(boxes,scores,self.confidence,self.nms)
        if len(keep)==0:return self._record([],started)
        result=[]
        for idx in np.asarray(keep).reshape(-1):
            i=int(idx);x,y,ww,hh=boxes[i];x1=max(0,x);y1=max(0,y);x2=min(w-1,x+ww);y2=min(h-1,y+hh)
            result.append({"class_id":class_ids[i],"class_name":ROAD_CLASSES[class_ids[i]],"confidence":float(scores[i]),"bbox":[x1,y1,x2,y2],"center":[(x1+x2)*.5,(y1+y2)*.5]})
        return self._record(result,started)


def create_camera_detector(config):
    """Factory: select implementation without changing callers/fusion code."""
    cfg=config or {}; backend=str(cfg.get("backend","yolov5_onnx_auto")).lower()
    if backend in ("none","null","disabled"):
        return NullCameraDetector()
    if backend in ("yolov5","yolov5_onnx","yolov5_onnx_auto","yolov5_onnx_opencv"):
        return YoloV5OnnxDetector(cfg.get("model","models/yolov5n.onnx"),cfg.get("input_size",640),cfg.get("confidence",.35),cfg.get("nms",.45))
    raise ValueError("Unsupported camera detector backend: %s" % backend)
