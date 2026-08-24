from __future__ import print_function
import argparse,os,sys,time,yaml
import numpy as np

PROJECT_ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:sys.path.insert(0,PROJECT_ROOT)
from roadside.camera_runtime_config import apply_camera_runtime_overrides

def main():
    parser=argparse.ArgumentParser(description="V0.6.12.8.2.2.70 camera reassociation shadow preflight")
    parser.add_argument("--config",default="config/roadside.yaml")
    parser.add_argument("--camera-model",default=None);args=parser.parse_args()
    with open(args.config,"r") as stream:config=yaml.safe_load(stream)
    config=apply_camera_runtime_overrides(config,"detector",args.camera_model)
    detection=config.get("camera_detection",{}) or {};model=str(detection.get("model",""))
    print("Camera detector preflight | backend=%s model=%s exists=%s"%(detection.get("backend","-"),os.path.abspath(model),os.path.isfile(model)))
    if not os.path.isfile(model):
        print("Camera detector preflight: BLOCKED | Camera model not found: %s"%model)
        return 2
    try:
        from roadside.camera_detector import create_camera_detector
        detector=create_camera_detector(detection)
    except Exception as exc:
        print("Camera detector preflight: BLOCKED | %s"%exc);return 2
    try:
        started=time.time();detections=detector.detect(np.zeros((720,1280,3),dtype=np.uint8))
        smoke_ms=(time.time()-started)*1000.0
    except Exception as exc:
        print("Camera detector preflight: BLOCKED_INFERENCE | %s"%exc);return 2
    print("Camera detector preflight: READY | detector=%s runtime=%s smoke=%.1fms detections=%d"%(detector.name,getattr(detector,"runtime","-"),smoke_ms,len(detections)))
    print("Run: python3.7 main.py --camera-source detector --camera-model %s"%model)
    return 0

if __name__=="__main__":sys.exit(main())
