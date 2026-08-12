from __future__ import print_function
import argparse,time,yaml,cv2
from roadside.carla_station import CarlaRoadsideStation
from roadside.camera_detector import create_camera_detector

def load_config(path):
    with open(path,"r") as fp:return yaml.safe_load(fp)

def main():
    ap=argparse.ArgumentParser(description="V0.4.1 pluggable RGB camera detection")
    ap.add_argument("--config",default="config/roadside.yaml");args=ap.parse_args();cfg=load_config(args.config)
    det_cfg=cfg.get("camera_detection",{});detector=create_camera_detector(det_cfg)
    station=CarlaRoadsideStation(cfg);station.start()
    print("RoadsideStation V0.4.1 camera detector started: %s"%detector.name);print("Press Q or ESC in the image window to exit.")
    try:
        last=None
        while True:
            camera,_,_=station.cache.snapshot()
            if camera is None:time.sleep(.02);continue
            fid,bgra=camera
            if fid==last:time.sleep(.005);continue
            last=fid;bgr=bgra[:,:,:3].copy();detections=detector.detect(bgr)
            for d in detections:
                x1,y1,x2,y2=d["bbox"];label="%s %.2f"%(d["class_name"],d["confidence"])
                cv2.rectangle(bgr,(x1,y1),(x2,y2),(255,255,0),2);cv2.putText(bgr,label,(x1,max(20,y1-7)),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,0),2,cv2.LINE_AA)
            cv2.putText(bgr,"V0.4.1 Camera Detector=%s | detections=%d"%(detector.name,len(detections)),(20,30),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,0),2,cv2.LINE_AA)
            cv2.imshow("RoadsideStation V0.4.1 Camera Detection",bgr)
            if cv2.waitKey(1)&0xff in (27,ord("q")):break
    finally:
        station.stop();cv2.destroyAllWindows();print("V0.4.1 camera detector stopped cleanly.")
if __name__=="__main__":main()
