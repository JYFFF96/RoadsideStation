from __future__ import print_function
import signal,sys,time,yaml
from roadside.carla_station import CarlaRoadsideStation
from roadside.fusion import SimpleFusion
from roadside.camera_fusion import CameraProjector
from roadside.messages import encode_object_list,encode_rsm
from roadside.mqtt_pub import MqttPublisher
_STOP_REQUESTED=False
def _request_stop(signum,frame):
 global _STOP_REQUESTED
 if not _STOP_REQUESTED:print("\nStop requested. Shutting down RoadsideStation...")
 _STOP_REQUESTED=True
def load_config(path="config/roadside.yaml"):
 with open(path,"r") as fp:return yaml.safe_load(fp)
def main():
 global _STOP_REQUESTED
 signal.signal(signal.SIGINT,_request_stop);signal.signal(signal.SIGTERM,_request_stop)
 config=load_config();sid=config["station"]["id"];station=CarlaRoadsideStation(config);fusion=SimpleFusion(sid,config["fusion"]);pub=MqttPublisher(config["mqtt"])
 print("RoadsideStation V0.3 starting...");station.start();fusion.set_world_transform(station.lidar_transform);fusion.set_candidate_validator(station.is_driving_roi);pub.connect()
 projector=None
 if station.camera_transform is not None:
  cc=config["camera"];projector=CameraProjector(cc.get("width",1280),cc.get("height",720),cc.get("fov",90),station.camera_transform)
 print("CARLA roadside sensors started: %d"%len(station.sensors));print("V0.3 Camera-LiDAR projection: enabled" if projector else "V0.3 Camera-LiDAR projection: disabled")
 print("Static background calibration: keep the scene empty until calibration is READY")
 last=0.0
 try:
  while not _STOP_REQUESTED:
   camera,lidar,radar=station.cache.snapshot();ol=fusion.fuse(lidar[1] if lidar else None,radar[1] if radar else None)
   visible=projector.annotate_candidates(fusion.last_dynamic_candidates) if projector else []
   oj=encode_object_list(ol);rj=encode_rsm(ol);now=time.time()
   if now-last>=1.0:
    s=fusion.last_stats;cf=camera[0] if camera else "-";bg=("READY/%d cells"%s["background_cells"] if s["background_ready"] else "LEARNING %.1fs"%s["background_remaining"])
    print("[RSU %s | %s] Camera:%s LiDAR:%d pts/%d clusters -> ROI:%d -> BG:%d Radar:%d Tracks:%d CamVisible:%d BG:%s"%(sid,station.map_name,cf,s["lidar_points"],s["lidar_clusters"],s["roi_candidates"],s["background_candidates"],s["radar_detections"],s["tracked_objects"],len(visible),bg))
    for o in ol.objects[:10]:print("  %-12s Xw=%8.2f Yw=%8.2f Zw=%6.2f vx=%6.2f vy=%6.2f conf=%.2f src=%s"%(o.object_id,o.x,o.y,o.z,o.vx,o.vy,o.confidence,"+".join(o.sources)))
    for c in visible[:5]:print("    camera projection: u=%7.1f v=%7.1f depth=%6.1fm world=(%.1f, %.1f, %.1f)"%(c["camera_u"],c["camera_v"],c["camera_depth"],c["x"],c["y"],c["z"]))
    last=now
   m=config["mqtt"];pub.publish(m["topic_object_list"],oj);pub.publish(m["topic_rsm"],rj);time.sleep(.05)
 except KeyboardInterrupt:_STOP_REQUESTED=True
 finally:
  print("Stopping RoadsideStation sensors and MQTT...")
  try:pub.close()
  except Exception as e:print("MQTT shutdown warning: %s"%e)
  try:station.stop()
  except Exception as e:print("Sensor shutdown warning: %s"%e)
  print("RoadsideStation stopped cleanly.")
 return 0
if __name__=="__main__":sys.exit(main())
