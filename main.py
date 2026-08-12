from __future__ import print_function
import signal,sys,time,yaml,math
import carla
from roadside.carla_station import CarlaRoadsideStation
from roadside.fusion import SimpleFusion
from roadside.camera_fusion import CameraProjector
from roadside.camera_lidar_association import associate_camera_to_lidar
from roadside.fused_objects import build_fused_object_list
from roadside.ground_truth_eval import GroundTruthEvaluator
from roadside.lidar_projection import project_lidar_tracks
from roadside.sim_camera_truth import make_truth_camera_objects
from roadside.messages import encode_object_list,encode_rsm
from roadside.mqtt_pub import MqttPublisher

_STOP_REQUESTED=False

def _request_stop(signum,frame):
 global _STOP_REQUESTED
 if not _STOP_REQUESTED:print("\nStop requested. Shutting down RoadsideStation...")
 _STOP_REQUESTED=True

def load_config(path="config/roadside.yaml"):
 with open(path,"r") as fp:return yaml.safe_load(fp)

def _try_load_configured_map(config):
 cc=config.get("carla",{});target=cc.get("map")
 if not target or not cc.get("load_world_on_start",False):return
 print("WARNING: load_world_on_start=true will reload the CARLA world and remove existing traffic.")
 client=carla.Client(cc.get("host","127.0.0.1"),int(cc.get("port",2000)));client.set_timeout(float(cc.get("timeout",60.0)))
 current=client.get_world().get_map().name.split("/")[-1];print("Experimental CARLA map switch enabled. Target map: %s"%target);print("Current CARLA map: %s"%current)
 if current!=target:
  print("Calling client.load_world('%s')..."%target);world=client.load_world(target);print("CARLA map switch completed: %s"%world.get_map().name.split("/")[-1]);time.sleep(2.0)

def _pct(v):return "-" if v is None else "%.1f%%"%(100.0*float(v))
def _meters(v):return "-" if v is None else "%.2fm"%float(v)

def _print_traffic_status(station,config):
 tc=config.get("traffic",{});mode=tc.get("mode","external");source=tc.get("source","carla_generate_traffic")
 try:
  vehicles=len(station.world.get_actors().filter("vehicle.*"));walkers=len(station.world.get_actors().filter("walker.pedestrian.*"))
 except Exception:
  vehicles=0;walkers=0
 print("Traffic mode: %s (source=%s)"%(mode,source))
 print("Attached to existing CARLA traffic: %d vehicles, %d walkers"%(vehicles,walkers))
 if mode=="external" and vehicles==0:
  print("NOTE: no vehicles are present. Start CARLA PythonAPI/examples/generate_traffic.py in another terminal.")

def main():
 global _STOP_REQUESTED
 signal.signal(signal.SIGINT,_request_stop);signal.signal(signal.SIGTERM,_request_stop)
 config=load_config();_try_load_configured_map(config);sid=config["station"]["id"];station=CarlaRoadsideStation(config);fusion=SimpleFusion(sid,config["fusion"]);pub=MqttPublisher(config["mqtt"])
 print("RoadsideStation V0.5.3 Ground Removal starting...")
 station.start();_print_traffic_status(station,config);fusion.set_world_transform(station.lidar_transform);fusion.set_radar_transform(station.radar_transform);fusion.set_ground_reference(station.junction_center.z if station.junction_center is not None else None);fusion.set_candidate_validator(station.is_driving_roi);pub.connect()
 if config.get("fusion",{}).get("ground_removal_enabled",True):
  gz=station.junction_center.z if station.junction_center is not None else None
  print("Ground removal: enabled reference_z=%s clearance=%.2fm"%(("-" if gz is None else "%.2f"%gz),float(config.get("fusion",{}).get("ground_clearance",0.30))))
 projector=None;width=0;height=0
 if station.camera_transform is not None:
  cc=config["camera"];width=int(cc.get("width",1280));height=int(cc.get("height",720));projector=CameraProjector(width,height,cc.get("fov",90),station.camera_transform)
 camera_id=config.get("camera",{}).get("id","CAM_01");camera_source=config.get("camera_fusion",{}).get("source","none");assoc_cfg=config.get("camera_lidar_association",{})
 eval_cfg=config.get("evaluation",{});evaluator=None
 if eval_cfg.get("enabled",True):
  def eval_center():
   if station.junction_center is not None:return station.junction_center
   if station.base_transform is not None:return station.base_transform.location
   return None
  evaluator=GroundTruthEvaluator(station.world,eval_center,eval_cfg)
 print("CARLA roadside sensors started: %d"%len(station.sensors));print("V0.5.3 CARLA evaluator: %s"%("enabled" if evaluator else "disabled"))
 if evaluator:print("Evaluation radius: %.1fm, bins=%s, truth-track gate: %.1fm"%(evaluator.radius,evaluator.range_bins,evaluator.match_distance))
 print("ARCH: external CARLA traffic -> roadside sensors -> ground removal -> perception/fusion -> ObjectList/RSM")
 print("ARCH: Ground Truth is evaluation-only and never enters perception/fusion/FusedObjectList.")
 print("Camera fusion source: %s"%camera_source)
 if camera_source=="carla_truth":print("NOTE: CamObjects is simulation truth visibility, NOT real camera detector recall.")
 print("Static background calibration: keep the scene empty until calibration is READY")
 last=0.0;last_eval=0.0;eval_interval=float(eval_cfg.get("report_interval",2.0))
 try:
  while not _STOP_REQUESTED:
   camera,lidar,radar=station.cache.snapshot();ol=fusion.fuse(lidar[1] if lidar else None,radar[1] if radar else None);camera_objects=[];pairs=[]
   if projector is not None and camera is not None:
    projected=project_lidar_tracks(projector,fusion.last_tracked_candidates,width,height)
    if camera_source=="carla_truth":
     cam_list=make_truth_camera_objects(station.world,projector,camera_id,width,height,frame_id=camera[0],timestamp=ol.timestamp);camera_objects=cam_list.objects
     for pair in associate_camera_to_lidar(camera_objects,projected,min_iou=assoc_cfg.get("min_iou",.05),max_center_distance=assoc_cfg.get("max_center_distance",120.0)):
      p=dict(pair);p["lidar_index"]=projected[pair["lidar_index"]]["source_index"];pairs.append(p)
   fol=build_fused_object_list(sid,fusion.last_tracked_candidates,ol.timestamp,camera_objects,pairs);oj=encode_object_list(ol);rj=encode_rsm(ol);now=time.time()
   if now-last>=1.0:
    s=fusion.last_stats;cf=camera[0] if camera else "-";bg=("READY/%d cells"%s["background_cells"] if s["background_ready"] else "LEARNING %.1fs"%s["background_remaining"]);rmin=s.get("radar_nearest_min");rmin_txt="-" if rmin is None else "%.2fm"%rmin
    print("[RSU %s | %s] Camera:%s LiDAR:%d pts -> Ground:-%d => %d pts/%d clusters -> ROI:%d -> BG:%d Radar:%d/%d world RadarMatched:%d Nearest:%s Tracks:%d Fused:%d CamObjects:%d CamMatched:%d BG:%s"%(sid,station.map_name,cf,s["lidar_points"],s.get("ground_removed_points",0),s.get("lidar_points_after_ground",s["lidar_points"]),s["lidar_clusters"],s["roi_candidates"],s["background_candidates"],s["radar_detections"],s.get("radar_world_points",0),s.get("radar_matched_objects",0),rmin_txt,s["tracked_objects"],len(fol.objects),len(camera_objects),len(pairs),bg))
    for idx,o in enumerate(fol.objects[:10]):
     t=fusion.last_tracked_candidates[idx];size=o.size;rs="-" if o.radar_speed is None else "%.2f"%o.radar_speed;cam="-" if o.camera is None else "%s box=%s"%(o.camera.get("cameraId","?"),o.camera.get("bbox"));near=t.get("radar_nearest_xy");near_txt="-" if near is None else "%.2f"%near;raw_speed=math.hypot(t.get("raw_vx",0),t.get("raw_vy",0));fused_speed=math.hypot(o.vx,o.vy)
     print("  %-12s type=%-7s pos=(%7.2f,%7.2f,%5.2f) vel=(%6.2f,%6.2f) speed=%.2f raw=%.2f size=(%.2f,%.2f,%.2f) radar=%s near=%sm hits=%d cam=%s conf=%.2f src=%s"%(o.object_id,o.object_type,o.x,o.y,o.z,o.vx,o.vy,fused_speed,raw_speed,size[0],size[1],size[2],rs,near_txt,int(t.get("radar_hits",0)),cam,o.confidence,"+".join(o.sources)))
    last=now
   if evaluator is not None and now-last_eval>=eval_interval:
    s=fusion.last_stats;ev=evaluator.evaluate(fusion.last_tracked_candidates,camera_objects,pairs,s.get("radar_matched_objects",0))
    print("[EVAL %.0fm] Truth:%d Tracks:%d Matched:%d Missed:%d FP:%d Recall:%s Precision:%s PosErr(avg/max):%s/%s RadarMatched:%d CamVisibleTruth:%d CamLiDAR:%d"%(evaluator.radius,ev["truth"],ev["detected"],ev["matched"],ev["missed"],ev["false_positive"],_pct(ev["recall"]),_pct(ev["precision"]),_meters(ev["mean_position_error"]),_meters(ev["max_position_error"]),ev["radar_matched"],ev["camera_visible"],ev["camera_lidar_matched"]))
    for b in ev.get("range_bins",[]):
     print("  [RANGE %02.0f-%02.0fm] Truth:%d Tracks:%d Matched:%d Missed:%d FP:%d Recall:%s Precision:%s PosErr:%s"%(b["min_range"],b["max_range"],b["truth"],b["detected"],b["matched"],b["missed"],b["false_positive"],_pct(b["recall"]),_pct(b["precision"]),_meters(b["mean_position_error"])))
    last_eval=now
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
