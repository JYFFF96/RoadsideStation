from __future__ import print_function
import signal,sys,time,yaml,math
import carla
from roadside.carla_station import CarlaRoadsideStation
from roadside.fusion import SimpleFusion
from roadside.detection_stability import DetectionStabilityDiagnostics
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
def _num(v):return "-" if v is None else "%.2f"%float(v)

def _print_traffic_status(station,config):
 tc=config.get("traffic",{});mode=tc.get("mode","external");source=tc.get("source","carla_generate_traffic")
 try:vehicles=len(station.world.get_actors().filter("vehicle.*"));walkers=len(station.world.get_actors().filter("walker.pedestrian.*"))
 except Exception:vehicles=0;walkers=0
 print("Traffic mode: %s (source=%s)"%(mode,source));print("Attached to existing CARLA traffic: %d vehicles, %d walkers"%(vehicles,walkers))
 if mode=="external" and vehicles==0:print("NOTE: no vehicles are present. Start a traffic generator in another terminal.")

def _print_stage(name,m):
 print("  [STAGE %-8s] Candidates:%d Matched:%d Missed:%d FP:%d Recall:%s Precision:%s"%(name,m["detected"],m["matched"],m["missed"],m["false_positive"],_pct(m["recall"]),_pct(m["precision"])))
 for b in m.get("range_bins",[]):
  print("    [%02.0f-%02.0fm] Cand:%d Match:%d Miss:%d FP:%d Recall:%s"%(b["min_range"],b["max_range"],b["detected"],b["matched"],b["missed"],b["false_positive"],_pct(b["recall"])))

def _print_detection_stability(ds):
 print("  [DETECTION STABILITY] Dyn:%d Persistent:%d New:%d OneFrameLost:%d Reassoc:%d Fragmented:%d Lost:%d Jump(avg/max):%.2f/%.2fm ExtentDelta:%.2f"%(ds.get("candidates",0),ds.get("persistent",0),ds.get("new",0),ds.get("one_frame_lost",0),ds.get("reassociated",0),ds.get("fragmented",0),ds.get("lost",0),float(ds.get("mean_jump",0.0)),float(ds.get("max_jump",0.0)),float(ds.get("mean_extent_delta",0.0))))

def _print_detection_drop(dd):
 print("  [DETECTION DROP] Truth:%d Pass:%d NoGeometry:%d ROIReject:%d ROILost:%d ScoreReject:%d ScoreLost:%d DynamicDrop:%d"%(dd.get("truth",0),dd.get("pass",0),dd.get("no_geometry_candidate",0),dd.get("roi_reject",0),dd.get("roi_lost",0),dd.get("score_reject",0),dd.get("score_lost",0),dd.get("dynamic_drop",0)))
 for b in dd.get("range_bins",[]):
  print("    [%02.0f-%02.0fm] Truth:%d Pass:%d NoGeometry:%d ROIReject:%d ScoreReject:%d DynamicDrop:%d"%(b.get("min_range",0.0),b.get("max_range",0.0),b.get("truth",0),b.get("pass",0),b.get("no_geometry_candidate",0),b.get("roi_reject",0)+b.get("roi_lost",0),b.get("score_reject",0)+b.get("score_lost",0),b.get("dynamic_drop",0)))

def main():
 global _STOP_REQUESTED
 signal.signal(signal.SIGINT,_request_stop);signal.signal(signal.SIGTERM,_request_stop)
 config=load_config();_try_load_configured_map(config);sid=config["station"]["id"];station=CarlaRoadsideStation(config);fusion=SimpleFusion(sid,config["fusion"]);pub=MqttPublisher(config["mqtt"])
 dc=config.get("detection_stability",{});detdiag=DetectionStabilityDiagnostics(dc.get("match_distance",3.5),dc.get("max_missed_frames",2),dc.get("fragmentation_distance",2.0));ds={}
 print("RoadsideStation V0.6.8 Detection Drop Reason Diagnostics starting...")
 station.start();_print_traffic_status(station,config);fusion.set_world_transform(station.lidar_transform);fusion.set_radar_transform(station.radar_transform);fusion.set_ground_reference(station.junction_center.z if station.junction_center is not None else None);fusion.set_candidate_validator(station.validate_driving_roi);pub.connect()
 fc=config.get("fusion",{})
 if fc.get("ground_removal_enabled",True):
  gz=station.junction_center.z if station.junction_center is not None else None;print("Ground removal: enabled reference_z=%s clearance=%.2fm"%(("-" if gz is None else "%.2f"%gz),float(fc.get("ground_clearance",0.30))))
 print("LiDAR clustering: %s"%("hybrid range-adaptive (near geometry filtered, mid 3D, far multi-scale BEV)" if fc.get("range_adaptive_clustering",False) else "fixed"))
 if fc.get("range_adaptive_clustering",False):print("LiDAR range bands: %s"%fc.get("range_bands",[]))
 near_band=(fc.get("range_bands") or [{}])[0]
 print("Teaching acceptance focus: 0-30m UNCHANGED | near geometry L=%.2f..%.1f W=%.2f..%.1f H=%.2f..%.1fm | merge gap=%.2fm"%(float(near_band.get("min_length",0.75)),float(near_band.get("max_length",6.5)),float(near_band.get("min_width",0.55)),float(near_band.get("max_width",3.4)),float(near_band.get("min_height",0.45)),float(near_band.get("max_height",2.6)),float(fc.get("near_merge_gap",0.65))))
 print("Road ROI margins: near=%.1fm mid=%.1fm far=%.1fm"%(float(fc.get("road_roi_margin",3.0)),float(fc.get("road_roi_margin_mid",4.2)),float(fc.get("road_roi_margin_far",4.5))))
 if fc.get("geometry_aware_roi_enabled",False):
  print("Geometry-aware ROI default/far: overlap>=%.2fm center_excess<=%.2fm"%(float(fc.get("geometry_aware_roi_min_overlap",0.25)),float(fc.get("geometry_aware_roi_max_center_excess",1.8))))
  print("Mid-range ROI rescue 30-50m: overlap>=%.2fm center_excess<=%.2fm half_width<=%.2fm"%(float(fc.get("geometry_aware_roi_mid_min_overlap",0.10)),float(fc.get("geometry_aware_roi_mid_max_center_excess",2.6)),float(fc.get("geometry_aware_roi_mid_max_half_width",2.0))))
 if fc.get("candidate_scoring_enabled",False):
  print("Far candidate scoring: >=%.0fm threshold=%.2f | >=%.0fm relaxed=%.2f"%(float(fc.get("candidate_scoring_min_range",50.0)),float(fc.get("candidate_scoring_threshold_far",0.46)),float(fc.get("candidate_scoring_far_relaxed_range",65.0)),float(fc.get("candidate_scoring_threshold_far_long",0.42))))
 else:print("Candidate scoring: disabled")
 print("Adaptive track persistence: %s | young=%d stable=%d far=%d frames | stable_hits=%d far>=%.0fm | low_score<%.2f edge>=%.2f | decay=%.2f"%("enabled" if fc.get("track_adaptive_coast_enabled",False) else "disabled",int(fc.get("track_coast_young_frames",1)),int(fc.get("track_coast_stable_frames",3)),int(fc.get("track_coast_far_frames",4)),int(fc.get("track_coast_stable_hits",3)),float(fc.get("track_coast_far_range",50.0)),float(fc.get("track_coast_low_score",0.55)),float(fc.get("track_coast_edge_ratio",0.90)),float(fc.get("track_coast_confidence_decay",.84))))
 print("Track Quality: %s | high>=%.2f medium>=%.2f | medium_coast<=%d low_coast<=%d | low coast requires hits>=%d | camera=+%.2f radar=+%.2f memory=%.1fs penalty=%.2f/miss"%("enabled" if fc.get("track_quality_enabled",True) else "disabled",float(fc.get("track_quality_high",.72)),float(fc.get("track_quality_medium",.50)),int(fc.get("track_quality_medium_coast_frames",2)),int(fc.get("track_quality_low_coast_frames",0)),int(fc.get("track_quality_low_min_hits_for_coast",3)),float(fc.get("track_quality_camera_bonus",.12)),float(fc.get("track_quality_radar_bonus",.08)),float(fc.get("track_quality_sensor_memory",1.5)),float(fc.get("track_quality_coast_penalty",.10))))
 print("Detection Stability diagnostics: observer-only | match<=%.1fm missed<=%d fragmentation<=%.1fm"%(detdiag.match_distance,detdiag.max_missed_frames,detdiag.fragmentation_distance))
 print("Detection Drop diagnostics: evaluation-only | Geometry -> ROI -> Score -> Dynamic | never feeds perception/fusion")
 print("Qt/C++ portability: quality/diagnostic policies use plain scalar candidate evidence only; no CARLA-specific logic.")
 print("Background filter: %s"%("enabled" if fc.get("background_filter_enabled",False) else "disabled"))
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
 print("CARLA roadside sensors started: %d"%len(station.sensors));print("V0.6.8 CARLA evaluator: %s"%("enabled" if evaluator else "disabled"))
 if evaluator:print("Evaluation radius: %.1fm, bins=%s, truth-track gate: %.1fm"%(evaluator.radius,evaluator.range_bins,evaluator.match_distance))
 print("ARCH: traffic -> ground removal -> road ROI -> far score -> Detection Stability(observer) -> tracker -> Track Quality -> Hit History Gate -> fusion")
 print("ARCH: Detection Drop diagnosis is GT evaluation-only and observes Geometry/ROI/Score/Dynamic outputs.")
 print("ARCH: Camera association writes generic confirmation evidence back to track state for the next cycle.")
 print("ARCH: Ground Truth is evaluation-only and never enters perception/fusion/FusedObjectList.")
 print("Camera fusion source: %s"%camera_source)
 if camera_source=="carla_truth":print("NOTE: CamObjects is simulation truth visibility, NOT real camera detector recall. Tracker receives only generic association confirmation, not truth actor data.")
 last=0.0;last_eval=0.0;eval_interval=float(eval_cfg.get("report_interval",2.0))
 try:
  while not _STOP_REQUESTED:
   camera,lidar,radar=station.cache.snapshot();ol=fusion.fuse(lidar[1] if lidar else None,radar[1] if radar else None);ds=detdiag.update(fusion.last_dynamic_candidates);camera_objects=[];pairs=[]
   if projector is not None and camera is not None:
    projected=project_lidar_tracks(projector,fusion.last_tracked_candidates,width,height)
    if camera_source=="carla_truth":
     cam_list=make_truth_camera_objects(station.world,projector,camera_id,width,height,frame_id=camera[0],timestamp=ol.timestamp);camera_objects=cam_list.objects
     for pair in associate_camera_to_lidar(camera_objects,projected,min_iou=assoc_cfg.get("min_iou",.05),max_center_distance=assoc_cfg.get("max_center_distance",120.0)):
      p=dict(pair);p["lidar_index"]=projected[pair["lidar_index"]]["source_index"];pairs.append(p)
   fusion.apply_camera_confirmations(pairs,timestamp=ol.timestamp)
   fol=build_fused_object_list(sid,fusion.last_tracked_candidates,ol.timestamp,camera_objects,pairs);oj=encode_object_list(ol);rj=encode_rsm(ol);now=time.time()
   if now-last>=1.0:
    s=fusion.last_stats;cf=camera[0] if camera else "-";rmin=s.get("radar_nearest_min");rmin_txt="-" if rmin is None else "%.2fm"%rmin;score_avg=s.get("candidate_score_avg");score_txt="-" if score_avg is None else "%.2f"%score_avg
    print("[RSU %s | %s] Camera:%s LiDAR:%d -> Ground:-%d => %d pts | Clusters:%d Geo:%d ROI:%d(+%d rescued) Reject:%d Score:%d(-%d avg=%s) Dyn:%d Tracks:%d | TrackLife N:%d U:%d C:%d S:%d D:%d | Radar:%d/%d Matched:%d Nearest:%s | Fused:%d Cam:%d/%d"%(sid,station.map_name,cf,s["lidar_points"],s.get("ground_removed_points",0),s.get("lidar_points_after_ground",s["lidar_points"]),s["lidar_clusters"],s.get("world_geometry_candidates",0),s["roi_candidates"],s.get("roi_rescued",0),s.get("roi_rejected",0),s.get("scored_candidates",s["roi_candidates"]),s.get("score_rejected",0),score_txt,s["background_candidates"],s["tracked_objects"],s.get("track_new",0),s.get("track_update",0),s.get("track_coast",0),s.get("track_suppress",0),s.get("track_drop",0),s["radar_detections"],s.get("radar_world_points",0),s.get("radar_matched_objects",0),rmin_txt,len(fol.objects),len(camera_objects),len(pairs)))
    _print_detection_stability(ds)
    print("  [TRACK QUALITY] Active:%d High:%d Medium:%d Low:%d Suppressed:%d AvgQuality:%.2f"%(s.get("track_quality_active",0),s.get("track_quality_high",0),s.get("track_quality_medium",0),s.get("track_quality_low",0),s.get("track_suppress",0),float(s.get("track_quality_avg",0.0))))
    print("  [TRACK LIFE GATE] low_hit_keep:%d low_new_drop:%d"%(s.get("track_low_hit_keep",0),s.get("track_low_new_drop",0)))
    if s.get("roi_rejection_reasons"):print("  ROI rejected reasons: %s"%s["roi_rejection_reasons"])
    if s.get("roi_rescued",0):print("  Geometry-aware ROI rescued: %d"%s.get("roi_rescued",0))
    if s.get("score_rejected",0):print("  Candidate score rejected: %d"%s.get("score_rejected",0))
    for idx,o in enumerate(fol.objects[:10]):
     t=fusion.last_tracked_candidates[idx];size=o.size;rs="-" if o.radar_speed is None else "%.2f"%o.radar_speed;cam="-" if o.camera is None else "%s box=%s"%(o.camera.get("cameraId","?"),o.camera.get("bbox"));near=t.get("radar_nearest_xy");near_txt="-" if near is None else "%.2f"%near;raw_speed=math.hypot(t.get("raw_vx",0),t.get("raw_vy",0));fused_speed=math.hypot(o.vx,o.vy);state=t.get("track_state","confirmed");allowed=int(t.get("coast_allowed",0));q=float(t.get("track_quality",0.0));sensors=t.get("track_sensors","L")
     print("  %-12s type=%-7s state=%-9s q=%.2f sensors=%-3s coast=%d/%d pos=(%7.2f,%7.2f,%5.2f) vel=(%6.2f,%6.2f) speed=%.2f raw=%.2f size=(%.2f,%.2f,%.2f) radar=%s near=%sm hits=%d cam=%s conf=%.2f src=%s"%(o.object_id,o.object_type,state,q,sensors,int(t.get("coast_frames",0)),allowed,o.x,o.y,o.z,o.vx,o.vy,fused_speed,raw_speed,size[0],size[1],size[2],rs,near_txt,int(t.get("radar_hits",0)),cam,o.confidence,"+".join(o.sources)))
    last=now
   if evaluator is not None and now-last_eval>=eval_interval:
    s=fusion.last_stats;ev=evaluator.evaluate(fusion.last_tracked_candidates,camera_objects,pairs,s.get("radar_matched_objects",0));geo=evaluator.evaluate_candidates(fusion.last_geometry_world);roi=evaluator.evaluate_candidates(fusion.last_roi_candidates);scored=evaluator.evaluate_candidates(fusion.last_scored_candidates);dyn=evaluator.evaluate_candidates(fusion.last_dynamic_candidates);dd=evaluator.analyze_detection_drop_reasons(fusion.last_geometry_world,fusion.last_roi_candidates,fusion.last_scored_candidates,fusion.last_dynamic_candidates,fusion.last_roi_rejections,fusion.last_score_rejections)
    print("[EVAL %.0fm] Truth:%d Tracks:%d Matched:%d Missed:%d FP:%d Recall:%s Precision:%s PosErr:%s/%s RadarMatched:%d CamVisibleTruth:%d CamLiDAR:%d"%(evaluator.radius,ev["truth"],ev["detected"],ev["matched"],ev["missed"],ev["false_positive"],_pct(ev["recall"]),_pct(ev["precision"]),_meters(ev["mean_position_error"]),_meters(ev["max_position_error"]),ev["radar_matched"],ev["camera_visible"],ev["camera_lidar_matched"]))
    _print_stage("GEOMETRY",geo);_print_stage("ROI",roi);_print_stage("SCORE",scored);_print_stage("DYNAMIC",dyn);_print_stage("TRACK",ev)
    _print_detection_stability(ds);_print_detection_drop(dd)
    print("  [TRACK LIFE] NEW:%d UPDATE:%d COAST:%d SUPPRESS:%d DROP:%d"%(s.get("track_new",0),s.get("track_update",0),s.get("track_coast",0),s.get("track_suppress",0),s.get("track_drop",0)))
    print("  [TRACK QUALITY] Active:%d High:%d Medium:%d Low:%d AvgQuality:%.2f"%(s.get("track_quality_active",0),s.get("track_quality_high",0),s.get("track_quality_medium",0),s.get("track_quality_low",0),float(s.get("track_quality_avg",0.0))))
    print("  [TRACK LIFE GATE] low_hit_keep:%d low_new_drop:%d"%(s.get("track_low_hit_keep",0),s.get("track_low_new_drop",0)))
    if s.get("roi_rejection_reasons"):print("  [ROI REJECT] %s"%s["roi_rejection_reasons"])
    if s.get("roi_rescued",0):print("  [ROI RESCUED] %d"%s.get("roi_rescued",0))
    if s.get("score_rejected",0):print("  [SCORE REJECT] %d"%s.get("score_rejected",0))
    if eval_cfg.get("roi_false_reject_diagnostics",False):
     lo=float(eval_cfg.get("roi_false_reject_min_range",30.0));hi=float(eval_cfg.get("roi_false_reject_max_range",50.0));limit=int(eval_cfg.get("roi_false_reject_max_print",6))
     false_rejects=evaluator.analyze_roi_false_rejections(fusion.last_roi_candidates,fusion.last_roi_rejections,lo,hi)
     print("  [ROI FALSE REJECT %02.0f-%02.0fm] %d"%(lo,hi,len(false_rejects)))
     for item in false_rejects[:max(0,limit)]:
      print("    truth=%s range=%.1fm reason=%s gate=%.2fm lateral=%s allowed=%s excess=%s overlap=%s margin=%s"%(item.get("actor_id"),float(item.get("truth_range",0.0)),item.get("reason"),float(item.get("candidate_distance",0.0)),_num(item.get("lateral")),_num(item.get("allowed_lateral")),_num(item.get("center_excess")),_num(item.get("bbox_overlap")),_num(item.get("roi_margin"))))
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