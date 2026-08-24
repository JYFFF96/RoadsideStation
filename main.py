from __future__ import print_function
import argparse,json,signal,sys,time,yaml,math
import carla
from roadside.carla_station import CarlaRoadsideStation
from roadside.fusion import SimpleFusion
from roadside.detection_stability import DetectionStabilityDiagnostics
from roadside.discovery_diagnostics import DiscoveryDiagnostics
from roadside.sparse_geometry_rescue import track_guided_sparse_rescue
from roadside.far_geometry_builder import build_far_geometry_candidates
from roadside.camera_fusion import CameraProjector
from roadside.camera_lidar_association import associate_camera_to_lidar
from roadside.camera_detector import create_camera_detector
from roadside.camera_runtime_config import apply_camera_runtime_overrides
from roadside.camera_objects import CameraObjectList
from roadside.camera_ground_initiation import CameraGroundInitiationShadow
from roadside.fused_objects import build_fused_object_list
from roadside.ground_truth_eval import GroundTruthEvaluator
from roadside.lidar_projection import project_lidar_tracks
from roadside.runtime_status import background_ready_banner
from roadside.radar_camera_support import annotate_radar_camera_support
from roadside.selected_camera_support import annotate_selected_camera_support
from roadside.sim_camera_truth import make_truth_camera_objects
from roadside.messages import encode_object_list,encode_rsm
from roadside.mqtt_pub import MqttPublisher
from roadside.v2x_events import V2XEventEngine,encode_v2x_event
from roadside.map_selection import carla_map_short_name,town05_switch_target

_STOP_REQUESTED=False

def _request_stop(signum,frame):
 global _STOP_REQUESTED
 if not _STOP_REQUESTED:print("\nStop requested. Shutting down RoadsideStation...")
 _STOP_REQUESTED=True

def load_config(path="config/roadside.yaml"):
 with open(path,"r") as fp:return yaml.safe_load(fp)

def _try_load_configured_map(config):
 cc=config.get("carla",{});target=cc.get("map","Town05_Opt")
 if not cc.get("load_world_on_start",False):return
 client=carla.Client(cc.get("host","127.0.0.1"),int(cc.get("port",2000)));client.set_timeout(float(cc.get("timeout",60.0)))
 current=carla_map_short_name(client.get_world().get_map().name)
 switch_target=town05_switch_target(current,target)
 print("Town05 startup map check | current=%s target=%s"%(current,target))
 if switch_target is None:
  print("Current world is already Town05; keeping the world and existing traffic.")
  return
 print("WARNING: switching to Town05 reloads the CARLA world and removes existing traffic.")
 print("Calling client.load_world('%s')..."%switch_target)
 world=client.load_world(switch_target)
 loaded=carla_map_short_name(world.get_map().name)
 print("CARLA map switch completed: %s"%loaded);time.sleep(2.0)

def _pct(v):return "-" if v is None else "%.1f%%"%(100.0*float(v))
def _meters(v):return "-" if v is None else "%.2fm"%float(v)
def _num(v):return "-" if v is None else "%.2f"%float(v)

def _print_traffic_status(station,config):
 tc=config.get("traffic",{});mode=tc.get("mode","external");source=tc.get("source","carla_generate_traffic")
 try:
  vehicle_actors=list(station.world.get_actors().filter("vehicle.*"));walker_actors=list(station.world.get_actors().filter("walker.pedestrian.*"));vehicles=len(vehicle_actors);walkers=len(walker_actors)
 except Exception:vehicle_actors=[];walker_actors=[];vehicles=0;walkers=0
 print("Traffic mode: %s (source=%s)"%(mode,source));print("Attached to existing CARLA traffic: %d vehicles, %d walkers"%(vehicles,walkers))
 center=getattr(station,"junction_center",None)
 if center is not None:
  def bins(actors):
   out=[0,0,0]
   for actor in actors:
    try:loc=actor.get_location();d=math.hypot(float(loc.x)-float(center.x),float(loc.y)-float(center.y))
    except Exception:continue
    if d<=30.0:out[0]+=1
    elif d<=50.0:out[1]+=1
    elif d<=80.0:out[2]+=1
   return out
  vb=bins(vehicle_actors);wb=bins(walker_actors);near=sum(vb)+sum(wb)
  print("Traffic near RSU | 00-30m V:%d P:%d | 30-50m V:%d P:%d | 50-80m V:%d P:%d | total:%d"%(vb[0],wb[0],vb[1],wb[1],vb[2],wb[2],near))
  if near<int(tc.get("nearby_participant_warning_threshold",10)):
   print("WARNING: too few traffic participants within 80m. Start tools/spawn_rsu_traffic.py and tools/spawn_multiclass_targets.py before main.py.")
 if mode=="external" and vehicles==0:print("NOTE: no vehicles are present. Start a traffic generator in another terminal.")

def _print_stage(name,m):
 print("  [STAGE %-8s] Candidates:%d Matched:%d Missed:%d FP:%d Recall:%s Precision:%s"%(name,m["detected"],m["matched"],m["missed"],m["false_positive"],_pct(m["recall"]),_pct(m["precision"])))
 for b in m.get("range_bins",[]):
  print("    [%02.0f-%02.0fm] Cand:%d Match:%d Miss:%d FP:%d Recall:%s"%(b["min_range"],b["max_range"],b["detected"],b["matched"],b["missed"],b["false_positive"],_pct(b["recall"])))

def _print_multiclass(m):
 parts=[]
 for name,b in sorted((m.get("class_metrics",{}) or {}).items()):
  parts.append("%s:T%d/M%d/Miss%d/R%s"%(name,b.get("truth",0),b.get("matched",0),b.get("missed",0),_pct(b.get("recall"))))
 print("  [MULTI-CLASS TRACK] %s"%(" | ".join(parts) if parts else "no truth objects"))

def _print_detection_stability(ds):
 print("  [DETECTION STABILITY] Dyn:%d Persistent:%d New:%d OneFrameLost:%d Reassoc:%d Fragmented:%d Lost:%d Jump(avg/max):%.2f/%.2fm ExtentDelta:%.2f"%(ds.get("candidates",0),ds.get("persistent",0),ds.get("new",0),ds.get("one_frame_lost",0),ds.get("reassociated",0),ds.get("fragmented",0),ds.get("lost",0),float(ds.get("mean_jump",0.0)),float(ds.get("max_jump",0.0)),float(ds.get("mean_extent_delta",0.0))))

def _print_detection_drop(dd):
 print("  [DETECTION DROP] Truth:%d Pass:%d NoGeometry:%d ROIReject:%d ROILost:%d ScoreReject:%d ScoreLost:%d DynamicDrop:%d"%(dd.get("truth",0),dd.get("pass",0),dd.get("no_geometry_candidate",0),dd.get("roi_reject",0),dd.get("roi_lost",0),dd.get("score_reject",0),dd.get("score_lost",0),dd.get("dynamic_drop",0)))
 for b in dd.get("range_bins",[]):
  print("    [%02.0f-%02.0fm] Truth:%d Pass:%d NoGeometry:%d ROIReject:%d ScoreReject:%d DynamicDrop:%d"%(b.get("min_range",0.0),b.get("max_range",0.0),b.get("truth",0),b.get("pass",0),b.get("no_geometry_candidate",0),b.get("roi_reject",0)+b.get("roi_lost",0),b.get("score_reject",0)+b.get("score_lost",0),b.get("dynamic_drop",0)))
 for name,b in sorted((dd.get("class_counts",{}) or {}).items()):
  print("    [MULTI-CLASS DROP %s] T:%d Pass:%d NoG:%d ROI:%d Score:%d Dynamic:%d"%(name,b.get("truth",0),b.get("pass",0),b.get("no_geometry_candidate",0),b.get("roi_reject",0)+b.get("roi_lost",0),b.get("score_reject",0)+b.get("score_lost",0),b.get("dynamic_drop",0)))

def _print_geometry_attribution(a):
 for name,b in sorted((a.get("classes",{}) or {}).items()):
  p=b.get("profile",{}) or {};pts=p.get("points",{}) or {};length=p.get("length",{}) or {};width=p.get("width",{}) or {};height=p.get("height",{}) or {};src=p.get("sources",{}) or {}
  print("  [MULTI-CLASS GEOMETRY %s] T:%d G:%d NoG:%d R:%s Pts(avg/min/max):%s/%s/%s LWH(avg):%s/%s/%s Modes:%s Src(C/S/R/T/F/O):%d/%d/%d/%d/%d/%d"%(name,b.get("truth",0),b.get("matched",0),b.get("no_geometry",0),_pct(b.get("recall")),_num(pts.get("mean")),_num(pts.get("min")),_num(pts.get("max")),_num(length.get("mean")),_num(width.get("mean")),_num(height.get("mean")),p.get("cluster_modes",{}),src.get("compact",0),src.get("sparse",0),src.get("recovery",0),src.get("temporal",0),src.get("far_builder",0),src.get("road_object",0)))
 fp=a.get("false_profile",{}) or {};points=fp.get("points",{}) or {}
 print("  [GEOMETRY UNATTRIBUTED] FP:%d Pts(avg):%s Modes:%s"%(a.get("false_positive",0),_num(points.get("mean")),fp.get("cluster_modes",{})))

def _print_far_admission_eval(a):
 print("  [FAR ADMISSION EVAL] Frames:%d WouldHold:%d WouldHoldTruth:%d WouldHoldFP:%d WouldConfirm:%d WouldConfirmTruth:%d WouldConfirmFP:%d Expired:%d ExpiredTruth:%d ExpiredFP:%d"%(a.get("frames",0),a.get("would_hold",0),a.get("would_hold_truth",0),a.get("would_hold_fp",0),a.get("would_confirm",0),a.get("would_confirm_truth",0),a.get("would_confirm_fp",0),a.get("expired",0),a.get("expired_truth",0),a.get("expired_fp",0)))
 jump=a.get("candidate_jump",{});gap=a.get("time_gap",{});frames=a.get("frame_gap",{})
 print("  [FAR ADMISSION MOTION] Samples:%d Jump(avg/p50/p90/max):%s/%s/%s/%sm TimeGap(avg/p90/max):%s/%s/%ss FrameGap(avg/p90/max):%s/%s/%s"%(jump.get("samples",0),_num(jump.get("mean")),_num(jump.get("p50")),_num(jump.get("p90")),_num(jump.get("max")),_num(gap.get("mean")),_num(gap.get("p90")),_num(gap.get("max")),_num(frames.get("mean")),_num(frames.get("p90")),_num(frames.get("max"))))

def _profile_stat(bucket,name,field="mean"):
 return _num((bucket.get(name,{}) or {}).get(field))

def _print_far_admission_profile(name,profile):
 truth=profile.get("truth",{}) or {};fp=profile.get("fp",{}) or {};label=name.upper()
 print("  [FAR ADMISSION PROFILE %s] Truth:%d FP:%d Score(avg/p50 T/F):%s/%s/%s/%s Points(avg/p90 T/F):%s/%s/%s/%s Range(avg T/F):%s/%sm Edge(avg T/F):%s/%s"%(label,truth.get("count",0),fp.get("count",0),_profile_stat(truth,"scores"),_profile_stat(truth,"scores","p50"),_profile_stat(fp,"scores"),_profile_stat(fp,"scores","p50"),_profile_stat(truth,"points"),_profile_stat(truth,"points","p90"),_profile_stat(fp,"points"),_profile_stat(fp,"points","p90"),_profile_stat(truth,"ranges"),_profile_stat(fp,"ranges"),_profile_stat(truth,"edge_ratios"),_profile_stat(fp,"edge_ratios")))
 print("  [FAR ADMISSION SOURCE %s] SizeLWH(avg T/F):%s/%s,%s/%s,%s/%s Recovery:%d/%d Sparse:%d/%d Temporal:%d/%d FarBuilder:%d/%d ScoreBypass:%d/%d Radar:%d/%d ModesT:%s ModesF:%s"%(label,_profile_stat(truth,"lengths"),_profile_stat(fp,"lengths"),_profile_stat(truth,"widths"),_profile_stat(fp,"widths"),_profile_stat(truth,"heights"),_profile_stat(fp,"heights"),truth.get("recovery",0),fp.get("recovery",0),truth.get("sparse",0),fp.get("sparse",0),truth.get("temporal",0),fp.get("temporal",0),truth.get("far_builder",0),fp.get("far_builder",0),truth.get("score_bypass",0),fp.get("score_bypass",0),truth.get("radar",0),fp.get("radar",0),truth.get("cluster_modes",{}),fp.get("cluster_modes",{})))

def _print_far_admission_risk_shadow(name,profile):
 truth=profile.get("truth",{}) or {};fp=profile.get("fp",{}) or {};label=name.upper()
 truth_total=int(truth.get("total",0));fp_total=int(fp.get("total",0));truth_kept=int(truth.get("kept",0));fp_rejected=int(fp.get("rejected",0))
 truth_rate=(float(truth_kept)/truth_total) if truth_total else None;fp_rate=(float(fp_rejected)/fp_total) if fp_total else None
 print("  [FAR ADMISSION EDGE-RISK SHADOW %s] TruthKeep:%d/%d(%s) FPReject:%d/%d(%s) HardEdge(T/F):%d/%d SoftRisk(T/F):%d/%d UnknownEdge(T/F):%d/%d"%(label,truth_kept,truth_total,_pct(truth_rate),fp_rejected,fp_total,_pct(fp_rate),truth.get("hard_edge",0),fp.get("hard_edge",0),truth.get("soft_risk",0),fp.get("soft_risk",0),truth.get("unknown_edge",0),fp.get("unknown_edge",0)))

def _print_far_admission_risk_classes(profile):
 parts=[]
 for name,b in sorted((profile or {}).items()):
  total=int(b.get("total",0));kept=int(b.get("kept",0));rate=(float(kept)/total) if total else None
  parts.append("%s:%d/%d(%s)"%(name,kept,total,_pct(rate)))
 print("  [FAR ADMISSION EDGE-RISK CLASS CONFIRM] TruthKeep %s"%(" | ".join(parts) if parts else "no truth candidates"))

def _print_sparse_geometry(s):
 print("  [SPARSE GEOMETRY] Built:%d ROI:%d Score:%d Dynamic:%d"%(s.get("sparse_rescue_candidates",0),s.get("sparse_rescue_roi",0),s.get("sparse_rescue_score",0),s.get("sparse_rescue_dynamic",0)))

def _print_road_object_recovery(s):
 print("  [ROAD-OBJECT RECOVERY] Mode:%s InputPts:%d Components:%d ShapePass:%d Pending:%d TemporalPass:%d Dedupe:%d CapReject:%d Built:%d BalancedShadow:%d Bands:%s"%('SHADOW' if s.get('road_object_recovery_shadow_mode',False) else 'ENFORCE',s.get("road_object_recovery_input",0),s.get("road_object_recovery_components",0),s.get("road_object_recovery_shape_pass",0),s.get("road_object_recovery_pending",0),s.get("road_object_recovery_temporal_pass",0),s.get("road_object_recovery_dedupe",0),s.get("road_object_recovery_cap_reject",0),s.get("road_object_recovery_built",0),s.get("road_object_recovery_balanced_built",0),s.get("road_object_recovery_balanced_bands",{})))
 print("  [ROAD-OBJECT ADAPTIVE TEMPORAL] Mode:SHADOW History:%d AccPts:%d Components:%d Shape:%d Temporal:%d Dedupe:%d Built:%d Bands:%s"%(s.get("road_object_recovery_adaptive_history_frames",0),s.get("road_object_recovery_adaptive_points",0),s.get("road_object_recovery_adaptive_components",0),s.get("road_object_recovery_adaptive_shape_pass",0),s.get("road_object_recovery_adaptive_temporal_pass",0),s.get("road_object_recovery_adaptive_dedupe",0),s.get("road_object_recovery_adaptive_built",0),s.get("road_object_recovery_adaptive_bands",{})))
 print("  [ROAD-OBJECT ADAPTIVE RANKING] Mode:SHADOW Built:%d Bands:%s"%(s.get("road_object_recovery_adaptive_ranked_built",0),s.get("road_object_recovery_adaptive_ranked_bands",{})))
 print("  [ROAD-OBJECT ADAPTIVE STRATIFIED] Mode:SHADOW Built:%d Bands:%s Heights:%s"%(s.get("road_object_recovery_adaptive_stratified_built",0),s.get("road_object_recovery_adaptive_stratified_bands",{}),s.get("road_object_recovery_adaptive_stratified_heights",{})))
 print("  [ROAD-OBJECT ADAPTIVE HYBRID] Mode:SHADOW Built:%d Bands:%s Sources:%s"%(s.get("road_object_recovery_adaptive_hybrid_built",0),s.get("road_object_recovery_adaptive_hybrid_bands",{}),s.get("road_object_recovery_adaptive_hybrid_sources",{})))
 print("  [ROAD-OBJECT HYBRID GATE] Mode:SHADOW Keep:%d Reject:%d Reasons:%s"%(s.get("road_object_recovery_adaptive_hybrid_gate_kept",0),s.get("road_object_recovery_adaptive_hybrid_gate_rejected",0),s.get("road_object_recovery_adaptive_hybrid_gate_reasons",{})))
 print("  [ROAD-OBJECT HYBRID TEMPORAL RESCUE] Mode:SHADOW Keep:%d Rescued:%d Sources:%s"%(s.get("road_object_recovery_adaptive_hybrid_rescue_kept",0),s.get("road_object_recovery_adaptive_hybrid_rescued",0),s.get("road_object_recovery_adaptive_hybrid_rescue_sources",{})))
 print("  [ROAD-OBJECT RESCUE GEOMETRY GATE] Mode:SHADOW Keep:%d Reject:%d Reasons:%s"%(s.get("road_object_recovery_adaptive_hybrid_geometry_gate_kept",0),s.get("road_object_recovery_adaptive_hybrid_geometry_gate_rejected",0),s.get("road_object_recovery_adaptive_hybrid_geometry_gate_reasons",{})))
 print("  [ROAD-OBJECT SELECTED OUTPUT] Mode:%s Policy:%s Built:%d Active:%d"%(
  "ENFORCING" if s.get("road_object_recovery_selected_output_enforcing",False) else "SHADOW",
  s.get("road_object_recovery_selected_output_policy","disabled"),
  s.get("road_object_recovery_selected_output_built",0),s.get("road_object_recovery_active_output_built",0)))
 print("  [SELECTED ADMISSION SCORE GATE] Mode:%s Threshold:%.2f Keep:%d Reject:%d"%(
  "ENFORCING" if s.get("selected_admission_score_enforcing",False) else "SHADOW",
  float(s.get("selected_admission_score_threshold",.20)),
  s.get("selected_admission_score_kept",0),s.get("selected_admission_score_rejected",0)))

def _dist3(profile,key):
 d=(profile or {}).get(key,{}) or {}
 return "%s/%s/%s"%(_num(d.get("p10")),_num(d.get("p50")),_num(d.get("p90")))

def _print_road_object_distribution(label,profile,count):
 print("    [ROAD-OBJECT DIST %s] N:%d Pts(p10/p50/p90):%s H:%s Long:%s Short:%s Range:%s"%(label,count,_dist3(profile,"points"),_dist3(profile,"height"),_dist3(profile,"long_side"),_dist3(profile,"short_side"),_dist3(profile,"range")))

def _print_road_object_gate(label,gate):
 if not gate.get("enabled",False):return
 truth=gate.get("truth",{}) or {};fp=gate.get("fp",{}) or {};classes=[]
 for name,b in sorted((gate.get("classes",{}) or {}).items()):classes.append("%s:%d/%d"%(name,b.get("kept",0),b.get("total",0)))
 print("    [ROAD-OBJECT PRECISION GATE %s] Keep:%d/%d TruthKeep:%d/%d FPReject:%d/%d Classes:%s"%(label,gate.get("kept",0),gate.get("candidates",0),truth.get("kept",0),truth.get("total",0),fp.get("rejected",0),fp.get("total",0)," | ".join(classes) if classes else "-"))
 tf=truth.get("failures",{}) or {};ff=fp.get("failures",{}) or {}
 print("    [ROAD-OBJECT GATE REJECT %s] Truth(points/height/range/invalid):%d/%d/%d/%d FP:%d/%d/%d/%d"%(label,tf.get("points",0),tf.get("height",0),tf.get("range",0),tf.get("invalid",0),ff.get("points",0),ff.get("height",0),ff.get("range",0),ff.get("invalid",0)))

def _print_road_object_ablations(label,ablations):
 for threshold,gate in sorted((ablations or {}).items(),key=lambda x:int(x[0])):
  truth=gate.get("truth",{}) or {};fp=gate.get("fp",{}) or {};kept=gate.get("kept",0)
  precision=(float(truth.get("kept",0))/kept) if kept else None
  print("    [ROAD-OBJECT GATE ABLATION %s PTS>=%s] TruthKeep:%d/%d FPReject:%d/%d KeptPrecision:%s"%(label,threshold,truth.get("kept",0),truth.get("total",0),fp.get("rejected",0),fp.get("total",0),_pct(precision)))

def _print_road_object_actor_coverage(items):
 for item in items or []:
  visible=item.get("visible_frames",0);matched=item.get("matched_frames",0);kept=item.get("gate_kept_frames",0);fail=item.get("gate_failures",{}) or {}
  ablation=" ".join("P%s:%d"%(key,value) for key,value in sorted((item.get("ablation_kept",{}) or {}).items(),key=lambda x:int(x[0])))
  print("    [ROAD-OBJECT ACTOR COVERAGE] id=%d type=%s range=%.1f..%.1fm Visible:%d RecoveryMatch:%d(%s) GateKeep:%d/%d(%s) Fail(P/H/R):%d/%d/%d Ablation:%s"%(item.get("actor_id",0),item.get("type_id","unknown"),float(item.get("range_min",0.0) or 0.0),float(item.get("range_max",0.0) or 0.0),visible,matched,_pct(float(matched)/visible if visible else None),kept,matched,_pct(float(kept)/matched if matched else None),fail.get("points",0),fail.get("height",0),fail.get("range",0),ablation or "-"))

def _stage_rate(value,total):return _pct(float(value)/total if total else None)

def _print_road_object_stage_attribution(report):
 for item in report.get("actors",[]) or []:
  visible=item.get("visible_frames",0);stages=item.get("stage_frames",{}) or {};raw=item.get("raw_frames",0)
  raw_avg=float(item.get("raw_points_total",0))/visible if visible else 0.0
  print("    [ROAD-OBJECT STAGE ACTOR] id=%d type=%s range=%.1f..%.1fm Visible:%d Raw:%d(%s pts_avg/max=%.1f/%d) Component:%d Shape:%d Temporal:%d DedupePass:%d Baseline:%d Balanced:%d Adaptive(C/S/T/D/O):%d/%d/%d/%d/%d"%(item.get("actor_id",0),item.get("type_id","unknown"),float(item.get("range_min",0.0) or 0.0),float(item.get("range_max",0.0) or 0.0),visible,raw,_stage_rate(raw,visible),raw_avg,item.get("raw_points_max",0),stages.get("component",0),stages.get("shape",0),stages.get("temporal",0),stages.get("dedupe_pass",0),stages.get("output",0),stages.get("balanced_output",0),stages.get("adaptive_component",0),stages.get("adaptive_shape",0),stages.get("adaptive_temporal",0),stages.get("adaptive_dedupe_pass",0),stages.get("adaptive_output",0)))
  print("      [ROAD-OBJECT RANKED ACTOR] id=%d Ranked:%d(%s)"%(item.get("actor_id",0),stages.get("adaptive_ranked_output",0),_stage_rate(stages.get("adaptive_ranked_output",0),visible)))
  print("      [ROAD-OBJECT STRATIFIED ACTOR] id=%d Stratified:%d(%s)"%(item.get("actor_id",0),stages.get("adaptive_stratified_output",0),_stage_rate(stages.get("adaptive_stratified_output",0),visible)))
  print("      [ROAD-OBJECT HYBRID ACTOR] id=%d Hybrid:%d(%s)"%(item.get("actor_id",0),stages.get("adaptive_hybrid_output",0),_stage_rate(stages.get("adaptive_hybrid_output",0),visible)))
  print("      [ROAD-OBJECT HYBRID-GATED ACTOR] id=%d Gated:%d(%s)"%(item.get("actor_id",0),stages.get("adaptive_hybrid_gated_output",0),_stage_rate(stages.get("adaptive_hybrid_gated_output",0),visible)))
  print("      [ROAD-OBJECT HYBRID-RESCUED ACTOR] id=%d Rescued:%d(%s)"%(item.get("actor_id",0),stages.get("adaptive_hybrid_rescued_output",0),_stage_rate(stages.get("adaptive_hybrid_rescued_output",0),visible)))
  print("      [ROAD-OBJECT HYBRID-GEOMETRY-GATED ACTOR] id=%d GeometryGated:%d(%s)"%(item.get("actor_id",0),stages.get("adaptive_hybrid_geometry_gated_output",0),_stage_rate(stages.get("adaptive_hybrid_geometry_gated_output",0),visible)))
  print("      [ROAD-OBJECT SELECTED ACTOR] id=%d Selected:%d(%s)"%(item.get("actor_id",0),stages.get("selected_output",0),_stage_rate(stages.get("selected_output",0),visible)))
 for band in report.get("range_bands",[]) or []:
  visible=band.get("visible_frames",0)
  print("    [ROAD-OBJECT RANGE STAGE %.0f-%.0fm] Actors:%d Visible:%d Raw:%d(%s) Component:%d(%s) Shape:%d(%s) Temporal:%d(%s) DedupePass:%d(%s) Baseline:%d(%s) Balanced:%d(%s) Adaptive(C/S/T/D/O):%d/%d/%d/%d/%d"%(band.get("min_range",0.0),band.get("max_range",0.0),band.get("actors",0),visible,band.get("raw_frames",0),_stage_rate(band.get("raw_frames",0),visible),band.get("component",0),_stage_rate(band.get("component",0),visible),band.get("shape",0),_stage_rate(band.get("shape",0),visible),band.get("temporal",0),_stage_rate(band.get("temporal",0),visible),band.get("dedupe_pass",0),_stage_rate(band.get("dedupe_pass",0),visible),band.get("output",0),_stage_rate(band.get("output",0),visible),band.get("balanced_output",0),_stage_rate(band.get("balanced_output",0),visible),band.get("adaptive_component",0),band.get("adaptive_shape",0),band.get("adaptive_temporal",0),band.get("adaptive_dedupe_pass",0),band.get("adaptive_output",0)))
  print("      [ROAD-OBJECT RANKED RANGE] Ranked:%d(%s)"%(band.get("adaptive_ranked_output",0),_stage_rate(band.get("adaptive_ranked_output",0),visible)))
  print("      [ROAD-OBJECT STRATIFIED RANGE] Stratified:%d(%s)"%(band.get("adaptive_stratified_output",0),_stage_rate(band.get("adaptive_stratified_output",0),visible)))
  print("      [ROAD-OBJECT HYBRID RANGE] Hybrid:%d(%s)"%(band.get("adaptive_hybrid_output",0),_stage_rate(band.get("adaptive_hybrid_output",0),visible)))
  print("      [ROAD-OBJECT HYBRID-GATED RANGE] Gated:%d(%s)"%(band.get("adaptive_hybrid_gated_output",0),_stage_rate(band.get("adaptive_hybrid_gated_output",0),visible)))
  print("      [ROAD-OBJECT HYBRID-RESCUED RANGE] Rescued:%d(%s)"%(band.get("adaptive_hybrid_rescued_output",0),_stage_rate(band.get("adaptive_hybrid_rescued_output",0),visible)))
  print("      [ROAD-OBJECT HYBRID-GEOMETRY-GATED RANGE] GeometryGated:%d(%s)"%(band.get("adaptive_hybrid_geometry_gated_output",0),_stage_rate(band.get("adaptive_hybrid_geometry_gated_output",0),visible)))
  print("      [ROAD-OBJECT SELECTED RANGE] Selected:%d(%s)"%(band.get("selected_output",0),_stage_rate(band.get("selected_output",0),visible)))

def _print_road_object_cap_comparison(report):
 for suffix,label in (("","FRAME"),("_run","RUN")):
  base=report.get("baseline"+suffix,{}) or {};balanced=report.get("balanced"+suffix,{}) or {}
  bp=float(base.get("matched",0))/base.get("candidates",0) if base.get("candidates",0) else None
  rp=float(balanced.get("matched",0))/balanced.get("candidates",0) if balanced.get("candidates",0) else None
  print("    [ROAD-OBJECT CAP COMPARE %s] Baseline C:%d M:%d FP:%d P:%s Classes:%s | Balanced C:%d M:%d FP:%d P:%s Classes:%s"%(label,base.get("candidates",0),base.get("matched",0),base.get("fp",0),_pct(bp),base.get("classes",{}),balanced.get("candidates",0),balanced.get("matched",0),balanced.get("fp",0),_pct(rp),balanced.get("classes",{})))
  adaptive=report.get("adaptive"+suffix,{}) or {}
  if adaptive:
   ap=float(adaptive.get("matched",0))/adaptive.get("candidates",0) if adaptive.get("candidates",0) else None
   print("    [ROAD-OBJECT ADAPTIVE COMPARE %s] C:%d M:%d FP:%d P:%s Classes:%s"%(label,adaptive.get("candidates",0),adaptive.get("matched",0),adaptive.get("fp",0),_pct(ap),adaptive.get("classes",{})))
  ranked=report.get("adaptive_ranked"+suffix,{}) or {}
  if ranked:
   rp=float(ranked.get("matched",0))/ranked.get("candidates",0) if ranked.get("candidates",0) else None
   print("    [ROAD-OBJECT RANKED COMPARE %s] C:%d M:%d FP:%d P:%s Classes:%s"%(label,ranked.get("candidates",0),ranked.get("matched",0),ranked.get("fp",0),_pct(rp),ranked.get("classes",{})))
  stratified=report.get("adaptive_stratified"+suffix,{}) or {}
  if stratified:
   sp=float(stratified.get("matched",0))/stratified.get("candidates",0) if stratified.get("candidates",0) else None
   print("    [ROAD-OBJECT STRATIFIED COMPARE %s] C:%d M:%d FP:%d P:%s Classes:%s"%(label,stratified.get("candidates",0),stratified.get("matched",0),stratified.get("fp",0),_pct(sp),stratified.get("classes",{})))
  hybrid=report.get("adaptive_hybrid"+suffix,{}) or {}
  if hybrid:
   hp=float(hybrid.get("matched",0))/hybrid.get("candidates",0) if hybrid.get("candidates",0) else None
   print("    [ROAD-OBJECT HYBRID COMPARE %s] C:%d M:%d FP:%d P:%s Classes:%s"%(label,hybrid.get("candidates",0),hybrid.get("matched",0),hybrid.get("fp",0),_pct(hp),hybrid.get("classes",{})))
  gated=report.get("adaptive_hybrid_gated"+suffix,{}) or {}
  if gated:
   gp=float(gated.get("matched",0))/gated.get("candidates",0) if gated.get("candidates",0) else None
   print("    [ROAD-OBJECT HYBRID-GATED COMPARE %s] C:%d M:%d FP:%d P:%s Classes:%s"%(label,gated.get("candidates",0),gated.get("matched",0),gated.get("fp",0),_pct(gp),gated.get("classes",{})))
  rescued=report.get("adaptive_hybrid_rescued"+suffix,{}) or {}
  if rescued:
   rsp=float(rescued.get("matched",0))/rescued.get("candidates",0) if rescued.get("candidates",0) else None
   print("    [ROAD-OBJECT HYBRID-RESCUED COMPARE %s] C:%d M:%d FP:%d P:%s Classes:%s"%(label,rescued.get("candidates",0),rescued.get("matched",0),rescued.get("fp",0),_pct(rsp),rescued.get("classes",{})))
  geometry_gated=report.get("adaptive_hybrid_geometry_gated"+suffix,{}) or {}
  if geometry_gated:
   ggp=float(geometry_gated.get("matched",0))/geometry_gated.get("candidates",0) if geometry_gated.get("candidates",0) else None
   print("    [ROAD-OBJECT HYBRID-GEOMETRY-GATED COMPARE %s] C:%d M:%d FP:%d P:%s Classes:%s"%(label,geometry_gated.get("candidates",0),geometry_gated.get("matched",0),geometry_gated.get("fp",0),_pct(ggp),geometry_gated.get("classes",{})))
  selected=report.get("selected"+suffix,{}) or {}
  if selected:
   slp=float(selected.get("matched",0))/selected.get("candidates",0) if selected.get("candidates",0) else None
   print("    [ROAD-OBJECT SELECTED COMPARE %s] C:%d M:%d FP:%d P:%s Classes:%s"%(label,selected.get("candidates",0),selected.get("matched",0),selected.get("fp",0),_pct(slp),selected.get("classes",{})))

def _print_selected_enforcement_attribution(report):
 for scope,label in (("frame","FRAME"),("run","RUN")):
  values=report.get(scope,{}) or {};parts=[]
  for key,name in (("roi","ROI"),("score","SCORE"),("dynamic","DYNAMIC"),
                   ("track_current","TRACK-CURRENT"),("track_ever","TRACK-EVER")):
   value=values.get(key,{}) or {}
   parts.append("%s C:%d M:%d FP:%d P:%s Classes:%s"%(
    name,value.get("candidates",0),value.get("matched",0),value.get("fp",0),
    _pct(value.get("precision")),value.get("classes",{})))
  print("    [SELECTED ENFORCEMENT ATTRIBUTION %s] %s"%(label," | ".join(parts)))
  paths=[]
  for key,name in (("score_near","NEAR"),("score_far","FAR"),
                   ("score_strict","STRICT"),("score_rescue","RESCUE"),
                   ("track_new","TRACK-NEW"),("track_confirmed","TRACK-CONFIRMED"),
                   ("track_coast","TRACK-COAST")):
   value=values.get(key,{}) or {}
   paths.append("%s C:%d M:%d FP:%d P:%s Classes:%s"%(
    name,value.get("candidates",0),value.get("matched",0),value.get("fp",0),
    _pct(value.get("precision")),value.get("classes",{})))
  print("    [SELECTED ADMISSION PATH ATTRIBUTION %s] %s"%(label," | ".join(paths)))

def _print_selected_admission_score_profile(report):
 if not report.get("enabled",False):return
 for scope,label in (("frame","FRAME"),("run","RUN")):
  value=report.get(scope,{}) or {};truth=value.get("truth_score",{}) or {};fp=value.get("fp_score",{}) or {}
  print("    [SELECTED ADMISSION SCORE %s] C:%d M:%d FP:%d P:%s ScoreT(avg/p50/p90):%s/%s/%s ScoreFP:%s/%s/%s"%(
   label,value.get("candidates",0),value.get("matched",0),value.get("fp",0),
   _pct(value.get("precision")),_num(truth.get("mean")),_num(truth.get("p50")),
   _num(truth.get("p90")),_num(fp.get("mean")),_num(fp.get("p50")),_num(fp.get("p90"))))
  parts=[]
  for threshold,item in sorted((value.get("thresholds",{}) or {}).items()):
   parts.append(">=%s C:%d M:%d FP:%d P:%s KeepT:%s"%(
    threshold,item.get("candidates",0),item.get("matched",0),item.get("fp",0),
    _pct(item.get("precision")),_pct(item.get("truth_retention"))))
  print("    [SELECTED ADMISSION ABLATION %s] %s"%(label," | ".join(parts) if parts else "-"))

def _print_selected_track_admission_profile(report):
 if not report.get("enabled",False):return
 for scope,label in (("frame","FRAME"),("run","RUN")):
  values=report.get(scope,{}) or {};parts=[]
  for key,name in (("hold","HOLD"),("confirm","CONFIRM"),
                   ("expired","EXPIRED"),("existing_track","TRACK-BYPASS"),
                   ("sensor","SENSOR-BYPASS")):
   value=values.get(key,{}) or {}
   parts.append("%s C:%d M:%d FP:%d P:%s Classes:%s"%(
    name,value.get("candidates",0),value.get("matched",0),value.get("fp",0),
    _pct(value.get("precision")),value.get("classes",{})))
  print("    [SELECTED NEW-TRACK ADMISSION %s] %s"%(label," | ".join(parts)))
 coverage=report.get("coverage",{}) or {};transitions=report.get("transitions",{}) or {}
 coverage_parts=[]
 for key,name in (("hold","HOLD"),("confirm","CONFIRM"),("expired","EXPIRED")):
  value=coverage.get(key,{}) or {}
  coverage_parts.append("%s Actors:%d Classes:%s"%(
   name,value.get("actors",0),value.get("classes",{})))
 print("    [SELECTED ADMISSION ACTOR COVERAGE] %s"%" | ".join(coverage_parts))
 transition_parts=[]
 for key,name in (("confirm","CONFIRM"),("expired","EXPIRED")):
  value=transitions.get(key,{}) or {}
  transition_parts.append("%s Total:%d FromT:%d FromFP:%d Unknown:%d NowT:%d NowFP:%d SameActor:%d StableFP:%d Changed:%d"%(
   name,value.get("total",0),value.get("origin_truth",0),value.get("origin_fp",0),
   value.get("origin_unknown",0),value.get("current_truth",0),value.get("current_fp",0),
   value.get("same_truth_actor",0),value.get("stable_fp",0),
   value.get("changed_label_or_actor",0)))
 print("    [SELECTED ADMISSION TRANSITIONS] Pending:%d | %s"%(
  report.get("pending_origins",0)," | ".join(transition_parts)))
 outcomes=report.get("actor_outcomes",{}) or {};parts=[]
 for key,name in (("confirm_only","CONFIRM-ONLY"),("expired_only","EXPIRED-ONLY"),
                  ("both","BOTH"),("unresolved","UNRESOLVED")):
  value=outcomes.get(key,{}) or {}
  parts.append("%s Actors:%d Classes:%s"%(
   name,value.get("actors",0),value.get("classes",{})))
 print("    [SELECTED ADMISSION ACTOR OUTCOMES] Held:%d EverConfirm:%d Coverage:%s | %s"%(
  outcomes.get("held_actors",0),outcomes.get("ever_confirmed",0),
  _pct(outcomes.get("confirmation_coverage"))," | ".join(parts)))
 for decision,buckets in sorted((report.get("outcome_features",{}) or {}).items()):
  for name,value in sorted((buckets or {}).items()):
   score=value.get("scores",{}) or {};points=value.get("points",{}) or {}
   height=value.get("height",{}) or {};rng=value.get("range",{}) or {}
   camera=value.get("camera",{}) or {}
   camera_iou=camera.get("iou",{}) or {};camera_dist=camera.get("center_distance",{}) or {}
   camera_conf=camera.get("confidence",{}) or {}
   print("    [SELECTED OUTCOME FEATURES %s %s] N:%d Score(avg/p10/p50/p90):%s/%s/%s/%s Points:%s/%s/%s/%s Height:%s/%s/%s/%s Range:%s/%s/%s/%s Camera(V/S):%d/%d Visible:%s Support:%s IoU(p50/p90):%s/%s Dist:%s/%s Conf:%s/%s Sources:%s Classes:%s Paths:%s Modes:%s"%(
    decision.upper(),name,value.get("samples",0),_num(score.get("mean")),
    _num(score.get("p10")),_num(score.get("p50")),_num(score.get("p90")),
    _num(points.get("mean")),_num(points.get("p10")),_num(points.get("p50")),
    _num(points.get("p90")),_num(height.get("mean")),_num(height.get("p10")),
    _num(height.get("p50")),_num(height.get("p90")),_num(rng.get("mean")),
    _num(rng.get("p10")),_num(rng.get("p50")),_num(rng.get("p90")),
    camera.get("visible",0),camera.get("supported",0),
    _pct(camera.get("visibility_rate")),_pct(camera.get("support_rate")),
    _num(camera_iou.get("p50")),_num(camera_iou.get("p90")),
    _num(camera_dist.get("p50")),_num(camera_dist.get("p90")),
    _num(camera_conf.get("p50")),_num(camera_conf.get("p90")),camera.get("sources",{}),
    camera.get("classes",{}),value.get("paths",{}),value.get("cluster_modes",{})))
   nearest_dist=camera.get("nearest_distance",{}) or {};nearest_iou=camera.get("nearest_iou",{}) or {};nearest_conf=camera.get("nearest_confidence",{}) or {}
   print("      [SELECTED CAMERA NEAREST %s %s] Dist(p10/p50/p90):%s/%s/%s IoU(p50/p90):%s/%s Conf(p50/p90):%s/%s Classes:%s | diagnostic-only, outside-gate boxes included"%(
    decision.upper(),name,_num(nearest_dist.get("p10")),_num(nearest_dist.get("p50")),_num(nearest_dist.get("p90")),_num(nearest_iou.get("p50")),_num(nearest_iou.get("p90")),_num(nearest_conf.get("p50")),_num(nearest_conf.get("p90")),camera.get("nearest_classes",{})))
   print("      [SELECTED CAMERA PROJECTION %s %s] Visible:%d Rejected:%s | diagnostic-only"%(
    decision.upper(),name,camera.get("visible",0),camera.get("projection_rejections",{})))
   print("      [SELECTED CAMERA RESCUE ABLATION %s %s] %s"%(
    decision.upper(),name,camera.get("rescue_ablations",{})))
 for name,value in sorted((report.get("camera_rescue_shadow",{}) or {}).items()):
  print("    [SELECTED CAMERA RESCUE SHADOW %s] IoU>=%.2f OR Dist<=%.0fpx | ExpiredOnly Actors:%d/%d Person:%d/%d | ExpiredSamples Person:%d/%d FP:%d/%d | ConfirmSamples Person:%d/%d FP:%d/%d"%(
   name,float(value.get("min_iou",0.0)),float(value.get("max_center_distance",0.0)),
   value.get("expired_only_actors_rescued",0),value.get("expired_only_actors",0),
   value.get("expired_only_person_actors_rescued",0),value.get("expired_only_person_actors",0),
   value.get("expired_person_samples_kept",0),value.get("expired_person_samples",0),
   value.get("expired_fp_samples_kept",0),value.get("expired_fp_samples",0),
   value.get("confirm_person_samples_kept",0),value.get("confirm_person_samples",0),
   value.get("confirm_fp_samples_kept",0),value.get("confirm_fp_samples",0)))
 camera_verdict=report.get("camera_rescue_deployment_verdict",{}) or {}
 if camera_verdict.get("enabled",False):
  values=camera_verdict.get("values",{}) or {};criteria=camera_verdict.get("criteria",{}) or {}
  print("    [SELECTED CAMERA RESCUE DEPLOYMENT VERDICT] Status:%s Rule:%s Samples:%d/%d VisiblePerson:%d/%d Opportunity:%s KeptPerson:%d/%d P:%s/>=%s FPReject:%s/>=%s ActorCoverage:%s/>=%s ConfirmFPReject:%s/>=%s Sources:%s Required:%s Reasons:%s | EVALUATION-ONLY"%(
   camera_verdict.get("status","BLOCKED"),camera_verdict.get("rule","-"),
   values.get("expired_person_samples",0),criteria.get("min_expired_person_samples",0),
   values.get("visible_person_samples",0),criteria.get("min_visible_person_samples",0),
   _pct(values.get("person_opportunity_rate")),
   values.get("kept_person_samples",0),criteria.get("min_kept_person_samples",0),
   _pct(values.get("kept_precision")),_pct(criteria.get("min_kept_precision")),
   _pct(values.get("expired_fp_rejection")),_pct(criteria.get("min_expired_fp_rejection")),
   _pct(values.get("expired_only_person_actor_coverage")),_pct(criteria.get("min_expired_only_person_actor_coverage")),
   _pct(values.get("confirm_fp_rejection")),_pct(criteria.get("min_confirm_fp_rejection")),
   values.get("camera_sources",{}),criteria.get("required_source","detector"),camera_verdict.get("reasons",[])))
 for name,value in sorted((report.get("delayed_reappearance_shadow",{}) or {}).items()):
  gap=value.get("time_gap",{}) or {};distance=value.get("match_distance",{}) or {}
  print("    [SELECTED DELAYED REAPPEARANCE %s] C:%d M:%d FP:%d P:%s Actors:%d Classes:%s | ExpiredOnly:%d/%d Person:%d/%d | Gap(p50/p90):%s/%ss Dist:%s/%sm"%(
   name,value.get("candidates",0),value.get("matched",0),value.get("fp",0),
   _pct(value.get("precision")),value.get("actors",0),value.get("classes",{}),
   value.get("expired_only_actors_rescued",0),value.get("expired_only_actors",0),
   value.get("expired_only_person_actors_rescued",0),value.get("expired_only_person_actors",0),
   _num(gap.get("p50")),_num(gap.get("p90")),_num(distance.get("p50")),
   _num(distance.get("p90"))))
  incremental=value.get("incremental",{}) or {};truth=incremental.get("truth_features",{}) or {};fp=incremental.get("fp_features",{}) or {}
  print("      [SELECTED DELAYED INCREMENTAL %s] C:%d M:%d FP:%d P:%s Actors:%d Classes:%s ExpiredOnly:%d Person:%d"%(
   name,incremental.get("candidates",0),incremental.get("matched",0),
   incremental.get("fp",0),_pct(incremental.get("precision")),
   incremental.get("actors",0),incremental.get("classes",{}),
   incremental.get("expired_only_actors_rescued",0),
   incremental.get("expired_only_person_actors_rescued",0)))
  for label,features in (("TRUTH",truth),("FP",fp)):
   print("        [SELECTED DELAYED FEATURES %s %s] N:%d Gap:%s/%s Dist:%s/%s Speed:%s/%s Score:%s/%s Points:%s/%s Height:%s/%s Range:%s/%s OriginScore:%s/%s OriginPts:%s/%s Classes:%s"%(
    name,label,features.get("samples",0),
    _num((features.get("time_gap",{}) or {}).get("p50")),_num((features.get("time_gap",{}) or {}).get("p90")),
    _num((features.get("match_distance",{}) or {}).get("p50")),_num((features.get("match_distance",{}) or {}).get("p90")),
    _num((features.get("apparent_speed",{}) or {}).get("p50")),_num((features.get("apparent_speed",{}) or {}).get("p90")),
    _num((features.get("score",{}) or {}).get("p50")),_num((features.get("score",{}) or {}).get("p90")),
    _num((features.get("points",{}) or {}).get("p50")),_num((features.get("points",{}) or {}).get("p90")),
    _num((features.get("height",{}) or {}).get("p50")),_num((features.get("height",{}) or {}).get("p90")),
    _num((features.get("range",{}) or {}).get("p50")),_num((features.get("range",{}) or {}).get("p90")),
    _num((features.get("origin_score",{}) or {}).get("p50")),_num((features.get("origin_score",{}) or {}).get("p90")),
    _num((features.get("origin_points",{}) or {}).get("p50")),_num((features.get("origin_points",{}) or {}).get("p90")),features.get("classes",{})))

  for gate,gate_value in sorted((incremental.get("risk_gate_ablations",{}) or {}).items()):
   print("        [SELECTED DELAYED RISK GATE %s %s] C:%d M:%d FP:%d P:%s KeepT:%s RejectFP:%s Actors:%d Classes:%s ExpiredOnly:%d Person:%d Rule:%s"%(
    name,gate,gate_value.get("candidates",0),gate_value.get("matched",0),
    gate_value.get("fp",0),_pct(gate_value.get("precision")),
    _pct(gate_value.get("truth_retention")),_pct(gate_value.get("fp_rejection")),
    gate_value.get("actors",0),gate_value.get("classes",{}),
    gate_value.get("expired_only_actors_rescued",0),
    gate_value.get("expired_only_person_actors_rescued",0),gate_value.get("rule",{})))
 verdict=report.get("delayed_reappearance_deployment_verdict",{}) or {}
 if verdict.get("enabled",False):
  values=verdict.get("values",{}) or {};criteria=verdict.get("criteria",{}) or {}
  print("    [SELECTED DELAYED DEPLOYMENT VERDICT] Status:%s Rule:%s/%s C:%d/%d P:%s/>=%s KeepT:%s/>=%s ExpiredOnly:%d/%d Person:%d/%d Reasons:%s | EVALUATION-ONLY"%(
   verdict.get("status","BLOCKED"),verdict.get("rule","-"),verdict.get("risk_gate","-"),
   values.get("candidates",0),criteria.get("min_candidates",0),
   _pct(values.get("precision")),_pct(criteria.get("min_precision")),
   _pct(values.get("truth_retention")),_pct(criteria.get("min_truth_retention")),
   values.get("expired_only_actors_rescued",0),criteria.get("min_expired_only_actors_rescued",0),
   values.get("expired_only_person_actors_rescued",0),criteria.get("min_expired_only_person_actors_rescued",0),
   verdict.get("reasons",[])))

def _adaptive_feature(profile,name):
 values=(profile or {}).get(name,{}) or {}
 return "%s/%s/%s/%s"%(_num(values.get("mean")),_num(values.get("p10")),
                         _num(values.get("p50")),_num(values.get("p90")))

def _print_adaptive_temporal_profile(report):
 if not report.get("enabled",False):return
 for key,label in (("frame","FRAME"),("run","RUN")):
  value=report.get(key,{}) or {}
  print("    [ROAD-OBJECT ADAPTIVE PROFILE %s] PreCap:%d Truth:%d FP:%d Precision:%s Bands:%s"%(label,value.get("candidates",0),value.get("matched",0),value.get("fp",0),_pct(value.get("precision")),value.get("bands",{})))
  profiles=[]
  for name,bucket in sorted((value.get("classes",{}) or {}).items()):profiles.append((name,bucket.get("samples",0),bucket.get("profile",{})))
  fp=value.get("false_profile",{}) or {};profiles.append(("FP",(fp.get("points",{}) or {}).get("samples",0),fp))
  for name,count,profile in profiles:
   print("      [ADAPTIVE FEATURE %s %s] N:%d Score(avg/p10/p50/p90):%s TotalPts:%s Current:%s History:%s Frames:%s Height:%s Range:%s SensorBands:%s"%(label,name,count,_adaptive_feature(profile,"rank_score"),_adaptive_feature(profile,"points"),_adaptive_feature(profile,"current_points"),_adaptive_feature(profile,"history_points"),_adaptive_feature(profile,"support_frames"),_adaptive_feature(profile,"height"),_adaptive_feature(profile,"range"),profile.get("bands",{})))

def _print_hybrid_selection_profile(report):
 if not report.get("enabled",False):return
 for key,label in (("frame","FRAME"),("run","RUN")):
  for source,value in sorted((report.get(key,{}) or {}).items()):
   print("    [ROAD-OBJECT HYBRID PROFILE %s %s] C:%d Truth:%d FP:%d Precision:%s Classes:%s"%(label,source,value.get("candidates",0),value.get("matched",0),value.get("fp",0),_pct(value.get("precision")),dict((name,b.get("samples",0)) for name,b in (value.get("classes",{}) or {}).items())))
   profiles=[]
   for name,bucket in sorted((value.get("classes",{}) or {}).items()):profiles.append((name,bucket.get("samples",0),bucket.get("profile",{})))
   fp=value.get("false_profile",{}) or {};profiles.append(("FP",(fp.get("points",{}) or {}).get("samples",0),fp))
   for name,count,profile in profiles:
    print("      [HYBRID FEATURE %s %s %s] N:%d Score:%s Pts:%s Current:%s History:%s Frames:%s Height:%s Long:%s Short:%s Range:%s"%(label,source,name,count,_adaptive_feature(profile,"rank_score"),_adaptive_feature(profile,"points"),_adaptive_feature(profile,"current_points"),_adaptive_feature(profile,"history_points"),_adaptive_feature(profile,"support_frames"),_adaptive_feature(profile,"height"),_adaptive_feature(profile,"long_side"),_adaptive_feature(profile,"short_side"),_adaptive_feature(profile,"range")))

def _print_hybrid_rescue_profile(report):
 if not report.get("enabled",False):return
 for key,label in (("frame","FRAME"),("run","RUN")):
  for source,value in sorted((report.get(key,{}) or {}).items()):
   print("    [ROAD-OBJECT RESCUE PROFILE %s %s] C:%d Truth:%d FP:%d Precision:%s Classes:%s"%(label,source,value.get("candidates",0),value.get("matched",0),value.get("fp",0),_pct(value.get("precision")),dict((name,b.get("samples",0)) for name,b in (value.get("classes",{}) or {}).items())))
   profiles=[]
   for name,bucket in sorted((value.get("classes",{}) or {}).items()):profiles.append((name,bucket.get("samples",0),bucket.get("profile",{})))
   fp=value.get("false_profile",{}) or {};profiles.append(("FP",(fp.get("points",{}) or {}).get("samples",0),fp))
   for name,count,profile in profiles:
    print("      [RESCUE FEATURE %s %s %s] N:%d Score:%s Pts:%s Current:%s History:%s Frames:%s Height:%s Long:%s Short:%s Area:%s Range:%s SensorRange:%s"%(label,source,name,count,_adaptive_feature(profile,"rank_score"),_adaptive_feature(profile,"points"),_adaptive_feature(profile,"current_points"),_adaptive_feature(profile,"history_points"),_adaptive_feature(profile,"support_frames"),_adaptive_feature(profile,"height"),_adaptive_feature(profile,"long_side"),_adaptive_feature(profile,"short_side"),_adaptive_feature(profile,"footprint_area"),_adaptive_feature(profile,"range"),_adaptive_feature(profile,"sensor_range")))
  for source,tests in sorted((report.get("ablations_"+key,{}) or {}).items()):
   for gate,value in sorted((tests or {}).items()):
    print("    [ROAD-OBJECT RESCUE ABLATION %s %s %s] TruthKeep:%d/%d FPReject:%d/%d Precision:%s"%(label,source,gate,value.get("truth_kept",0),value.get("truth",0),value.get("fp_rejected",0),value.get("fp",0),_pct(value.get("precision"))))

def _print_truth_lifecycle(report):
 if not report.get("enabled",False):return
 counts=report.get("counts",{}) or {};totals=report.get("totals",{}) or {}
 print("  [TRUTH LIFECYCLE] Active:%d Entered:%d BoundaryExit:%d UnexpectedExit:%d Teleport:%d | Totals:%s"%(report.get("active",0),counts.get("entered",0),counts.get("boundary_exit",0),counts.get("unexpected_exit",0),counts.get("teleport",0),totals))
 for kind in ("unexpected_exit","teleport"):
  for item in report.get(kind,[]) or []:
   print("    [TRUTH CHURN] Kind:%s id=%d role=%s type=%s range=%.1fm jump=%s"%(kind,item.get("actor_id",0),item.get("role","-"),item.get("type_id","-"),float(item.get("range",0.0) or 0.0),_num(item.get("jump_distance"))))

def _print_road_object_profile(a):
 session=a.get("benchmark_session",{}) or {}
 if session.get("reset",False):
  print("  [ROAD-OBJECT BENCHMARK SESSION] Reset:1 Generation:%d Actors:%d"%(session.get("generation",0),session.get("actors",0)))
 precision=(float(a.get('matched',0))/a.get('geometry',0)) if a.get('geometry',0) else None
 print("  [ROAD-OBJECT SHADOW EVAL] Candidates:%d TruthMatched:%d FP:%d Precision:%s"%(a.get('geometry',0),a.get('matched',0),a.get('false_positive',0),_pct(precision)))
 for name,b in sorted((a.get('classes',{}) or {}).items()):
  if not b.get('matched',0):continue
  _print_road_object_distribution("FRAME-"+name,b.get('profile',{}),b.get('matched',0))
 _print_road_object_distribution("FRAME-FP",a.get('false_profile',{}),a.get('false_positive',0))
 _print_road_object_gate("FRAME",a.get("precision_gate_shadow",{}))
 cumulative=a.get("cumulative",{}) or {}
 for name,b in sorted((cumulative.get("classes",{}) or {}).items()):_print_road_object_distribution("RUN-"+name,b.get("profile",{}),b.get("matched_samples",0))
 fp=(cumulative.get("false_profile",{}) or {}).get("points",{}) or {}
 _print_road_object_distribution("RUN-FP",cumulative.get("false_profile",{}),fp.get("samples",0))
 _print_road_object_gate("RUN",cumulative.get("precision_gate_shadow",{}))
 _print_road_object_ablations("RUN",cumulative.get("gate_ablations",{}))
 _print_road_object_actor_coverage(cumulative.get("actor_coverage",[]))

def _print_test_targets(evaluator):
 targets=evaluator.test_targets();print("Evaluation benchmark targets: %d tagged actor(s)"%len(targets))
 for item in sorted(targets,key=lambda x:x.get("actor_id",0)):
  print("  [TEST TARGET] id=%d role=%s type=%s pos=(%.2f,%.2f,%.2f) range=%.2fm"%(item.get("actor_id",0),item.get("role","-"),item.get("type_id","-"),item.get("x",0.0),item.get("y",0.0),item.get("z",0.0),item.get("range",0.0)))

def _print_discovery_diagnostics(d):
 print("  [DISCOVERY SOURCE] TrackRescue B:%d R:%d S:%d D:%d | NewDiscovery B:%d R:%d S:%d D:%d"%(d.get("track_rescue_built",0),d.get("track_rescue_roi",0),d.get("track_rescue_score",0),d.get("track_rescue_dynamic",0),d.get("new_discovery_built",0),d.get("new_discovery_roi",0),d.get("new_discovery_score",0),d.get("new_discovery_dynamic",0)))
 print("  [DISCOVERY TRACK] New:%d Confirmed:%d OneFrameDrop:%d Active:%d"%(d.get("discovery_track_new",0),d.get("discovery_track_confirmed",0),d.get("discovery_track_one_frame_drop",0),d.get("discovery_track_active",0)))

def _print_rescue_gate():
 g=getattr(track_guided_sparse_rescue,"last_stats",{}) or {};mid=g.get("mid",{}) or {};far=g.get("far",{}) or {}
 print("  [RESCUE GATE] Eligible:%d QualityBlock:%d StreakBlock:%d SupportBlock:%d GeometryBlock:%d Built:%d"%(g.get("eligible",0),g.get("quality_block",0),g.get("streak_block",0),g.get("support_block",0),g.get("geometry_block",0),g.get("built",0)))
 print("    [30-50m] Eligible:%d QualityBlock:%d StreakBlock:%d SupportBlock:%d GeometryBlock:%d Built:%d"%(mid.get("eligible",0),mid.get("quality_block",0),mid.get("streak_block",0),mid.get("support_block",0),mid.get("geometry_block",0),mid.get("built",0)))
 print("    [50-80m] Eligible:%d QualityBlock:%d StreakBlock:%d SupportBlock:%d GeometryBlock:%d Built:%d"%(far.get("eligible",0),far.get("quality_block",0),far.get("streak_block",0),far.get("support_block",0),far.get("geometry_block",0),far.get("built",0)))

def _print_far_geometry():
 g=getattr(build_far_geometry_candidates,"last_stats",{}) or {}
 print("  [FAR GEOMETRY] InputPts:%d Components:%d TemplatePass:%d Dedupe:%d Built:%d"%(g.get("input_points",0),g.get("components",0),g.get("template_pass",0),g.get("dedupe",0),g.get("built",0)))

def main():
 global _STOP_REQUESTED
 parser=argparse.ArgumentParser(description="RoadsideStation runtime")
 parser.add_argument("--config",default="config/roadside.yaml")
 parser.add_argument("--camera-source",choices=["none","carla_truth","detector"],default=None)
 parser.add_argument("--camera-model",default=None)
 parser.add_argument("--sensor-sync",choices=["latest","aligned"],default="aligned",
                     help="aligned selects each camera at the LiDAR frame; latest is a legacy diagnostic mode")
 args=parser.parse_args()
 signal.signal(signal.SIGINT,_request_stop);signal.signal(signal.SIGTERM,_request_stop)
 config=apply_camera_runtime_overrides(load_config(args.config),args.camera_source,args.camera_model);_try_load_configured_map(config);sid=config["station"]["id"];station=CarlaRoadsideStation(config);fusion=SimpleFusion(sid,config["fusion"]);pub=MqttPublisher(config["mqtt"]);event_engine=V2XEventEngine(sid,config.get("v2x_events",{}))
 dc=config.get("detection_stability",{});detdiag=DetectionStabilityDiagnostics(dc.get("match_distance",3.5),dc.get("max_missed_frames",2),dc.get("fragmentation_distance",2.0));ds={};discdiag=DiscoveryDiagnostics();dds={}
 print("RoadsideStation V0.6.12.8.2.2.59 Camera Ground Initiation Shadow starting...")
 station.start();_print_traffic_status(station,config);fusion.set_world_transform(station.lidar_transform);fusion.set_radar_transform(station.radar_transform);fusion.set_ground_reference(station.junction_center.z if station.junction_center is not None else None);fusion.set_candidate_validator(station.validate_driving_roi);pub.connect()
 fc=config.get("fusion",{});eval_cfg=config.get("evaluation",{})
 if fc.get("ground_removal_enabled",True):
  gz=station.junction_center.z if station.junction_center is not None else None;print("Ground removal: enabled reference_z=%s clearance=%.2fm"%(("-" if gz is None else "%.2f"%gz),float(fc.get("ground_clearance",0.30))))
 road_mode='shadow' if fc.get('road_object_recovery_shadow_mode',True) else 'enforcing'
 print("Road-Object Geometry Recovery: %s/%s | range=%.0f..%.0fm low_clearance=%.2fm temporal_frames=%d max=%d"%('enabled' if fc.get('road_object_recovery_enabled',False) else 'disabled',road_mode,float(fc.get('road_object_recovery_min_range',5.0)),float(fc.get('road_object_recovery_max_range',45.0)),float(fc.get('road_object_recovery_ground_clearance',.05)),int(fc.get('road_object_recovery_temporal_frames',2)),int(fc.get('road_object_recovery_max_candidates',12))))
 print("Range-Balanced Recovery Cap: %s | bands=%s | Shadow only, unused quota refills globally"%("shadow" if fc.get("road_object_recovery_balanced_cap_shadow",False) else "disabled",fc.get("road_object_recovery_balanced_bands",[])))
 print("LiDAR clustering: %s"%("hybrid range-adaptive (near geometry filtered, mid 3D, far multi-scale BEV)" if fc.get("range_adaptive_clustering",False) else "fixed"))
 if fc.get("range_adaptive_clustering",False):print("LiDAR range bands: %s"%fc.get("range_bands",[]))
 near_band=(fc.get("range_bands") or [{}])[0]
 print("LiDAR near-band geometry: 0-30m L=%.2f..%.1f W=%.2f..%.1f H=%.2f..%.1fm | physical blind zone is reported separately; cameras/radar cover it"%(float(near_band.get("min_length",0.75)),float(near_band.get("max_length",6.5)),float(near_band.get("min_width",0.55)),float(near_band.get("max_width",3.4)),float(near_band.get("min_height",0.45)),float(near_band.get("max_height",2.6))))
 radar_init_mode="shadow" if fc.get("radar_initiation_shadow_mode",True) else "enforcing"
 print("Near Radar Track Initiation: %s/%s | range=%.0f..%.0fm points>=%d frames>=%d | |radial speed|>=%.2fm/s | LiDAR dedupe<=%.1fm"%(
  "enabled" if fc.get("radar_initiation_enabled",False) else "disabled",radar_init_mode,
  float(fc.get("radar_initiation_min_range",2.0)),float(fc.get("radar_initiation_max_range",30.0)),
  int(fc.get("radar_initiation_min_points",2)),int(fc.get("radar_initiation_required_frames",2)),
 float(fc.get("radar_initiation_min_abs_speed",.6)),float(fc.get("radar_initiation_dedupe_distance",3.0))))
 print("Sparse Radar Return Initiation: %s | single point | |radial speed|>=%.2fm/s | frames>=%d ttl=%.1fs | road ROI + LiDAR dedupe"%(
  "enabled" if fc.get("radar_initiation_single_point_enabled",False) else "disabled",
  float(fc.get("radar_initiation_single_point_min_abs_speed",.2)),
  int(fc.get("radar_initiation_single_point_required_frames",3)),
 float(fc.get("radar_initiation_single_point_ttl",1.0))))
 print("Motion-Seed Radar Bridge: %s | seed speed>=%.2fm/s | frames=%s gates=%s | Shadow only, Tracker/ObjectList unchanged"%(
  "enabled" if fc.get("radar_initiation_seed_bridge_shadow_enabled",False) else "disabled",
  float(fc.get("radar_initiation_single_point_min_abs_speed",.2)),
  fc.get("radar_initiation_seed_bridge_required_frames",[2,3]),
  fc.get("radar_initiation_seed_bridge_match_gates",[2.5,4.0,6.0])))
 print("Singleton-to-Component Radar Bridge: %s | gates=%s | Shadow only, Tracker/ObjectList unchanged"%(
  "enabled" if fc.get("radar_initiation_seed_to_component_shadow_enabled",False) else "disabled",
  fc.get("radar_initiation_seed_to_component_match_gates",[2.5,4.0])))
 print("Radar Singleton Camera Support: %s | both cameras, generic 2D association | Shadow only"%(
  "enabled" if fc.get("radar_initiation_camera_support_shadow_enabled",False) else "disabled"))
 print("Camera Ground Initiation: %s | range=%.0f..%.0fm bottom-center ray/ground plane | Shadow only"%(
  "enabled" if fc.get("camera_ground_initiation_shadow_enabled",False) else "disabled",
  float(fc.get("camera_ground_initiation_min_range",2.0)),
  float(fc.get("camera_ground_initiation_max_range",30.0))))
 print("Sparse Geometry Rescue: %s | range=%.0f..%.0fm stable_hits>=%d | mid(q>=%.2f streak<=%d) far(q>=%.2f streak<=%d) | radius mid/far=%.1f/%.1fm | points mid/far>=%d/%d | score_bonus=%.2f"%("enabled" if fc.get("sparse_geometry_rescue_enabled",False) else "disabled",float(fc.get("sparse_geometry_rescue_min_range",30.0)),float(fc.get("sparse_geometry_rescue_max_range",80.0)),int(fc.get("sparse_geometry_rescue_min_track_hits",3)),float(fc.get("sparse_geometry_rescue_mid_min_quality",0.55)),int(fc.get("sparse_geometry_rescue_mid_max_streak",2)),float(fc.get("sparse_geometry_rescue_far_min_quality",0.47)),int(fc.get("sparse_geometry_rescue_far_max_streak",3)),float(fc.get("sparse_geometry_rescue_mid_radius",2.2)),float(fc.get("sparse_geometry_rescue_far_radius",3.0)),int(fc.get("sparse_geometry_rescue_mid_min_points",3)),int(fc.get("sparse_geometry_rescue_far_min_points",2)),float(fc.get("sparse_geometry_rescue_score_bonus",0.08))))
 print("Far Geometry Builder: %s | range=%.0f..%.0fm cell=%.2fm neighbor=%d min_points=%d max=%d"%("enabled" if fc.get("far_geometry_builder_enabled",True) else "disabled",float(fc.get("far_geometry_builder_min_range",50.0)),float(fc.get("far_geometry_builder_max_range",80.0)),float(fc.get("far_geometry_builder_cell_size",1.0)),int(fc.get("far_geometry_builder_neighbor_cells",1)),int(fc.get("far_geometry_builder_min_points",2)),int(fc.get("far_geometry_builder_max_candidates",30))))
 print("Far Geometry Recovery: %s | bridge<=%.1fm z_gate<=%.1fm fragments<=%d max=%d"%("enabled" if fc.get("far_geometry_recovery_enabled",False) else "disabled",float(fc.get("far_geometry_recovery_bridge_distance",3.0)),float(fc.get("far_geometry_recovery_z_gate",1.0)),int(fc.get("far_geometry_recovery_max_fragments",3)),int(fc.get("far_geometry_recovery_max_candidates",8))))
 print("Recovery Quality Gate: %s | unsupported current points>=%d | stable track hits>=%d q>=%.2f"%("enabled" if fc.get("far_recovery_quality_gate_enabled",False) else "disabled",int(fc.get("far_recovery_quality_min_current_points_without_support",4)),int(fc.get("far_recovery_quality_track_min_hits",3)),float(fc.get("far_recovery_quality_track_min_quality",0.47))))
 admission_enabled=bool(fc.get("far_track_admission_enabled",False));admission_shadow=bool(fc.get("far_track_admission_shadow_mode",False))
 admission_mode="shadow" if admission_enabled and admission_shadow else ("enforcing" if admission_enabled else "disabled")
 print("Far New-Track Admission: %s | range>=%.0fm frames=%d match<=%.1fm | strong points>=%d score>=%.2f"%(admission_mode,float(fc.get("far_track_admission_min_range",50.0)),int(fc.get("far_track_admission_required_frames",2)),float(fc.get("far_track_admission_match_gate",2.5)),int(fc.get("far_track_admission_strong_min_points",10)),float(fc.get("far_track_admission_strong_min_score",0.72))))
 print("Far New Object Discovery: %s | range=%.0f..%.0fm | cell mid/far=%.2f/%.2fm | points mid/far>=%d/%d | max=%d"%("enabled" if fc.get("far_sparse_discovery_enabled",False) else "disabled",float(fc.get("far_sparse_discovery_min_range",30.0)),float(fc.get("far_sparse_discovery_max_range",80.0)),float(fc.get("far_sparse_discovery_mid_cell",0.90)),float(fc.get("far_sparse_discovery_far_cell",1.20)),int(fc.get("far_sparse_discovery_mid_min_points",4)),int(fc.get("far_sparse_discovery_far_min_points",3)),int(fc.get("far_sparse_discovery_max_candidates",40))))
 print("Discovery Diagnostics: observer-only | split TrackRescue/NewDiscovery stages + discovery-born track lifecycle")
 print("Road ROI margins: near=%.1fm mid=%.1fm far=%.1fm"%(float(fc.get("road_roi_margin",3.0)),float(fc.get("road_roi_margin_mid",4.2)),float(fc.get("road_roi_margin_far",4.5))))
 if fc.get("far_roi_adaptive_corridor_enabled",False):
  print("Far ROI Adaptive Corridor: %.0f..%.0fm margin %.1f..%.1fm"%(float(fc.get("far_roi_adaptive_min_range",50.0)),float(fc.get("far_roi_adaptive_max_range",80.0)),float(fc.get("far_roi_adaptive_base_margin",4.5)),float(fc.get("far_roi_adaptive_max_margin",5.5))))
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
 print("Far Admission Decision diagnostics: evaluation-only | truth labels never feed admission/tracking/fusion")
 print("Far Admission Feature Profiling: evaluation-only | score/points/range/shape/source truth-vs-FP")
 print("Far Admission Edge-Risk Shadow: evaluation-only | hard>=%.2f soft>=%.2f with score<%.2f or risky source | never filters Tracker input"%(float(eval_cfg.get("far_admission_edge_hard_ratio",.65)),float(eval_cfg.get("far_admission_edge_soft_ratio",.35)),float(eval_cfg.get("far_admission_edge_soft_score",.68))))
 print("Road-Object Precision Gate Shadow: evaluation-only | points>=%d height<=%.2fm range<=%.1fm | never filters ROI/Tracker/ObjectList"%(int(eval_cfg.get("road_object_gate_min_points",10)),float(eval_cfg.get("road_object_gate_max_height",.45)),float(eval_cfg.get("road_object_gate_max_range",25.0))))
 print("Road-Object Stage Attribution: evaluation-only | raw_radius=%.1fm stage_gate=%.1fm bands=%s"%(float(eval_cfg.get("road_object_raw_support_radius",1.5)),float(eval_cfg.get("road_object_stage_match_distance",2.0)),eval_cfg.get("road_object_stage_range_bins",[25.0,35.0,45.0])))
 print("Road-Object Rescue Profiler: %s | Truth Lifecycle Diagnostics: %s"%("enabled" if eval_cfg.get("road_object_hybrid_rescue_feature_profiling",False) else "disabled","enabled" if eval_cfg.get("truth_lifecycle_diagnostics",False) else "disabled"))
 print("Selected Admission Camera-Support Profiling: %s | evaluator-only; never changes admission or tracking"%("enabled" if eval_cfg.get("selected_track_admission_camera_profiling",False) else "disabled"))
 print("Selected Admission Camera-Rescue: Shadow ablations only | rules=%s"%eval_cfg.get("selected_track_admission_camera_rescue_ablations",[]))
 print("Selected Delayed Reappearance: Shadow only | rules=%s"%fc.get("selected_track_admission_delayed_reappearance_ablations",[]))
 print("Selected Delayed-Risk Policy: Shadow only | rule=%s gate=%s | never confirms or filters Tracker input"%(fc.get("selected_delayed_reappearance_selected_rule","-"),fc.get("selected_delayed_reappearance_selected_risk_gate",{})))
 print("Multi-Class Safety Baseline: vehicle + VRU + configured road obstacles | LiDAR unknowns remain unknown_obstacle")
 print("Qt/C++ portability: rescue/discovery/far-builder/quality/diagnostic logic uses scalar point/track evidence only; no CARLA actor data.")
 print("Background filter: %s"%("enabled" if fc.get("background_filter_enabled",False) else "disabled"))
 camera_runtimes={}
 for cc in station.camera_configs():
  camera_id=str(cc.get("id","CAM_01"));transform=station.camera_transforms.get(camera_id)
  if transform is None:continue
  width=int(cc.get("width",1280));height=int(cc.get("height",720))
  camera_runtimes[camera_id]={"config":cc,"width":width,"height":height,
   "projector":CameraProjector(width,height,cc.get("fov",90),transform)}
 primary_camera_id=next(iter(camera_runtimes),None)
 camera_source=config.get("camera_fusion",{}).get("source","none");assoc_cfg=config.get("camera_lidar_association",{})
 camera_detector=None;camera_detection_frames={};camera_detection_objects={}
 if camera_source=="detector" and config.get("camera_detection",{}).get("enabled",True):
  try:
   camera_detector=create_camera_detector(config.get("camera_detection",{}));print("Camera detector active in fusion loop: %s model=%s"%(camera_detector.name,config.get("camera_detection",{}).get("model","-")))
  except Exception as exc:
   print("WARNING: camera detector unavailable; camera classification disabled (no truth fallback): %s"%exc);camera_source="none"
 evaluator=None
 if eval_cfg.get("enabled",True):
  def eval_center():
   if station.junction_center is not None:return station.junction_center
   if station.base_transform is not None:return station.base_transform.location
   return None
  evaluator=GroundTruthEvaluator(station.world,eval_center,eval_cfg)
  _print_test_targets(evaluator)
 print("CARLA roadside sensors started: %d | cameras=%s"%(len(station.sensors),sorted(camera_runtimes)));print("V0.6.11.2 CARLA evaluator: %s"%("enabled" if evaluator else "disabled"))
 if evaluator:print("Evaluation radius: %.1fm, bins=%s, truth-track gate: %.1fm"%(evaluator.radius,evaluator.range_bins,evaluator.match_distance))
 print("ARCH: traffic -> ground removal -> clustering -> V0.6.11.1 range-aware rescue + V0.6.11.2 far geometry builder + V0.6.10 current-frame discovery -> road ROI -> far score -> tracker -> fusion")
 print("ARCH: Discovery Diagnostics observes source stages and discovery-born track lifecycle only.")
 print("ARCH: Far admission is observer-only in shadow mode; every dynamic candidate still reaches Tracker.")
 print("ARCH: Far admission uses LiDAR frame IDs; repeated reads of one frame cannot confirm a pending target.")
 print("ARCH: Selected new-track admission is Shadow-only; first-frame holds and repeat confirmations never filter Tracker input.")
 print("ARCH: Detection Stability/Drop diagnostics remain observer/evaluation-only.")
 print("ARCH: Camera association writes generic confirmation evidence back to track state for the next cycle.")
 print("ARCH: Ground Truth is evaluation-only and never enters perception/fusion/FusedObjectList.")
 print("Camera fusion source: %s"%camera_source)
 camera_ground_shadow=CameraGroundInitiationShadow(fc)
 print("V2X Event Engine: %s | AVW + SLW | canonical FusedObjectList input"%("enabled" if event_engine.enabled else "disabled"))
 print("Sensor snapshot mode: %s%s"%(args.sensor_sync," (default multi-camera alignment)" if args.sensor_sync=="aligned" else " (legacy diagnostic)"))
 if camera_source=="carla_truth":print("NOTE: CamObjects is simulation truth visibility, NOT real camera detector recall. Tracker receives only generic association confirmation, not truth actor data.")
 last=0.0;last_eval=0.0;last_json_sample=0.0;background_ready_announced=False;eval_interval=float(eval_cfg.get("report_interval",2.0));output_diag=config.get("output_diagnostics",{}) or {}
 try:
  while not _STOP_REQUESTED:
   cameras,lidar,radar=(station.cache.snapshot_all_aligned() if args.sensor_sync=="aligned" else station.cache.snapshot_all());ol=fusion.fuse(lidar[1] if lidar else None,radar[1] if radar else None,frame_id=lidar[0] if lidar else None,radar_frame_id=radar[0] if radar else None);ds=detdiag.update(fusion.last_dynamic_candidates);dds=discdiag.update(fusion.last_geometry_world,fusion.last_roi_candidates,fusion.last_scored_candidates,fusion.last_dynamic_candidates,fusion.last_tracked_candidates);camera_objects=[];camera_objects_by_id={};pairs=[]
   if evaluator is not None and radar is not None:
    evaluator.observe_radar_seed_bridge(
     fusion.last_radar_seed_bridge_shadow_candidates,frame_id=radar[0])
   if evaluator is not None and lidar is not None and (eval_cfg.get("far_admission_decision_diagnostics",False) or eval_cfg.get("far_admission_feature_profiling",False) or eval_cfg.get("far_admission_edge_risk_shadow",False)):
    evaluator.observe_far_admission_decisions(fusion.last_far_admission_rejections,fusion.last_far_admission_candidates,fusion.last_far_admission_expired_candidates,frame_id=lidar[0])
   for camera_id,runtime in camera_runtimes.items():
    camera=cameras.get(camera_id)
    if camera is None:continue
    projector=runtime["projector"];width=runtime["width"];height=runtime["height"]
    projected=project_lidar_tracks(projector,fusion.last_tracked_candidates,width,height)
    local_objects=[]
    if camera_source=="carla_truth":
     cam_list=make_truth_camera_objects(station.world,projector,camera_id,width,height,frame_id=camera[0],timestamp=ol.timestamp,obstacle_patterns=eval_cfg.get("obstacle_actor_patterns",[]));local_objects=cam_list.objects
    elif camera_source=="detector" and camera_detector is not None:
     if camera_detection_frames.get(camera_id)!=camera[0]:
      detections=camera_detector.detect(camera[1][:,:,:3].copy());cam_list=CameraObjectList.from_detections(camera_id,detections,timestamp=ol.timestamp,frame_id=camera[0]);camera_detection_objects[camera_id]=cam_list.objects;camera_detection_frames[camera_id]=camera[0]
     local_objects=list(camera_detection_objects.get(camera_id,[]))
    camera_objects_by_id[camera_id]=list(local_objects);camera_offset=len(camera_objects);camera_objects.extend(local_objects)
    for pair in associate_camera_to_lidar(local_objects,projected,min_iou=assoc_cfg.get("min_iou",.05),max_center_distance=assoc_cfg.get("max_center_distance",120.0)):
     p=dict(pair);p["camera_index"]=camera_offset+int(pair["camera_index"]);p["lidar_index"]=projected[pair["lidar_index"]]["source_index"];p["camera_id"]=camera_id;pairs.append(p)
   radar_camera_views=[]
   for camera_id,runtime in camera_runtimes.items():
    if cameras.get(camera_id) is None:continue
    radar_camera_views.append({"camera_id":camera_id,
     "projector":runtime["projector"],"width":runtime["width"],
     "height":runtime["height"],
     "camera_objects":camera_objects_by_id.get(camera_id,[]),
     "camera_source":camera_source,"frame_id":cameras.get(camera_id)[0]})
   radar_camera_candidates=annotate_radar_camera_support(
    fusion.last_radar_camera_support_shadow_candidates,radar_camera_views,
    camera_source=camera_source,min_iou=assoc_cfg.get("min_iou",.05),
    max_center_distance=assoc_cfg.get("max_center_distance",120.0))
   if evaluator is not None and radar is not None:
    evaluator.observe_radar_camera_support(
     radar_camera_candidates,frame_id=radar[0])
   camera_ground_token=tuple((view["camera_id"],view["frame_id"])
                             for view in radar_camera_views)
   camera_ground_candidates=camera_ground_shadow.update(
    radar_camera_views,list(fusion.last_roi_candidates)+list(fusion.last_tracked_candidates),
    station.junction_center.z if station.junction_center is not None else 0.0,
    validator=station.validate_driving_roi,frame_token=camera_ground_token)
   if evaluator is not None:
    evaluator.observe_camera_ground_initiation(
     camera_ground_candidates,frame_id=camera_ground_token)
   selected_camera_stats={"held":0,"visible":0,"supported":0,"source":camera_source}
   selected_held=fusion.last_selected_track_admission_rejections
   if eval_cfg.get("selected_track_admission_camera_profiling",False):
    primary_runtime=camera_runtimes.get(primary_camera_id);primary_camera=cameras.get(primary_camera_id) if primary_camera_id else None
    selected_held,selected_camera_stats=annotate_selected_camera_support(
     selected_held,(primary_runtime or {}).get("projector") if primary_camera is not None else None,
     camera_objects_by_id.get(primary_camera_id,[]),
     (primary_runtime or {}).get("width",0),(primary_runtime or {}).get("height",0),
     camera_source=camera_source,min_iou=assoc_cfg.get("min_iou",.05),
     max_center_distance=assoc_cfg.get("max_center_distance",120.0))
   if evaluator is not None and lidar is not None and eval_cfg.get("selected_track_admission_profiling",False):
    evaluator.observe_selected_track_admission(selected_held,fusion.last_selected_track_admission_candidates,fusion.last_selected_track_admission_expired_candidates,frame_id=lidar[0])
    evaluator.observe_selected_delayed_reappearance(
     fusion.last_selected_delayed_reappearance_candidates,
     fusion.last_selected_track_admission_candidates,frame_id=lidar[0])
   fusion.apply_camera_confirmations(pairs,timestamp=ol.timestamp)
   fol=build_fused_object_list(
    sid,fusion.last_tracked_candidates,ol.timestamp,camera_objects,pairs,
    frame_id=(lidar[0] if lidar else None),coordinate_frame="carla_world")
   # Every downstream consumer uses the same post-association
   # object list. Camera class/size/source evidence must not disappear at MQTT.
   oj=encode_object_list(fol);rj=encode_rsm(fol);now=time.time()
   if output_diag.get("fused_json_sample_enabled",True) and fol.objects and now-last_json_sample>=float(output_diag.get("fused_json_sample_interval",5.0)):
    print("[FUSED OUTPUT SAMPLE] %s"%json.dumps(fol.objects[0].to_dict(),ensure_ascii=False,separators=(",",":")));last_json_sample=now
   if now-last>=1.0:
    s=fusion.last_stats;camera_frames=",".join("%s:%s"%(cid,(cameras.get(cid) or ("-",))[0]) for cid in sorted(camera_runtimes));primary_camera=cameras.get(primary_camera_id) if primary_camera_id else None;cf=camera_frames or "-";rmin=s.get("radar_nearest_min");rmin_txt="-" if rmin is None else "%.2fm"%rmin;score_avg=s.get("candidate_score_avg");score_txt="-" if score_avg is None else "%.2f"%score_avg
    sync_frames=(int(primary_camera[0])-int(lidar[0])) if primary_camera is not None and lidar is not None else None
    sync_seconds=(float(primary_camera[2])-float(lidar[2])) if primary_camera is not None and lidar is not None and primary_camera[2] is not None and lidar[2] is not None else None
    print("  [SENSOR SYNC] Mode:%s Camera-LiDAR FrameDelta:%s TimeDelta:%s"%(args.sensor_sync,("-" if sync_frames is None else "%+d"%sync_frames),("-" if sync_seconds is None else "%+.3fs"%sync_seconds)))
    print("[RSU %s | %s] Camera:%s LiDAR:%d -> Ground:-%d => %d pts | Clusters:%d Geo:%d ROI:%d(+%d rescued) Reject:%d Score:%d(-%d avg=%s) Dyn:%d Tracks:%d | TrackLife N:%d U:%d C:%d S:%d D:%d | Radar:%d/%d Matched:%d Nearest:%s | Fused:%d Cam:%d/%d"%(sid,station.map_name,cf,s["lidar_points"],s.get("ground_removed_points",0),s.get("lidar_points_after_ground",s["lidar_points"]),s["lidar_clusters"],s.get("world_geometry_candidates",0),s["roi_candidates"],s.get("roi_rescued",0),s.get("roi_rejected",0),s.get("scored_candidates",s["roi_candidates"]),s.get("score_rejected",0),score_txt,s["background_candidates"],s["tracked_objects"],s.get("track_new",0),s.get("track_update",0),s.get("track_coast",0),s.get("track_suppress",0),s.get("track_drop",0),s["radar_detections"],s.get("radar_world_points",0),s.get("radar_matched_objects",0),rmin_txt,len(fol.objects),len(camera_objects),len(pairs)))
    if s.get("background_ready",False) and not background_ready_announced:
     for banner_line in background_ready_banner():print(banner_line)
     background_ready_announced=True
    print("  [BACKGROUND] Status:%s Remaining:%.1fs Cells:%d Rejected:%d"%(
     "READY" if s.get("background_ready",False) else "LEARNING",float(s.get("background_remaining",0.0)),int(s.get("background_cells",0)),int(s.get("background_rejected",0))))
    print("  [RADAR INIT] Mode:%s RangePts:%d Components:%d Accepted:%d SingleCand:%d Pending:%d Confirmed:%d SingleConfirm:%d Moving:%d StaticReject:%d DedupeReject:%d ROIReject:%d Emitted:%d SingleEmit:%d"%(
     "SHADOW" if s.get("radar_initiation_shadow_mode",True) else "ENFORCE",
     s.get("radar_initiation_range_points",0),s.get("radar_initiation_components",0),
     s.get("radar_initiation_clusters",0),s.get("radar_initiation_single_point_candidates",0),
     s.get("radar_initiation_pending",0),s.get("radar_initiation_confirmed",0),
     s.get("radar_initiation_single_point_confirmed",0),
     s.get("radar_initiation_moving",0),s.get("radar_initiation_static_rejected",0),
     s.get("radar_initiation_dedupe_rejected",0),s.get("radar_initiation_roi_rejected",0),
     s.get("radar_initiation_emitted",0),s.get("radar_initiation_single_point_emitted",0)))
    radar_speed_p50=s.get("radar_initiation_speed_p50");radar_speed_max=s.get("radar_initiation_speed_max");radar_speed_counts=s.get("radar_initiation_speed_shadow_counts",{}) or {}
    radar_speed_shadow=" ".join(">=%sm/s:%d"%(key,radar_speed_counts[key]) for key in sorted(radar_speed_counts,key=float)) or "-"
    print("  [RADAR SPEED SHADOW] ConfirmedAbsSpeed P50:%s Max:%s | WouldMove %s | TrackerInput:UNCHANGED"%(
     "-" if radar_speed_p50 is None else "%.2fm/s"%float(radar_speed_p50),
     "-" if radar_speed_max is None else "%.2fm/s"%float(radar_speed_max),radar_speed_shadow))
    radar_cumulative=s.get("radar_initiation_cumulative",{}) or {};single_speed=radar_cumulative.get("single_point_speed_counts",{}) or {};expired_hits=radar_cumulative.get("single_point_expired_hits",{}) or {}
    single_speed_text=" ".join(">=%sm/s:%d"%(key,single_speed[key]) for key in sorted(single_speed,key=float)) or "-"
    print("  [RADAR SINGLE LIFECYCLE CUMULATIVE] Frames:%d SingleComponents:%d MixedMovingComponents:%d BuriedMovingPoints:%d Candidates:%d Started:%d Matched:%d BelowSpeedNearPending:%d Expired:%d Hits(1/2/3+):%d/%d/%d Confirmed:%d Emitted:%d | SingletonSpeeds %s | OutputPolicy:UNCHANGED"%(
     radar_cumulative.get("frames",0),radar_cumulative.get("single_point_components",0),
     radar_cumulative.get("mixed_moving_components",0),radar_cumulative.get("moving_points_in_multi_components",0),
     radar_cumulative.get("single_point_candidates",0),radar_cumulative.get("single_point_started",0),
     radar_cumulative.get("single_point_matched",0),radar_cumulative.get("single_point_below_speed_near_pending",0),
     radar_cumulative.get("single_point_expired",0),expired_hits.get("1",0),expired_hits.get("2",0),expired_hits.get("3+",0),
     radar_cumulative.get("single_point_confirmed",0),radar_cumulative.get("single_point_emitted",0),single_speed_text))
    bridge=s.get("radar_seed_bridge_shadow",{}) or {};bridge_expired=bridge.get("expired_hits",{}) or {};bridge_rules=bridge.get("rules",{}) or {}
    print("  [RADAR MOTION-SEED BRIDGE SHADOW] Frames:%d Seeds:%d Matches:%d BelowSpeedMatches:%d Expired:%d Hits(1/2/3+):%d/%d/%d | TrackerInput:UNCHANGED"%(
     bridge.get("frames",0),bridge.get("seeds",0),bridge.get("matches",0),bridge.get("below_speed_matches",0),bridge.get("expired",0),bridge_expired.get("1",0),bridge_expired.get("2",0),bridge_expired.get("3+",0)))
    for rule in sorted(bridge_rules,key=int):
     value=bridge_rules[rule]
     print("    [BRIDGE GATE %.1fm %s-FRAME] Confirmed:%d DedupeReject:%d ROIReject:%d WouldEmit:%d | OutputPolicy:UNCHANGED"%(
      float(fc.get("radar_initiation_match_gate",2.5)),rule,value.get("confirmed",0),value.get("dedupe_rejected",0),value.get("roi_rejected",0),value.get("would_emit",0)))
    for gate,value in sorted((bridge.get("gate_ablation",{}) or {}).items(),key=lambda item:float(item[0])):
     gate_expired=value.get("expired_hits",{}) or {}
     print("    [BRIDGE GATE %.1fm LIFECYCLE] Seeds:%d Matches:%d BelowSpeedMatches:%d Expired:%d Hits(1/2/3+):%d/%d/%d"%(
      float(gate),value.get("seeds",0),value.get("matches",0),value.get("below_speed_matches",0),value.get("expired",0),gate_expired.get("1",0),gate_expired.get("2",0),gate_expired.get("3+",0)))
     for rule,rule_value in sorted((value.get("rules",{}) or {}).items(),key=lambda item:int(item[0])):
      print("      [BRIDGE GATE %.1fm %s-FRAME] Confirmed:%d DedupeReject:%d ROIReject:%d WouldEmit:%d | OutputPolicy:UNCHANGED"%(
       float(gate),rule,rule_value.get("confirmed",0),rule_value.get("dedupe_rejected",0),rule_value.get("roi_rejected",0),rule_value.get("would_emit",0)))
    morph=s.get("radar_seed_to_component_shadow",{}) or {}
    for gate,value in sorted(morph.items(),key=lambda item:float(item[0])):
     avg_points=(float(value.get("matched_points",0))/value.get("matches",1)) if value.get("matches",0) else 0.0
     print("    [SINGLETON->COMPONENT GATE %.1fm] Seeds:%d Matches:%d MovingMatches:%d AvgPoints:%.1f Expired:%d DedupeReject:%d ROIReject:%d WouldEmit:%d | OutputPolicy:UNCHANGED"%(
      float(gate),value.get("seeds",0),value.get("matches",0),value.get("moving_matches",0),avg_points,value.get("expired",0),value.get("dedupe_rejected",0),value.get("roi_rejected",0),value.get("would_emit",0)))
    radar_camera=s.get("radar_camera_support_shadow",{}) or {};radar_camera_eval=evaluator.report_radar_camera_support() if evaluator is not None else {}
    print("    [RADAR CAMERA SUPPORT SHADOW] Raw:%d DedupeReject:%d ROIReject:%d Eligible:%d | Visible:%d Supported:%d Rate:%s SupportedTruth:%d FP:%d Precision:%s Source:%s | TrackerInput:UNCHANGED"%(
     radar_camera.get("raw",0),radar_camera.get("dedupe_rejected",0),radar_camera.get("roi_rejected",0),radar_camera.get("eligible",0),radar_camera_eval.get("visible",0),radar_camera_eval.get("supported",0),_pct(radar_camera_eval.get("support_rate")),radar_camera_eval.get("supported_truth",0),radar_camera_eval.get("supported_fp",0),_pct(radar_camera_eval.get("supported_precision")),radar_camera_eval.get("sources",{})))
    camera_ground=camera_ground_shadow.report();camera_ground_eval=evaluator.report_camera_ground_initiation() if evaluator is not None else {}
    print("    [CAMERA GROUND INIT SHADOW] Frames:%d Det:%d ClassReject:%d ConfReject:%d ProjectionReject:%d RangeReject:%d CrossCamDedupe:%d LiDARDedupe:%d ROIReject:%d WouldEmit:%d | Truth:%d FP:%d Precision:%s Classes:%s Source:%s | TrackerInput:UNCHANGED"%(
     camera_ground.get("frames",0),camera_ground.get("detections",0),camera_ground.get("class_rejected",0),camera_ground.get("confidence_rejected",0),camera_ground.get("projection_rejected",0),camera_ground.get("range_rejected",0),camera_ground.get("cross_camera_deduped",0),camera_ground.get("lidar_deduped",0),camera_ground.get("roi_rejected",0),camera_ground.get("would_emit",0),camera_ground_eval.get("matched",0),camera_ground_eval.get("fp",0),_pct(camera_ground_eval.get("precision")),camera_ground_eval.get("classes",{}),camera_ground_eval.get("sources",{})))
    _print_sparse_geometry(s);_print_road_object_recovery(s);_print_discovery_diagnostics(dds);_print_rescue_gate();_print_far_geometry();_print_detection_stability(ds)
    print("  [TRACK QUALITY] Active:%d High:%d Medium:%d Low:%d Suppressed:%d AvgQuality:%.2f"%(s.get("track_quality_active",0),s.get("track_quality_high",0),s.get("track_quality_medium",0),s.get("track_quality_low",0),s.get("track_suppress",0),float(s.get("track_quality_avg",0.0))))
    print("  [TRACK LIFE GATE] low_hit_keep:%d low_new_drop:%d"%(s.get("track_low_hit_keep",0),s.get("track_low_new_drop",0)))
    print("  [FAR TRACK ADMISSION] Mode:%s Pending:%d WouldHold:%d WouldConfirm:%d Expired:%d SensorBypass:%d StrongBypass:%d TrackBypass:%d TrackerInput:%d"%("SHADOW" if s.get("far_admission_shadow_mode",False) else "ENFORCE",s.get("far_admission_pending",0),s.get("far_admission_held",0),s.get("far_admission_confirmed",0),s.get("far_admission_expired",0),s.get("far_admission_sensor_bypass",0),s.get("far_admission_strong_bypass",0),s.get("far_admission_track_bypass",0),s.get("far_admission_tracker_input",0)))
    print("  [SELECTED NEW-TRACK ADMISSION] Mode:%s Pending:%d WouldHold:%d WouldConfirm:%d Expired:%d SensorBypass:%d TrackBypass:%d TrackerInput:%d"%("SHADOW" if s.get("selected_track_admission_shadow_mode",False) else "ENFORCE",s.get("selected_track_admission_pending",0),s.get("selected_track_admission_held",0),s.get("selected_track_admission_confirmed",0),s.get("selected_track_admission_expired",0),s.get("selected_track_admission_sensor_bypass",0),s.get("selected_track_admission_track_bypass",0),s.get("selected_track_admission_tracker_input",0)))
    print("  [SELECTED ADMISSION CAMERA SHADOW] Source:%s Held:%d Visible:%d Supported:%d"%(selected_camera_stats.get("source","none"),selected_camera_stats.get("held",0),selected_camera_stats.get("visible",0),selected_camera_stats.get("supported",0)))
    for rule,value in sorted((fusion.last_selected_delayed_reappearance_stats or {}).items()):
     print("  [SELECTED DELAYED REAPPEARANCE SHADOW %s] TTL:%.1fs Gate:%.1fm Eligible:%d Pending:%d Confirm:%d Expired:%d"%(rule,value.get("ttl",0.0),value.get("match_gate",0.0),value.get("eligible",0),value.get("pending",0),value.get("confirmed",0),value.get("expired",0)))
     if value.get("selected_risk_shadow",False):
      print("  [SELECTED DELAYED RISK POLICY SHADOW %s] WouldKeep:%d WouldReject:%d Rule:%s | TrackerInput:UNCHANGED"%(rule,value.get("would_keep",0),value.get("would_reject",0),value.get("risk_gate",{})))
    if s.get("roi_rejection_reasons"):print("  ROI rejected reasons: %s"%s["roi_rejection_reasons"])
    if s.get("roi_rescued",0):print("  Geometry-aware ROI rescued: %d"%s.get("roi_rescued",0))
    if s.get("score_rejected",0):print("  Candidate score rejected: %d"%s.get("score_rejected",0))
    for idx,o in enumerate(fol.objects[:10]):
     t=fusion.last_tracked_candidates[idx];size=o.size;rs="-" if o.radar_speed is None else "%.2f"%o.radar_speed;cam="-" if o.camera is None else "%s box=%s"%(o.camera.get("cameraId","?"),o.camera.get("bbox"));near=t.get("radar_nearest_xy");near_txt="-" if near is None else "%.2f"%near;raw_speed=math.hypot(t.get("raw_vx",0),t.get("raw_vy",0));fused_speed=math.hypot(o.vx,o.vy);state=t.get("track_state","confirmed");allowed=int(t.get("coast_allowed",0));q=float(t.get("track_quality",0.0));sensors=t.get("track_sensors","L")
     print("  %-12s type=%-7s state=%-9s q=%.2f sensors=%-3s coast=%d/%d pos=(%7.2f,%7.2f,%5.2f) vel=(%6.2f,%6.2f) speed=%.2f raw=%.2f size=(%.2f,%.2f,%.2f) radar=%s near=%sm hits=%d cam=%s conf=%.2f src=%s"%(o.object_id,o.object_type,state,q,sensors,int(t.get("coast_frames",0)),allowed,o.x,o.y,o.z,o.vx,o.vy,fused_speed,raw_speed,size[0],size[1],size[2],rs,near_txt,int(t.get("radar_hits",0)),cam,o.confidence,"+".join(o.sources)))
    last=now
   if evaluator is not None and now-last_eval>=eval_interval:
    s=fusion.last_stats;ev=evaluator.evaluate(fusion.last_tracked_candidates,camera_objects,pairs,s.get("radar_matched_objects",0));geo=evaluator.evaluate_candidates(fusion.last_geometry_world);roi=evaluator.evaluate_candidates(fusion.last_roi_candidates);scored=evaluator.evaluate_candidates(fusion.last_scored_candidates);dyn=evaluator.evaluate_candidates(fusion.last_dynamic_candidates);ga=evaluator.analyze_geometry_attribution(fusion.last_geometry_world);road_ga=evaluator.analyze_road_object_recovery(fusion.last_road_object_recovery_candidates);selected_attr=evaluator.analyze_selected_enforcement_attribution(fusion.last_roi_candidates,fusion.last_scored_candidates,fusion.last_dynamic_candidates,fusion.last_tracked_candidates);selected_score=evaluator.analyze_selected_admission_score_profile(list(fusion.last_scored_candidates)+list(fusion.last_score_rejections));road_diag=fusion.road_object_recovery_diagnostics_world();road_stage=evaluator.analyze_road_object_recovery_stages(road_diag);road_stages=road_diag.get("stages",{}) or {};road_cap=evaluator.analyze_road_object_cap_comparison(fusion.last_road_object_recovery_candidates,road_stages.get("balanced_output",[]),road_stages.get("adaptive_output",[]),road_stages.get("adaptive_ranked_output",[]),road_stages.get("adaptive_stratified_output",[]),road_stages.get("adaptive_hybrid_output",[]),road_stages.get("adaptive_hybrid_gated_output",[]),road_stages.get("adaptive_hybrid_rescued_output",[]),road_stages.get("adaptive_hybrid_geometry_gated_output",[]),road_stages.get("selected_output",[]));road_adaptive=evaluator.analyze_road_object_adaptive_profile(road_stages.get("adaptive_dedupe_pass",[]));road_hybrid=evaluator.analyze_road_object_hybrid_profile(road_stages.get("adaptive_hybrid_output",[]));road_rescue=evaluator.analyze_road_object_hybrid_rescue_profile(road_stages.get("adaptive_hybrid_rescued_output",[]));truth_lifecycle=evaluator.analyze_truth_lifecycle();dd=evaluator.analyze_detection_drop_reasons(fusion.last_geometry_world,fusion.last_roi_candidates,fusion.last_scored_candidates,fusion.last_dynamic_candidates,fusion.last_roi_rejections,fusion.last_score_rejections)
    print("[EVAL %.0fm] Truth:%d Tracks:%d Matched:%d Missed:%d FP:%d Recall:%s Precision:%s PosErr:%s/%s RadarMatched:%d CamVisibleTruth:%d CamLiDAR:%d"%(evaluator.radius,ev["truth"],ev["detected"],ev["matched"],ev["missed"],ev["false_positive"],_pct(ev["recall"]),_pct(ev["precision"]),_meters(ev["mean_position_error"]),_meters(ev["max_position_error"]),ev["radar_matched"],ev["camera_visible"],ev["camera_lidar_matched"]))
    _print_multiclass(ev)
    _print_stage("GEOMETRY",geo);_print_stage("ROI",roi);_print_stage("SCORE",scored);_print_stage("DYNAMIC",dyn);_print_stage("TRACK",ev)
    _print_sparse_geometry(s);_print_road_object_recovery(s);_print_road_object_profile(road_ga);_print_road_object_stage_attribution(road_stage);_print_road_object_cap_comparison(road_cap);_print_selected_enforcement_attribution(selected_attr);_print_selected_admission_score_profile(selected_score);_print_adaptive_temporal_profile(road_adaptive);_print_hybrid_selection_profile(road_hybrid);_print_hybrid_rescue_profile(road_rescue);_print_truth_lifecycle(truth_lifecycle);_print_discovery_diagnostics(dds);_print_rescue_gate();_print_far_geometry();_print_detection_stability(ds);_print_geometry_attribution(ga);_print_detection_drop(dd)
    _print_selected_track_admission_profile(evaluator.report_selected_track_admission())
    for rule,value in sorted(evaluator.report_radar_seed_bridge().items()):
     print("  [RADAR BRIDGE TRUTH %s] Candidates:%d Matched:%d FP:%d Precision:%s | EvaluationOnly"%(
      rule,value.get("candidates",0),value.get("matched",0),value.get("fp",0),_pct(value.get("precision"))))
    if eval_cfg.get("far_admission_decision_diagnostics",False) or eval_cfg.get("far_admission_feature_profiling",False) or eval_cfg.get("far_admission_edge_risk_shadow",False):
     admission_report=evaluator.report_far_admission_decisions(reset=True)
     if eval_cfg.get("far_admission_decision_diagnostics",False):_print_far_admission_eval(admission_report)
     if eval_cfg.get("far_admission_feature_profiling",False):
      profiles=admission_report.get("feature_profiles",{})
     _print_far_admission_profile("hold",profiles.get("would_hold",{}));_print_far_admission_profile("confirm",profiles.get("would_confirm",{}));_print_far_admission_profile("expired",profiles.get("expired",{}))
     if eval_cfg.get("far_admission_edge_risk_shadow",False):
      risk=admission_report.get("edge_risk_shadow",{})
      _print_far_admission_risk_shadow("hold",risk.get("would_hold",{}));_print_far_admission_risk_shadow("confirm",risk.get("would_confirm",{}));_print_far_admission_risk_shadow("expired",risk.get("expired",{}))
      _print_far_admission_risk_classes(admission_report.get("edge_risk_classes",{}).get("would_confirm",{}))
    print("  [TRACK LIFE] NEW:%d UPDATE:%d COAST:%d SUPPRESS:%d DROP:%d"%(s.get("track_new",0),s.get("track_update",0),s.get("track_coast",0),s.get("track_suppress",0),s.get("track_drop",0)))
    print("  [TRACK QUALITY] Active:%d High:%d Medium:%d Low:%d AvgQuality:%.2f"%(s.get("track_quality_active",0),s.get("track_quality_high",0),s.get("track_quality_medium",0),s.get("track_quality_low",0),float(s.get("track_quality_avg",0.0))))
    print("  [TRACK LIFE GATE] low_hit_keep:%d low_new_drop:%d"%(s.get("track_low_hit_keep",0),s.get("track_low_new_drop",0)))
    print("  [FAR TRACK ADMISSION] Mode:%s Pending:%d WouldHold:%d WouldConfirm:%d Expired:%d SensorBypass:%d StrongBypass:%d TrackBypass:%d TrackerInput:%d"%("SHADOW" if s.get("far_admission_shadow_mode",False) else "ENFORCE",s.get("far_admission_pending",0),s.get("far_admission_held",0),s.get("far_admission_confirmed",0),s.get("far_admission_expired",0),s.get("far_admission_sensor_bypass",0),s.get("far_admission_strong_bypass",0),s.get("far_admission_track_bypass",0),s.get("far_admission_tracker_input",0)))
    print("  [SELECTED NEW-TRACK ADMISSION] Mode:%s Pending:%d WouldHold:%d WouldConfirm:%d Expired:%d SensorBypass:%d TrackBypass:%d TrackerInput:%d"%("SHADOW" if s.get("selected_track_admission_shadow_mode",False) else "ENFORCE",s.get("selected_track_admission_pending",0),s.get("selected_track_admission_held",0),s.get("selected_track_admission_confirmed",0),s.get("selected_track_admission_expired",0),s.get("selected_track_admission_sensor_bypass",0),s.get("selected_track_admission_track_bypass",0),s.get("selected_track_admission_tracker_input",0)))
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
   m=config["mqtt"];pub.publish(m["topic_object_list"],oj);pub.publish(m["topic_rsm"],rj)
   ego_speed=(config.get("v2x_events",{}) or {}).get("test_ego_speed_kmh")
   for event in event_engine.update(fol,{"speed_kmh":ego_speed} if ego_speed is not None else {}):
    payload=encode_v2x_event(event);pub.publish(m.get("topic_event","roadside/%s/event"%sid),payload);print("[V2X EVENT] %s"%payload)
   time.sleep(.05)
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
