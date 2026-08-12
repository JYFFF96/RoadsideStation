from __future__ import print_function
import math,time,statistics
from collections import defaultdict
from .models import DetectedObject,ObjectList
from .perception import voxel_cluster_lidar,adaptive_voxel_cluster_lidar,merge_lidar_clusters
from .tracking import NearestTracker

class PersistentStaticFilter(object):
 def __init__(self,calibration_seconds=6.,cell_size=1.,occupancy_ratio=.45,moving_radar_speed=1.2,neighbor_radius_cells=2,**kwargs):self.calibration_seconds=float(calibration_seconds);self.cell_size=float(cell_size);self.occupancy_ratio=float(occupancy_ratio);self.moving_radar_speed=float(moving_radar_speed);self.neighbor_radius_cells=int(neighbor_radius_cells);self.started_at=None;self.frames=0;self.counts=defaultdict(int);self.static_cells=set();self.ready=False
 def _key(self,x,y):return(int(math.floor(x/self.cell_size)),int(math.floor(y/self.cell_size)))
 def _near_static(self,k):
  x,y=k;r=max(0,self.neighbor_radius_cells);return any((x+a,y+b) in self.static_cells for a in range(-r,r+1) for b in range(-r,r+1))
 def update_and_filter(self,c,now):
  if self.started_at is None:self.started_at=now
  if not self.ready:
   self.frames+=1
   for k in set(self._key(x["x"],x["y"]) for x in c):self.counts[k]+=1
   if now-self.started_at>=self.calibration_seconds:
    th=max(2,int(math.ceil(self.frames*self.occupancy_ratio)));self.static_cells=set(k for k,n in self.counts.items() if n>=th);self.ready=True
   return []
  return [x for x in c if not(self._near_static(self._key(x["x"],x["y"])) and not(x.get("radar_radial_velocity") is not None and abs(x["radar_radial_velocity"])>=self.moving_radar_speed))]
 def remaining_seconds(self,now):return 0. if self.ready else self.calibration_seconds if self.started_at is None else max(0.,self.calibration_seconds-(now-self.started_at))

class SimpleFusion(object):
 def __init__(self,station_id,config):
  self.station_id=station_id;self.config=config
  self.tracker=NearestTracker(config.get("track_match_distance",4.),config.get("track_max_age",1.5),config.get("track_max_speed",20.),config.get("velocity_alpha",.25),config.get("extent_alpha",.25),config.get("extent_shrink_alpha",.05),config.get("extent_lock_hits",5),config.get("radar_velocity_alpha",.35),config.get("velocity_window",5),config.get("position_alpha",.45),config.get("stationary_speed",.35))
  self.background=PersistentStaticFilter(**config);self.world_transform=None;self.radar_matrix=None;self.radar_origin=None;self.ground_reference_z=None;self.candidate_validator=None
  self.last_stats={};self.last_geometry_world=[];self.last_roi_candidates=[];self.last_scored_candidates=[];self.last_dynamic_candidates=[];self.last_tracked_candidates=[];self.last_roi_rejections=[];self.last_score_rejections=[]
 def set_world_transform(self,t):
  if t is None:self.world_transform=None;return
  self.world_transform={"x":float(t.location.x),"y":float(t.location.y),"z":float(t.location.z),"yaw":math.radians(float(t.rotation.yaw))}
 def set_ground_reference(self,z):self.ground_reference_z=None if z is None else float(z)
 def set_radar_transform(self,t):
  if t is None:self.radar_matrix=None;self.radar_origin=None;return
  try:self.radar_matrix=t.get_matrix()
  except Exception:self.radar_matrix=None
  self.radar_origin=(float(t.location.x),float(t.location.y),float(t.location.z))
 def set_candidate_validator(self,v):self.candidate_validator=v
 def _to_world(self,x,y,z):
  if self.world_transform is None:return x,y,z
  t=self.world_transform;c=math.cos(t["yaw"]);s=math.sin(t["yaw"]);return t["x"]+c*x-s*y,t["y"]+s*x+c*y,t["z"]+z
 def _sensor_range(self,x,y):
  if self.world_transform is None:return math.hypot(float(x),float(y))
  return math.hypot(float(x)-self.world_transform["x"],float(y)-self.world_transform["y"])
 def _remove_ground_points(self,points):
  if points is None:return None,0
  total=len(points)
  if total==0:return points,0
  if not self.config.get("ground_removal_enabled",True) or self.ground_reference_z is None or self.world_transform is None:return points,0
  clearance=float(self.config.get("ground_clearance",0.30));sensor_z=float(self.world_transform["z"]);cut_local=float(self.ground_reference_z)+clearance-sensor_z
  kept=[p for p in points if float(p[2])>cut_local]
  return kept,total-len(kept)
 def _radar_point_to_world(self,d):
  if self.radar_matrix is None:return None
  m=self.radar_matrix;x=float(d["x"]);y=float(d["y"]);z=float(d["z"])
  wx=m[0][0]*x+m[0][1]*y+m[0][2]*z+m[0][3];wy=m[1][0]*x+m[1][1]*y+m[1][2]*z+m[1][3];wz=m[2][0]*x+m[2][1]*y+m[2][2]*z+m[2][3]
  ox,oy,oz=self.radar_origin;dx=wx-ox;dy=wy-oy;dz=wz-oz;n=math.sqrt(dx*dx+dy*dy+dz*dz)
  if n<1e-3:return None
  return {"x":wx,"y":wy,"z":wz,"velocity":float(d.get("velocity",0.0)),"los_x":dx/n,"los_y":dy/n,"los_z":dz/n}
 def _associate_radar_world(self,clusters,radar_detections):
  points=[]
  for d in radar_detections or []:
   p=self._radar_point_to_world(d)
   if p is not None:points.append(p)
  max_d=float(self.config.get("radar_match_distance",4.0));max_z=float(self.config.get("radar_match_z",2.5));min_hits=int(self.config.get("radar_min_hits",1));out=[];matched=0
  for c in clusters:
   near=[];nearest_xy=None;nearest_3d=None
   for p in points:
    dxy=math.hypot(p["x"]-c["x"],p["y"]-c["y"]);d3=math.sqrt(dxy*dxy+(p["z"]-c["z"])**2)
    if nearest_xy is None or dxy<nearest_xy:nearest_xy=dxy
    if nearest_3d is None or d3<nearest_3d:nearest_3d=d3
    if abs(p["z"]-c["z"])>max_z:continue
    if dxy<=max_d:near.append((dxy,p))
   item=dict(c);item["radar_nearest_xy"]=nearest_xy;item["radar_nearest_3d"]=nearest_3d;item["radar_hits"]=0
   if len(near)>=min_hits:
    near.sort(key=lambda x:x[0]);use=[p for _,p in near[:max(1,min(8,len(near)))]]
    item["radar_radial_velocity"]=float(statistics.median([p["velocity"] for p in use]));item["radar_los_x"]=sum(p["los_x"] for p in use)/len(use);item["radar_los_y"]=sum(p["los_y"] for p in use)/len(use);item["radar_hits"]=len(near);matched+=1
   out.append(item)
  return out,len(points),matched
 def _looks_like_pole(self,e):
  ex,ey,ez=[float(v) for v in e];hl=max(ex,ey);hs=min(ex,ey);c=self.config
  return hs<c.get("pole_short_max",.75) and hl<c.get("pole_long_max",2.5) and ez>c.get("pole_height_min",1.5)
 def _validate_candidate(self,wx,wy,wz,e):
  if not self.candidate_validator:return True,"ok",{}
  try:
   result=self.candidate_validator(wx,wy,wz,e)
   if isinstance(result,tuple):
    ok=bool(result[0]);reason=result[1] if len(result)>1 else ("ok" if ok else "rejected");details=result[2] if len(result)>2 else {};return ok,reason,details
   return bool(result),("ok" if result else "rejected"),{}
  except Exception as exc:return False,"validator_error",{"error":str(exc)}
 def _cluster(self,clean_points,c):
  if c.get("range_adaptive_clustering",False):
   return adaptive_voxel_cluster_lidar(clean_points,c.get("range_bands",[]),c.get("lidar_min_z",-7.5),c.get("lidar_max_z",2.),c.get("max_range",80.),c.get("vehicle_max_length",8.),c.get("vehicle_max_width",4.),c.get("vehicle_max_height",4.),c.get("max_objects",120))
  return voxel_cluster_lidar(clean_points,c.get("voxel_size",.8),c.get("cluster_min_points",6),c.get("lidar_min_z",-7.5),c.get("lidar_max_z",2.),c.get("max_range",70.),c.get("vehicle_min_length",.6),c.get("vehicle_max_length",8.),c.get("vehicle_min_width",.4),c.get("vehicle_max_width",4.),c.get("vehicle_min_height",.25),c.get("vehicle_max_height",4.),c.get("max_objects",80))
 def _candidate_score(self,item):
  e=[float(v) for v in item.get("extent",[0,0,0])];hl=max(e[0],e[1]);hs=min(e[0],e[1]);h=e[2];points=int(item.get("point_count",0));votes=int(item.get("scale_votes",1));details=item.get("roi_details",{}) or {};score=0.0
  if 1.2<=hl<=7.5:score+=0.22
  elif 0.7<=hl<=8.0:score+=0.10
  if 0.55<=hs<=3.4:score+=0.20
  elif 0.30<=hs<=3.8:score+=0.08
  if 0.45<=h<=3.5:score+=0.20
  elif 0.20<=h<=3.8:score+=0.08
  if points>=8:score+=0.16
  elif points>=4:score+=0.11
  elif points>=2:score+=0.05
  if votes>=2:score+=0.17
  else:score+=0.04
  lateral=details.get("lateral");allowed=details.get("allowed_lateral")
  if lateral is not None and allowed not in (None,0):
   ratio=float(lateral)/max(0.01,float(allowed))
   if ratio<=0.65:score+=0.05
   elif ratio<=0.85:score+=0.03
  return min(1.0,score)
 def _score_candidates(self,items):
  c=self.config
  if not c.get("candidate_scoring_enabled",False):return [dict(x) for x in items],[]
  min_range=float(c.get("candidate_scoring_min_range",50.0));threshold=float(c.get("candidate_scoring_threshold_far",0.48));kept=[];rejected=[]
  for src in items:
   item=dict(src);rng=self._sensor_range(item["x"],item["y"]);item["sensor_range"]=rng
   if rng<min_range:
    item["candidate_score"]=1.0;item["candidate_score_bypass"]=True;kept.append(item);continue
   score=self._candidate_score(item);item["candidate_score"]=score;item["candidate_score_bypass"]=False
   if score>=threshold:kept.append(item)
   else:
    r=dict(item);r["reason"]="candidate_score";rejected.append(r)
  return kept,rejected
 def fuse(self,lidar_points,radar_detections,timestamp=None):
  now=time.time() if timestamp is None else float(timestamp);c=self.config
  clean_points,ground_removed=self._remove_ground_points(lidar_points);raw=self._cluster(clean_points,c)
  filtered=[x for x in raw if not self._looks_like_pole(x.get("extent",[0,0,0]))]
  clusters=merge_lidar_clusters(filtered,c.get("cluster_merge_gap",1.4),c.get("merged_vehicle_max_length",14.),c.get("merged_vehicle_max_width",4.2),c.get("merged_vehicle_max_height",4.2),c.get("near_merge_range",30.0),c.get("near_merge_gap",0.65),c.get("near_merged_vehicle_max_length",7.5),c.get("near_merged_vehicle_max_width",3.2)) if c.get("cluster_merge_enabled",True) else filtered
  world_clusters=[];accepted=[];roi_rejections=[];roi_rescued=0
  for i in clusters:
   wx,wy,wz=self._to_world(i["x"],i["y"],i["z"]);e=i.get("extent",[0,0,0]);item={"x":wx,"y":wy,"z":wz,"confidence":.72,"sources":["lidar"],"point_count":i.get("point_count",0),"extent":e,"cluster_mode":i.get("cluster_mode","3d"),"scale_votes":int(i.get("scale_votes",1)),"scale_modes":list(i.get("scale_modes",[i.get("cluster_mode","3d")]))};world_clusters.append(dict(item))
   ok,reason,details=self._validate_candidate(wx,wy,wz,e)
   if not ok:
    rej=dict(item);rej["reason"]=reason;rej["details"]=details;roi_rejections.append(rej);continue
   item["roi_reason"]=reason;item["roi_details"]=details
   if details.get("geometry_rescued",False):roi_rescued+=1
   accepted.append(item)
  self.last_geometry_world=[dict(x) for x in world_clusters];self.last_roi_rejections=roi_rejections;self.last_roi_candidates=[dict(x) for x in accepted]
  scored,score_rejections=self._score_candidates(accepted);self.last_scored_candidates=[dict(x) for x in scored];self.last_score_rejections=[dict(x) for x in score_rejections]
  assoc,radar_world_count,radar_matched=self._associate_radar_world(scored,radar_detections);roi=[]
  for item in assoc:
   if item.get("radar_radial_velocity") is not None:item["confidence"]=.90;item["sources"]=["lidar","radar"]
   roi.append(item)
  if c.get("background_filter_enabled",False):
   dyn=self.background.update_and_filter(roi,now);background_ready=self.background.ready;background_remaining=self.background.remaining_seconds(now);background_cells=len(self.background.static_cells)
  else:
   dyn=[dict(x) for x in roi];background_ready=True;background_remaining=0.0;background_cells=0
  self.last_dynamic_candidates=[dict(x) for x in dyn];tracked=self.tracker.update(dyn,now);self.last_tracked_candidates=[dict(x) for x in tracked]
  objs=[DetectedObject(i["id"],i["x"],i["y"],i["z"],vx=i["vx"],vy=i["vy"],object_type="unknown",confidence=i["confidence"],sources=i["sources"]) for i in tracked]
  nearest=[x.get("radar_nearest_xy") for x in roi if x.get("radar_nearest_xy") is not None];reasons=defaultdict(int)
  for r in roi_rejections:reasons[r.get("reason","rejected")]+=1
  score_values=[float(x.get("candidate_score",1.0)) for x in scored if not x.get("candidate_score_bypass",False)]
  self.last_stats={"lidar_points":0 if lidar_points is None else len(lidar_points),"ground_removed_points":ground_removed,"lidar_points_after_ground":0 if clean_points is None else len(clean_points),"raw_lidar_clusters":len(raw),"geometry_clusters":len(filtered),"lidar_clusters":len(clusters),"world_geometry_candidates":len(world_clusters),"roi_candidates":len(accepted),"roi_rejected":len(roi_rejections),"roi_rescued":roi_rescued,"roi_rejection_reasons":dict(reasons),"scored_candidates":len(scored),"score_rejected":len(score_rejections),"candidate_scoring_enabled":bool(c.get("candidate_scoring_enabled",False)),"candidate_score_avg":(sum(score_values)/len(score_values) if score_values else None),"background_candidates":len(dyn),"background_rejected":max(0,len(roi)-len(dyn)),"background_ready":background_ready,"background_remaining":background_remaining,"background_cells":background_cells,"background_filter_enabled":bool(c.get("background_filter_enabled",False)),"range_adaptive_clustering":bool(c.get("range_adaptive_clustering",False)),"radar_detections":0 if not radar_detections else len(radar_detections),"radar_world_points":radar_world_count,"radar_matched_objects":radar_matched,"radar_nearest_min":min(nearest) if nearest else None,"tracked_objects":len(objs)}
  return ObjectList(self.station_id,objs,timestamp=now)
