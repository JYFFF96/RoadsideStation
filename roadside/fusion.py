from __future__ import print_function
import math,time
from collections import defaultdict
from .models import DetectedObject,ObjectList
from .perception import voxel_cluster_lidar,merge_lidar_clusters,associate_radar
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
  return [x for x in c if not(self._near_static(self._key(x["x"],x["y"])) and not(x.get("radar_speed") is not None and abs(x["radar_speed"])>=self.moving_radar_speed))]
 def remaining_seconds(self,now):return 0. if self.ready else self.calibration_seconds if self.started_at is None else max(0.,self.calibration_seconds-(now-self.started_at))

class SimpleFusion(object):
 def __init__(self,station_id,config):
  self.station_id=station_id;self.config=config;self.tracker=NearestTracker(config.get("track_match_distance",4.),config.get("track_max_age",1.5),config.get("track_max_speed",20.),config.get("velocity_alpha",.35),config.get("extent_alpha",.25));self.background=PersistentStaticFilter(**config);self.world_transform=None;self.candidate_validator=None;self.last_stats={};self.last_dynamic_candidates=[];self.last_tracked_candidates=[]
 def set_world_transform(self,t):
  if t is None:self.world_transform=None;return
  self.world_transform={"x":float(t.location.x),"y":float(t.location.y),"z":float(t.location.z),"yaw":math.radians(float(t.rotation.yaw))}
 def set_candidate_validator(self,v):self.candidate_validator=v
 def _to_world(self,x,y,z):
  if self.world_transform is None:return x,y,z
  t=self.world_transform;c=math.cos(t["yaw"]);s=math.sin(t["yaw"]);return t["x"]+c*x-s*y,t["y"]+s*x+c*y,t["z"]+z
 def fuse(self,lidar_points,radar_detections,timestamp=None):
  now=time.time() if timestamp is None else float(timestamp);c=self.config
  raw=voxel_cluster_lidar(lidar_points,c.get("voxel_size",.8),c.get("cluster_min_points",6),c.get("lidar_min_z",-7.5),c.get("lidar_max_z",2.),c.get("max_range",70.),c.get("vehicle_min_length",.6),c.get("vehicle_max_length",8.),c.get("vehicle_min_width",.4),c.get("vehicle_max_width",4.),c.get("vehicle_min_height",.25),c.get("vehicle_max_height",4.),c.get("max_objects",80))
  clusters=merge_lidar_clusters(raw,c.get("cluster_merge_gap",1.8),c.get("merged_vehicle_max_length",14.),c.get("merged_vehicle_max_width",4.5),c.get("merged_vehicle_max_height",4.5)) if c.get("cluster_merge_enabled",True) else raw
  assoc=associate_radar(clusters,radar_detections,c.get("radar_match_distance",3.));roi=[]
  for i in assoc:
   r=i.get("radar");src=["lidar"] if r is None else ["lidar","radar"];conf=.72 if r is None else .90;rs=None if r is None else float(r.get("velocity",0));wx,wy,wz=self._to_world(i["x"],i["y"],i["z"]);e=i.get("extent",[0,0,0])
   if self.candidate_validator:
    try:
     if not self.candidate_validator(wx,wy,wz,e):continue
    except Exception:continue
   roi.append({"x":wx,"y":wy,"z":wz,"radar_speed":rs,"confidence":conf,"sources":src,"point_count":i.get("point_count",0),"extent":e})
  dyn=self.background.update_and_filter(roi,now);self.last_dynamic_candidates=[dict(x) for x in dyn];tracked=self.tracker.update(dyn,now);self.last_tracked_candidates=[dict(x) for x in tracked]
  objs=[DetectedObject(i["id"],i["x"],i["y"],i["z"],vx=i["vx"],vy=i["vy"],object_type="unknown",confidence=i["confidence"],sources=i["sources"]) for i in tracked]
  self.last_stats={"lidar_points":0 if lidar_points is None else len(lidar_points),"raw_lidar_clusters":len(raw),"lidar_clusters":len(clusters),"roi_candidates":len(roi),"background_candidates":len(dyn),"background_ready":self.background.ready,"background_remaining":self.background.remaining_seconds(now),"background_cells":len(self.background.static_cells),"radar_detections":0 if not radar_detections else len(radar_detections),"tracked_objects":len(objs)}
  return ObjectList(self.station_id,objs,timestamp=now)
