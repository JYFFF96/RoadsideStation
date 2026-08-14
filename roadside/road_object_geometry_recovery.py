from __future__ import print_function

from collections import defaultdict, deque
import math

import numpy as np


class RoadObjectGeometryRecovery(object):
    """Recover compact low road objects removed by the normal ground cut.

    The channel consumes LiDAR points and scalar geometry only. It never reads
    CARLA actors or evaluation truth, so the same logic can be ported to the
    real-device implementation.
    """

    def __init__(self):
        self.history = []
        self.last_frame = None
        self.last_output = []
        self.last_input_points = np.empty((0,3),dtype=np.float32)
        self.last_stage_outputs = self._empty_stages()
        self.last_stats = self._empty_stats()

    @staticmethod
    def _empty_stats():
        return {"input_points":0,"components":0,"shape_pass":0,"pending":0,
                "temporal_pass":0,"dedupe":0,"cap_reject":0,"built":0}

    @staticmethod
    def _empty_stages():
        return {"component":[],"shape":[],"temporal":[],"dedupe_pass":[],"output":[]}

    def _clear_diagnostics(self):
        self.last_input_points=np.empty((0,3),dtype=np.float32)
        self.last_stage_outputs=self._empty_stages()

    @staticmethod
    def _distance(a, b):
        return math.hypot(float(a["x"])-float(b["x"]),
                          float(a["y"])-float(b["y"]))

    def update(self, points, existing_geometry, ground_cut_local, config=None, frame_id=None):
        c=config or {};stats=self._empty_stats()
        if frame_id is not None and frame_id==self.last_frame:
            return [dict(x) for x in self.last_output]
        self.last_frame=frame_id
        if not c.get("road_object_recovery_enabled",False) or points is None or ground_cut_local is None:
            self.history=[];self.last_output=[];self._clear_diagnostics();self.last_stats=stats;return []
        pts=np.asarray(points,dtype=np.float32)
        if pts.size==0:
            self.history=[];self.last_output=[];self._clear_diagnostics();self.last_stats=stats;return []
        min_range=float(c.get("road_object_recovery_min_range",5.0))
        max_range=float(c.get("road_object_recovery_max_range",45.0))
        low=float(ground_cut_local)-float(c.get("ground_clearance",.30))+float(c.get("road_object_recovery_ground_clearance",.05))
        high=low+float(c.get("road_object_recovery_max_height_above_ground",1.50))
        ranges=np.sqrt(pts[:,0]*pts[:,0]+pts[:,1]*pts[:,1])
        mask=(ranges>=min_range)&(ranges<=max_range)&(pts[:,2]>low)&(pts[:,2]<=high)
        pts=pts[mask];self.last_input_points=pts.copy();stats["input_points"]=len(pts)
        if len(pts)==0:
            self.history=[];self.last_output=[];self.last_stage_outputs=self._empty_stages();self.last_stats=stats;return []

        cell=max(.10,float(c.get("road_object_recovery_cell_size",.30)))
        neighbor=max(1,int(c.get("road_object_recovery_neighbor_cells",1)))
        keys=np.floor(pts[:,:2]/cell).astype(np.int32);buckets=defaultdict(list)
        for index,key in enumerate(keys):buckets[(int(key[0]),int(key[1]))].append(index)
        occupied=set(buckets);visited=set();components=[]
        offsets=[(x,y) for x in range(-neighbor,neighbor+1) for y in range(-neighbor,neighbor+1)]
        for start in occupied:
            if start in visited:continue
            visited.add(start);queue=deque([start]);indices=[]
            while queue:
                current=queue.popleft();indices.extend(buckets[current]);cx,cy=current
                for dx,dy in offsets:
                    nxt=(cx+dx,cy+dy)
                    if nxt in occupied and nxt not in visited:visited.add(nxt);queue.append(nxt)
            components.append(indices)
        stats["components"]=len(components);candidates=[];component_items=[]
        min_points=max(2,int(c.get("road_object_recovery_min_points",2)))
        max_points=max(min_points,int(c.get("road_object_recovery_max_points",24)))
        for indices in components:
            cp=pts[indices]
            pmin=cp.min(axis=0);pmax=cp.max(axis=0);extent=pmax-pmin
            ex,ey,ez=map(float,extent);long_side=max(ex,ey);short_side=min(ex,ey)
            center=cp.mean(axis=0)
            component_items.append({"x":float(center[0]),"y":float(center[1]),"z":float(center[2]),
                                    "point_count":len(cp),"current_point_count":len(cp),
                                    "extent":[ex,ey,ez],"cluster_mode":"road_object_component"})
            if len(cp)<min_points or len(cp)>max_points:continue
            if not (float(c.get("road_object_recovery_min_length",.08))<=long_side<=float(c.get("road_object_recovery_max_length",1.50))):continue
            if not (float(c.get("road_object_recovery_min_width",.02))<=short_side<=float(c.get("road_object_recovery_max_width",1.20))):continue
            if not (float(c.get("road_object_recovery_min_height",.02))<=ez<=float(c.get("road_object_recovery_max_height",1.40))):continue
            candidates.append({"x":float(center[0]),"y":float(center[1]),"z":float(center[2]),
                               "point_count":len(cp),"current_point_count":len(cp),
                               "extent":[ex,ey,ez],"cluster_mode":"road_object_low",
                               "scale_votes":1,"scale_modes":["road_object_low"],
                               "road_object_recovered":True})
        stats["shape_pass"]=len(candidates)

        required=max(1,int(c.get("road_object_recovery_temporal_frames",2)))
        gate=float(c.get("road_object_recovery_temporal_gate",.80));used=set();next_history=[];stable=[]
        for item in candidates:
            best=None
            for index,old in enumerate(self.history):
                if index in used:continue
                distance=self._distance(item,old)
                if distance<=gate and (best is None or distance<best[0]):best=(distance,index,old)
            hits=1
            if best is not None:
                used.add(best[1]);hits=int(best[2].get("hits",1))+1
            entry=dict(item);entry["hits"]=hits;next_history.append(entry)
            if hits>=required:
                item["road_object_temporal_hits"]=hits;stable.append(item)
            else:stats["pending"]+=1
        self.history=next_history;stats["temporal_pass"]=len(stable)

        dedupe=float(c.get("road_object_recovery_dedupe_distance",1.0));dedupe_pass=[]
        occupied=list(existing_geometry or [])
        for item in sorted(stable,key=lambda x:x.get("point_count",0),reverse=True):
            if any(self._distance(item,old)<=dedupe for old in occupied):
                stats["dedupe"]+=1;continue
            dedupe_pass.append(item);occupied.append(item)
        limit=max(0,int(c.get("road_object_recovery_max_candidates",12)))
        out=dedupe_pass[:limit];stats["cap_reject"]=max(0,len(dedupe_pass)-len(out))
        self.last_stage_outputs={"component":[dict(x) for x in component_items],
                                 "shape":[dict(x) for x in candidates],
                                 "temporal":[dict(x) for x in stable],
                                 "dedupe_pass":[dict(x) for x in dedupe_pass],
                                 "output":[dict(x) for x in out]}
        stats["built"]=len(out);self.last_output=[dict(x) for x in out];self.last_stats=stats;return out
