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
        self.adaptive_point_history = []
        self.last_frame = None
        self.last_output = []
        self.last_input_points = np.empty((0,3),dtype=np.float32)
        self.last_stage_outputs = self._empty_stages()
        self.last_stats = self._empty_stats()

    @staticmethod
    def _empty_stats():
        return {"input_points":0,"components":0,"shape_pass":0,"pending":0,
                "temporal_pass":0,"dedupe":0,"cap_reject":0,"built":0,
                "balanced_built":0,"balanced_band_counts":{},
                "adaptive_history_frames":0,"adaptive_points":0,
                "adaptive_components":0,"adaptive_shape_pass":0,
                "adaptive_temporal_pass":0,"adaptive_dedupe":0,
                "adaptive_built":0,"adaptive_band_counts":{},
                "adaptive_ranked_built":0,"adaptive_ranked_band_counts":{},
                "adaptive_stratified_built":0,"adaptive_stratified_band_counts":{},
                "adaptive_stratified_height_counts":{},
                "adaptive_hybrid_built":0,"adaptive_hybrid_band_counts":{},
                "adaptive_hybrid_source_counts":{},
                "adaptive_hybrid_gate_kept":0,"adaptive_hybrid_gate_rejected":0,
                "adaptive_hybrid_gate_reasons":{},
                "adaptive_hybrid_rescue_kept":0,"adaptive_hybrid_rescued":0,
                "adaptive_hybrid_rescue_sources":{},
                "adaptive_hybrid_geometry_gate_kept":0,
                "adaptive_hybrid_geometry_gate_rejected":0,
                "adaptive_hybrid_geometry_gate_reasons":{},
                "selected_output_built":0,"selected_output_policy":"disabled",
                "selected_output_enforcing":False,"active_output_built":0,
                "active_output_policy":"baseline"}

    @staticmethod
    def _empty_stages():
        return {"component":[],"shape":[],"temporal":[],"dedupe_pass":[],
                "output":[],"balanced_output":[],"adaptive_component":[],
                "adaptive_shape":[],"adaptive_temporal":[],
                "adaptive_dedupe_pass":[],"adaptive_output":[],
                "adaptive_ranked_output":[],"adaptive_stratified_output":[],
                "adaptive_hybrid_output":[],"adaptive_hybrid_gated_output":[],
                "adaptive_hybrid_rescued_output":[],
                "adaptive_hybrid_geometry_gated_output":[],"selected_output":[]}


    def _clear_diagnostics(self):
        self.last_input_points=np.empty((0,3),dtype=np.float32)
        self.last_stage_outputs=self._empty_stages()

    @staticmethod
    def _distance(a, b):
        return math.hypot(float(a["x"])-float(b["x"]),
                          float(a["y"])-float(b["y"]))

    @staticmethod
    def _sensor_range(item):
        return math.hypot(float(item.get("x",0.0)),float(item.get("y",0.0)))

    def _balanced_cap(self, items, limit, config):
        if not config.get("road_object_recovery_balanced_cap_shadow",False) or limit<=0:return []
        bands=config.get("road_object_recovery_balanced_bands",[]) or []
        parsed=[];lower=float(config.get("road_object_recovery_min_range",5.0))
        for value in bands:
            try:upper=float(value.get("max_range"));quota=max(0,int(value.get("quota",0)))
            except (AttributeError,TypeError,ValueError):continue
            if upper<=lower:continue
            parsed.append((lower,upper,quota));lower=upper
        selected=[];selected_ids=set()
        for index,(low,high,quota) in enumerate(parsed):
            candidates=[item for item in items if id(item) not in selected_ids and
                        low<=self._sensor_range(item)<high]
            for item in candidates[:quota]:
                if len(selected)>=limit:break
                selected.append(item);selected_ids.add(id(item))
        for item in items:
            if len(selected)>=limit:break
            if id(item) not in selected_ids:selected.append(item);selected_ids.add(id(item))
        return selected

    def _adaptive_bands(self, config):
        if not config.get("road_object_recovery_adaptive_temporal_shadow",False):return []
        result=[]
        for value in config.get("road_object_recovery_adaptive_temporal_bands",[]) or []:
            try:
                low=float(value.get("min_range"));high=float(value.get("max_range"))
                frames=max(1,int(value.get("history_frames",1)))
                required=max(1,min(frames,int(value.get("min_support_frames",2))))
                max_points=max(2,int(value.get("max_points",48)))
            except (AttributeError,TypeError,ValueError):continue
            if high>low:result.append((low,high,frames,required,max_points))
        return sorted(result,key=lambda item:item[0])

    @staticmethod
    def _adaptive_rank_score(item, config):
        try:height=float((item.get("extent",[]) or [0.0,0.0,0.0])[2])
        except (IndexError,TypeError,ValueError):height=9.0
        points=float(item.get("point_count",0) or 0);current=float(item.get("current_point_count",0) or 0)
        frames=float(item.get("support_frames",0) or 0);target=float(config.get("road_object_adaptive_rank_target_points",5.0))
        if height<=float(config.get("road_object_adaptive_rank_low_height",.30)):height_score=1.0
        elif height<=float(config.get("road_object_adaptive_rank_mid_height",.75)):height_score=.55
        else:height_score=.15
        point_score=max(0.0,1.0-abs(points-target)/max(1.0,target+3.0))
        current_score=max(0.0,1.0-abs(current-2.0)/4.0)
        frame_score=min(1.0,frames/max(1.0,float(config.get("road_object_adaptive_rank_full_support_frames",4))))
        return (.55*height_score+.20*point_score+.15*current_score+.10*frame_score)

    def _adaptive_ranked_cap(self, items, limit, config):
        if not config.get("road_object_recovery_adaptive_ranking_shadow",False):return []
        for item in items:item["adaptive_rank_score"]=self._adaptive_rank_score(item,config)
        ranked=sorted(items,key=lambda value:(value.get("adaptive_rank_score",0.0),
                      value.get("support_frames",0),-abs(float(value.get("point_count",0))-5.0)),reverse=True)
        return self._balanced_cap(ranked,limit,config) or ranked[:limit]

    @staticmethod
    def _candidate_height(item):
        try:return float((item.get("extent",[]) or [0.0,0.0,0.0])[2])
        except (IndexError,TypeError,ValueError):return 9.0

    def _adaptive_stratified_cap(self, items, limit, config):
        """Reserve low/elevated geometry slots without using object labels."""
        if not config.get("road_object_recovery_adaptive_stratified_shadow",False) or limit<=0:return []
        for item in items:item["adaptive_rank_score"]=self._adaptive_rank_score(item,config)
        ranked=sorted(items,key=lambda value:(value.get("adaptive_rank_score",0.0),
                      value.get("support_frames",0),-abs(float(value.get("point_count",0))-5.0)),reverse=True)
        threshold=float(config.get("road_object_adaptive_stratified_height",.30))
        elevated_quota=max(0,int(config.get("road_object_adaptive_stratified_elevated_quota",2)))
        bands=config.get("road_object_recovery_balanced_bands",[]) or []
        parsed=[];lower=float(config.get("road_object_recovery_min_range",5.0))
        for value in bands:
            try:upper=float(value.get("max_range"));quota=max(0,int(value.get("quota",0)))
            except (AttributeError,TypeError,ValueError):continue
            if upper<=lower:continue
            parsed.append((lower,upper,quota));lower=upper
        selected=[];selected_ids=set()
        for low,high,quota in parsed:
            candidates=[item for item in ranked if low<=self._sensor_range(item)<high]
            elevated=[item for item in candidates if self._candidate_height(item)>threshold]
            low_items=[item for item in candidates if self._candidate_height(item)<=threshold]
            low_quota=max(0,quota-min(quota,elevated_quota))
            band_selected=0
            for pool,count in ((low_items,low_quota),(elevated,min(quota,elevated_quota)),(candidates,quota)):
                for item in pool:
                    if len(selected)>=limit or count<=0 or band_selected>=quota:break
                    if id(item) in selected_ids:continue
                    selected.append(item);selected_ids.add(id(item));count-=1;band_selected+=1
        for item in ranked:
            if len(selected)>=limit:break
            if id(item) not in selected_ids:selected.append(item);selected_ids.add(id(item))
        return selected

    def _adaptive_hybrid_cap(self, baseline_items, adaptive_items, limit, config):
        """Combine near baseline stability with far Adaptive ranking in Shadow."""
        if not config.get("road_object_recovery_adaptive_hybrid_shadow",False) or limit<=0:return []
        split=float(config.get("road_object_adaptive_hybrid_split_range",25.0))
        near=sorted([item for item in baseline_items if self._sensor_range(item)<split],
                    key=lambda value:value.get("point_count",0),reverse=True)
        for item in adaptive_items:item["adaptive_rank_score"]=self._adaptive_rank_score(item,config)
        far=sorted([item for item in adaptive_items if self._sensor_range(item)>=split],
                   key=lambda value:(value.get("adaptive_rank_score",0.0),
                   value.get("support_frames",0)),reverse=True)
        fallback=sorted([item for item in baseline_items if self._sensor_range(item)>=split],
                        key=lambda value:value.get("point_count",0),reverse=True)
        dedupe=float(config.get("road_object_recovery_dedupe_distance",1.0));pool=[]
        for source,items in (("near_baseline",near),("far_ranked",far),("far_baseline_fallback",fallback)):
            for item in items:
                if any(self._distance(item,old)<=dedupe for old in pool):continue
                item["adaptive_hybrid_source"]=source;pool.append(item)
        return self._balanced_cap(pool,limit,config) or pool[:limit]

    def _adaptive_hybrid_gate(self, items, config):
        """Source-aware scalar gate. Labels and simulator truth are never used."""
        if not config.get("road_object_recovery_adaptive_hybrid_gate_shadow",False):return [],{}
        kept=[];reasons={}
        for original in items:
            item=original;source=str(item.get("adaptive_hybrid_source","unknown"));failed=[]
            points=float(item.get("point_count",0) or 0);height=self._candidate_height(item)
            extent=list(item.get("extent",[]) or [])
            try:short_side=min(float(extent[0]),float(extent[1]))
            except (IndexError,TypeError,ValueError):short_side=0.0
            if source=="near_baseline":
                if points<float(config.get("road_object_hybrid_gate_near_min_points",8)):failed.append("near_points")
                compact=height<=float(config.get("road_object_hybrid_gate_near_low_height",.45))
                wide=short_side>=float(config.get("road_object_hybrid_gate_near_min_short_side",.50))
                if not (compact or wide):failed.append("near_shape")
            else:
                frames=float(item.get("support_frames",0) or 0)
                current=float(item.get("current_point_count",0) or 0);distance=self._sensor_range(item)
                stable=frames>=float(config.get("road_object_hybrid_gate_far_stable_frames",4))
                close=(distance<=float(config.get("road_object_hybrid_gate_far_close_range",32.0)) and
                       points>=float(config.get("road_object_hybrid_gate_far_close_min_points",5)) and
                       current>=float(config.get("road_object_hybrid_gate_far_close_current_points",2)))
                if not (stable or close):failed.append("far_support")
            item["adaptive_hybrid_gate_keep"]=not failed
            item["adaptive_hybrid_gate_failures"]=list(failed)
            if failed:
                for reason in failed:reasons[reason]=reasons.get(reason,0)+1
            else:kept.append(item)
        return kept,reasons

    def _adaptive_hybrid_temporal_rescue(self, items, config):
        """Recover persistent sparse candidates rejected by the strict Shadow gate."""
        if not config.get("road_object_recovery_adaptive_hybrid_rescue_shadow",False):return [],{}
        output=[];sources={}
        for item in items:
            if item.get("adaptive_hybrid_gate_keep",False):output.append(item);continue
            source=str(item.get("adaptive_hybrid_source","unknown"));rescue=False
            points=float(item.get("point_count",0) or 0)
            if source=="near_baseline":
                hits=float(item.get("road_object_temporal_hits",0) or 0)
                rescue=(hits>=float(config.get("road_object_hybrid_rescue_near_temporal_hits",3)) and
                        points>=float(config.get("road_object_hybrid_rescue_near_min_points",5)))
            else:
                frames=float(item.get("support_frames",0) or 0)
                current=float(item.get("current_point_count",0) or 0);distance=self._sensor_range(item)
                rescue=(distance<=float(config.get("road_object_hybrid_rescue_far_max_range",32.0)) and
                        frames>=float(config.get("road_object_hybrid_rescue_far_support_frames",3)) and
                        points>=float(config.get("road_object_hybrid_rescue_far_min_points",4)) and
                        current>=float(config.get("road_object_hybrid_rescue_far_current_points",1)))
            if rescue:
                item["adaptive_hybrid_temporal_rescue"]=True;output.append(item)
                sources[source]=sources.get(source,0)+1
        return output,sources

    def _adaptive_hybrid_rescue_geometry_gate(self, items, config):
        """Filter only temporal-rescue additions in a second Shadow branch."""
        if not config.get("road_object_recovery_adaptive_hybrid_geometry_gate_shadow",False):return [],{}
        output=[];reasons={}
        for item in items:
            if not item.get("adaptive_hybrid_temporal_rescue",False):
                output.append(item);continue
            source=str(item.get("adaptive_hybrid_source","unknown"));keep=False;reason="unknown_source"
            if source=="near_baseline":
                extent=list(item.get("extent",[]) or [])
                try:area=max(0.0,float(extent[0]))*max(0.0,float(extent[1]))
                except (IndexError,TypeError,ValueError):area=0.0
                keep=area>=float(config.get("road_object_hybrid_geometry_gate_near_min_area",.02))
                reason="near_area"
            else:
                keep=self._sensor_range(item)<=float(config.get("road_object_hybrid_geometry_gate_far_max_range",32.0))
                reason="far_range"
            item["adaptive_hybrid_geometry_gate_keep"]=keep
            item["adaptive_hybrid_geometry_gate_reason"]=(None if keep else reason)
            if keep:output.append(item)
            else:reasons[reason]=reasons.get(reason,0)+1
        return output,reasons

    @staticmethod
    def _selected_shadow_output(stages, config):
        """Resolve a named future-output policy while production stays Shadow."""
        if not config.get("road_object_recovery_selected_output_shadow",False):return [],"disabled"
        policy=str(config.get("road_object_recovery_selected_output_policy",
                              "adaptive_hybrid_geometry_gated"))
        choices={"baseline":stages.get("baseline",[]),
                 "adaptive_hybrid_gated":stages.get("adaptive_hybrid_gated",[]),
                 "adaptive_hybrid_rescued":stages.get("adaptive_hybrid_rescued",[]),
                 "adaptive_hybrid_geometry_gated":stages.get("adaptive_hybrid_geometry_gated",[])}
        if policy not in choices:policy="baseline"
        return list(choices[policy]),policy

    @staticmethod
    def _active_output(baseline, selected, selected_policy, config):
        """Choose the recovery output without silently bypassing the baseline."""
        enforcing=bool(config.get("road_object_recovery_selected_output_enforcing",False))
        if enforcing and selected_policy!="disabled":
            output=[]
            for source in selected:
                item=dict(source)
                item["road_object_selected_enforced"]=True
                item["road_object_selected_policy"]=selected_policy
                output.append(item)
            return output,selected_policy,True
        return list(baseline),"baseline",False

    @staticmethod
    def _voxelized_history_points(frames, low, high, voxel):
        """Return spatially unique points and the source-frame set per voxel."""
        cells={};voxel=max(.02,float(voxel))
        for source,frame in enumerate(frames):
            if frame is None or len(frame)==0:continue
            ranges=np.sqrt(frame[:,0]*frame[:,0]+frame[:,1]*frame[:,1])
            for point in frame[(ranges>=low)&(ranges<high)]:
                key=tuple(np.floor(point/voxel).astype(np.int32).tolist())
                entry=cells.get(key)
                if entry is None:cells[key]=[point.copy(),set([source])]
                else:
                    entry[1].add(source)
                    if source==0:entry[0]=point.copy()
        points=np.asarray([entry[0] for entry in cells.values()],dtype=np.float32)
        return points,[entry[1] for entry in cells.values()]

    @staticmethod
    def _component_indices(points, cell, neighbor):
        if points is None or len(points)==0:return []
        keys=np.floor(points[:,:2]/cell).astype(np.int32);buckets=defaultdict(list)
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
        return components

    def _adaptive_temporal_candidates(self, pts, existing_geometry, config, stats):
        bands=self._adaptive_bands(config)
        empty={"component":[],"shape":[],"temporal":[],"dedupe_pass":[],
               "output":[],"ranked_output":[],"stratified_output":[]}
        if not bands:
            self.adaptive_point_history=[]
            return empty
        max_frames=max(item[2] for item in bands)
        keep=max(0,max_frames-1)
        recent=self.adaptive_point_history[-keep:] if keep else []
        frames=[pts]+list(reversed(recent))
        stats["adaptive_history_frames"]=len(frames)
        voxel=float(config.get("road_object_recovery_adaptive_voxel_size",.08))
        cell=max(.10,float(config.get("road_object_recovery_cell_size",.30)))
        neighbor=max(1,int(config.get("road_object_recovery_neighbor_cells",1)))
        min_points=max(2,int(config.get("road_object_recovery_min_points",2)))
        components=[];shapes=[];stable=[]
        for low,high,frame_count,required,max_points in bands:
            ap,sources=self._voxelized_history_points(frames[:frame_count],low,high,voxel)
            stats["adaptive_points"]+=len(ap)
            for indices in self._component_indices(ap,cell,neighbor):
                cp=ap[indices];pmin=cp.min(axis=0);pmax=cp.max(axis=0);extent=pmax-pmin
                ex,ey,ez=map(float,extent);long_side=max(ex,ey);short_side=min(ex,ey);center=cp.mean(axis=0)
                source_frames=set()
                for index in indices:source_frames.update(sources[index])
                current_points=sum(1 for index in indices if 0 in sources[index])
                base={"x":float(center[0]),"y":float(center[1]),"z":float(center[2]),
                      "point_count":len(cp),"current_point_count":current_points,
                      "temporal_point_count":max(0,len(cp)-current_points),
                      "extent":[ex,ey,ez],"support_frames":len(source_frames),
                      "adaptive_band":"%.0f-%.0fm"%(low,high)}
                component=dict(base);component["cluster_mode"]="road_object_adaptive_component";components.append(component)
                if current_points<1 or len(cp)<min_points or len(cp)>max_points:continue
                if not (float(config.get("road_object_recovery_min_length",.08))<=long_side<=float(config.get("road_object_recovery_max_length",1.50))):continue
                if not (float(config.get("road_object_recovery_min_width",.02))<=short_side<=float(config.get("road_object_recovery_max_width",1.20))):continue
                if not (float(config.get("road_object_recovery_min_height",.02))<=ez<=float(config.get("road_object_recovery_max_height",1.40))):continue
                candidate=dict(base);candidate.update({"cluster_mode":"road_object_adaptive_temporal",
                    "scale_votes":1,"scale_modes":["road_object_adaptive_temporal"],
                    "road_object_recovered":True,"range_adaptive_temporal_shadow":True})
                shapes.append(candidate)
                if len(source_frames)>=required:
                    candidate["road_object_temporal_hits"]=len(source_frames);stable.append(candidate)
        stats["adaptive_components"]=len(components);stats["adaptive_shape_pass"]=len(shapes)
        stats["adaptive_temporal_pass"]=len(stable)
        dedupe=float(config.get("road_object_recovery_dedupe_distance",1.0));dedupe_pass=[]
        occupied=list(existing_geometry or [])
        for item in sorted(stable,key=lambda value:value.get("point_count",0),reverse=True):
            if any(self._distance(item,old)<=dedupe for old in occupied):
                stats["adaptive_dedupe"]+=1;continue
            dedupe_pass.append(item);occupied.append(item)
        limit=max(0,int(config.get("road_object_recovery_max_candidates",12)))
        output=self._balanced_cap(dedupe_pass,limit,config) or dedupe_pass[:limit]
        ranked_output=self._adaptive_ranked_cap(dedupe_pass,limit,config)
        stratified_output=self._adaptive_stratified_cap(dedupe_pass,limit,config)
        stats["adaptive_built"]=len(output);counts={}
        for low,high,_,_,_ in bands:
            label="%.0f-%.0fm"%(low,high)
            counts[label]=sum(1 for item in output if low<=self._sensor_range(item)<high)
        stats["adaptive_band_counts"]=counts
        stats["adaptive_ranked_built"]=len(ranked_output);ranked_counts={}
        for low,high,_,_,_ in bands:
            label="%.0f-%.0fm"%(low,high)
            ranked_counts[label]=sum(1 for item in ranked_output if low<=self._sensor_range(item)<high)
        stats["adaptive_ranked_band_counts"]=ranked_counts
        stats["adaptive_stratified_built"]=len(stratified_output);stratified_counts={}
        for low,high,_,_,_ in bands:
            label="%.0f-%.0fm"%(low,high)
            stratified_counts[label]=sum(1 for item in stratified_output if low<=self._sensor_range(item)<high)
        stats["adaptive_stratified_band_counts"]=stratified_counts
        height_threshold=float(config.get("road_object_adaptive_stratified_height",.30))
        stats["adaptive_stratified_height_counts"]={
            "low":sum(1 for item in stratified_output if self._candidate_height(item)<=height_threshold),
            "elevated":sum(1 for item in stratified_output if self._candidate_height(item)>height_threshold)}
        self.adaptive_point_history.append(pts.copy())
        self.adaptive_point_history=self.adaptive_point_history[-keep:] if keep else []
        return {"component":components,"shape":shapes,"temporal":stable,
                "dedupe_pass":dedupe_pass,"output":output,"ranked_output":ranked_output,
                "stratified_output":stratified_output}

    def update(self, points, existing_geometry, ground_cut_local, config=None, frame_id=None):
        c=config or {};stats=self._empty_stats()
        if frame_id is not None and frame_id==self.last_frame:
            return [dict(x) for x in self.last_output]
        self.last_frame=frame_id
        if not c.get("road_object_recovery_enabled",False) or points is None or ground_cut_local is None:
            self.history=[];self.adaptive_point_history=[];self.last_output=[];self._clear_diagnostics();self.last_stats=stats;return []
        pts=np.asarray(points,dtype=np.float32)
        if pts.size==0:
            self.history=[];self.adaptive_point_history=[];self.last_output=[];self._clear_diagnostics();self.last_stats=stats;return []
        min_range=float(c.get("road_object_recovery_min_range",5.0))
        max_range=float(c.get("road_object_recovery_max_range",45.0))
        low=float(ground_cut_local)-float(c.get("ground_clearance",.30))+float(c.get("road_object_recovery_ground_clearance",.05))
        high=low+float(c.get("road_object_recovery_max_height_above_ground",1.50))
        ranges=np.sqrt(pts[:,0]*pts[:,0]+pts[:,1]*pts[:,1])
        mask=(ranges>=min_range)&(ranges<=max_range)&(pts[:,2]>low)&(pts[:,2]<=high)
        pts=pts[mask];self.last_input_points=pts.copy();stats["input_points"]=len(pts)
        if len(pts)==0:
            self.history=[];self.adaptive_point_history=[];self.last_output=[];self.last_stage_outputs=self._empty_stages();self.last_stats=stats;return []

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
        balanced=self._balanced_cap(dedupe_pass,limit,c);stats["balanced_built"]=len(balanced)
        lower=min_range;counts={}
        for value in c.get("road_object_recovery_balanced_bands",[]) or []:
            try:upper=float(value.get("max_range"))
            except (AttributeError,TypeError,ValueError):continue
            label="%.0f-%.0fm"%(lower,upper);counts[label]=sum(1 for item in balanced if lower<=self._sensor_range(item)<upper);lower=upper
        stats["balanced_band_counts"]=counts
        adaptive=self._adaptive_temporal_candidates(pts,existing_geometry,c,stats)
        hybrid=self._adaptive_hybrid_cap(dedupe_pass,adaptive["dedupe_pass"],limit,c)
        hybrid_gated,hybrid_gate_reasons=self._adaptive_hybrid_gate(hybrid,c)
        hybrid_rescued,hybrid_rescue_sources=self._adaptive_hybrid_temporal_rescue(hybrid,c)
        hybrid_geometry_gated,hybrid_geometry_gate_reasons=self._adaptive_hybrid_rescue_geometry_gate(hybrid_rescued,c)
        selected_output,selected_policy=self._selected_shadow_output({
            "baseline":out,"adaptive_hybrid_gated":hybrid_gated,
            "adaptive_hybrid_rescued":hybrid_rescued,
            "adaptive_hybrid_geometry_gated":hybrid_geometry_gated},c)
        active_output,active_policy,selected_enforcing=self._active_output(
            out,selected_output,selected_policy,c)
        stats["adaptive_hybrid_built"]=len(hybrid);hybrid_counts={};hybrid_lower=min_range
        for value in c.get("road_object_recovery_balanced_bands",[]) or []:
            try:hybrid_upper=float(value.get("max_range"))
            except (AttributeError,TypeError,ValueError):continue
            label="%.0f-%.0fm"%(hybrid_lower,hybrid_upper)
            hybrid_counts[label]=sum(1 for item in hybrid if hybrid_lower<=self._sensor_range(item)<hybrid_upper)
            hybrid_lower=hybrid_upper
        stats["adaptive_hybrid_band_counts"]=hybrid_counts;source_counts={}
        for item in hybrid:
            source=str(item.get("adaptive_hybrid_source","unknown"))
            source_counts[source]=source_counts.get(source,0)+1
        stats["adaptive_hybrid_source_counts"]=source_counts
        stats["adaptive_hybrid_gate_kept"]=len(hybrid_gated)
        stats["adaptive_hybrid_gate_rejected"]=max(0,len(hybrid)-len(hybrid_gated))
        stats["adaptive_hybrid_gate_reasons"]=hybrid_gate_reasons
        stats["adaptive_hybrid_rescue_kept"]=len(hybrid_rescued)
        stats["adaptive_hybrid_rescued"]=max(0,len(hybrid_rescued)-len(hybrid_gated))
        stats["adaptive_hybrid_rescue_sources"]=hybrid_rescue_sources
        stats["adaptive_hybrid_geometry_gate_kept"]=len(hybrid_geometry_gated)
        stats["adaptive_hybrid_geometry_gate_rejected"]=max(0,len(hybrid_rescued)-len(hybrid_geometry_gated))
        stats["adaptive_hybrid_geometry_gate_reasons"]=hybrid_geometry_gate_reasons
        stats["selected_output_built"]=len(selected_output);stats["selected_output_policy"]=selected_policy
        stats["selected_output_enforcing"]=selected_enforcing
        stats["active_output_built"]=len(active_output);stats["active_output_policy"]=active_policy
        self.last_stage_outputs={"component":[dict(x) for x in component_items],
                                 "shape":[dict(x) for x in candidates],
                                 "temporal":[dict(x) for x in stable],
                                 "dedupe_pass":[dict(x) for x in dedupe_pass],
                                 "output":[dict(x) for x in out],
                                 "balanced_output":[dict(x) for x in balanced],
                                 "adaptive_component":[dict(x) for x in adaptive["component"]],
                                 "adaptive_shape":[dict(x) for x in adaptive["shape"]],
                                 "adaptive_temporal":[dict(x) for x in adaptive["temporal"]],
                                 "adaptive_dedupe_pass":[dict(x) for x in adaptive["dedupe_pass"]],
                                 "adaptive_output":[dict(x) for x in adaptive["output"]],
                                 "adaptive_ranked_output":[dict(x) for x in adaptive["ranked_output"]],
                                 "adaptive_stratified_output":[dict(x) for x in adaptive["stratified_output"]],
                                 "adaptive_hybrid_output":[dict(x) for x in hybrid],
                                 "adaptive_hybrid_gated_output":[dict(x) for x in hybrid_gated],
                                 "adaptive_hybrid_rescued_output":[dict(x) for x in hybrid_rescued],
                                 "adaptive_hybrid_geometry_gated_output":[dict(x) for x in hybrid_geometry_gated],
                                 "selected_output":[dict(x) for x in selected_output]}
        stats["built"]=len(out);self.last_output=[dict(x) for x in active_output];self.last_stats=stats;return active_output
