from __future__ import print_function
import math,time
from collections import deque

class NearestTracker(object):
    def __init__(self,max_distance=4.0,max_age=1.5,max_speed=20.0,velocity_alpha=.35,extent_alpha=.25,extent_shrink_alpha=.05,extent_lock_hits=5,radar_velocity_alpha=.35,velocity_window=5,position_alpha=.45,stationary_speed=.35,coast_frames=0,coast_confidence_decay=.88,adaptive_coast_enabled=False,coast_young_frames=1,coast_stable_frames=3,coast_far_frames=4,coast_stable_hits=3,coast_far_range=50.0,coast_low_score=0.55,coast_edge_ratio=0.90):
        self.max_distance=float(max_distance);self.max_age=float(max_age);self.max_speed=float(max_speed);self.velocity_alpha=float(velocity_alpha)
        self.extent_alpha=float(extent_alpha);self.extent_shrink_alpha=float(extent_shrink_alpha);self.extent_lock_hits=int(extent_lock_hits);self.radar_velocity_alpha=float(radar_velocity_alpha)
        self.velocity_window=max(2,int(velocity_window));self.position_alpha=float(position_alpha);self.stationary_speed=float(stationary_speed)
        self.coast_frames=max(0,int(coast_frames));self.coast_confidence_decay=float(coast_confidence_decay);self.adaptive_coast_enabled=bool(adaptive_coast_enabled)
        self.coast_young_frames=max(0,int(coast_young_frames));self.coast_stable_frames=max(0,int(coast_stable_frames));self.coast_far_frames=max(0,int(coast_far_frames));self.coast_stable_hits=max(1,int(coast_stable_hits));self.coast_far_range=float(coast_far_range);self.coast_low_score=float(coast_low_score);self.coast_edge_ratio=float(coast_edge_ratio)
        self._tracks={};self._next_id=1;self.last_stats={"new":0,"update":0,"coast":0,"drop":0,"suppress":0}
    def _clamp_velocity(self,vx,vy):
        s=math.hypot(vx,vy)
        if s<=self.max_speed or s<1e-6:return vx,vy
        k=self.max_speed/s;return vx*k,vy*k
    def _smooth_extent(self,old,cur,hits):
        prev=old.get("extent",cur);out=[]
        for i in range(3):
            grow=float(cur[i])>=float(prev[i]);a=self.extent_alpha if grow or hits<self.extent_lock_hits else self.extent_shrink_alpha
            out.append((1.0-a)*float(prev[i])+a*float(cur[i]))
        return out
    def _history_velocity(self,hist):
        if len(hist)<2:return 0.0,0.0
        first=hist[0];last=hist[-1];dt=max(1e-3,last[0]-first[0])
        return self._clamp_velocity((last[1]-first[1])/dt,(last[2]-first[2])/dt)
    def _apply_radar_radial(self,vx,vy,det):
        rv=det.get("radar_radial_velocity");lx=det.get("radar_los_x");ly=det.get("radar_los_y")
        if rv is None or lx is None or ly is None:return vx,vy
        norm=math.hypot(float(lx),float(ly))
        if norm<1e-6:return vx,vy
        lx=float(lx)/norm;ly=float(ly)/norm;measured=float(rv);pred=vx*lx+vy*ly;err=measured-pred;a=self.radar_velocity_alpha
        return self._clamp_velocity(vx+a*err*lx,vy+a*err*ly)
    def _track_quality(self,t):
        d=t.get("last_det",{}) or {};score=float(d.get("candidate_score",1.0));bypass=bool(d.get("candidate_score_bypass",False));details=d.get("roi_details",{}) or {}
        lateral=details.get("lateral");allowed=details.get("allowed_lateral");edge_ratio=0.0
        if lateral is not None and allowed not in (None,0):edge_ratio=float(lateral)/max(.01,float(allowed))
        return score,bypass,edge_ratio,bool(details.get("geometry_rescued",False)),float(d.get("sensor_range",0.0))
    def _allowed_coast(self,t):
        if not self.adaptive_coast_enabled:return self.coast_frames
        hits=int(t.get("hits",1));score,bypass,edge_ratio,rescued,rng=self._track_quality(t)
        if hits<2:return 0
        allowed=self.coast_young_frames if hits<self.coast_stable_hits else self.coast_stable_frames
        if hits>=self.coast_stable_hits and rng>=self.coast_far_range:allowed=self.coast_far_frames
        if (not bypass and score<self.coast_low_score) or edge_ratio>=self.coast_edge_ratio or rescued:allowed=min(allowed,1)
        return max(0,int(allowed))
    def _coast_item(self,tid,t,now):
        dt=max(0.0,now-float(t.get("timestamp",now)));item=dict(t.get("last_det",{}));allowed=self._allowed_coast(t)
        item.update({"id":tid,"x":float(t["x"])+float(t["vx"])*dt,"y":float(t["y"])+float(t["vy"])*dt,"z":float(t["z"]),"raw_x":float(t["x"]),"raw_y":float(t["y"]),"raw_z":float(t["z"]),"raw_vx":float(t["vx"]),"raw_vy":float(t["vy"]),"vx":float(t["vx"]),"vy":float(t["vy"]),"extent":list(t.get("extent",[0,0,0])),"track_hits":int(t.get("hits",1)),"track_state":"coast","coast_frames":int(t.get("misses",0)),"coast_allowed":allowed})
        item["confidence"]=float(item.get("confidence",.72))*(self.coast_confidence_decay**max(1,int(t.get("misses",1))))
        item["sources"]=list(item.get("sources",["lidar"]))
        return item
    def update(self,detections,timestamp=None):
        now=time.time() if timestamp is None else float(timestamp);unmatched=set(self._tracks);results=[];pending=[];stats={"new":0,"update":0,"coast":0,"drop":0,"suppress":0}
        for di,det in enumerate(detections):
            for tid in unmatched:
                t=self._tracks[tid];dt=max(1e-3,now-t["timestamp"]);px=t["x"]+t["vx"]*dt;py=t["y"]+t["vy"]*dt;d=math.hypot(det["x"]-px,det["y"]-py)
                if d<self.max_distance:pending.append((d,di,tid))
        pending.sort();assigned_det={};assigned_track=set()
        for d,di,tid in pending:
            if di not in assigned_det and tid not in assigned_track:assigned_det[di]=tid;assigned_track.add(tid)
        for di,det in enumerate(detections):
            tid=assigned_det.get(di);old=self._tracks.get(tid) if tid else None;raw_x=float(det["x"]);raw_y=float(det["y"]);raw_z=float(det.get("z",0))
            if old is None:
                tid="vehicle_%03d"%self._next_id;self._next_id+=1;x=raw_x;y=raw_y;z=raw_z;vx=0.0;vy=0.0;raw_vx=0.0;raw_vy=0.0;extent=list(det.get("extent",[0,0,0]));hits=1;hist=deque(maxlen=self.velocity_window);hist.append((now,raw_x,raw_y));stats["new"]+=1
            else:
                pa=self.position_alpha;x=(1-pa)*old["x"]+pa*raw_x;y=(1-pa)*old["y"]+pa*raw_y;z=(1-pa)*old["z"]+pa*raw_z
                hist=deque(old.get("history",[]),maxlen=self.velocity_window);hist.append((now,raw_x,raw_y));raw_vx,raw_vy=self._history_velocity(hist)
                a=self.velocity_alpha;vx=(1-a)*old["vx"]+a*raw_vx;vy=(1-a)*old["vy"]+a*raw_vy;vx,vy=self._clamp_velocity(vx,vy)
                hits=int(old.get("hits",1))+1;extent=self._smooth_extent(old,list(det.get("extent",old.get("extent",[0,0,0]))),hits);stats["update"]+=1
            pre_radar_vx,pre_radar_vy=vx,vy;vx,vy=self._apply_radar_radial(vx,vy,det)
            if det.get("radar_radial_velocity") is None and math.hypot(raw_vx,raw_vy)<self.stationary_speed and math.hypot(vx,vy)<self.stationary_speed*1.5:vx=0.0;vy=0.0
            last_det=dict(det);last_det.pop("id",None)
            self._tracks[tid]={"x":x,"y":y,"z":z,"vx":vx,"vy":vy,"extent":extent,"hits":hits,"timestamp":now,"history":hist,"misses":0,"last_det":last_det}
            item=dict(det);item.update({"id":tid,"x":x,"y":y,"z":z,"raw_x":raw_x,"raw_y":raw_y,"raw_z":raw_z,"raw_vx":raw_vx,"raw_vy":raw_vy,"track_vx_before_radar":pre_radar_vx,"track_vy_before_radar":pre_radar_vy,"vx":vx,"vy":vy,"extent":extent,"track_hits":hits,"track_state":"new" if old is None else "update","coast_frames":0,"coast_allowed":self._allowed_coast(self._tracks[tid])});results.append(item)
        for tid in list(unmatched-assigned_track):
            t=self._tracks.get(tid)
            if t is None:continue
            t["misses"]=int(t.get("misses",0))+1;allowed=self._allowed_coast(t)
            if now-float(t.get("timestamp",now))<=self.max_age and t["misses"]<=allowed:
                results.append(self._coast_item(tid,t,now));stats["coast"]+=1
            elif now-float(t.get("timestamp",now))<=self.max_age:stats["suppress"]+=1
        stale=[tid for tid,t in self._tracks.items() if now-t["timestamp"]>self.max_age]
        for tid in stale:del self._tracks[tid];stats["drop"]+=1
        self.last_stats=stats
        return results
