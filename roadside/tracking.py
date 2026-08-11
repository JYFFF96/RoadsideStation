from __future__ import print_function
import math,time

class NearestTracker(object):
    def __init__(self,max_distance=4.0,max_age=1.5,max_speed=20.0,velocity_alpha=.35,extent_alpha=.25,extent_shrink_alpha=.05,extent_lock_hits=5):
        self.max_distance=float(max_distance);self.max_age=float(max_age);self.max_speed=float(max_speed);self.velocity_alpha=float(velocity_alpha)
        self.extent_alpha=float(extent_alpha);self.extent_shrink_alpha=float(extent_shrink_alpha);self.extent_lock_hits=int(extent_lock_hits)
        self._tracks={};self._next_id=1
    def _clamp_velocity(self,vx,vy):
        s=math.hypot(vx,vy)
        if s<=self.max_speed or s<1e-6:return vx,vy
        k=self.max_speed/s;return vx*k,vy*k
    def _smooth_extent(self,old,cur,hits):
        prev=old.get("extent",cur);out=[]
        for i in range(3):
            grow=float(cur[i])>=float(prev[i])
            a=self.extent_alpha if grow or hits<self.extent_lock_hits else self.extent_shrink_alpha
            out.append((1.0-a)*float(prev[i])+a*float(cur[i]))
        return out
    def update(self,detections,timestamp=None):
        now=time.time() if timestamp is None else float(timestamp);unmatched=set(self._tracks);results=[];pending=[]
        for di,det in enumerate(detections):
            for tid in unmatched:
                t=self._tracks[tid];dt=max(1e-3,now-t["timestamp"]);px=t["x"]+t["vx"]*dt;py=t["y"]+t["vy"]*dt;d=math.hypot(det["x"]-px,det["y"]-py)
                if d<self.max_distance:pending.append((d,di,tid))
        pending.sort();assigned_det={};assigned_track=set()
        for d,di,tid in pending:
            if di not in assigned_det and tid not in assigned_track:assigned_det[di]=tid;assigned_track.add(tid)
        for di,det in enumerate(detections):
            tid=assigned_det.get(di);old=self._tracks.get(tid) if tid else None
            if old is None:
                tid="vehicle_%03d"%self._next_id;self._next_id+=1;vx=float(det.get("vx",0));vy=float(det.get("vy",0));extent=list(det.get("extent",[0,0,0]));hits=1
            else:
                dt=max(1e-3,now-old["timestamp"]);rvx=(float(det["x"])-old["x"])/dt;rvy=(float(det["y"])-old["y"])/dt;rvx,rvy=self._clamp_velocity(rvx,rvy)
                rs=det.get("radar_speed")
                if rs is not None:
                    ss=math.hypot(rvx,rvy);ra=min(abs(float(rs)),self.max_speed)
                    if ss>.2:rvx*=ra/ss;rvy*=ra/ss
                a=self.velocity_alpha;vx=(1-a)*old["vx"]+a*rvx;vy=(1-a)*old["vy"]+a*rvy;vx,vy=self._clamp_velocity(vx,vy)
                hits=int(old.get("hits",1))+1;extent=self._smooth_extent(old,list(det.get("extent",old.get("extent",[0,0,0]))),hits)
            self._tracks[tid]={"x":float(det["x"]),"y":float(det["y"]),"z":float(det.get("z",0)),"vx":vx,"vy":vy,"extent":extent,"hits":hits,"timestamp":now}
            item=dict(det);item.update({"id":tid,"vx":vx,"vy":vy,"extent":extent,"track_hits":hits});results.append(item)
        stale=[tid for tid,t in self._tracks.items() if now-t["timestamp"]>self.max_age]
        for tid in stale:del self._tracks[tid]
        return results
