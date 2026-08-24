from __future__ import print_function

import json
import math


class V2XEventEngine(object):
    """Build sensor-only V2X warning events from an ObjectList.

    The JSON envelope follows Dachuan event2hmi field names. Topic ownership is
    deliberately left in configuration until the device-side MQTT direction is
    confirmed with the vendor.
    """
    VEHICLE_TYPES=set(("vehicle","car","truck","bus","motorcycle"))
    PEDESTRIAN_TYPES=set(("person","pedestrian"))
    CYCLIST_TYPES=set(("bicycle","cyclist"))

    def __init__(self, station_id, config=None):
        self.station_id=str(station_id);self.config=config or {}
        self.enabled=bool(self.config.get("enabled",False))
        self._stationary_since={};self._last_emitted={};self._event_count=0
        self._vru_presence_hits=0
        self._hlw_presence_hits=0

    def _envelope(self, category, event_sort, description, timestamp,
                  direction=-1, speed=None, extra=None):
        self._event_count+=1
        data={"category":str(category),"event_sort":int(event_sort),
              "event_count":self._event_count,"sid":self.station_id,
              "direc":int(direction),"description":str(description),
              "time":int(timestamp)}
        if speed is not None:data["speed"]=int(round(float(speed)))
        data.update(extra or {})
        return {"type":"event","data":data}

    def _cooldown_ready(self, key, now):
        cooldown=float(self.config.get("cooldown_seconds",5.0))
        return now-float(self._last_emitted.get(key,-1e30))>=cooldown

    def _credible_road_obstacle(self, obj, config):
        if str(obj.object_type).lower()!="unknown_obstacle":return False
        if str(getattr(obj,"track_state","confirmed")).lower()!="confirmed":return False
        if int(getattr(obj,"age",1))<int(config.get("min_track_age",3)):return False
        if float(getattr(obj,"confidence",0.0))<float(config.get("min_confidence",.70)):
            return False
        if math.hypot(float(obj.vx),float(obj.vy))>float(
                config.get("max_stationary_speed_mps",.5)):return False
        size=list(getattr(obj,"size",[]) or [])
        if len(size)<3:return False
        length,width,height=[float(value) for value in size[:3]]
        if not (float(config.get("min_length_m",.20))<=length<=
                float(config.get("max_length_m",2.5))):return False
        if not (float(config.get("min_width_m",.15))<=width<=
                float(config.get("max_width_m",2.0))):return False
        if not (float(config.get("min_height_m",.08))<=height<=
                float(config.get("max_height_m",2.0))):return False
        if length/max(width,.01)>float(config.get("max_aspect_ratio",6.0)):
            return False
        evidence=dict(getattr(obj,"perception_evidence",{}) or {})
        selected=bool(evidence.get("roadObjectSelectedEver",False))
        multisensor=len(set(getattr(obj,"sources",[]) or []))>=2
        quality=float(evidence.get("trackQuality",0.0))>=float(
            config.get("min_track_quality",.75))
        return selected or multisensor or quality

    def update(self, object_list, ego=None):
        if not self.enabled:return []
        ego=ego or {};now=float(object_list.timestamp);events=[];active=set()
        vrucw=self.config.get("vrucw",{}) or {}
        if vrucw.get("enabled",True):
            vru=[]
            for obj in object_list.objects:
                object_type=str(obj.object_type).lower()
                if (object_type in self.PEDESTRIAN_TYPES or
                        object_type in self.CYCLIST_TYPES):vru.append(obj)
            if vru:self._vru_presence_hits+=1
            else:self._vru_presence_hits=0
            required=max(1,int(vrucw.get("required_updates",2)))
            key=("VRUCW","road_presence")
            if (self._vru_presence_hits>=required and
                    self._cooldown_ready(key,now)):
                participant_type=(3 if any(str(obj.object_type).lower() in
                                           self.PEDESTRIAN_TYPES for obj in vru)
                                  else 2)
                events.append(self._envelope(
                    "VRUCW",10,("请注意行人" if participant_type==3
                                 else "请注意非机动车"),now,
                    int(vrucw.get("direction",1)),
                    float(ego.get("speed_kmh",0.0)),
                    {"ptc_type":participant_type,
                     # Dachuan's example spells the table field as spc_type.
                     # Keep both spellings until the RSU firmware is verified.
                     "spc_type":participant_type,
                     "participant_count":len(vru),
                     "trigger_mode":"road_presence"}))
                self._last_emitted[key]=now
        hlw=self.config.get("hlw",{}) or {}
        if hlw.get("enabled",True):
            obstacles=[obj for obj in object_list.objects
                       if self._credible_road_obstacle(obj,hlw)]
            if obstacles:self._hlw_presence_hits+=1
            else:self._hlw_presence_hits=0
            required=max(1,int(hlw.get("required_updates",3)))
            key=("HLW","road_obstacle_presence")
            if (self._hlw_presence_hits>=required and
                    self._cooldown_ready(key,now)):
                events.append(self._envelope(
                    "HLW",8,"道路存在障碍物",now,
                    int(hlw.get("direction",1)),
                    float(ego.get("speed_kmh",0.0)),
                    {"event_type":int(hlw.get("event_type",37)),
                     "obstacle_count":len(obstacles),
                     "trigger_mode":"road_obstacle_presence"}))
                self._last_emitted[key]=now
        avw=self.config.get("avw",{}) or {}
        if avw.get("enabled",True):
            max_speed=float(avw.get("max_stationary_speed_mps",.5))
            dwell=float(avw.get("dwell_seconds",5.0))
            direction=int(avw.get("direction",1))
            for obj in object_list.objects:
                if str(obj.object_type).lower() not in self.VEHICLE_TYPES:continue
                object_id=str(obj.object_id);active.add(object_id)
                speed=math.hypot(float(obj.vx),float(obj.vy))
                if speed>max_speed:
                    self._stationary_since.pop(object_id,None);continue
                since=self._stationary_since.setdefault(object_id,now)
                key=("AVW",object_id)
                if now-since>=dwell and self._cooldown_ready(key,now):
                    events.append(self._envelope(
                        "AVW",6,"请注意前方异常车辆",now,direction,
                        float(ego.get("speed_kmh",0.0)),
                        {"object_id":object_id,"stationary_seconds":round(now-since,2)}))
                    self._last_emitted[key]=now
            for object_id in list(self._stationary_since):
                if object_id not in active:self._stationary_since.pop(object_id,None)
        slw=self.config.get("slw",{}) or {}
        if slw.get("enabled",True) and ego.get("speed_kmh") is not None:
            speed=float(ego["speed_kmh"]);limit=int(slw.get("speed_limit_kmh",40))
            flag=2 if speed>float(limit) else 1;key=("SLW",flag)
            if self._cooldown_ready(key,now):
                events.append(self._envelope(
                    "SLW",9,"请注意限速",now,-1,None,
                    {"speed_limit":limit,"spd_Flag":flag,"speed":int(round(speed))}))
                self._last_emitted[key]=now
        return events


def encode_v2x_event(event):
    return json.dumps(event,ensure_ascii=False,separators=(",",":"))
