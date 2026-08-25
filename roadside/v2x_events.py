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
        self._last_emitted={};self._event_count=0
        self._vru_presence_hits=0
        self._hlw_presence_hits=0
        self._avw_presence_since=None
        self.last_diagnostics={}

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

    def _credible_vru(self, obj, config):
        object_type=str(obj.object_type).lower()
        if (object_type not in self.PEDESTRIAN_TYPES and
                object_type not in self.CYCLIST_TYPES):return False
        if (config.get("require_confirmed",True) and
                str(getattr(obj,"track_state","confirmed")).lower()!=
                "confirmed"):return False
        # Detector class alone is insufficient: the .76 obstacle-only run
        # repeatedly labelled a low, thin LiDAR cluster as a person. Keep
        # legacy ObjectList inputs compatible, but gate FusedObject geometry.
        size=list(getattr(obj,"size",[]) or [])
        if len(size)>=3 and any(float(value)>0.0 for value in size[:3]):
            length,width,height=[float(value) for value in size[:3]]
            if not (float(config.get("min_height_m",.45))<=height<=
                    float(config.get("max_height_m",2.60))):return False
            if length>float(config.get("max_length_m",1.50)):return False
            if width>float(config.get("max_width_m",1.20)):return False
        return True

    def _stopped_vehicle_geometry(self, obj, config):
        if not config.get("vehicle_geometry_fallback_enabled",True):return False
        if str(obj.object_type).lower()!="unknown_obstacle":return False
        if str(getattr(obj,"track_state","confirmed")).lower()!="confirmed":return False
        if int(getattr(obj,"age",1))<int(config.get("geometry_min_track_age",5)):
            return False
        if float(getattr(obj,"confidence",0.0))<float(
                config.get("geometry_min_confidence",.70)):return False
        size=list(getattr(obj,"size",[]) or [])
        if len(size)<3:return False
        length,width,height=[float(value) for value in size[:3]]
        if not (float(config.get("geometry_min_length_m",2.8))<=length<=
                float(config.get("geometry_max_length_m",7.5))):return False
        if not (float(config.get("geometry_min_width_m",1.2))<=width<=
                float(config.get("geometry_max_width_m",3.2))):return False
        if not (float(config.get("geometry_min_height_m",1.0))<=height<=
                float(config.get("geometry_max_height_m",3.0))):return False
        evidence=dict(getattr(obj,"perception_evidence",{}) or {})
        quality=float(evidence.get("trackQuality",0.0))
        return quality>=float(config.get("geometry_min_track_quality",.60))

    def update(self, object_list, ego=None):
        if not self.enabled:return []
        ego=ego or {};now=float(object_list.timestamp);events=[]
        vrucw=self.config.get("vrucw",{}) or {}
        if vrucw.get("enabled",True):
            vru=[]
            for obj in object_list.objects:
                if self._credible_vru(obj,vrucw):vru.append(obj)
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
            typed=[];geometry=[]
            for obj in object_list.objects:
                speed=math.hypot(float(obj.vx),float(obj.vy))
                if speed>max_speed:continue
                if (str(obj.object_type).lower() in self.VEHICLE_TYPES and
                        str(getattr(obj,"track_state","confirmed")).lower()==
                        "confirmed"):typed.append(obj)
                elif self._stopped_vehicle_geometry(obj,avw):geometry.append(obj)
            stopped=typed+geometry
            if stopped:
                if self._avw_presence_since is None:self._avw_presence_since=now
            else:self._avw_presence_since=None
            elapsed=(0.0 if self._avw_presence_since is None else
                     now-self._avw_presence_since)
            self.last_diagnostics["avw"]={"typed":len(typed),
                "geometry":len(geometry),"stopped":len(stopped),
                "dwell_seconds":round(elapsed,2)}
            key=("AVW","stopped_vehicle_presence")
            if (self._avw_presence_since is not None and
                    now-self._avw_presence_since>=dwell and
                    self._cooldown_ready(key,now)):
                events.append(self._envelope(
                    "AVW",6,"请注意前方异常车辆",now,direction,
                    float(ego.get("speed_kmh",0.0)),
                    {"object_id":str(stopped[0].object_id),
                     "vehicle_count":len(stopped),
                     "stationary_seconds":round(now-self._avw_presence_since,2),
                     "classification_source":("camera_label" if typed else
                                              "lidar_vehicle_geometry"),
                     "trigger_mode":"stopped_vehicle_presence"}))
                self._last_emitted[key]=now
        slw=self.config.get("slw",{}) or {}
        speed_available=ego.get("speed_kmh") is not None
        self.last_diagnostics["slw"]={"speed_available":speed_available,
            "speed_limit_kmh":int(slw.get("speed_limit_kmh",40))}
        if slw.get("enabled",True) and speed_available:
            speed=float(ego["speed_kmh"]);limit=int(slw.get("speed_limit_kmh",40))
            flag=2 if speed>float(limit) else 1;key=("SLW",flag)
            self.last_diagnostics["slw"].update({"speed_kmh":speed,
                                                  "spd_Flag":flag})
            if self._cooldown_ready(key,now):
                events.append(self._envelope(
                    "SLW",9,"请注意限速",now,-1,None,
                    {"speed_limit":limit,"spd_Flag":flag,"speed":int(round(speed))}))
                self._last_emitted[key]=now
        return events


def encode_v2x_event(event):
    return json.dumps(event,ensure_ascii=False,separators=(",",":"))
