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

    def __init__(self, station_id, config=None):
        self.station_id=str(station_id);self.config=config or {}
        self.enabled=bool(self.config.get("enabled",False))
        self._stationary_since={};self._last_emitted={};self._event_count=0

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

    def update(self, object_list, ego=None):
        if not self.enabled:return []
        ego=ego or {};now=float(object_list.timestamp);events=[];active=set()
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
