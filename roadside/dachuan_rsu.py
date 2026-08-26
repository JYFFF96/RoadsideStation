from __future__ import print_function

import hashlib
import json
import math
import uuid


_PTC_TYPES={"vehicle":1,"car":1,"truck":1,"bus":1,"motorcycle":1,
            "bicycle":2,"cyclist":2,"person":3,"pedestrian":3}


def _clamp(value,minimum,maximum):
    return max(minimum,min(maximum,int(value)))


def _participant_id(value):
    text=str(value)
    digits="".join(ch for ch in text if ch.isdigit())
    if digits:return int(digits)%256
    return int(hashlib.sha1(text.encode("utf-8")).hexdigest()[:2],16)


class DachuanRsuBridge(object):
    """Encode the canonical FusedObjectList for Dachuan MEC-RSU MQTT.

    event2hmi is a device-to-HMI result, not an RSU input. Participant causes
    are sent as RSM, while road events/signs are sent as RSI.
    """
    def __init__(self,config=None):
        self.config=dict(config or {});self.enabled=bool(self.config.get("enabled",False))
        self.ref_lat=self.config.get("reference_latitude_deg")
        self.ref_lon=self.config.get("reference_longitude_deg")
        self.ref_elevation_m=float(self.config.get("reference_elevation_m",0.0))
        self.origin_x=None;self.origin_y=None;self.origin_z=0.0;self.msg_cnt=0;self._last_rsm=None
        self.last_diagnostic={}
        if self.enabled and (self.ref_lat is None or self.ref_lon is None):
            raise ValueError("Dachuan RSU publishing requires surveyed reference_latitude_deg and reference_longitude_deg")

    def set_world_origin(self,x,y,z=0.0):
        self.origin_x=float(x);self.origin_y=float(y);self.origin_z=float(z)

    def _next_count(self):
        value=self.msg_cnt%128;self.msg_cnt=(self.msg_cnt+1)%128;return value

    def _topic(self,message_type):
        source=("milliRadar" if message_type.lower()=="rsm" else "event")
        template=self.config.get("topic_template",
            "command/traffic/{source}/req/{uuid}/{message_type}")
        return template.format(source=source,uuid=str(uuid.uuid4()),
                               message_type=message_type.lower())

    def _geodetic(self,x,y):
        if self.origin_x is None or self.origin_y is None:
            raise ValueError("Dachuan RSU world origin is not configured")
        dx=float(x)-self.origin_x;dy=float(y)-self.origin_y
        angle=math.radians(float(self.config.get("world_x_heading_from_east_deg",0.0)))
        east=dx*math.cos(angle)-dy*math.sin(angle)
        north=dx*math.sin(angle)+dy*math.cos(angle)
        radius=6378137.0;lat0=math.radians(float(self.ref_lat))
        lat=float(self.ref_lat)+math.degrees(north/radius)
        lon=float(self.ref_lon)+math.degrees(east/(radius*max(.01,math.cos(lat0))))
        return int(round(lat*1e7)),int(round(lon*1e7))

    def _east_north(self,x,y):
        angle=math.radians(float(self.config.get("world_x_heading_from_east_deg",0.0)))
        return (float(x)*math.cos(angle)-float(y)*math.sin(angle),
                float(x)*math.sin(angle)+float(y)*math.cos(angle))

    def _vehicle_geometry(self,obj):
        size=list(getattr(obj,"size",[]) or [])
        if len(size)<3:return False
        length,width,height=[float(v) for v in size[:3]]
        return 2.8<=length<=7.5 and 1.2<=width<=3.2 and 1.0<=height<=3.0

    def _ptc_type(self,obj):
        name=str(getattr(obj,"object_type","unknown_obstacle")).lower()
        value=_PTC_TYPES.get(name)
        if value is None and self._vehicle_geometry(obj):value=1
        return value

    def build_rsm(self,object_list):
        if not self.enabled:return None
        now=float(object_list.timestamp);hz=max(.1,float(self.config.get("rsm_publish_hz",10.0)))
        if self._last_rsm is not None and now-self._last_rsm<1.0/hz:return None
        self._last_rsm=now;participants=[]
        for obj in object_list.objects[:16]:
            ptc_type=self._ptc_type(obj)
            if ptc_type is None:continue
            lat,lon=self._geodetic(obj.x,obj.y)
            speed=math.hypot(float(obj.vx),float(obj.vy))
            east_speed,north_speed=self._east_north(obj.vx,obj.vy)
            # China V2X heading uses clockwise degrees from geographic north.
            heading=(math.degrees(math.atan2(east_speed,north_speed))%360.0
                     if speed>1e-3 else 0.0)
            size=list(getattr(obj,"size",[]) or [0.0,0.0,0.0])
            sources=set(getattr(obj,"sources",[]) or [])
            source=3 if "camera" in sources else 4
            participants.append({"ptcType":ptc_type,
                "ptcId":_participant_id(obj.object_id),"source":source,
                "id":str(obj.object_id),"secMark":int(round((now%60.0)*1000.0)),
                "pos":{"offsetLL":{"choiceID":7,"position_LatLon":
                    {"lat":lat,"long":lon}},"offsetV":{"choiceID":7,
                    "elevation":int(round((float(obj.z)-self.origin_z)*10.0))},
                    "posConfidence":{"pos":15},
                    "speed":_clamp(round(speed/0.02),0,8191),
                    "heading":_clamp(round(heading/0.0125),0,28799),
                    "size":{"width":_clamp(round(float(size[1])*100),0,1023),
                            "length":_clamp(round(float(size[0])*100),0,4095),
                            "height":_clamp(round(float(size[2])/0.05),0,127)}}})
        ref_lat=int(round(float(self.ref_lat)*1e7));ref_lon=int(round(float(self.ref_lon)*1e7))
        payload={"type":"RSM","value":{"category":"RSM",
            "msgCnt":self._next_count(),"id":str(self.config.get("rsm_id","dc-rsm-roadside-1")),
            "refPos":{"lat":ref_lat,"long":ref_lon},"participants":participants}}
        self.last_diagnostic={"message_type":"RSM","participants":len(participants)}
        return self._topic("rsm"),json.dumps(payload,ensure_ascii=False,separators=(",",":"))

    def build_rsi(self,event):
        if not self.enabled:return None
        data=dict((event or {}).get("data",{}) or {});category=str(data.get("category",""))
        lat=int(round(float(self.ref_lat)*1e7));lon=int(round(float(self.ref_lon)*1e7))
        value={"category":"RSI","msgCnt":self._next_count(),"rtes":[],"rtss":[]}
        if category=="HLW":
            value["rtes"].append({"rteId":int(data.get("event_count",1))%256,
                "eventType":int(data.get("event_type",37)),"eventSource":5,
                "eventPos":{"offsetLL":{"choiceID":7,"position_LatLon":
                    {"lat":lat,"long":lon}},"offsetV":{"choiceID":7,
                    "elevation":int(round(self.ref_elevation_m*10.0))}},
                "eventRadius":int(self.config.get("event_radius_m",100)),
                "description":str(data.get("description","道路存在障碍物")),"priority":1})
        elif category=="SLW":
            sign_type=self.config.get("slw_sign_type")
            if sign_type is None:
                self.last_diagnostic={"message_type":"RSI","category":"SLW",
                                      "suppressed":"missing_slw_sign_type"}
                return None
            value["rtss"].append({"rtsId":int(data.get("event_count",1))%256,
                "signType":int(sign_type),"signPos":{"offsetLL":{"present":7,
                    "lon":lon,"lat":lat},"offsetV":{"present":7,
                    "elevation":int(round(self.ref_elevation_m*10.0))}},
                "description":"限速%dkm/h"%int(data.get("speed_limit",0)),"priority":1})
        else:
            self.last_diagnostic={"message_type":"RSI","category":category,
                                  "suppressed":"participant_warning_uses_rsm"}
            return None
        payload={"type":"RSI","value":value}
        self.last_diagnostic={"message_type":"RSI","category":category}
        return self._topic("rsi"),json.dumps(payload,ensure_ascii=False,separators=(",",":"))
