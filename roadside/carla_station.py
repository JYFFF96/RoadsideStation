from __future__ import print_function

import math
import carla
from .sensors import SensorCache, image_to_bgra, lidar_to_xyz, radar_to_cartesian


def _combine_transform(base, offset):
    return carla.Transform(
        carla.Location(x=float(base.location.x)+float(offset.get("x",0)), y=float(base.location.y)+float(offset.get("y",0)), z=float(base.location.z)+float(offset.get("z",0))),
        carla.Rotation(pitch=float(base.rotation.pitch)+float(offset.get("pitch",0)), yaw=float(base.rotation.yaw)+float(offset.get("yaw",0)), roll=float(base.rotation.roll)+float(offset.get("roll",0))))


def _angle_diff(a,b):
    d=(float(a)-float(b)+180.0)%360.0-180.0
    return abs(d)


class CarlaRoadsideStation(object):
    def __init__(self, config):
        self.config=config; self.cache=SensorCache(); self.client=None; self.world=None; self.sensors=[]
        self.base_transform=None; self.map_name=None; self.camera_transform=None; self.lidar_transform=None; self.radar_transform=None; self.world_map=None; self.junction_center=None

    def _attach_current_world(self):
        self.world=self.client.get_world(); self.world_map=self.world.get_map(); self.map_name=self.world_map.name.split("/")[-1]
        requested=self.config.get("carla",{}).get("map")
        if requested and requested!=self.map_name:
            print("WARNING: CARLA is running map %s, config prefers %s."%(self.map_name,requested))
            print("RoadsideStation will NOT call load_world(); start CARLA with the desired map instead.")

    def _configure_map_layers(self):
        cfg=self.config.get("environment",{})
        if cfg.get("unload_foliage",False):
            try:
                self.world.unload_map_layer(carla.MapLayer.Foliage)
                print("Map layer disabled: Foliage")
            except Exception as exc:
                print("WARNING: unable to unload Foliage layer: %s"%exc)

    def _junction_candidates(self):
        seen={};
        for wp in self.world_map.generate_waypoints(2.0):
            if not wp.is_junction: continue
            j=wp.get_junction()
            if j is None or j.id in seen: continue
            try: pairs=j.get_waypoints(carla.LaneType.Driving)
            except Exception: pairs=[]
            headings=[]
            for pair in pairs:
                if not pair: continue
                try: headings.append(float(pair[0].transform.rotation.yaw)%360.0)
                except Exception: pass
            bins=[]
            for h in headings:
                if all(_angle_diff(h,b)>35.0 for b in bins): bins.append(h)
            box=j.bounding_box; area=max(1.0,float(box.extent.x)*2.0)*max(1.0,float(box.extent.y)*2.0)
            score=len(bins)*1000.0+min(area,500.0)
            seen[j.id]=(score,j,wp,len(bins),area)
        items=list(seen.values()); items.sort(key=lambda x:x[0],reverse=True); return items

    def _find_junction_transform(self):
        cfg=self.config["station"]; candidates=self._junction_candidates()
        if not candidates: raise RuntimeError("No junction found in map %s"%self.map_name)
        cross=[x for x in candidates if x[3]>=4]
        pool=cross or candidates; index=int(cfg.get("junction_index",0))%len(pool); score,junction,wp,dirs,area=pool[index]
        center=junction.bounding_box.location; self.junction_center=carla.Location(x=center.x,y=center.y,z=center.z)
        yaw=float(wp.transform.rotation.yaw); lateral=float(cfg.get("lateral_offset",7.0)); height=float(cfg.get("height",8.0)); r=math.radians(yaw)
        x=float(center.x)-math.sin(r)*lateral; y=float(center.y)+math.cos(r)*lateral; z=float(center.z)+height
        sensor_yaw=math.degrees(math.atan2(float(center.y)-y,float(center.x)-x))
        print("Selected junction id=%s directions=%d area=%.1f center=(%.2f,%.2f)"%(junction.id,dirs,area,center.x,center.y))
        return carla.Transform(carla.Location(x=x,y=y,z=z),carla.Rotation(pitch=0.0,yaw=sensor_yaw,roll=0.0))

    def _resolve_base_transform(self):
        cfg=self.config["station"]
        if cfg.get("deployment","manual") in ("auto_junction","auto_cross_junction"): return self._find_junction_transform()
        t=cfg["transform"]; return carla.Transform(carla.Location(x=float(t.get("x",0)),y=float(t.get("y",0)),z=float(t.get("z",8))),carla.Rotation(pitch=float(t.get("pitch",0)),yaw=float(t.get("yaw",0)),roll=float(t.get("roll",0))))

    def validate_driving_roi(self,x,y,z,extent=None):
        if self.world_map is None:return True,"ok",{}
        cfg=self.config.get("fusion",{});margin=float(cfg.get("road_roi_margin",2.5));min_above=float(cfg.get("road_min_height",-.8));max_above=float(cfg.get("road_max_height",3.5))
        loc=carla.Location(x=float(x),y=float(y),z=float(z)); wp=self.world_map.get_waypoint(loc,project_to_road=True,lane_type=carla.LaneType.Driving)
        if wp is None:return False,"no_waypoint",{}
        lane=wp.transform.location; lateral=math.hypot(float(x)-lane.x,float(y)-lane.y); allowed=float(wp.lane_width)*.5+margin;dz=float(z)-float(lane.z)
        details={"lateral":lateral,"allowed_lateral":allowed,"dz":dz,"lane_width":float(wp.lane_width)}
        if lateral>allowed:return False,"lateral",details
        if dz<min_above:return False,"below_road",details
        if dz>max_above:return False,"above_road",details
        return True,"ok",details

    def is_driving_roi(self,x,y,z,extent=None):
        return self.validate_driving_roi(x,y,z,extent)[0]

    def start(self):
        cc=self.config["carla"];self.client=carla.Client(cc.get("host","127.0.0.1"),int(cc.get("port",2000)));self.client.set_timeout(float(cc.get("timeout",60.0)))
        self._attach_current_world();self._configure_map_layers();blueprints=self.world.get_blueprint_library();self.base_transform=self._resolve_base_transform()
        print("RSU deployment: map=%s x=%.2f y=%.2f z=%.2f yaw=%.1f"%(self.map_name,self.base_transform.location.x,self.base_transform.location.y,self.base_transform.location.z,self.base_transform.rotation.yaw))
        if self.config["camera"].get("enabled",True):
            cfg=self.config["camera"];bp=blueprints.find("sensor.camera.rgb");bp.set_attribute("image_size_x",str(cfg.get("width",1280)));bp.set_attribute("image_size_y",str(cfg.get("height",720)));bp.set_attribute("fov",str(cfg.get("fov",90)));self.camera_transform=_combine_transform(self.base_transform,cfg["transform"]);a=self.world.spawn_actor(bp,self.camera_transform);a.listen(lambda d:self.cache.set_camera(d.frame,image_to_bgra(d)));self.sensors.append(a)
        if self.config["lidar"].get("enabled",True):
            cfg=self.config["lidar"];bp=blueprints.find("sensor.lidar.ray_cast")
            for key,attr in [("channels","channels"),("range","range"),("points_per_second","points_per_second"),("rotation_frequency","rotation_frequency"),("upper_fov","upper_fov"),("lower_fov","lower_fov")]:
                if key in cfg: bp.set_attribute(attr,str(cfg[key]))
            self.lidar_transform=_combine_transform(self.base_transform,cfg["transform"])
            print("LiDAR config: channels=%s pps=%s range=%sm vertical_fov=[%s,%s] height=%.2fm"%(cfg.get("channels"),cfg.get("points_per_second"),cfg.get("range"),cfg.get("lower_fov","default"),cfg.get("upper_fov","default"),self.lidar_transform.location.z))
            a=self.world.spawn_actor(bp,self.lidar_transform);a.listen(lambda d:self.cache.set_lidar(d.frame,lidar_to_xyz(d)));self.sensors.append(a)
        if self.config["radar"].get("enabled",True):
            cfg=self.config["radar"];bp=blueprints.find("sensor.other.radar");bp.set_attribute("horizontal_fov",str(cfg["horizontal_fov"]));bp.set_attribute("vertical_fov",str(cfg["vertical_fov"]));bp.set_attribute("range",str(cfg["range"]));self.radar_transform=_combine_transform(self.base_transform,cfg["transform"]);a=self.world.spawn_actor(bp,self.radar_transform);a.listen(lambda d:self.cache.set_radar(d.frame,radar_to_cartesian(d)));self.sensors.append(a)

    def stop(self):
        for sensor in self.sensors:
            try:sensor.stop();sensor.destroy()
            except Exception:pass
        self.sensors=[]
