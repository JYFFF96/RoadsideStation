from __future__ import print_function

import math

import carla

from .sensors import SensorCache, image_to_bgra, lidar_to_xyz, radar_to_cartesian


def _combine_transform(base, offset):
    return carla.Transform(
        carla.Location(
            x=float(base.location.x) + float(offset.get("x", 0)),
            y=float(base.location.y) + float(offset.get("y", 0)),
            z=float(base.location.z) + float(offset.get("z", 0))),
        carla.Rotation(
            pitch=float(base.rotation.pitch) + float(offset.get("pitch", 0)),
            yaw=float(base.rotation.yaw) + float(offset.get("yaw", 0)),
            roll=float(base.rotation.roll) + float(offset.get("roll", 0))))


class CarlaRoadsideStation(object):
    def __init__(self, config):
        self.config = config
        self.cache = SensorCache()
        self.client = None
        self.world = None
        self.sensors = []
        self.base_transform = None
        self.map_name = None

    def _ensure_map(self):
        requested = self.config["carla"].get("map")
        self.world = self.client.get_world()
        current_name = self.world.get_map().name.split("/")[-1]
        if requested and current_name != requested:
            print("Loading CARLA map: %s" % requested)
            self.world = self.client.load_world(requested)
        self.map_name = self.world.get_map().name.split("/")[-1]

    def _find_junction_transform(self):
        station_cfg = self.config["station"]
        world_map = self.world.get_map()
        waypoints = world_map.generate_waypoints(2.0)
        junction_waypoints = []
        seen = set()
        for waypoint in waypoints:
            if not waypoint.is_junction:
                continue
            junction = waypoint.get_junction()
            if junction is None or junction.id in seen:
                continue
            seen.add(junction.id)
            junction_waypoints.append(waypoint)

        if not junction_waypoints:
            raise RuntimeError("No junction found in map %s" % self.map_name)

        index = int(station_cfg.get("junction_index", 0)) % len(junction_waypoints)
        waypoint = junction_waypoints[index]
        location = waypoint.transform.location
        yaw = float(waypoint.transform.rotation.yaw)
        lateral = float(station_cfg.get("lateral_offset", 5.0))
        height = float(station_cfg.get("height", 8.0))
        yaw_rad = math.radians(yaw)

        # Shift from lane center toward the roadside using the lane normal.
        x = float(location.x) - math.sin(yaw_rad) * lateral
        y = float(location.y) + math.cos(yaw_rad) * lateral
        z = float(location.z) + height
        return carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0))

    def _resolve_base_transform(self):
        station_cfg = self.config["station"]
        if station_cfg.get("deployment", "manual") == "auto_junction":
            return self._find_junction_transform()
        cfg = station_cfg["transform"]
        return carla.Transform(
            carla.Location(x=float(cfg.get("x", 0)),
                           y=float(cfg.get("y", 0)),
                           z=float(cfg.get("z", 8))),
            carla.Rotation(pitch=float(cfg.get("pitch", 0)),
                           yaw=float(cfg.get("yaw", 0)),
                           roll=float(cfg.get("roll", 0))))

    def start(self):
        carla_cfg = self.config["carla"]
        self.client = carla.Client(carla_cfg.get("host", "127.0.0.1"),
                                   int(carla_cfg.get("port", 2000)))
        self.client.set_timeout(float(carla_cfg.get("timeout", 10.0)))
        self._ensure_map()
        blueprints = self.world.get_blueprint_library()
        self.base_transform = self._resolve_base_transform()

        print("RSU deployment: map=%s x=%.2f y=%.2f z=%.2f yaw=%.1f" % (
            self.map_name,
            self.base_transform.location.x,
            self.base_transform.location.y,
            self.base_transform.location.z,
            self.base_transform.rotation.yaw))

        if self.config["camera"].get("enabled", True):
            cfg = self.config["camera"]
            bp = blueprints.find("sensor.camera.rgb")
            bp.set_attribute("image_size_x", str(cfg.get("width", 1280)))
            bp.set_attribute("image_size_y", str(cfg.get("height", 720)))
            bp.set_attribute("fov", str(cfg.get("fov", 90)))
            actor = self.world.spawn_actor(bp, _combine_transform(self.base_transform, cfg["transform"]))
            actor.listen(lambda data: self.cache.set_camera(data.frame, image_to_bgra(data)))
            self.sensors.append(actor)

        if self.config["lidar"].get("enabled", True):
            cfg = self.config["lidar"]
            bp = blueprints.find("sensor.lidar.ray_cast")
            for key, attr in [("channels", "channels"), ("range", "range"),
                              ("points_per_second", "points_per_second"),
                              ("rotation_frequency", "rotation_frequency")]:
                bp.set_attribute(attr, str(cfg[key]))
            actor = self.world.spawn_actor(bp, _combine_transform(self.base_transform, cfg["transform"]))
            actor.listen(lambda data: self.cache.set_lidar(data.frame, lidar_to_xyz(data)))
            self.sensors.append(actor)

        if self.config["radar"].get("enabled", True):
            cfg = self.config["radar"]
            bp = blueprints.find("sensor.other.radar")
            bp.set_attribute("horizontal_fov", str(cfg["horizontal_fov"]))
            bp.set_attribute("vertical_fov", str(cfg["vertical_fov"]))
            bp.set_attribute("range", str(cfg["range"]))
            actor = self.world.spawn_actor(bp, _combine_transform(self.base_transform, cfg["transform"]))
            actor.listen(lambda data: self.cache.set_radar(data.frame, radar_to_cartesian(data)))
            self.sensors.append(actor)

    def stop(self):
        for sensor in self.sensors:
            try:
                sensor.stop()
                sensor.destroy()
            except Exception:
                pass
        self.sensors = []
