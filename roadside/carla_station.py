from __future__ import print_function

import carla

from .sensors import SensorCache, image_to_bgra, lidar_to_xyz, radar_to_cartesian


def _transform(carla_module, base, offset):
    return carla_module.Transform(
        carla_module.Location(
            x=float(base.get("x", 0)) + float(offset.get("x", 0)),
            y=float(base.get("y", 0)) + float(offset.get("y", 0)),
            z=float(base.get("z", 0)) + float(offset.get("z", 0))),
        carla_module.Rotation(
            pitch=float(base.get("pitch", 0)) + float(offset.get("pitch", 0)),
            yaw=float(base.get("yaw", 0)) + float(offset.get("yaw", 0)),
            roll=float(base.get("roll", 0)) + float(offset.get("roll", 0))))


class CarlaRoadsideStation(object):
    def __init__(self, config):
        self.config = config
        self.cache = SensorCache()
        self.client = None
        self.world = None
        self.sensors = []

    def start(self):
        carla_cfg = self.config["carla"]
        self.client = carla.Client(carla_cfg.get("host", "127.0.0.1"),
                                   int(carla_cfg.get("port", 2000)))
        self.client.set_timeout(float(carla_cfg.get("timeout", 10.0)))
        self.world = self.client.get_world()
        blueprints = self.world.get_blueprint_library()
        base = self.config["station"]["transform"]

        if self.config["camera"].get("enabled", True):
            cfg = self.config["camera"]
            bp = blueprints.find("sensor.camera.rgb")
            bp.set_attribute("image_size_x", str(cfg.get("width", 1280)))
            bp.set_attribute("image_size_y", str(cfg.get("height", 720)))
            bp.set_attribute("fov", str(cfg.get("fov", 90)))
            actor = self.world.spawn_actor(bp, _transform(carla, base, cfg["transform"]))
            actor.listen(lambda data: self.cache.set_camera(data.frame, image_to_bgra(data)))
            self.sensors.append(actor)

        if self.config["lidar"].get("enabled", True):
            cfg = self.config["lidar"]
            bp = blueprints.find("sensor.lidar.ray_cast")
            for key, attr in [("channels", "channels"), ("range", "range"),
                              ("points_per_second", "points_per_second"),
                              ("rotation_frequency", "rotation_frequency")]:
                bp.set_attribute(attr, str(cfg[key]))
            actor = self.world.spawn_actor(bp, _transform(carla, base, cfg["transform"]))
            actor.listen(lambda data: self.cache.set_lidar(data.frame, lidar_to_xyz(data)))
            self.sensors.append(actor)

        if self.config["radar"].get("enabled", True):
            cfg = self.config["radar"]
            bp = blueprints.find("sensor.other.radar")
            bp.set_attribute("horizontal_fov", str(cfg["horizontal_fov"]))
            bp.set_attribute("vertical_fov", str(cfg["vertical_fov"]))
            bp.set_attribute("range", str(cfg["range"]))
            actor = self.world.spawn_actor(bp, _transform(carla, base, cfg["transform"]))
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
