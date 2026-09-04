"""Own and safely control the single reference vehicle used by warning scenes."""
from __future__ import print_function

import math
import time

from .sim_ego import EGO_ROLE, find_ego_actor


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, float(value)))


def _angle_delta_degrees(target, current):
    return (float(target) - float(current) + 180.0) % 360.0 - 180.0


def _distance2d(first, second):
    return math.hypot(float(first.x) - float(second.x),
                      float(first.y) - float(second.y))


def _forward_score(waypoint, center):
    location = waypoint.transform.location
    forward = waypoint.transform.get_forward_vector()
    dx = float(center.x) - float(location.x)
    dy = float(center.y) - float(location.y)
    distance = max(.01, math.hypot(dx, dy))
    return (dx * float(forward.x) + dy * float(forward.y)) / distance


def _straightest_successor(current, candidates):
    """Choose a deterministic continuation without lane-changing at a fork."""
    candidates = list(candidates)
    if not candidates:
        return None
    yaw = float(current.transform.rotation.yaw)
    return min(candidates, key=lambda item: (
        abs(_angle_delta_degrees(item.transform.rotation.yaw, yaw)),
        str(getattr(item, "road_id", "")),
        int(getattr(item, "lane_id", 0))))


def _relative_xy(transform, location):
    yaw = math.radians(float(transform.rotation.yaw))
    dx = float(location.x) - float(transform.location.x)
    dy = float(location.y) - float(transform.location.y)
    return (dx * math.cos(yaw) + dy * math.sin(yaw),
            -dx * math.sin(yaw) + dy * math.cos(yaw))


class ScenarioEgo(object):
    """Waypoint lane follower with a separate emergency-stop layer."""
    def __init__(self, world, role_name=EGO_ROLE):
        self.world = world
        self.role_name = role_name
        self.actor = None
        self.owned = False
        self.world_map = None
        self.target_speed_kmh = 0.0
        self.explicit_hazards = []
        self.last_status = {}
        self._last_report = 0.0
        self.verbose = False

    def _candidate_spawn_waypoints(self, world_map, center, targets):
        candidates = []
        for target in targets:
            try:
                waypoint = world_map.get_waypoint(
                    target.get_location(), project_to_road=True)
            except (RuntimeError, TypeError):
                waypoint = world_map.get_waypoint(target.get_location())
            if waypoint is None:
                continue
            for gap in (45.0, 55.0, 65.0):
                try:
                    candidates.extend(waypoint.previous(gap))
                except RuntimeError:
                    pass
        incoming = []
        for waypoint in world_map.generate_waypoints(3.0):
            location = waypoint.transform.location
            distance = _distance2d(location, center)
            if (not waypoint.is_junction and 40.0 <= distance <= 70.0 and
                    _forward_score(waypoint, center) > .75):
                incoming.append(waypoint)
        incoming.sort(key=lambda item: (
            abs(_distance2d(item.transform.location, center) - 55.0),
            int(getattr(item, "road_id", 0)), int(getattr(item, "lane_id", 0))))
        candidates.extend(incoming)
        result = []
        for waypoint in candidates:
            if all(_distance2d(waypoint.transform.location, old.transform.location) >= 3.0
                   for old in result):
                result.append(waypoint)
        return result

    def start(self, client, world_map, center, speed_kmh=18.0, tm_port=8000,
              targets=(), verbose=False):
        import carla
        del client, tm_port
        self.verbose = bool(verbose)
        self.actor = find_ego_actor(self.world, self.role_name)
        if self.actor is not None:
            print("[EGO] Reusing id=%d role=%s; original owner retains control" %
                  (self.actor.id, self.role_name))
            return self.actor
        candidates = self._candidate_spawn_waypoints(world_map, center, targets)
        blueprints = [bp for bp in self.world.get_blueprint_library().filter("vehicle.*")
                      if bp.has_attribute("number_of_wheels") and
                      bp.get_attribute("number_of_wheels").as_int() == 4]
        blueprints.sort(key=lambda bp: (bp.id != "vehicle.tesla.model3", bp.id))
        if not blueprints:
            raise RuntimeError("No four-wheel blueprint available for ego")
        blueprint = blueprints[0]
        blueprint.set_attribute("role_name", self.role_name)
        if blueprint.has_attribute("color"):
            blueprint.set_attribute("color", "30,110,240")
        for waypoint in candidates:
            source = waypoint.transform
            transform = carla.Transform(
                carla.Location(x=source.location.x, y=source.location.y,
                               z=source.location.z + .35),
                carla.Rotation(pitch=source.rotation.pitch, yaw=source.rotation.yaw,
                               roll=source.rotation.roll))
            self.actor = self.world.try_spawn_actor(blueprint, transform)
            if self.actor is not None:
                self.owned = True
                break
        if self.actor is None:
            raise RuntimeError("Cannot spawn ego on an incoming lane; clear traffic near the junction")
        try:
            self.world_map = world_map
            self.target_speed_kmh = max(0.0, float(speed_kmh))
            self.explicit_hazards = [item for item in targets if item is not None]
            self.actor.set_autopilot(False)
            if self.target_speed_kmh <= 0:
                self.actor.apply_control(carla.VehicleControl(
                    throttle=0.0, brake=1.0, hand_brake=True))
            else:
                self.update_control()
            print("[EGO] Created id=%d role=%s lane_follower=ON target_speed=%.1fkm/h" %
                  (self.actor.id, self.role_name, self.target_speed_kmh))
            print("[EGO] Safety brake watches vehicles, pedestrians and scene props.")
            return self.actor
        except Exception:
            self.close()
            raise

    def set_hazards(self, actors):
        self.explicit_hazards = [item for item in actors if item is not None]

    def _all_hazards(self):
        result = []
        seen = set((int(self.actor.id),))
        groups = []
        try:
            groups.append(self.world.get_actors().filter("vehicle.*"))
            groups.append(self.world.get_actors().filter("walker.pedestrian.*"))
        except RuntimeError:
            groups = []
        groups.append(self.explicit_hazards)
        for group in groups:
            for actor in group:
                try:
                    actor_id = int(actor.id)
                    if actor_id in seen or not actor.is_alive:
                        continue
                    seen.add(actor_id)
                    result.append(actor)
                except (AttributeError, RuntimeError):
                    continue
        return result

    def _nearest_hazard(self, transform, speed_mps):
        ego_extent = self.actor.bounding_box.extent
        lane_half_width = 1.35
        try:
            waypoint = self.world_map.get_waypoint(
                transform.location, project_to_road=True)
            lane_half_width = max(1.1, min(2.0, float(waypoint.lane_width) * .48))
        except (AttributeError, RuntimeError, TypeError):
            pass
        watch_distance = min(35.0, max(8.0,
            speed_mps * speed_mps / 12.0 + .35 * speed_mps + 4.0))
        nearest = None
        for actor in self._all_hazards():
            try:
                longitudinal, lateral = _relative_xy(transform, actor.get_location())
                extent = actor.bounding_box.extent
                clearance = longitudinal - float(ego_extent.x) - float(extent.x)
                lateral_limit = lane_half_width + min(1.0, float(extent.y))
                if (clearance >= -0.5 and clearance <= watch_distance and
                        abs(lateral) <= lateral_limit):
                    item = (clearance, actor, lateral)
                    if nearest is None or item[0] < nearest[0]:
                        nearest = item
            except (AttributeError, RuntimeError):
                continue
        return nearest, watch_distance

    def update_control(self):
        """Apply one non-blocking lane-follow/safety update (call around 10 Hz)."""
        import carla
        if self.actor is None or not self.actor.is_alive:
            return {"mode": "missing"}
        if not self.owned:
            return {"mode": "reused_no_control"}
        if self.target_speed_kmh <= 0:
            self.actor.apply_control(carla.VehicleControl(
                throttle=0.0, brake=1.0, hand_brake=True))
            self.last_status = {"mode": "parked", "speed_kmh": 0.0}
            return dict(self.last_status)
        transform = self.actor.get_transform()
        velocity = self.actor.get_velocity()
        speed_mps = math.sqrt(float(velocity.x) ** 2 + float(velocity.y) ** 2 +
                              float(velocity.z) ** 2)
        speed_kmh = 3.6 * speed_mps
        waypoint = self.world_map.get_waypoint(transform.location, project_to_road=True)
        if waypoint is None:
            self.actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            self.last_status = {"mode": "no_waypoint", "speed_kmh": speed_kmh}
            return dict(self.last_status)
        lane_error = _distance2d(transform.location, waypoint.transform.location)
        next_waypoint = _straightest_successor(waypoint, waypoint.next(4.0))
        if next_waypoint is None:
            self.actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            self.last_status = {"mode": "route_end", "speed_kmh": speed_kmh,
                                "lane_error_m": lane_error}
            return dict(self.last_status)
        target = next_waypoint.transform.location
        target_yaw = math.degrees(math.atan2(float(target.y) - float(transform.location.y),
                                             float(target.x) - float(transform.location.x)))
        heading_error = _angle_delta_degrees(target_yaw, transform.rotation.yaw)
        steer = _clamp(heading_error / 38.0, -.80, .80)
        hazard, watch_distance = self._nearest_hazard(transform, speed_mps)
        traffic_light_stop = False
        try:
            if self.actor.is_at_traffic_light():
                state = self.actor.get_traffic_light_state()
                traffic_light_stop = state in (carla.TrafficLightState.Red,
                                               carla.TrafficLightState.Yellow)
        except (AttributeError, RuntimeError):
            pass
        lane_departure = lane_error > max(1.25, float(waypoint.lane_width) * .48)
        if hazard is not None or traffic_light_stop or lane_departure:
            brake, throttle = 1.0, 0.0
            mode = ("hazard_stop" if hazard is not None else
                    ("traffic_light_stop" if traffic_light_stop else "lane_departure_stop"))
        else:
            error = self.target_speed_kmh - speed_kmh
            throttle = _clamp(.18 + error * .025, 0.0, .55) if error > 0 else 0.0
            brake = _clamp(-error * .06, 0.0, .6) if error < -1.0 else 0.0
            mode = "lane_follow"
        self.actor.apply_control(carla.VehicleControl(
            throttle=throttle, steer=steer, brake=brake, hand_brake=False))
        self.last_status = {"mode": mode, "speed_kmh": speed_kmh,
            "lane_error_m": lane_error, "heading_error_deg": heading_error,
            "steer": steer, "watch_distance_m": watch_distance,
            "hazard_id": (int(hazard[1].id) if hazard is not None else None),
            "hazard_clearance_m": (float(hazard[0]) if hazard is not None else None)}
        now = time.time()
        if self.verbose and now - self._last_report >= 1.0:
            print("[EGO CONTROL] mode=%s speed=%.1fkm/h lane_error=%.2fm steer=%.2f hazard=%s clearance=%s" %
                  (mode, speed_kmh, lane_error, steer, self.last_status["hazard_id"],
                   ("-" if hazard is None else "%.1fm" % hazard[0])))
            self._last_report = now
        return dict(self.last_status)

    def close(self):
        if self.owned and self.actor is not None:
            try:
                self.actor.destroy()
            except RuntimeError:
                pass
        self.actor = None
        self.owned = False
        self.explicit_hazards = []
