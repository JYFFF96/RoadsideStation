"""Own or reuse the single simulation reference vehicle for warning scenarios."""
from __future__ import print_function

import math

from .sim_ego import EGO_ROLE, find_ego_actor


class ScenarioEgo(object):
    def __init__(self, world, role_name=EGO_ROLE):
        self.world = world
        self.role_name = role_name
        self.actor = None
        self.owned = False

    def start(self, client, world_map, center, speed_kmh=25.0, tm_port=8000,
              targets=()):
        import carla
        self.actor = find_ego_actor(self.world, self.role_name)
        if self.actor is not None:
            print("[EGO] Reusing id=%d role=%s; original owner retains control" %
                  (self.actor.id, self.role_name))
            return self.actor
        # Prefer the same lane behind a stopped target, so AVW has a clear ego/target relation.
        candidates = []
        for target in targets:
            wp = world_map.get_waypoint(target.get_location())
            if wp is not None:
                for gap in (25.0, 35.0, 45.0):
                    candidates.extend(wp.previous(gap))
        incoming = []
        for wp in world_map.generate_waypoints(3.0):
            loc = wp.transform.location
            dx, dy = center.x - loc.x, center.y - loc.y
            distance = math.hypot(dx, dy)
            forward = wp.transform.get_forward_vector()
            if (not wp.is_junction and 30.0 <= distance <= 60.0 and
                    (dx * forward.x + dy * forward.y) / max(distance, .01) > .65):
                incoming.append(wp)
        incoming.sort(key=lambda wp: abs(wp.transform.location.distance(center) - 45.0))
        candidates.extend(incoming)
        blueprints = [bp for bp in self.world.get_blueprint_library().filter("vehicle.*")
                      if bp.has_attribute("number_of_wheels") and
                      bp.get_attribute("number_of_wheels").as_int() == 4]
        blueprints.sort(key=lambda bp: (bp.id != "vehicle.tesla.model3", bp.id))
        if not blueprints:
            raise RuntimeError("No four-wheel blueprint available for ego")
        bp = blueprints[0]
        bp.set_attribute("role_name", self.role_name)
        if bp.has_attribute("color"):
            bp.set_attribute("color", "30,110,240")
        for wp in candidates:
            tf = wp.transform
            tf.location.z += .35
            self.actor = self.world.try_spawn_actor(bp, tf)
            if self.actor is not None:
                self.owned = True
                break
        if self.actor is None:
            raise RuntimeError("Cannot spawn ego on an incoming lane; clear traffic near the junction")
        try:
            if speed_kmh <= 0:
                self.actor.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            else:
                tm = client.get_trafficmanager(tm_port)
                tm.auto_lane_change(self.actor, False)
                tm.set_desired_speed(self.actor, float(speed_kmh))
                self.actor.set_autopilot(True, tm_port)
            print("[EGO] Created id=%d role=%s desired_speed=%.1fkm/h TM=%d" %
                  (self.actor.id, self.role_name, speed_kmh, tm_port))
            print("[EGO] Traffic Manager controls steering/braking; events use measured speed.")
            return self.actor
        except Exception:
            self.close()
            raise

    def close(self):
        if self.owned and self.actor is not None:
            try:
                self.actor.destroy()
            except RuntimeError:
                pass
        self.actor = None
        self.owned = False
