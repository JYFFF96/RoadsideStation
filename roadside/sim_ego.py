from __future__ import print_function

import math

EGO_ROLE = "rsu_test_ego"


def find_ego_actor(world, role_name=EGO_ROLE):
    """Select one explicit role; never silently pick an arbitrary traffic car."""
    if world is None:
        return None
    actors = [actor for actor in world.get_actors().filter("vehicle.*")
              if actor.attributes.get("role_name", "") == role_name
              and actor.is_alive]
    if len(actors) > 1:
        raise ValueError("Multiple ego vehicles with role %s; stop duplicate scenarios" % role_name)
    return actors[0] if actors else None


def read_ego_state(world, role_name=EGO_ROLE):
    """Simulation reference state only, never an OBU identity or detected target."""
    actor = find_ego_actor(world, role_name)
    if actor is None:
        return {}
    transform = actor.get_transform()
    velocity = actor.get_velocity()
    extent = actor.bounding_box.extent
    center = actor.bounding_box.location
    # Transform the local bounding-box centre, including nonzero blueprint offsets.
    matrix = transform.get_matrix()
    local = (center.x, center.y, center.z, 1.0)
    bbox = [sum(matrix[row][col] * local[col] for col in range(4))
            for row in range(3)]
    return {"actor_id": actor.id, "role_name": role_name,
            "x": transform.location.x, "y": transform.location.y,
            "z": transform.location.z, "yaw_deg": transform.rotation.yaw,
            "vx": velocity.x, "vy": velocity.y,
            "speed_kmh": 3.6 * math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2),
            "bbox_x": bbox[0], "bbox_y": bbox[1], "bbox_z": bbox[2],
            "half_length": extent.x, "half_width": extent.y,
            "half_height": extent.z, "source": "CARLA_EGO"}


def is_ego_detection(obj, ego):
    """Conservative spatial self mask for events, not for RSM or perception.

    Fused IDs are tracker IDs, not CARLA actor IDs. Restrict the mask to the
    reference vehicle footprint; pedestrians/cyclists must never be masked.
    """
    if not ego or "bbox_x" not in ego:
        return False
    if str(obj.object_type).lower() not in (
            "vehicle", "car", "truck", "bus", "motorcycle", "unknown_obstacle"):
        return False
    yaw = math.radians(ego["yaw_deg"])
    dx, dy = float(obj.x) - ego["bbox_x"], float(obj.y) - ego["bbox_y"]
    longitudinal = dx * math.cos(yaw) + dy * math.sin(yaw)
    lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return (abs(longitudinal) <= ego["half_length"] + .35 and
            abs(lateral) <= ego["half_width"] + .25 and
            abs(float(obj.z) - ego["bbox_z"]) <= ego["half_height"] + 1.0)


def find_test_ego_speed_kmh(world,role_name="rsu_test_speeding_vehicle"):
    """Read a tagged CARLA vehicle as a simulation-only OBU speed source."""
    if world is None:return None
    try:actors=world.get_actors().filter("vehicle.*")
    except Exception:return None
    for actor in actors:
        try:
            if str(actor.attributes.get("role_name",""))!=str(role_name):continue
            velocity=actor.get_velocity()
            return 3.6*math.sqrt(float(velocity.x)**2+float(velocity.y)**2+
                                 float(velocity.z)**2)
        except Exception:continue
    return None
