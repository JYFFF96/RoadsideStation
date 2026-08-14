from __future__ import print_function


VEHICLE_TYPES = set(("car", "bus", "truck"))
VRU_TYPES = set(("person", "bicycle", "motorcycle"))
ROAD_OBJECT_TYPES = VEHICLE_TYPES | VRU_TYPES | set(("unknown_obstacle",))


def carla_actor_class(actor_or_type):
    """Map a CARLA actor/type id to the public road-object taxonomy."""
    type_id = getattr(actor_or_type, "type_id", actor_or_type)
    tid = str(type_id or "").lower()
    if tid.startswith("walker.pedestrian."):
        return "person"
    if "crossbike" in tid or "diamondback" in tid or "omafiets" in tid:
        return "bicycle"
    if "motorcycle" in tid or "harley" in tid or "kawasaki" in tid or "vespa" in tid or "yamaha" in tid:
        return "motorcycle"
    if "bus" in tid:
        return "bus"
    if "truck" in tid or "carlacola" in tid or "firetruck" in tid or "sprinter" in tid:
        return "truck"
    if tid.startswith("vehicle."):
        return "car"
    if tid.startswith("static.prop.") or tid.startswith("dynamic.prop."):
        return "unknown_obstacle"
    return "unknown_obstacle"


def object_group(object_type):
    name = str(object_type or "unknown_obstacle")
    if name in VEHICLE_TYPES:return "vehicle"
    if name in VRU_TYPES:return "vru"
    return "obstacle"


def iter_carla_road_actors(world, obstacle_patterns=None):
    """Yield deduplicated CARLA actors that are valid road-object truth.

    This helper is simulation-only. Static props are opt-in because normal map
    infrastructure must not automatically become road-obstacle truth.
    """
    seen = set()
    patterns = ["vehicle.*", "walker.pedestrian.*"] + list(obstacle_patterns or [])
    for pattern in patterns:
        try:actors = world.get_actors().filter(pattern)
        except Exception:actors = []
        for actor in actors:
            actor_id = getattr(actor, "id", id(actor))
            if actor_id in seen:continue
            seen.add(actor_id)
            yield actor
