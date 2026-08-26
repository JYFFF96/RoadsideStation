from __future__ import print_function

import math


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
