from __future__ import print_function


def carla_map_short_name(name):
    """Normalize a CARLA asset path or short map name."""
    return str(name or "").replace("\\", "/").rstrip("/").split("/")[-1]


def is_town05_map(name):
    """Accept Town05 and optimized/packaged Town05 name variants."""
    return carla_map_short_name(name).lower().startswith("town05")


def town05_switch_target(current_name, configured_target="Town05_Opt"):
    """Return None when already in Town05, otherwise the configured target."""
    if is_town05_map(current_name):
        return None
    target = carla_map_short_name(configured_target) or "Town05_Opt"
    if not is_town05_map(target):
        target = "Town05_Opt"
    return target
