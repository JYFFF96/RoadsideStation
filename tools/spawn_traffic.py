from __future__ import print_function

import argparse
import math
import random
import time

import carla


def _get_traffic_manager(client, preferred_port):
    ports = [preferred_port]
    for port in range(8000, 8011):
        if port not in ports:
            ports.append(port)

    last_error = None
    for port in ports:
        try:
            tm = client.get_trafficmanager(port)
            return tm, port
        except RuntimeError as exc:
            last_error = exc
            print("Traffic Manager port %d unavailable: %s" % (port, exc))

    raise RuntimeError("No free Traffic Manager port found in 8000-8010: %s" % last_error)


def _vehicle_blueprints(world):
    # Prefer ordinary passenger cars for deterministic perception tests.
    preferred_prefixes = (
        "vehicle.tesla.model3",
        "vehicle.lincoln.mkz",
        "vehicle.audi.tt",
        "vehicle.mini.cooper",
        "vehicle.mercedes.coupe",
        "vehicle.nissan.patrol",
        "vehicle.toyota.prius",
    )
    normal = []
    fallback = []
    for bp in world.get_blueprint_library().filter("vehicle.*"):
        if not bp.has_attribute("number_of_wheels"):
            continue
        try:
            wheels = int(bp.get_attribute("number_of_wheels").as_int())
        except Exception:
            continue
        if wheels != 4:
            continue
        fallback.append(bp)
        if bp.id.startswith(preferred_prefixes):
            normal.append(bp)
    return normal or fallback


def _cleanup_existing_vehicles(client, world):
    vehicles = list(world.get_actors().filter("vehicle.*"))
    if not vehicles:
        return 0
    commands = [carla.command.DestroyActor(actor.id) for actor in vehicles]
    client.apply_batch_sync(commands, True)
    return len(vehicles)


def _distance2d(a, b):
    dx = float(a.location.x) - float(b.location.x)
    dy = float(a.location.y) - float(b.location.y)
    return math.hypot(dx, dy)


def _safe_spawn_points(world, minimum_spacing):
    """Return lane-aligned spawn points away from junction interiors.

    Town03's roundabout is useful for perception, but starting many vehicles
    inside the junction can immediately create pile-ups. We therefore start
    traffic on ordinary driving lanes and keep initial vehicles separated.
    Traffic Manager can still drive them through the roundabout afterwards.
    """
    world_map = world.get_map()
    candidates = []
    for transform in world_map.get_spawn_points():
        waypoint = world_map.get_waypoint(
            transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving)
        if waypoint is None or waypoint.is_junction:
            continue
        # Use the map waypoint transform so heading is exactly lane aligned.
        transform = waypoint.transform
        transform.location.z += 0.35
        candidates.append(transform)

    random.shuffle(candidates)
    selected = []
    for transform in candidates:
        if all(_distance2d(transform, other) >= minimum_spacing for other in selected):
            selected.append(transform)
    return selected


def main():
    parser = argparse.ArgumentParser(description="Spawn stable CARLA traffic for RoadsideStation testing")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--vehicles", type=int, default=20)
    parser.add_argument("--walkers", type=int, default=0)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--speed-diff", type=float, default=30.0,
                        help="Global percentage speed reduction; 30 means 30%% slower")
    parser.add_argument("--min-spacing", type=float, default=15.0,
                        help="Minimum initial distance between spawned vehicles")
    parser.add_argument("--keep-existing", action="store_true",
                        help="Do not remove vehicles left from previous test runs")
    args = parser.parse_args()

    random.seed(args.seed)
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()

    if not args.keep_existing:
        removed = _cleanup_existing_vehicles(client, world)
        if removed:
            print("Removed %d existing/stale vehicles from previous runs." % removed)
            time.sleep(0.5)

    tm, tm_port = _get_traffic_manager(client, args.tm_port)
    tm.set_global_distance_to_leading_vehicle(5.0)
    tm.global_percentage_speed_difference(float(args.speed_diff))
    tm.set_hybrid_physics_mode(False)
    try:
        tm.set_random_device_seed(args.seed)
    except Exception:
        pass
    print("Traffic Manager connected on port %d" % tm_port)

    blueprints = _vehicle_blueprints(world)
    if not blueprints:
        raise RuntimeError("No four-wheel vehicle blueprints found")

    spawn_points = _safe_spawn_points(world, float(args.min_spacing))
    if not spawn_points:
        raise RuntimeError("No safe non-junction driving-lane spawn points found")

    actors = []
    requested = min(args.vehicles, len(spawn_points))
    for transform in spawn_points:
        if len(actors) >= requested:
            break
        bp = random.choice(blueprints)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "roadside_test")
        if bp.has_attribute("color"):
            colors = bp.get_attribute("color").recommended_values
            if colors:
                bp.set_attribute("color", random.choice(colors))

        actor = world.try_spawn_actor(bp, transform)
        if actor is None:
            continue
        actor.set_autopilot(True, tm_port)
        try:
            # Keep the traffic boring and predictable: no opportunistic lane
            # changes, generous following distance, and reduced speed.
            tm.auto_lane_change(actor, False)
            tm.distance_to_leading_vehicle(actor, 5.0)
            tm.vehicle_percentage_speed_difference(actor, float(args.speed_diff))
        except Exception:
            pass
        actors.append(actor)

    print("Spawned %d safe lane-aligned vehicles on %s. Press Ctrl+C to remove them." % (
        len(actors), world.get_map().name.split("/")[-1]))
    print("Initial junction spawns are excluded; minimum spacing=%.1fm; speed reduced %.0f%%." % (
        args.min_spacing, args.speed_diff))

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        print("Destroying %d vehicles..." % len(actors))
        for actor in actors:
            try:
                actor.destroy()
            except Exception:
                pass


if __name__ == "__main__":
    main()
