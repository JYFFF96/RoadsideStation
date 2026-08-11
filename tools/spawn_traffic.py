from __future__ import print_function

import argparse
import math
import random
import signal
import sys
import time

import carla


_STOP_REQUESTED = False


def _request_stop(signum, frame):
    global _STOP_REQUESTED
    if not _STOP_REQUESTED:
        print("\nStop requested (signal %s). Cleaning up..." % signum)
    _STOP_REQUESTED = True


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
    preferred_prefixes = (
        "vehicle.tesla.model3", "vehicle.lincoln.mkz", "vehicle.audi.tt",
        "vehicle.mini.cooper", "vehicle.mercedes.coupe",
        "vehicle.nissan.patrol", "vehicle.toyota.prius",
    )
    normal, fallback = [], []
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


def _destroy_actors(client, actors):
    ids = []
    for actor in actors:
        try:
            if actor is not None and actor.is_alive:
                ids.append(actor.id)
        except Exception:
            pass
    if not ids:
        print("No spawned vehicles need cleanup.")
        return
    print("Destroying %d vehicles..." % len(ids))
    try:
        responses = client.apply_batch_sync(
            [carla.command.DestroyActor(actor_id) for actor_id in ids], True)
        errors = [r.error for r in responses if getattr(r, "error", None)]
        if errors:
            print("Cleanup completed with %d CARLA errors." % len(errors))
        else:
            print("Cleanup complete.")
    except Exception as exc:
        print("Batch cleanup failed: %s" % exc)
        print("CARLA may have stopped; stale actors will disappear when CARLA restarts.")


def _distance2d(a, b):
    return math.hypot(float(a.location.x) - float(b.location.x),
                      float(a.location.y) - float(b.location.y))


def _safe_spawn_points(world, minimum_spacing):
    world_map = world.get_map()
    candidates = []
    for transform in world_map.get_spawn_points():
        waypoint = world_map.get_waypoint(transform.location, project_to_road=True,
                                          lane_type=carla.LaneType.Driving)
        if waypoint is None or waypoint.is_junction:
            continue
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
    global _STOP_REQUESTED
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    parser = argparse.ArgumentParser(description="Spawn stable CARLA traffic for RoadsideStation testing")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--vehicles", type=int, default=20)
    parser.add_argument("--walkers", type=int, default=0)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--speed-diff", type=float, default=30.0)
    parser.add_argument("--min-spacing", type=float, default=15.0)
    parser.add_argument("--keep-existing", action="store_true")
    args = parser.parse_args()

    random.seed(args.seed)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    actors = []

    try:
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

        requested = min(args.vehicles, len(spawn_points))
        for transform in spawn_points:
            if _STOP_REQUESTED or len(actors) >= requested:
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
            actors.append(actor)
            actor.set_autopilot(True, tm_port)
            try:
                tm.auto_lane_change(actor, False)
                tm.distance_to_leading_vehicle(actor, 5.0)
                tm.vehicle_percentage_speed_difference(actor, float(args.speed_diff))
            except Exception:
                pass

        print("Spawned %d safe lane-aligned vehicles on %s." % (
            len(actors), world.get_map().name.split("/")[-1]))
        print("Press Ctrl+C once to stop and remove all spawned vehicles.")
        print("Do not use Ctrl+Z; it suspends the process without cleanup.")

        while not _STOP_REQUESTED:
            # Short sleep makes signal handling responsive even on Python 3.7.
            time.sleep(0.1)
    except KeyboardInterrupt:
        _STOP_REQUESTED = True
    finally:
        _destroy_actors(client, actors)

    print("Traffic generator stopped cleanly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
