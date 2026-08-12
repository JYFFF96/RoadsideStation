from __future__ import print_function

import argparse
import random
import signal
import sys
import time

import carla

_STOP_REQUESTED = False


def _request_stop(signum, frame):
    global _STOP_REQUESTED
    if not _STOP_REQUESTED:
        print("\nStop requested. Cleaning up RoadsideStation traffic...")
    _STOP_REQUESTED = True


def _get_tm(client, preferred):
    """Connect to a Traffic Manager port, falling back when a stale TM owns one."""
    last = None
    ports = [preferred] + [p for p in range(8000, 8011) if p != preferred]
    for port in ports:
        try:
            return client.get_trafficmanager(port), port
        except RuntimeError as exc:
            last = exc
            print("Traffic Manager port %d unavailable: %s" % (port, exc))
    raise RuntimeError("No free Traffic Manager port: %s" % last)


def _vehicle_blueprints(world, safe=True):
    """Follow the spirit of CARLA examples/generate_traffic.py blueprint filtering."""
    blueprints = list(world.get_blueprint_library().filter("vehicle.*"))
    if not safe:
        return blueprints

    safe_bps = []
    for bp in blueprints:
        try:
            if bp.has_attribute("number_of_wheels"):
                if int(bp.get_attribute("number_of_wheels").as_int()) != 4:
                    continue
            # Keep normal passenger/commercial vehicles. Exclude a few unusual models
            # that are less useful for roadside perception tests.
            bid = bp.id.lower()
            if any(x in bid for x in ("carlacola", "cybertruck", "t2", "sprinter")):
                continue
            safe_bps.append(bp)
        except Exception:
            pass
    return safe_bps or blueprints


def _prepare_blueprint(bp, rng):
    if bp.has_attribute("color"):
        colors = bp.get_attribute("color").recommended_values
        if colors:
            bp.set_attribute("color", rng.choice(colors))
    if bp.has_attribute("driver_id"):
        drivers = bp.get_attribute("driver_id").recommended_values
        if drivers:
            bp.set_attribute("driver_id", rng.choice(drivers))
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "roadside_autopilot")
    return bp


def _destroy(client, actor_ids):
    if not actor_ids:
        print("No spawned vehicles need cleanup.")
        return
    print("Destroying %d vehicles..." % len(actor_ids))
    try:
        client.apply_batch_sync(
            [carla.command.DestroyActor(actor_id) for actor_id in actor_ids], True)
        print("Cleanup complete.")
    except Exception as exc:
        print("Cleanup failed: %s" % exc)


def main():
    global _STOP_REQUESTED
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    parser = argparse.ArgumentParser(
        description="RoadsideStation CARLA official-style Traffic Manager generator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--vehicles", "-n", type=int, default=30)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--speed-diff", type=float, default=30.0,
                        help="Traffic Manager percentage speed reduction")
    parser.add_argument("--distance", type=float, default=3.0,
                        help="global following distance in metres")
    parser.add_argument("--safe", action="store_true", default=True,
                        help="filter to normal four-wheel vehicles (default enabled)")
    parser.add_argument("--all-blueprints", action="store_true",
                        help="disable safe vehicle filtering")
    parser.add_argument("--keep-existing", action="store_true",
                        help="do not remove existing vehicle actors before spawning")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    spawned_ids = []

    try:
        world = client.get_world()
        map_name = world.get_map().name.split("/")[-1]
        print("RoadsideStation V0.4.4 official-style traffic generator")
        print("CARLA map: %s" % map_name)

        if not args.keep_existing:
            existing = list(world.get_actors().filter("vehicle.*"))
            if existing:
                ids = [a.id for a in existing]
                client.apply_batch_sync(
                    [carla.command.DestroyActor(actor_id) for actor_id in ids], True)
                print("Removed %d existing/stale vehicles." % len(ids))
                time.sleep(0.5)

        tm, tm_port = _get_tm(client, args.tm_port)
        print("Traffic Manager connected on port %d" % tm_port)
        tm.set_global_distance_to_leading_vehicle(args.distance)
        tm.global_percentage_speed_difference(args.speed_diff)
        tm.set_hybrid_physics_mode(False)
        try:
            tm.set_random_device_seed(args.seed)
        except Exception:
            pass

        blueprints = _vehicle_blueprints(world, safe=not args.all_blueprints)
        spawn_points = list(world.get_map().get_spawn_points())
        if not blueprints:
            raise RuntimeError("No vehicle blueprints available")
        if not spawn_points:
            raise RuntimeError("No map spawn points available")

        # This is intentionally the same basic strategy as CARLA's official
        # generate_traffic.py: use map spawn points and let Traffic Manager own
        # routing. Do NOT call set_path(), force junction branches, or manually
        # construct waypoints here.
        rng.shuffle(spawn_points)
        requested = min(args.vehicles, len(spawn_points))
        if args.vehicles > len(spawn_points):
            print("Requested %d vehicles but map has only %d spawn points; limiting request." %
                  (args.vehicles, len(spawn_points)))

        batch = []
        for transform in spawn_points[:requested]:
            bp = _prepare_blueprint(rng.choice(blueprints), rng)
            batch.append(
                carla.command.SpawnActor(bp, transform).then(
                    carla.command.SetAutopilot(
                        carla.command.FutureActor, True, tm_port)))

        responses = client.apply_batch_sync(batch, True)
        failures = 0
        for response in responses:
            if response.error:
                failures += 1
            else:
                spawned_ids.append(response.actor_id)

        print("Spawned %d/%d vehicles using CARLA map spawn points." %
              (len(spawned_ids), requested))
        if failures:
            print("%d spawn points were occupied or unavailable." % failures)
        print("Routing is controlled entirely by CARLA Traffic Manager (no RoadsideStation set_path).")
        print("Safe four-wheel blueprint filtering: %s" % ("ON" if not args.all_blueprints else "OFF"))
        print("Press Ctrl+C to stop and remove all vehicles spawned by this script.")

        while not _STOP_REQUESTED:
            time.sleep(0.1)

    except KeyboardInterrupt:
        _STOP_REQUESTED = True
    finally:
        _destroy(client, spawned_ids)

    print("Traffic generator stopped cleanly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
