from __future__ import print_function

import argparse
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
    output = []
    for bp in world.get_blueprint_library().filter("vehicle.*"):
        if not bp.has_attribute("number_of_wheels"):
            continue
        try:
            wheels = int(bp.get_attribute("number_of_wheels").as_int())
        except Exception:
            continue
        if wheels != 4:
            continue
        output.append(bp)
    return output


def main():
    parser = argparse.ArgumentParser(description="Spawn stable CARLA traffic for RoadsideStation testing")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--vehicles", type=int, default=20)
    parser.add_argument("--walkers", type=int, default=0)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--speed-diff", type=float, default=20.0,
                        help="Global percentage speed reduction; 20 means 20%% slower")
    args = parser.parse_args()

    random.seed(args.seed)
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()

    tm, tm_port = _get_traffic_manager(client, args.tm_port)
    tm.set_global_distance_to_leading_vehicle(4.0)
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

    # CARLA map spawn points are lane-aligned. Keeping the original transforms is
    # important: manually offsetting them can place vehicles in walls or curbs.
    spawn_points = list(world.get_map().get_spawn_points())
    random.shuffle(spawn_points)

    actors = []
    count = min(args.vehicles, len(spawn_points))
    for transform in spawn_points[:count]:
        bp = random.choice(blueprints)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "autopilot")
        if bp.has_attribute("color"):
            colors = bp.get_attribute("color").recommended_values
            if colors:
                bp.set_attribute("color", random.choice(colors))
        actor = world.try_spawn_actor(bp, transform)
        if actor is None:
            continue
        actor.set_autopilot(True, tm_port)
        try:
            tm.auto_lane_change(actor, True)
            tm.distance_to_leading_vehicle(actor, 4.0)
        except Exception:
            pass
        actors.append(actor)

    print("Spawned %d lane-aligned vehicles on %s. Press Ctrl+C to remove them." % (
        len(actors), world.get_map().name.split("/")[-1]))
    print("Traffic is intentionally slowed by %.0f%% for roadside-perception testing." % args.speed_diff)

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
