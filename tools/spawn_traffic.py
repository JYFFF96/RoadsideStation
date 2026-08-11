from __future__ import print_function

import argparse
import random
import time

import carla


def main():
    parser = argparse.ArgumentParser(description="Spawn CARLA traffic for RoadsideStation testing")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--vehicles", type=int, default=20)
    parser.add_argument("--walkers", type=int, default=0)
    parser.add_argument("--tm-port", type=int, default=8000)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    tm = client.get_trafficmanager(args.tm_port)
    tm.set_global_distance_to_leading_vehicle(2.5)

    blueprints = [bp for bp in world.get_blueprint_library().filter("vehicle.*")
                  if bp.has_attribute("number_of_wheels")]
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)

    actors = []
    count = min(args.vehicles, len(spawn_points))
    for transform in spawn_points[:count]:
        bp = random.choice(blueprints)
        if bp.has_attribute("color"):
            colors = bp.get_attribute("color").recommended_values
            if colors:
                bp.set_attribute("color", random.choice(colors))
        actor = world.try_spawn_actor(bp, transform)
        if actor is not None:
            actor.set_autopilot(True, args.tm_port)
            actors.append(actor)

    print("Spawned %d vehicles. Press Ctrl+C to remove them." % len(actors))
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
