from __future__ import print_function

import argparse
import math
import random
import signal
import sys
import time

import carla
import yaml

_STOP = False


def _stop(signum, frame):
    global _STOP
    if not _STOP:
        print("\nStop requested. Cleaning up RSU local traffic...")
    _STOP = True


def _load_config(path):
    with open(path, "r") as fp:
        return yaml.safe_load(fp)


def _angle_diff(a, b):
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _junction_candidates(world_map):
    seen = {}
    for wp in world_map.generate_waypoints(2.0):
        if not wp.is_junction:
            continue
        j = wp.get_junction()
        if j is None or j.id in seen:
            continue
        try:
            pairs = j.get_waypoints(carla.LaneType.Driving)
        except Exception:
            pairs = []
        headings = []
        for pair in pairs:
            try:
                headings.append(float(pair[0].transform.rotation.yaw) % 360.0)
            except Exception:
                pass
        bins = []
        for h in headings:
            if all(_angle_diff(h, b) > 35.0 for b in bins):
                bins.append(h)
        box = j.bounding_box
        area = max(1.0, float(box.extent.x) * 2.0) * max(1.0, float(box.extent.y) * 2.0)
        seen[j.id] = (len(bins) * 1000.0 + min(area, 500.0), j, len(bins), area)
    items = list(seen.values())
    items.sort(key=lambda x: x[0], reverse=True)
    return items


def _resolve_rsu_center(world_map, config):
    sc = config.get("station", {})
    deployment = sc.get("deployment", "manual")
    if deployment in ("auto_junction", "auto_cross_junction"):
        candidates = _junction_candidates(world_map)
        if not candidates:
            raise RuntimeError("No junction found for RSU local traffic")
        cross = [x for x in candidates if x[2] >= 4]
        pool = cross or candidates
        _, junction, dirs, area = pool[int(sc.get("junction_index", 0)) % len(pool)]
        c = junction.bounding_box.location
        print("RSU junction id=%s directions=%d area=%.1f center=(%.2f, %.2f)" %
              (junction.id, dirs, area, c.x, c.y))
        return carla.Location(x=c.x, y=c.y, z=c.z)
    t = sc.get("transform", {})
    return carla.Location(x=float(t.get("x", 0)), y=float(t.get("y", 0)), z=float(t.get("z", 0)))


def _distance2d(a, b):
    return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))


def _safe_blueprints(world):
    out = []
    for bp in world.get_blueprint_library().filter("vehicle.*"):
        try:
            if bp.has_attribute("number_of_wheels") and int(bp.get_attribute("number_of_wheels")) != 4:
                continue
            bid = bp.id.lower()
            if any(x in bid for x in ("microlino", "carlacola", "cybertruck", "t2", "sprinter", "firetruck", "ambulance")):
                continue
            out.append(bp)
        except Exception:
            pass
    return sorted(out, key=lambda x: x.id)


def _prepare(bp, rng):
    if bp.has_attribute("color"):
        vals = bp.get_attribute("color").recommended_values
        if vals:
            bp.set_attribute("color", rng.choice(vals))
    if bp.has_attribute("driver_id"):
        vals = bp.get_attribute("driver_id").recommended_values
        if vals:
            bp.set_attribute("driver_id", rng.choice(vals))
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "autopilot")
    return bp


def _destroy(client, ids):
    if ids:
        client.apply_batch([carla.command.DestroyActor(x) for x in ids])


def main():
    global _STOP
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    ap = argparse.ArgumentParser(description="V0.4.7 RSU-local CARLA Traffic Manager traffic")
    ap.add_argument("--config", default="config/roadside.yaml")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--tm-port", type=int, default=8000)
    ap.add_argument("--vehicles", "-n", type=int, default=30)
    ap.add_argument("--spawn-radius", type=float, default=130.0,
                    help="prefer CARLA map spawn points within this distance of the RSU junction")
    ap.add_argument("--report-radius", type=float, default=80.0)
    ap.add_argument("--safe", action="store_true", default=True)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--keep-existing", action="store_true")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    cfg = _load_config(args.config)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    vehicles = []

    try:
        world = client.get_world()
        world_map = world.get_map()
        center = _resolve_rsu_center(world_map, cfg)
        print("RoadsideStation V0.4.7 - CARLA TM Local Spawn")
        print("Map: %s" % world_map.name.split("/")[-1])
        print("Driving: official CARLA Traffic Manager autopilot only")
        print("No custom path, no waypoint routing, no steering override, no density recycle.")

        if not args.keep_existing:
            existing = list(world.get_actors().filter("vehicle.*"))
            if existing:
                _destroy(client, [a.id for a in existing])
                print("Removed %d existing vehicles." % len(existing))
                time.sleep(0.5)

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        try:
            traffic_manager.set_random_device_seed(args.seed)
        except Exception:
            pass

        blueprints = _safe_blueprints(world)
        if not blueprints:
            raise RuntimeError("No safe vehicle blueprints found")

        all_spawns = list(world_map.get_spawn_points())
        local_spawns = [sp for sp in all_spawns if _distance2d(sp.location, center) <= args.spawn_radius]
        # Important: these are untouched map-defined spawn transforms, exactly the same
        # type used by CARLA's official generate_traffic.py. We only change their order.
        rng.shuffle(local_spawns)
        rng.shuffle(all_spawns)
        ordered = local_spawns + [sp for sp in all_spawns if sp not in local_spawns]
        print("Map spawn points: %d | preferred within %.0fm: %d" %
              (len(all_spawns), args.spawn_radius, len(local_spawns)))

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        batch = []
        for sp in ordered[:args.vehicles]:
            bp = _prepare(rng.choice(blueprints), rng)
            batch.append(SpawnActor(bp, sp).then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        for response in client.apply_batch_sync(batch, False):
            if response.error:
                print("Spawn warning: %s" % response.error)
            else:
                vehicles.append(response.actor_id)

        print("Spawned: %d/%d" % (len(vehicles), args.vehicles))
        print("Press Ctrl+C to stop and remove these vehicles.")

        last = 0.0
        while not _STOP:
            now = time.time()
            if now - last >= 2.0:
                actors = {a.id: a for a in world.get_actors().filter("vehicle.*")}
                alive = 0
                within = 0
                for aid in vehicles:
                    a = actors.get(aid)
                    if a is None:
                        continue
                    alive += 1
                    if _distance2d(a.get_location(), center) <= args.report_radius:
                        within += 1
                print("Traffic | alive:%d/%d | within %.0fm:%d" %
                      (alive, len(vehicles), args.report_radius, within))
                last = now
            time.sleep(0.2)

    except KeyboardInterrupt:
        _STOP = True
    finally:
        print("Destroying %d traffic vehicles..." % len(vehicles))
        try:
            _destroy(client, vehicles)
        except Exception as exc:
            print("Cleanup warning: %s" % exc)
        print("Traffic stopped cleanly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
