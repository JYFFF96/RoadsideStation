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


def _balanced_spawn_order(spawns, center, rng):
    """Round-robin CARLA map spawns across the RSU evaluation bands."""
    buckets = [[], [], [], []]
    for sp in spawns:
        distance = _distance2d(sp.location, center)
        index = 0 if distance <= 30.0 else 1 if distance <= 50.0 else 2 if distance <= 80.0 else 3
        buckets[index].append(sp)
    for bucket in buckets:
        rng.shuffle(bucket)
    ordered = []
    while any(buckets):
        for bucket in buckets:
            if bucket:
                ordered.append(bucket.pop())
    return ordered


def _range_counts(actors, center):
    counts = [0, 0, 0]
    for actor in actors:
        distance = _distance2d(actor.get_location(), center)
        if distance <= 30.0:counts[0] += 1
        elif distance <= 50.0:counts[1] += 1
        elif distance <= 80.0:counts[2] += 1
    return counts


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
        client.apply_batch_sync([carla.command.DestroyActor(x) for x in ids], True)


def main():
    global _STOP
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    ap = argparse.ArgumentParser(description="V0.6.12.8.1 balanced local RSU traffic")
    ap.add_argument("--config", default="config/roadside.yaml")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--tm-port", type=int, default=8000)
    ap.add_argument("--vehicles", "-n", type=int, default=45)
    ap.add_argument("--spawn-radius", type=float, default=130.0)
    ap.add_argument("--recycle-radius", type=float, default=170.0,
                    help="destroy owned vehicles after they leave this radius")
    ap.add_argument("--recycle", action="store_true",
                    help="opt-in legacy destroy/respawn behavior")
    ap.add_argument("--report-radius", type=float, default=80.0)
    ap.add_argument("--following-distance", type=float, default=5.0)
    ap.add_argument("--speed-diff", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--keep-existing", action="store_true")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    cfg = _load_config(args.config)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    vehicles = set()

    try:
        world = client.get_world()
        world_map = world.get_map()
        center = _resolve_rsu_center(world_map, cfg)
        print("RoadsideStation V0.6.12.8.1 - BALANCED LOCAL TRAFFIC")
        print("Map: %s" % world_map.name.split("/")[-1])
        print("Driving: CARLA Traffic Manager autopilot only")
        print("Spawn radius=%.0fm, report radius=%.0fm, recycle=%s" %
              (args.spawn_radius, args.report_radius,
               ("ON >%.0fm" % args.recycle_radius) if args.recycle else "OFF"))
        print("No custom path / waypoint routing / steering override.")

        if not args.keep_existing:
            existing = list(world.get_actors().filter("vehicle.*"))
            if existing:
                _destroy(client, [a.id for a in existing])
                print("Removed %d existing vehicles." % len(existing))
                time.sleep(0.5)

        tm = client.get_trafficmanager(args.tm_port)
        tm.set_global_distance_to_leading_vehicle(args.following_distance)
        tm.global_percentage_speed_difference(args.speed_diff)
        tm.set_hybrid_physics_mode(False)
        try:
            tm.set_random_device_seed(args.seed)
        except Exception:
            pass

        blueprints = _safe_blueprints(world)
        all_spawns = list(world_map.get_spawn_points())
        local_spawns = [sp for sp in all_spawns if _distance2d(sp.location, center) <= args.spawn_radius]
        local_spawns = _balanced_spawn_order(local_spawns, center, rng)
        if not blueprints:
            raise RuntimeError("No safe vehicle blueprints found")
        if not local_spawns:
            raise RuntimeError("No CARLA spawn points within %.0fm" % args.spawn_radius)
        print("Map spawn points:%d | strict-local spawn points:%d" % (len(all_spawns), len(local_spawns)))

        target = min(args.vehicles, len(local_spawns))
        if args.vehicles > len(local_spawns):
            print("Requested %d, local region supports at most %d simultaneous spawn slots." %
                  (args.vehicles, target))

        spawn_cursor = [0]

        def spawn_one():
            occupied = [a.get_location() for a in world.get_actors().filter("vehicle.*")]
            total = len(local_spawns)
            for offset in range(total):
                index = (spawn_cursor[0] + offset) % total
                sp = local_spawns[index]
                if any(_distance2d(sp.location, p) < 9.0 for p in occupied):
                    continue
                bp = _prepare(rng.choice(blueprints), rng)
                try:
                    actor = world.try_spawn_actor(bp, sp)
                except Exception:
                    actor = None
                if actor is None:
                    continue
                actor.set_autopilot(True, tm.get_port())
                vehicles.add(actor.id)
                spawn_cursor[0] = (index + 1) % total
                return actor.id
            return None

        attempts = 0
        while len(vehicles) < target and attempts < target * 10:
            attempts += 1
            if spawn_one() is None:
                time.sleep(0.05)
        print("Initial spawned: %d/%d" % (len(vehicles), target))
        print("Initial spawn order is balanced across 00-30m, 30-50m, 50-80m and outer feeder roads.")
        print("Recycle: %s." % ("ON (legacy opt-in)" if args.recycle else "OFF"))
        print("Press Ctrl+C to stop and remove these vehicles.")

        last_report = 0.0
        last_spawn = 0.0
        while not _STOP:
            actors = {a.id: a for a in world.get_actors().filter("vehicle.*")}
            gone = []
            outside = []
            for aid in list(vehicles):
                a = actors.get(aid)
                if a is None:
                    gone.append(aid)
                elif args.recycle and _distance2d(a.get_location(), center) > args.recycle_radius:
                    outside.append(aid)
            for aid in gone:
                vehicles.discard(aid)
            if outside:
                _destroy(client, outside)
                for aid in outside:
                    vehicles.discard(aid)

            now = time.time()
            spawned = 0
            if len(vehicles) < target and now - last_spawn >= 0.75:
                # Refill gently to avoid dumping many cars onto the same road at once.
                for _ in range(min(2, target - len(vehicles))):
                    if spawn_one() is None:
                        break
                    spawned += 1
                last_spawn = now

            if now - last_report >= 2.0:
                current = {a.id: a for a in world.get_actors().filter("vehicle.*")}
                within = 0
                farthest = 0.0
                alive = []
                for aid in vehicles:
                    a = current.get(aid)
                    if a is None:
                        continue
                    alive.append(a)
                    d = _distance2d(a.get_location(), center)
                    farthest = max(farthest, d)
                    if d <= args.report_radius:
                        within += 1
                counts = _range_counts(alive, center)
                print("Traffic | alive:%d/%d | 00-30m:%d 30-50m:%d 50-80m:%d | within %.0fm:%d | farthest:%.1fm | recycled:%d respawned:%d" %
                      (len(vehicles), target, counts[0], counts[1], counts[2],
                       args.report_radius, within, farthest, len(outside), spawned))
                last_report = now
            time.sleep(0.2)

    except KeyboardInterrupt:
        _STOP = True
    finally:
        print("Destroying %d traffic vehicles..." % len(vehicles))
        try:
            _destroy(client, list(vehicles))
        except Exception as exc:
            print("Cleanup warning: %s" % exc)
        print("Traffic stopped cleanly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
