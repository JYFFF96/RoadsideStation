from __future__ import print_function

# Repeatable CARLA traffic benchmark for the junction selected by RoadsideStation.
# Uses CARLA Traffic Manager only: no custom steering/path override.

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
        print("\nStop requested. Cleaning up junction benchmark traffic...")
    _STOP = True


def _load_config(path):
    with open(path, "r") as fp:
        return yaml.safe_load(fp)


def _angle_diff(a, b):
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _distance2d(a, b):
    return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))


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


def _resolve_center(world_map, config):
    sc = config.get("station", {})
    if sc.get("deployment", "manual") in ("auto_junction", "auto_cross_junction"):
        candidates = _junction_candidates(world_map)
        if not candidates:
            raise RuntimeError("No junction found")
        cross = [x for x in candidates if x[2] >= 4]
        pool = cross or candidates
        _, junction, dirs, area = pool[int(sc.get("junction_index", 0)) % len(pool)]
        c = junction.bounding_box.location
        print("Benchmark junction id=%s directions=%d area=%.1f center=(%.2f, %.2f)" %
              (junction.id, dirs, area, c.x, c.y))
        return carla.Location(x=c.x, y=c.y, z=c.z)
    t = sc.get("transform", {})
    return carla.Location(x=float(t.get("x", 0)), y=float(t.get("y", 0)), z=float(t.get("z", 0)))


def _safe_blueprints(world):
    result = []
    for bp in world.get_blueprint_library().filter("vehicle.*"):
        try:
            if bp.has_attribute("number_of_wheels") and int(bp.get_attribute("number_of_wheels")) != 4:
                continue
            name = bp.id.lower()
            if any(x in name for x in ("microlino", "carlacola", "cybertruck", "t2", "sprinter", "firetruck", "ambulance")):
                continue
            result.append(bp)
        except Exception:
            pass
    return sorted(result, key=lambda x: x.id)


def _prepare(bp, rng):
    if bp.has_attribute("color"):
        values = bp.get_attribute("color").recommended_values
        if values:
            bp.set_attribute("color", rng.choice(values))
    if bp.has_attribute("driver_id"):
        values = bp.get_attribute("driver_id").recommended_values
        if values:
            bp.set_attribute("driver_id", rng.choice(values))
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "roadside_autopilot")
    return bp


def _destroy(client, ids):
    if ids:
        client.apply_batch_sync([carla.command.DestroyActor(x) for x in ids], True)


def _bin_counts(actors, center, bins):
    counts = [0] * len(bins)
    for actor in actors:
        d = _distance2d(actor.get_location(), center)
        low = 0.0
        for i, high in enumerate(bins):
            if low <= d < high:
                counts[i] += 1
                break
            low = high
    return counts


def main():
    global _STOP
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    ap = argparse.ArgumentParser(description="Repeatable RoadsideStation junction traffic benchmark")
    ap.add_argument("--config", default="config/roadside.yaml")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--tm-port", type=int, default=8000)
    ap.add_argument("--vehicles", "-n", type=int, default=45)
    ap.add_argument("--spawn-radius", type=float, default=115.0)
    ap.add_argument("--recycle-radius", type=float, default=135.0)
    ap.add_argument("--report-radius", type=float, default=80.0)
    ap.add_argument("--following-distance", type=float, default=4.0)
    ap.add_argument("--speed-diff", type=float, default=20.0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--keep-existing", action="store_true")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    cfg = _load_config(args.config)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    owned = set()

    try:
        world = client.get_world()
        world_map = world.get_map()
        center = _resolve_center(world_map, cfg)
        print("RoadsideStation Junction Traffic Benchmark V0.5.7")
        print("Map: %s | seed:%d" % (world_map.name.split("/")[-1], args.seed))
        print("Target vehicles:%d | spawn<=%.0fm recycle>%.0fm report<=%.0fm" %
              (args.vehicles, args.spawn_radius, args.recycle_radius, args.report_radius))
        print("Traffic Manager autopilot only; deterministic spawn order/blueprint choices for the same seed.")

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
        local_spawns = [sp for sp in world_map.get_spawn_points()
                        if _distance2d(sp.location, center) <= args.spawn_radius]
        local_spawns.sort(key=lambda sp: (_distance2d(sp.location, center), sp.location.x, sp.location.y))
        if not blueprints:
            raise RuntimeError("No safe vehicle blueprints found")
        if not local_spawns:
            raise RuntimeError("No spawn points near selected junction")

        # Fixed seed shuffle gives repeatable but spatially mixed spawn choices.
        rng.shuffle(local_spawns)
        target = min(args.vehicles, len(local_spawns))
        print("Local CARLA spawn slots:%d | benchmark target:%d" % (len(local_spawns), target))

        def spawn_one():
            occupied = [a.get_location() for a in world.get_actors().filter("vehicle.*")]
            for sp in local_spawns:
                if any(_distance2d(sp.location, p) < 8.0 for p in occupied):
                    continue
                bp = _prepare(rng.choice(blueprints), rng)
                actor = world.try_spawn_actor(bp, sp)
                if actor is None:
                    continue
                actor.set_autopilot(True, tm.get_port())
                owned.add(actor.id)
                return True
            return False

        attempts = 0
        while len(owned) < target and attempts < target * 12:
            attempts += 1
            if not spawn_one():
                time.sleep(0.05)
        print("Initial spawned: %d/%d" % (len(owned), target))
        print("Press Ctrl+C to stop; benchmark-owned vehicles will be removed.")

        last_report = 0.0
        last_refill = 0.0
        while not _STOP:
            actors = {a.id: a for a in world.get_actors().filter("vehicle.*")}
            outside = []
            for aid in list(owned):
                actor = actors.get(aid)
                if actor is None:
                    owned.discard(aid)
                elif _distance2d(actor.get_location(), center) > args.recycle_radius:
                    outside.append(aid)
            if outside:
                _destroy(client, outside)
                for aid in outside:
                    owned.discard(aid)

            now = time.time()
            if len(owned) < target and now - last_refill >= 0.5:
                for _ in range(min(3, target - len(owned))):
                    if not spawn_one():
                        break
                last_refill = now

            if now - last_report >= 2.0:
                current = {a.id: a for a in world.get_actors().filter("vehicle.*")}
                alive = [current[aid] for aid in owned if aid in current]
                counts = _bin_counts(alive, center, [30.0, 50.0, 80.0])
                within = sum(counts)
                print("BENCH Truth-like density | alive:%d/%d | <=80m:%d | 00-30:%d 30-50:%d 50-80:%d | recycled:%d" %
                      (len(alive), target, within, counts[0], counts[1], counts[2], len(outside)))
                last_report = now
            time.sleep(0.2)

    except KeyboardInterrupt:
        _STOP = True
    finally:
        print("Destroying %d benchmark vehicles..." % len(owned))
        try:
            _destroy(client, list(owned))
        except Exception as exc:
            print("Cleanup warning: %s" % exc)
        print("Junction benchmark stopped cleanly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
