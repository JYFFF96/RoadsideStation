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
            if not pair:
                continue
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
        score = len(bins) * 1000.0 + min(area, 500.0)
        seen[j.id] = (score, j, len(bins), area)
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
        index = int(sc.get("junction_index", 0)) % len(pool)
        _, junction, dirs, area = pool[index]
        c = junction.bounding_box.location
        print("RSU junction id=%s directions=%d area=%.1f center=(%.2f, %.2f)" %
              (junction.id, dirs, area, c.x, c.y))
        return carla.Location(x=c.x, y=c.y, z=c.z)
    t = sc.get("transform", {})
    return carla.Location(x=float(t.get("x", 0)), y=float(t.get("y", 0)), z=float(t.get("z", 0)))


def _get_tm(client, preferred):
    last = None
    for port in [preferred] + [p for p in range(8000, 8011) if p != preferred]:
        try:
            return client.get_trafficmanager(port), port
        except RuntimeError as exc:
            last = exc
            print("Traffic Manager port %d unavailable: %s" % (port, exc))
    raise RuntimeError("No Traffic Manager port available: %s" % last)


def _blueprints(world):
    out = []
    for bp in world.get_blueprint_library().filter("vehicle.*"):
        try:
            if bp.has_attribute("number_of_wheels") and int(bp.get_attribute("number_of_wheels").as_int()) != 4:
                continue
            bid = bp.id.lower()
            if any(x in bid for x in ("carlacola", "cybertruck", "t2", "sprinter")):
                continue
            out.append(bp)
        except Exception:
            pass
    return out


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
        bp.set_attribute("role_name", "rsu_local_autopilot")
    return bp


def _distance2d(a, b):
    return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))


def _destroy(client, ids):
    if not ids:
        return
    client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], True)


def main():
    global _STOP
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    ap = argparse.ArgumentParser(description="V0.4.6 RSU local CARLA traffic generator")
    ap.add_argument("--config", default="config/roadside.yaml")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--tm-port", type=int, default=8000)
    ap.add_argument("--vehicles", "-n", type=int, default=30)
    ap.add_argument("--radius", type=float, default=180.0,
                    help="initial spawn radius around the RSU junction in metres")
    ap.add_argument("--recycle-radius", type=float, default=230.0,
                    help="destroy/recycle owned vehicles after they leave this radius")
    ap.add_argument("--speed-diff", type=float, default=30.0)
    ap.add_argument("--distance", type=float, default=3.0)
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
        center = _resolve_rsu_center(world_map, cfg)
        print("RoadsideStation V0.4.6 RSU Local Traffic Generator")
        print("CARLA map: %s" % world_map.name.split("/")[-1])
        print("Local traffic center: (%.2f, %.2f), spawn radius=%.1fm, recycle radius=%.1fm" %
              (center.x, center.y, args.radius, args.recycle_radius))

        if not args.keep_existing:
            existing = list(world.get_actors().filter("vehicle.*"))
            if existing:
                _destroy(client, [a.id for a in existing])
                print("Removed %d existing/stale vehicles." % len(existing))
                time.sleep(0.5)

        tm, tm_port = _get_tm(client, args.tm_port)
        tm.set_global_distance_to_leading_vehicle(args.distance)
        tm.global_percentage_speed_difference(args.speed_diff)
        tm.set_hybrid_physics_mode(False)
        try:
            tm.set_random_device_seed(args.seed)
        except Exception:
            pass
        print("Traffic Manager connected on port %d" % tm_port)
        print("Routing remains 100%% CARLA Traffic Manager; this script never calls set_path().")

        bps = _blueprints(world)
        all_spawns = list(world_map.get_spawn_points())
        local_spawns = [sp for sp in all_spawns if _distance2d(sp.location, center) <= args.radius]
        local_spawns.sort(key=lambda sp: _distance2d(sp.location, center))
        print("Map spawn points: %d; RSU-local spawn points: %d" % (len(all_spawns), len(local_spawns)))
        if not bps:
            raise RuntimeError("No safe vehicle blueprints")
        if not local_spawns:
            raise RuntimeError("No CARLA spawn points inside %.1fm; increase --radius" % args.radius)

        def spawn_one():
            candidates = list(local_spawns)
            rng.shuffle(candidates)
            actors = world.get_actors()
            occupied = [a.get_location() for a in actors.filter("vehicle.*")]
            for sp in candidates:
                if any(_distance2d(sp.location, p) < 8.0 for p in occupied):
                    continue
                bp = _prepare(rng.choice(bps), rng)
                try:
                    actor = world.try_spawn_actor(bp, sp)
                except Exception:
                    actor = None
                if actor is None:
                    continue
                actor.set_autopilot(True, tm_port)
                owned.add(actor.id)
                return True
            return False

        # Initial fill. More vehicles than local spawn points cannot exist at once
        # without unsafe overlap, so the actual count may be below the requested target.
        attempts = 0
        while len(owned) < args.vehicles and attempts < max(50, args.vehicles * 8):
            attempts += 1
            if not spawn_one():
                time.sleep(0.05)
        print("Initial local traffic: %d/%d vehicles." % (len(owned), args.vehicles))
        print("Auto-recycle: ON. Vehicles leaving %.1fm are destroyed and replenished locally." % args.recycle_radius)
        print("Press Ctrl+C to stop and remove vehicles owned by this script.")

        last_report = 0.0
        while not _STOP:
            actors = {a.id: a for a in world.get_actors().filter("vehicle.*")}
            gone = []
            outside = []
            for aid in list(owned):
                actor = actors.get(aid)
                if actor is None:
                    gone.append(aid)
                elif _distance2d(actor.get_location(), center) > args.recycle_radius:
                    outside.append(aid)
            for aid in gone:
                owned.discard(aid)
            if outside:
                _destroy(client, outside)
                for aid in outside:
                    owned.discard(aid)

            # Refill gradually; Traffic Manager still owns all routing decisions.
            missing = max(0, args.vehicles - len(owned))
            for _ in range(min(missing, 5)):
                if not spawn_one():
                    break

            now = time.time()
            if now - last_report >= 2.0:
                inside_sensor = 0
                for aid in owned:
                    actor = actors.get(aid)
                    if actor is not None and _distance2d(actor.get_location(), center) <= 80.0:
                        inside_sensor += 1
                print("Local traffic: %d/%d | within 80m: %d | recycled this cycle: %d" %
                      (len(owned), args.vehicles, inside_sensor, len(outside)))
                last_report = now
            time.sleep(0.25)

    except KeyboardInterrupt:
        _STOP = True
    finally:
        print("Destroying %d RSU-local vehicles..." % len(owned))
        try:
            _destroy(client, list(owned))
        except Exception as exc:
            print("Cleanup warning: %s" % exc)
        print("RSU local traffic stopped cleanly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
