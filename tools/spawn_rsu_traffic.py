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


def _get_tm(client, preferred):
    last = None
    for port in [preferred] + [p for p in range(8000, 8011) if p != preferred]:
        try:
            return client.get_trafficmanager(port), port
        except RuntimeError as exc:
            last = exc
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
    if ids:
        client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], True)


def _heading_group(yaw):
    # Four broad approach directions. Opposite lanes remain separate groups.
    return int(((float(yaw) % 360.0) + 45.0) // 90.0) % 4


def main():
    global _STOP
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    ap = argparse.ArgumentParser(description="V0.4.7 stable RSU intersection traffic generator")
    ap.add_argument("--config", default="config/roadside.yaml")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--tm-port", type=int, default=8000)
    ap.add_argument("--vehicles", "-n", type=int, default=32,
                    help="total locally owned vehicles")
    ap.add_argument("--target-80m", type=int, default=14,
                    help="desired minimum vehicle count inside 80m")
    ap.add_argument("--inner-radius", type=float, default=80.0)
    ap.add_argument("--entry-min", type=float, default=35.0)
    ap.add_argument("--entry-max", type=float, default=115.0)
    ap.add_argument("--recycle-radius", type=float, default=150.0)
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
        print("RoadsideStation V0.4.7 Stable Intersection Traffic")
        print("CARLA map: %s" % world_map.name.split("/")[-1])
        print("Target: >=%d vehicles inside %.0fm; entry band %.0f-%.0fm; recycle %.0fm" %
              (args.target_80m, args.inner_radius, args.entry_min, args.entry_max, args.recycle_radius))

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
        print("Routing: CARLA Traffic Manager only (no set_path / no custom steering).")

        bps = _blueprints(world)
        all_spawns = list(world_map.get_spawn_points())
        entry_spawns = []
        inner_spawns = []
        groups = {0: [], 1: [], 2: [], 3: []}
        for sp in all_spawns:
            d = _distance2d(sp.location, center)
            if d <= args.inner_radius:
                inner_spawns.append(sp)
            if args.entry_min <= d <= args.entry_max:
                entry_spawns.append(sp)
                groups[_heading_group(sp.rotation.yaw)].append(sp)
        active_groups = [g for g in groups if groups[g]]
        print("Spawn points: map=%d inner=%d entry=%d direction-groups=%d" %
              (len(all_spawns), len(inner_spawns), len(entry_spawns), len(active_groups)))
        for g in active_groups:
            print("  approach group %d: %d spawn points" % (g, len(groups[g])))
        if not bps or not entry_spawns:
            raise RuntimeError("Insufficient vehicle blueprints or entry spawn points")

        group_cursor = [0]

        def spawn_one(prefer_inner=False):
            actors_now = world.get_actors().filter("vehicle.*")
            occupied = [a.get_location() for a in actors_now]
            candidates = []
            # Density recovery first tries legal spawn points already inside 80m.
            if prefer_inner and inner_spawns:
                candidates.extend(inner_spawns)
            # Then rotate through approach directions to avoid feeding only one road.
            if active_groups:
                start = group_cursor[0] % len(active_groups)
                for off in range(len(active_groups)):
                    g = active_groups[(start + off) % len(active_groups)]
                    pool = list(groups[g])
                    rng.shuffle(pool)
                    candidates.extend(pool)
                group_cursor[0] += 1
            else:
                candidates.extend(entry_spawns)
            seen = set()
            for sp in candidates:
                key = (round(sp.location.x, 1), round(sp.location.y, 1), round(sp.rotation.yaw, 1))
                if key in seen:
                    continue
                seen.add(key)
                # Conservative separation prevents spawn collisions/traffic chaos.
                if any(_distance2d(sp.location, p) < 9.0 for p in occupied):
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
                return actor.id
            return None

        # Fill the local population without overlapping vehicles.
        attempts = 0
        while len(owned) < args.vehicles and attempts < args.vehicles * 12:
            attempts += 1
            if spawn_one(prefer_inner=(len(owned) < args.target_80m)) is None:
                time.sleep(0.05)
        print("Initial traffic: %d/%d" % (len(owned), args.vehicles))
        print("Density controller ON. Low 80m density triggers local legal-spawn replenishment.")
        print("Press Ctrl+C to stop and remove vehicles owned by this script.")

        last_report = 0.0
        last_refill = 0.0
        while not _STOP:
            actor_list = world.get_actors().filter("vehicle.*")
            actors = {a.id: a for a in actor_list}
            gone = []
            outside = []
            inside = []
            for aid in list(owned):
                actor = actors.get(aid)
                if actor is None:
                    gone.append(aid)
                    continue
                d = _distance2d(actor.get_location(), center)
                if d > args.recycle_radius:
                    outside.append(aid)
                elif d <= args.inner_radius:
                    inside.append(aid)
            for aid in gone:
                owned.discard(aid)
            if outside:
                _destroy(client, outside)
                for aid in outside:
                    owned.discard(aid)
                time.sleep(0.05)

            now = time.time()
            deficit = max(0, args.target_80m - len(inside))
            total_missing = max(0, args.vehicles - len(owned))
            spawned = 0
            # Rate limit replenishment. This avoids suddenly dumping many cars into
            # the junction while still recovering density within a few seconds.
            if now - last_refill >= 0.75:
                budget = min(3, max(deficit, total_missing))
                for _ in range(budget):
                    if len(owned) >= args.vehicles:
                        break
                    aid = spawn_one(prefer_inner=(deficit > 0))
                    if aid is None:
                        break
                    spawned += 1
                if budget:
                    last_refill = now

            if now - last_report >= 2.0:
                # Re-read after spawn/destroy for truthful reporting.
                current = {a.id: a for a in world.get_actors().filter("vehicle.*")}
                inside_now = 0
                bands = [0, 0, 0]
                for aid in owned:
                    a = current.get(aid)
                    if a is None:
                        continue
                    d = _distance2d(a.get_location(), center)
                    if d <= args.inner_radius:
                        inside_now += 1
                        bands[0] += 1
                    elif d <= args.entry_max:
                        bands[1] += 1
                    else:
                        bands[2] += 1
                state = "OK" if inside_now >= args.target_80m else "REFILL"
                print("Traffic %s | total:%d/%d | within %.0fm:%d target:%d | 80-%.0fm:%d | outer:%d | recycled:%d spawned:%d" %
                      (state, len(owned), args.vehicles, args.inner_radius, inside_now,
                       args.target_80m, args.entry_max, bands[1], bands[2], len(outside), spawned))
                last_report = now
            time.sleep(0.20)

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
