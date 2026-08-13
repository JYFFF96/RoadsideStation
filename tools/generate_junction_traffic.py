from __future__ import print_function

# Stable CARLA traffic benchmark for RoadsideStation perception evaluation.
# Traffic Manager/autopilot only: no custom steering or path override.
# V0.5.8 adds deterministic sync ticking, safer spawning and traffic health stats.

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


def _speed(actor):
    try:
        v = actor.get_velocity()
        return math.sqrt(float(v.x) * float(v.x) + float(v.y) * float(v.y) + float(v.z) * float(v.z))
    except Exception:
        return 0.0


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


def _destroy(client, ids, do_tick=False):
    if ids:
        client.apply_batch_sync([carla.command.DestroyActor(x) for x in ids], do_tick)


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


def _is_safe_spawn(world_map, sp, center, safe_radius, spawn_radius):
    d = _distance2d(sp.location, center)
    if d < safe_radius or d > spawn_radius:
        return False
    try:
        wp = world_map.get_waypoint(sp.location, project_to_road=False, lane_type=carla.LaneType.Driving)
        if wp is None or wp.is_junction:
            return False
    except Exception:
        return False
    return True


def _configure_vehicle_tm(tm, actor, following_distance, speed_diff):
    # Keep all rules scalar/explicit so the same policy is easy to port to Qt/C++ later.
    try:
        tm.distance_to_leading_vehicle(actor, float(following_distance))
    except Exception:
        pass
    try:
        tm.vehicle_percentage_speed_difference(actor, float(speed_diff))
    except Exception:
        pass
    try:
        tm.ignore_lights_percentage(actor, 0.0)
    except Exception:
        pass
    try:
        tm.ignore_signs_percentage(actor, 0.0)
    except Exception:
        pass
    try:
        tm.auto_lane_change(actor, False)
    except Exception:
        pass
    try:
        tm.random_left_lanechange_percentage(actor, 0.0)
        tm.random_right_lanechange_percentage(actor, 0.0)
    except Exception:
        pass


def _attach_collision_sensor(world, actor, collision_sensors, health, pair_last_time):
    try:
        bp = world.get_blueprint_library().find("sensor.other.collision")
        sensor = world.spawn_actor(bp, carla.Transform(), attach_to=actor)

        def _on_collision(event):
            try:
                other_id = int(event.other_actor.id) if event.other_actor is not None else -1
            except Exception:
                other_id = -1
            a = int(actor.id)
            key = (min(a, other_id), max(a, other_id))
            now = time.time()
            # CARLA may emit repeated contacts for the same physical collision.
            if now - pair_last_time.get(key, 0.0) >= 2.0:
                pair_last_time[key] = now
                health["collisions"] += 1
        sensor.listen(_on_collision)
        collision_sensors[actor.id] = sensor
    except Exception as exc:
        print("Collision sensor warning for vehicle %s: %s" % (actor.id, exc))


def main():
    global _STOP
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    ap = argparse.ArgumentParser(description="Stable RoadsideStation junction traffic benchmark V0.5.8")
    ap.add_argument("--config", default="config/roadside.yaml")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--tm-port", type=int, default=8000)
    ap.add_argument("--mode", choices=("benchmark", "demo", "stress"), default="benchmark")
    ap.add_argument("--vehicles", "-n", type=int, default=None,
                    help="override mode vehicle count")
    ap.add_argument("--spawn-radius", type=float, default=None)
    ap.add_argument("--junction-safe-radius", type=float, default=25.0)
    ap.add_argument("--recycle-radius", type=float, default=None)
    ap.add_argument("--report-radius", type=float, default=80.0)
    ap.add_argument("--following-distance", type=float, default=8.0)
    ap.add_argument("--speed-diff", type=float, default=30.0)
    ap.add_argument("--spawn-clearance", type=float, default=10.0)
    ap.add_argument("--fixed-delta", type=float, default=0.05)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--keep-existing", action="store_true")
    ap.add_argument("--async-mode", action="store_true",
                    help="do not enable synchronous CARLA/TM ticking")
    ap.add_argument("--no-collision-monitor", action="store_true")
    args = ap.parse_args()

    mode_defaults = {
        "benchmark": {"vehicles": 40, "spawn_radius": 115.0},
        "demo": {"vehicles": 80, "spawn_radius": 180.0},
        "stress": {"vehicles": 150, "spawn_radius": 300.0},
    }
    md = mode_defaults[args.mode]
    if args.vehicles is None:
        args.vehicles = md["vehicles"]
    if args.spawn_radius is None:
        args.spawn_radius = md["spawn_radius"]
    if args.recycle_radius is None:
        args.recycle_radius = args.spawn_radius + 25.0

    rng = random.Random(args.seed)
    cfg = _load_config(args.config)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    owned = set()
    collision_sensors = {}
    health = {"collisions": 0}
    pair_last_time = {}
    stop_since = {}
    original_settings = None
    sync_enabled = False
    tm = None

    try:
        world = client.get_world()
        world_map = world.get_map()
        center = _resolve_center(world_map, cfg)
        print("RoadsideStation Junction Traffic Benchmark V0.5.8")
        print("Map:%s | mode:%s | seed:%d" % (world_map.name.split("/")[-1], args.mode, args.seed))
        print("Target vehicles:%d | spawn %.0f..%.0fm | recycle>%.0fm | report<=%.0fm" %
              (args.vehicles, args.junction_safe_radius, args.spawn_radius,
               args.recycle_radius, args.report_radius))
        print("Safety profile: follow=%.1fm speed_diff=%.0f%% spawn_clearance=%.1fm" %
              (args.following_distance, args.speed_diff, args.spawn_clearance))
        print("Traffic Manager autopilot only; no custom steering/path override.")

        if not args.keep_existing:
            existing = list(world.get_actors().filter("vehicle.*"))
            if existing:
                _destroy(client, [a.id for a in existing], False)
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

        if not args.async_mode:
            original_settings = world.get_settings()
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = float(args.fixed_delta)
            world.apply_settings(settings)
            tm.set_synchronous_mode(True)
            sync_enabled = True
            print("CARLA sync: ENABLED | fixed_delta=%.3fs | TrafficManager sync: ENABLED" % args.fixed_delta)
        else:
            print("CARLA sync: disabled by --async-mode")

        blueprints = _safe_blueprints(world)
        local_spawns = [sp for sp in world_map.get_spawn_points()
                        if _is_safe_spawn(world_map, sp, center,
                                          args.junction_safe_radius, args.spawn_radius)]
        local_spawns.sort(key=lambda sp: (_distance2d(sp.location, center),
                                          sp.location.x, sp.location.y))
        if not blueprints:
            raise RuntimeError("No safe vehicle blueprints found")
        if not local_spawns:
            raise RuntimeError("No safe spawn points near selected junction")

        rng.shuffle(local_spawns)
        target = min(args.vehicles, len(local_spawns))
        if target < args.vehicles:
            print("NOTE: requested %d vehicles but only %d safe local spawn slots are available." %
                  (args.vehicles, target))
        print("Safe CARLA spawn slots:%d | benchmark target:%d" % (len(local_spawns), target))

        def destroy_vehicle(aid):
            sensor = collision_sensors.pop(aid, None)
            if sensor is not None:
                try:
                    sensor.stop()
                except Exception:
                    pass
                try:
                    sensor.destroy()
                except Exception:
                    pass
            _destroy(client, [aid], False)
            owned.discard(aid)
            stop_since.pop(aid, None)

        def spawn_one():
            occupied = [a.get_location() for a in world.get_actors().filter("vehicle.*")]
            for sp in local_spawns:
                if any(_distance2d(sp.location, p) < args.spawn_clearance for p in occupied):
                    continue
                bp = _prepare(rng.choice(blueprints), rng)
                actor = world.try_spawn_actor(bp, sp)
                if actor is None:
                    continue
                actor.set_autopilot(True, tm.get_port())
                _configure_vehicle_tm(tm, actor, args.following_distance, args.speed_diff)
                owned.add(actor.id)
                if not args.no_collision_monitor and args.mode != "stress":
                    _attach_collision_sensor(world, actor, collision_sensors, health, pair_last_time)
                return True
            return False

        attempts = 0
        while len(owned) < target and attempts < target * 12:
            attempts += 1
            if not spawn_one():
                if sync_enabled:
                    world.tick()
                else:
                    time.sleep(0.05)
        if sync_enabled:
            world.tick()
        print("Initial spawned: %d/%d" % (len(owned), target))
        print("Press Ctrl+C to stop; benchmark-owned vehicles will be removed and sync mode restored.")

        last_report = 0.0
        last_refill = 0.0
        recycled_total = 0
        while not _STOP:
            if sync_enabled:
                world.tick()
            else:
                time.sleep(0.05)

            actors = {a.id: a for a in world.get_actors().filter("vehicle.*")}
            outside = []
            now = time.time()
            for aid in list(owned):
                actor = actors.get(aid)
                if actor is None:
                    owned.discard(aid)
                    stop_since.pop(aid, None)
                    sensor = collision_sensors.pop(aid, None)
                    if sensor is not None:
                        try:
                            sensor.stop()
                            sensor.destroy()
                        except Exception:
                            pass
                    continue
                if _distance2d(actor.get_location(), center) > args.recycle_radius:
                    outside.append(aid)
                    continue
                if _speed(actor) < 0.2:
                    stop_since.setdefault(aid, now)
                else:
                    stop_since.pop(aid, None)

            for aid in outside:
                destroy_vehicle(aid)
            recycled_total += len(outside)

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
                stopped_now = sum(1 for a in alive if _speed(a) < 0.2)
                stuck_15s = sum(1 for aid in owned if aid in stop_since and now - stop_since[aid] >= 15.0)
                offroad = 0
                for actor in alive:
                    try:
                        wp = world_map.get_waypoint(actor.get_location(), project_to_road=False,
                                                    lane_type=carla.LaneType.Driving)
                        if wp is None:
                            offroad += 1
                    except Exception:
                        pass
                print("BENCH TrafficHealth | alive:%d/%d <=80m:%d | 00-30:%d 30-50:%d 50-80:%d | stopped:%d stuck15s:%d offroad:%d collisions:%d recycled:%d" %
                      (len(alive), target, within, counts[0], counts[1], counts[2],
                       stopped_now, stuck_15s, offroad, health["collisions"], recycled_total))
                last_report = now

    except KeyboardInterrupt:
        _STOP = True
    finally:
        print("Destroying %d benchmark vehicles and %d collision sensors..." %
              (len(owned), len(collision_sensors)))
        for sensor in list(collision_sensors.values()):
            try:
                sensor.stop()
            except Exception:
                pass
            try:
                sensor.destroy()
            except Exception:
                pass
        collision_sensors.clear()
        try:
            _destroy(client, list(owned), False)
        except Exception as exc:
            print("Cleanup warning: %s" % exc)
        if sync_enabled:
            try:
                tm.set_synchronous_mode(False)
            except Exception:
                pass
            if original_settings is not None:
                try:
                    world.apply_settings(original_settings)
                    print("CARLA world settings restored.")
                except Exception as exc:
                    print("World settings restore warning: %s" % exc)
        print("Junction benchmark stopped cleanly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
