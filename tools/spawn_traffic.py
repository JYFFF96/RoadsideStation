from __future__ import print_function
import argparse, math, random, signal, sys, time
import carla

_STOP_REQUESTED = False


def _request_stop(signum, frame):
    global _STOP_REQUESTED
    if not _STOP_REQUESTED:
        print("\nStop requested. Cleaning up...")
    _STOP_REQUESTED = True


def _angle_diff(a, b):
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _get_tm(client, preferred):
    last = None
    for port in [preferred] + [x for x in range(8000, 8011) if x != preferred]:
        try:
            return client.get_trafficmanager(port), port
        except RuntimeError as exc:
            last = exc
            print("Traffic Manager port %d unavailable: %s" % (port, exc))
    raise RuntimeError("No free Traffic Manager port: %s" % last)


def _blueprints(world):
    out = []
    for bp in world.get_blueprint_library().filter("vehicle.*"):
        try:
            if bp.has_attribute("number_of_wheels") and int(bp.get_attribute("number_of_wheels").as_int()) == 4:
                out.append(bp)
        except Exception:
            pass
    return out


def _find_cross_junction(world):
    candidates = []
    seen = set()
    for wp in world.get_map().generate_waypoints(2.0):
        if not wp.is_junction:
            continue
        junction = wp.get_junction()
        if junction is None or junction.id in seen:
            continue
        seen.add(junction.id)
        try:
            pairs = list(junction.get_waypoints(carla.LaneType.Driving))
        except Exception:
            pairs = []
        headings = []
        for entry, _ in pairs:
            yaw = float(entry.transform.rotation.yaw) % 360.0
            if all(_angle_diff(yaw, h) > 35.0 for h in headings):
                headings.append(yaw)
        area = max(1.0, junction.bounding_box.extent.x * 2.0) * max(1.0, junction.bounding_box.extent.y * 2.0)
        score = len(headings) * 1000.0 + min(area, 500.0)
        candidates.append((score, len(headings), junction, pairs))
    if not candidates:
        raise RuntimeError("No junction found")
    candidates.sort(key=lambda x: x[0], reverse=True)
    pool = [x for x in candidates if x[1] >= 4] or candidates
    _, directions, junction, pairs = pool[0]
    center = junction.bounding_box.location
    print("Selected traffic junction id=%s center=(%.2f, %.2f) directions=%d" %
          (junction.id, center.x, center.y, directions))
    return junction, pairs


def _pick_straight(options, reference_yaw):
    if not options:
        return None
    return min(options, key=lambda w: _angle_diff(w.transform.rotation.yaw, reference_yaw))


def _walk_previous(wp, distance, step=4.0):
    cur = wp
    travelled = 0.0
    ref_yaw = float(wp.transform.rotation.yaw)
    visited = set()
    while travelled < distance:
        key = (cur.road_id, cur.section_id, cur.lane_id, int(cur.s * 2.0))
        if key in visited:
            break
        visited.add(key)
        options = cur.previous(min(step, distance - travelled))
        nxt = _pick_straight(options, ref_yaw)
        if nxt is None:
            break
        cur = nxt
        ref_yaw = float(cur.transform.rotation.yaw)
        travelled += step
    return cur


def _walk_next(wp, distance, step=4.0):
    cur = wp
    travelled = 0.0
    ref_yaw = float(wp.transform.rotation.yaw)
    visited = set()
    path = []
    while travelled < distance:
        key = (cur.road_id, cur.section_id, cur.lane_id, int(cur.s * 2.0))
        if key in visited:
            break
        visited.add(key)
        options = cur.next(min(step, distance - travelled))
        nxt = _pick_straight(options, ref_yaw)
        if nxt is None:
            break
        cur = nxt
        ref_yaw = float(cur.transform.rotation.yaw)
        path.append(carla.Location(x=cur.transform.location.x,
                                   y=cur.transform.location.y,
                                   z=cur.transform.location.z))
        travelled += step
    return cur, path


def _build_straight_routes(junction_pairs, approach_distance=65.0, exit_distance=90.0):
    """Create explicit lane-to-lane straight-through routes across one junction.

    Junction.get_waypoints() gives the exact entry/exit lane connection.  We only
    keep pairs whose heading changes little, so Traffic Manager never needs to
    choose a branch while inside the junction.
    """
    routes = []
    used = set()
    for entry, exit_wp in junction_pairs:
        in_yaw = float(entry.transform.rotation.yaw)
        out_yaw = float(exit_wp.transform.rotation.yaw)
        if _angle_diff(in_yaw, out_yaw) > 35.0:
            continue
        key = (entry.road_id, entry.lane_id, exit_wp.road_id, exit_wp.lane_id)
        if key in used:
            continue
        used.add(key)
        spawn_wp = _walk_previous(entry, approach_distance)
        _, downstream = _walk_next(exit_wp, exit_distance)
        path = [carla.Location(x=entry.transform.location.x,
                               y=entry.transform.location.y,
                               z=entry.transform.location.z),
                carla.Location(x=exit_wp.transform.location.x,
                               y=exit_wp.transform.location.y,
                               z=exit_wp.transform.location.z)] + downstream
        routes.append({"spawn_wp": spawn_wp, "entry": entry, "exit": exit_wp, "path": path})
    return routes


def _spawn_offsets(route, count, spacing=22.0):
    """Return several spawn waypoints upstream on one exact lane."""
    result = []
    base = route["spawn_wp"]
    for i in range(count):
        wp = _walk_previous(base, i * spacing)
        if wp is None or wp.is_junction:
            continue
        result.append(wp)
    return result


def _cleanup_existing(client, world):
    vehicles = list(world.get_actors().filter("vehicle.*"))
    if vehicles:
        client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in vehicles], True)
    return len(vehicles)


def _destroy(client, actors):
    ids = []
    for actor in actors:
        try:
            if actor and actor.is_alive:
                ids.append(actor.id)
        except Exception:
            pass
    if not ids:
        print("No spawned vehicles need cleanup.")
        return
    print("Destroying %d vehicles..." % len(ids))
    try:
        client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], True)
        print("Cleanup complete.")
    except Exception as exc:
        print("Cleanup failed: %s" % exc)


def main():
    global _STOP_REQUESTED
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    parser = argparse.ArgumentParser(description="RoadsideStation explicit straight-route CARLA traffic generator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--vehicles", type=int, default=24)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--speed-diff", type=float, default=40.0)
    parser.add_argument("--spacing", type=float, default=24.0)
    parser.add_argument("--keep-existing", action="store_true")
    args = parser.parse_args()

    random.seed(args.seed)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    actors = []

    try:
        world = client.get_world()
        junction, pairs = _find_cross_junction(world)
        routes = _build_straight_routes(pairs)
        if not routes:
            raise RuntimeError("No straight-through lane connections found at selected junction")
        print("Built %d explicit straight-through lane routes." % len(routes))
        for idx, route in enumerate(routes):
            print("  Route %d: road/lane %s/%s -> %s/%s, path points=%d" %
                  (idx + 1, route["entry"].road_id, route["entry"].lane_id,
                   route["exit"].road_id, route["exit"].lane_id, len(route["path"])))

        if not args.keep_existing:
            removed = _cleanup_existing(client, world)
            if removed:
                print("Removed %d existing/stale vehicles." % removed)
                time.sleep(.5)

        tm, tm_port = _get_tm(client, args.tm_port)
        tm.set_global_distance_to_leading_vehicle(8.0)
        tm.global_percentage_speed_difference(args.speed_diff)
        tm.set_hybrid_physics_mode(False)
        try:
            tm.set_random_device_seed(args.seed)
        except Exception:
            pass

        bps = _blueprints(world)
        if not bps:
            raise RuntimeError("No four-wheel vehicle blueprints found")

        per_route = max(1, int(math.ceil(float(args.vehicles) / len(routes))))
        spawn_jobs = []
        for route_index, route in enumerate(routes):
            for wp in _spawn_offsets(route, per_route, args.spacing):
                spawn_jobs.append((route_index, route, wp))
        random.shuffle(spawn_jobs)

        for route_index, route, wp in spawn_jobs:
            if _STOP_REQUESTED or len(actors) >= args.vehicles:
                break
            transform = wp.transform
            transform.location.z += 0.35
            bp = random.choice(bps)
            if bp.has_attribute("role_name"):
                bp.set_attribute("role_name", "roadside_route_%02d" % (route_index + 1))
            actor = world.try_spawn_actor(bp, transform)
            if actor is None:
                continue
            actors.append(actor)
            actor.set_autopilot(True, tm_port)
            try:
                tm.auto_lane_change(actor, False)
                tm.random_left_lanechange_percentage(actor, 0.0)
                tm.random_right_lanechange_percentage(actor, 0.0)
                tm.distance_to_leading_vehicle(actor, 8.0)
                tm.vehicle_percentage_speed_difference(actor, args.speed_diff)
                tm.set_path(actor, route["path"])
            except Exception as exc:
                print("Route setup warning for actor %s: %s" % (actor.id, exc))

        print("Spawned %d explicit-route vehicles on %s." %
              (len(actors), world.get_map().name.split("/")[-1]))
        print("Only straight junction lane connections are used; no branch selection occurs inside the junction.")
        print("Press Ctrl+C to stop and remove all spawned vehicles.")
        while not _STOP_REQUESTED:
            time.sleep(.1)
    except KeyboardInterrupt:
        _STOP_REQUESTED = True
    finally:
        _destroy(client, actors)
    print("Traffic generator stopped cleanly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
