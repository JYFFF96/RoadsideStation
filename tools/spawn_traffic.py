from __future__ import print_function
import argparse, math, random, signal, sys, time
import carla
_STOP_REQUESTED=False

def _request_stop(signum,frame):
    global _STOP_REQUESTED
    if not _STOP_REQUESTED: print("\nStop requested. Cleaning up...")
    _STOP_REQUESTED=True

def _get_tm(client,p):
    last=None
    for port in [p]+[x for x in range(8000,8011) if x!=p]:
        try: return client.get_trafficmanager(port),port
        except RuntimeError as e: last=e; print("Traffic Manager port %d unavailable: %s"%(port,e))
    raise RuntimeError("No free Traffic Manager port: %s"%last)

def _blueprints(world):
    result=[]
    for bp in world.get_blueprint_library().filter("vehicle.*"):
        try:
            if bp.has_attribute("number_of_wheels") and int(bp.get_attribute("number_of_wheels").as_int())==4: result.append(bp)
        except Exception: pass
    return result

def _dist(a,b): return math.hypot(a.location.x-b.location.x,a.location.y-b.location.y)
def _dist_xy(t,x,y): return math.hypot(t.location.x-x,t.location.y-y)

def _spawn_points(world,spacing,near_rsu=False,rsu_x=-27.01,rsu_y=61.21,radius=250.0):
    wm=world.get_map(); candidates=[]
    for raw in wm.get_spawn_points():
        wp=wm.get_waypoint(raw.location,project_to_road=True,lane_type=carla.LaneType.Driving)
        if wp is None or wp.is_junction: continue
        t=wp.transform; t.location.z+=0.35
        if near_rsu and _dist_xy(t,rsu_x,rsu_y)>radius: continue
        candidates.append(t)
    # Local mode prioritizes the closest usable lanes, so traffic density is
    # concentrated around the RSU instead of spread across all Town10HD.
    if near_rsu:
        candidates.sort(key=lambda t:_dist_xy(t,rsu_x,rsu_y))
        # add a little deterministic variation among similarly close points
        bands=[]
        for i in range(0,len(candidates),8):
            band=candidates[i:i+8]; random.shuffle(band); bands.extend(band)
        candidates=bands
    else: random.shuffle(candidates)
    selected=[]
    for t in candidates:
        if all(_dist(t,o)>=spacing for o in selected): selected.append(t)
    return selected

def _cleanup_existing(client,world):
    vehicles=list(world.get_actors().filter("vehicle.*"))
    if vehicles: client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in vehicles],True)
    return len(vehicles)

def _destroy(client,actors):
    ids=[]
    for a in actors:
        try:
            if a and a.is_alive: ids.append(a.id)
        except Exception: pass
    if not ids: print("No spawned vehicles need cleanup."); return
    print("Destroying %d vehicles..."%len(ids))
    try: client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids],True); print("Cleanup complete.")
    except Exception as e: print("Cleanup failed: %s"%e)

def main():
    global _STOP_REQUESTED
    signal.signal(signal.SIGINT,_request_stop); signal.signal(signal.SIGTERM,_request_stop)
    p=argparse.ArgumentParser(description="RoadsideStation CARLA traffic generator")
    p.add_argument("--host",default="127.0.0.1"); p.add_argument("--port",type=int,default=2000)
    p.add_argument("--vehicles",type=int,default=20); p.add_argument("--tm-port",type=int,default=8000)
    p.add_argument("--seed",type=int,default=42); p.add_argument("--speed-diff",type=float,default=30.0)
    p.add_argument("--min-spacing",type=float,default=15.0); p.add_argument("--keep-existing",action="store_true")
    p.add_argument("--near-rsu",action="store_true",help="concentrate spawns around the RoadsideStation RSU")
    p.add_argument("--rsu-x",type=float,default=-27.01); p.add_argument("--rsu-y",type=float,default=61.21)
    p.add_argument("--radius",type=float,default=250.0,help="local spawn radius around RSU in metres")
    args=p.parse_args(); random.seed(args.seed)
    client=carla.Client(args.host,args.port); client.set_timeout(10.0); actors=[]
    try:
        world=client.get_world()
        if not args.keep_existing:
            n=_cleanup_existing(client,world)
            if n: print("Removed %d existing/stale vehicles."%n); time.sleep(.5)
        tm,tm_port=_get_tm(client,args.tm_port); tm.set_global_distance_to_leading_vehicle(5.0)
        tm.global_percentage_speed_difference(args.speed_diff); tm.set_hybrid_physics_mode(False)
        try: tm.set_random_device_seed(args.seed)
        except Exception: pass
        print("Traffic Manager connected on port %d"%tm_port)
        points=_spawn_points(world,args.min_spacing,args.near_rsu,args.rsu_x,args.rsu_y,args.radius)
        bps=_blueprints(world)
        if not points: raise RuntimeError("No suitable spawn points found; increase --radius or reduce --min-spacing")
        requested=min(args.vehicles,len(points))
        for t in points:
            if _STOP_REQUESTED or len(actors)>=requested: break
            bp=random.choice(bps)
            if bp.has_attribute("role_name"): bp.set_attribute("role_name","roadside_test")
            a=world.try_spawn_actor(bp,t)
            if not a: continue
            actors.append(a); a.set_autopilot(True,tm_port)
            try:
                tm.auto_lane_change(a,False); tm.distance_to_leading_vehicle(a,5.0); tm.vehicle_percentage_speed_difference(a,args.speed_diff)
            except Exception: pass
        map_name=world.get_map().name.split("/")[-1]
        print("Spawned %d vehicles on %s."%(len(actors),map_name))
        if args.near_rsu:
            print("LOCAL RSU mode: center=(%.2f, %.2f), radius=%.0fm, candidates=%d."%(args.rsu_x,args.rsu_y,args.radius,len(points)))
        print("Press Ctrl+C once to stop and remove all spawned vehicles.")
        while not _STOP_REQUESTED: time.sleep(.1)
    except KeyboardInterrupt: _STOP_REQUESTED=True
    finally: _destroy(client,actors)
    print("Traffic generator stopped cleanly."); return 0
if __name__=="__main__": sys.exit(main())
