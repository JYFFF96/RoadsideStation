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
        try:return client.get_trafficmanager(port),port
        except RuntimeError as e:last=e;print("Traffic Manager port %d unavailable: %s"%(port,e))
    raise RuntimeError("No free Traffic Manager port: %s"%last)

def _blueprints(world):
    out=[]
    for bp in world.get_blueprint_library().filter("vehicle.*"):
        try:
            if bp.has_attribute("number_of_wheels") and int(bp.get_attribute("number_of_wheels").as_int())==4:out.append(bp)
        except Exception:pass
    return out

def _dist(a,b):return math.hypot(a.location.x-b.location.x,a.location.y-b.location.y)
def _dist_xy(t,x,y):return math.hypot(t.location.x-x,t.location.y-y)
def _angle_diff(a,b):return abs((float(a)-float(b)+180.0)%360.0-180.0)

def _find_cross_center(world):
    wm=world.get_map();seen={}
    for wp in wm.generate_waypoints(2.0):
        if not wp.is_junction:continue
        j=wp.get_junction()
        if j is None or j.id in seen:continue
        try:pairs=j.get_waypoints(carla.LaneType.Driving)
        except Exception:pairs=[]
        headings=[]
        for pair in pairs:
            try:headings.append(float(pair[0].transform.rotation.yaw)%360.0)
            except Exception:pass
        bins=[]
        for h in headings:
            if all(_angle_diff(h,b)>35.0 for b in bins):bins.append(h)
        area=max(1.0,j.bounding_box.extent.x*2.0)*max(1.0,j.bounding_box.extent.y*2.0)
        seen[j.id]=(len(bins)*1000.0+min(area,500.0),j,len(bins))
    vals=list(seen.values());vals.sort(key=lambda x:x[0],reverse=True)
    if not vals:raise RuntimeError("No junction found")
    pool=[x for x in vals if x[2]>=4] or vals
    j=pool[0][1];c=j.bounding_box.location
    return float(c.x),float(c.y),j.id

def _spawn_points(world,spacing,cx,cy,radius):
    wm=world.get_map();cand=[]
    for raw in wm.get_spawn_points():
        wp=wm.get_waypoint(raw.location,project_to_road=True,lane_type=carla.LaneType.Driving)
        if wp is None or wp.is_junction:continue
        t=wp.transform;t.location.z+=.35
        d=_dist_xy(t,cx,cy)
        if d>radius or d<35.0:continue
        # Prefer approaches that are close enough to reach the selected junction.
        cand.append((d,t,wp))
    cand.sort(key=lambda x:x[0]);selected=[]
    for d,t,wp in cand:
        if all(_dist(t,o[1])>=spacing for o in selected):selected.append((d,t,wp))
    return selected

def _choose_straight(options,current_yaw):
    if not options:return None
    return min(options,key=lambda w:_angle_diff(w.transform.rotation.yaw,current_yaw))

def _make_straight_path(wp,step=4.0,max_distance=220.0):
    """Build a lane-following path, choosing the smallest yaw change at branches/junctions."""
    path=[];cur=wp;travel=0.0;last_yaw=float(cur.transform.rotation.yaw)
    while travel<max_distance:
        opts=cur.next(step)
        nxt=_choose_straight(opts,last_yaw)
        if nxt is None:break
        path.append(carla.Location(x=nxt.transform.location.x,y=nxt.transform.location.y,z=nxt.transform.location.z))
        last_yaw=float(nxt.transform.rotation.yaw);cur=nxt;travel+=step
    return path

def _cleanup_existing(client,world):
    vehicles=list(world.get_actors().filter("vehicle.*"))
    if vehicles:client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in vehicles],True)
    return len(vehicles)

def _destroy(client,actors):
    ids=[]
    for a in actors:
        try:
            if a and a.is_alive:ids.append(a.id)
        except Exception:pass
    if not ids:print("No spawned vehicles need cleanup.");return
    print("Destroying %d vehicles..."%len(ids))
    try:client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids],True);print("Cleanup complete.")
    except Exception as e:print("Cleanup failed: %s"%e)

def main():
    global _STOP_REQUESTED
    signal.signal(signal.SIGINT,_request_stop);signal.signal(signal.SIGTERM,_request_stop)
    p=argparse.ArgumentParser(description="RoadsideStation fixed-route CARLA traffic generator")
    p.add_argument("--host",default="127.0.0.1");p.add_argument("--port",type=int,default=2000);p.add_argument("--vehicles",type=int,default=24);p.add_argument("--tm-port",type=int,default=8000)
    p.add_argument("--seed",type=int,default=42);p.add_argument("--speed-diff",type=float,default=35.0);p.add_argument("--min-spacing",type=float,default=18.0);p.add_argument("--radius",type=float,default=180.0);p.add_argument("--keep-existing",action="store_true")
    p.add_argument("--fixed-routes",action="store_true",default=True,help="use Traffic Manager set_path() with straight lane-following routes")
    args=p.parse_args();random.seed(args.seed);client=carla.Client(args.host,args.port);client.set_timeout(10.0);actors=[]
    try:
        world=client.get_world();cx,cy,jid=_find_cross_center(world);print("Selected traffic junction id=%s center=(%.2f, %.2f)"%(jid,cx,cy))
        if not args.keep_existing:
            n=_cleanup_existing(client,world)
            if n:print("Removed %d existing/stale vehicles."%n);time.sleep(.5)
        tm,tm_port=_get_tm(client,args.tm_port);tm.set_global_distance_to_leading_vehicle(8.0);tm.global_percentage_speed_difference(args.speed_diff);tm.set_hybrid_physics_mode(False)
        try:tm.set_random_device_seed(args.seed)
        except Exception:pass
        points=_spawn_points(world,args.min_spacing,cx,cy,args.radius);bps=_blueprints(world)
        if not points:raise RuntimeError("No suitable approach spawn points found")
        requested=min(args.vehicles,len(points))
        for _,t,wp in points:
            if _STOP_REQUESTED or len(actors)>=requested:break
            bp=random.choice(bps)
            if bp.has_attribute("role_name"):bp.set_attribute("role_name","roadside_route_test")
            a=world.try_spawn_actor(bp,t)
            if not a:continue
            actors.append(a);a.set_autopilot(True,tm_port)
            try:
                tm.auto_lane_change(a,False);tm.random_left_lanechange_percentage(a,0.0);tm.random_right_lanechange_percentage(a,0.0);tm.distance_to_leading_vehicle(a,8.0);tm.vehicle_percentage_speed_difference(a,args.speed_diff)
                route=_make_straight_path(wp)
                if route:tm.set_path(a,route)
            except Exception as exc:print("Route setup warning for actor %s: %s"%(a.id,exc))
        print("Spawned %d fixed-route vehicles on %s."%(len(actors),world.get_map().name.split("/")[-1]));print("Routes follow lane waypoints and choose the straightest branch at junctions.");print("Press Ctrl+C to stop and remove all spawned vehicles.")
        while not _STOP_REQUESTED:time.sleep(.1)
    except KeyboardInterrupt:_STOP_REQUESTED=True
    finally:_destroy(client,actors)
    print("Traffic generator stopped cleanly.");return 0
if __name__=="__main__":sys.exit(main())
