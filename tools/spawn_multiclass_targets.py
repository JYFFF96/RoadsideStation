from __future__ import print_function

import argparse
import math
import os
import random
import sys
import time

# Direct execution sets sys.path[0] to tools/. Add the repository root so the
# documented `python3.7 tools/spawn_multiclass_targets.py` command can import
# the roadside package without requiring a separate PYTHONPATH entry for it.
PROJECT_ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:sys.path.insert(0,PROJECT_ROOT)

import carla
import yaml

from roadside.carla_station import CarlaRoadsideStation


def load_config(path):
    with open(path, "r") as fp:return yaml.safe_load(fp)


def distance(a, b):return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))


def junction_center(client, config):
    station = CarlaRoadsideStation(config)
    station.client = client
    station._attach_current_world()
    station._find_junction_transform()
    return station.world, station.world_map, station.junction_center


def unique_locations(locations, minimum_spacing=1.5):
    result=[]
    for loc in locations:
        if loc is not None and all(distance(loc,other)>=minimum_spacing for other in result):result.append(loc)
    return result


def balanced_approach_order(locations, center, rng):
    buckets=[[],[],[],[]]
    for loc in locations:
        angle=math.atan2(float(loc.y)-float(center.y),float(loc.x)-float(center.x))
        buckets[int(((angle+math.pi)/(math.pi/2.0)))%4].append(loc)
    for bucket in buckets:rng.shuffle(bucket)
    ordered=[]
    while any(buckets):
        for bucket in buckets:
            if bucket:ordered.append(bucket.pop())
    return ordered


def opposite_crossing_destination(start, locations, center, used):
    sx=float(start.x)-float(center.x);sy=float(start.y)-float(center.y);sn=max(.01,math.hypot(sx,sy))
    ranked=[]
    for loc in locations:
        dx=float(loc.x)-float(center.x);dy=float(loc.y)-float(center.y);dn=max(.01,math.hypot(dx,dy))
        cosine=(sx*dx+sy*dy)/(sn*dn)
        if cosine>-0.35:continue
        key=(round(float(loc.x),2),round(float(loc.y),2))
        ranked.append((1 if key in used else 0,cosine,abs(dn-sn),key,loc))
    if not ranked:
        return carla.Location(x=2.0*float(center.x)-float(start.x),
                              y=2.0*float(center.y)-float(start.y),z=float(start.z))
    selected=min(ranked,key=lambda item:item[:3]);used.add(selected[3]);return selected[4]


def spawn_walkers(world, world_map, center, count, rng, mode="ai", speed=1.2, launch_interval=0.8):
    library = world.get_blueprint_library();blueprints = list(library.filter("walker.pedestrian.*"))
    try:locations = [x for x in world_map.get_crosswalks() if distance(x, center) <= 45.0]
    except Exception:locations = []
    if not locations:
        locations = [world.get_random_location_from_navigation() for _ in range(max(10, count * 5))]
        locations = [x for x in locations if x is not None and distance(x, center) <= 45.0]
    locations=unique_locations(locations);spawn_locations=balanced_approach_order(locations,center,rng)
    controller_bp=None
    if mode=="ai":
        try:controller_bp=library.find("controller.ai.walker")
        except Exception:print("WARNING: controller.ai.walker unavailable; using staggered manual WalkerControl.")
    actors=[];movements=[];used_destinations=set()
    for loc in spawn_locations:
        if len(actors) >= count:break
        bp = rng.choice(blueprints)
        if bp.has_attribute("is_invincible"):bp.set_attribute("is_invincible", "false")
        if bp.has_attribute("role_name"):bp.set_attribute("role_name","rsu_test_walker")
        transform = carla.Transform(carla.Location(x=loc.x, y=loc.y, z=loc.z + 0.35))
        actor = world.try_spawn_actor(bp, transform)
        if actor is None:continue
        if mode=="static":
            actors.append(actor)
            continue
        destination=opposite_crossing_destination(loc,locations,center,used_destinations)
        controller=None
        if controller_bp is not None:
            try:controller=world.try_spawn_actor(controller_bp,carla.Transform(),attach_to=actor)
            except Exception:controller=None
        dx=float(destination.x)-float(loc.x);dy=float(destination.y)-float(loc.y);norm=max(.01,math.hypot(dx,dy))
        movements.append({"actor":actor,"controller":controller,"destination":destination,
                          "direction":carla.Vector3D(x=dx/norm,y=dy/norm,z=0.0),
                          "speed":float(speed),"delay":len(actors)*float(launch_interval),"started":False})
        actors.append(actor)
    return actors,movements


def launch_due_walkers(movements, elapsed):
    for movement in movements:
        if movement["started"] or elapsed<movement["delay"]:continue
        controller=movement["controller"]
        movement["started"]=True
        try:
            if controller is not None:
                controller.start();controller.go_to_location(movement["destination"]);controller.set_max_speed(movement["speed"])
            else:
                movement["actor"].apply_control(carla.WalkerControl(
                    direction=movement["direction"],speed=movement["speed"],jump=False))
            print("  WALKER START id=%d delay=%.1fs mode=%s"%(movement["actor"].id,movement["delay"],"ai" if controller is not None else "manual"))
        except Exception as exc:
            if controller is None:
                print("WARNING: failed to start walker id=%d: %s"%(movement["actor"].id,exc));continue
            print("WARNING: AI start failed for walker id=%d (%s); trying manual control."%(movement["actor"].id,exc))
            try:
                if controller is not None:controller.stop()
                movement["actor"].apply_control(carla.WalkerControl(
                    direction=movement["direction"],speed=movement["speed"],jump=False))
            except Exception as fallback_exc:print("WARNING: failed to start walker id=%d: %s"%(movement["actor"].id,fallback_exc))


def obstacle_blueprints(library):
    words=("trafficcone","barrier","debris","garbage","trash","box","shoppingcart")
    return [bp for bp in library.filter("static.prop.*")
            if any(word in bp.id.lower() for word in words)]


def spawn_obstacles(world, world_map, center, count, rng):
    blueprints=obstacle_blueprints(world.get_blueprint_library())
    if not blueprints:
        print("WARNING: no suitable static road-obstacle blueprints are available in this CARLA build.")
        return []
    points=[]
    for wp in world_map.generate_waypoints(2.0):
        d=distance(wp.transform.location,center)
        if 12.0<=d<=38.0 and all(distance(wp.transform.location,x.transform.location)>8.0 for x in points):
            points.append(wp)
    rng.shuffle(points);actors=[]
    for wp in points:
        if len(actors)>=count:break
        bp=rng.choice(blueprints);transform=wp.transform
        if bp.has_attribute("role_name"):bp.set_attribute("role_name","rsu_test_obstacle")
        transform.location.z+=0.20
        actor=world.try_spawn_actor(bp,transform)
        if actor is not None:actors.append(actor)
    return actors


def main():
    parser=argparse.ArgumentParser(description="V0.6.12.8.2.2.76 event-focused HLW targets")
    parser.add_argument("--config",default="config/roadside.yaml")
    parser.add_argument("--walkers",type=int,default=12)
    parser.add_argument("--obstacles",type=int,default=6)
    parser.add_argument("--seed",type=int,default=42)
    parser.add_argument("--walker-mode",choices=("static","manual","ai"),default="manual",
                        help="static holds walkers in place; manual moves across roads; ai requires a pedestrian navmesh")
    parser.add_argument("--walker-speed",type=float,default=1.2)
    parser.add_argument("--walker-launch-interval",type=float,default=0.8)
    args=parser.parse_args();rng=random.Random(args.seed);config=load_config(args.config);cc=config.get("carla",{})
    client=carla.Client(cc.get("host","127.0.0.1"),int(cc.get("port",2000)));client.set_timeout(float(cc.get("timeout",60.0)))
    world,world_map,center=junction_center(client,config)
    walkers,movements=spawn_walkers(world,world_map,center,max(0,args.walkers),rng,args.walker_mode,max(.1,args.walker_speed),max(0.0,args.walker_launch_interval))
    obstacles=spawn_obstacles(world,world_map,center,max(0,args.obstacles),rng);actors=walkers+obstacles
    print("V0.6.12.8.2.2.76 targets active: seed=%d walkers=%d road_obstacles=%d walker_mode=%s launch_interval=%.1fs"%(args.seed,len(walkers),len(obstacles),args.walker_mode,args.walker_launch_interval))
    movement_by_id=dict((x["actor"].id,x) for x in movements)
    for label,items in (("walker",walkers),("obstacle",obstacles)):
        for actor in items:
            loc=actor.get_location();suffix=""
            if actor.id in movement_by_id:
                movement=movement_by_id[actor.id];dest=movement["destination"]
                suffix=" dest=(%.2f,%.2f) delay=%.1fs controller=%s"%(dest.x,dest.y,movement["delay"],"ai" if movement["controller"] is not None else "manual")
            print("  TARGET %s id=%d type=%s pos=(%.2f,%.2f,%.2f) range=%.2fm%s"%(label,actor.id,actor.type_id,loc.x,loc.y,loc.z,distance(loc,center),suffix))
    print("Keep this process running; Ctrl+C removes only these test targets.")
    started_at=time.time()
    try:
        while True:launch_due_walkers(movements,time.time()-started_at);time.sleep(0.1)
    except KeyboardInterrupt:pass
    finally:
        for movement in movements:
            controller=movement["controller"]
            if controller is not None:
                try:controller.stop()
                except Exception:pass
        for movement in movements:
            controller=movement["controller"]
            if controller is not None:
                try:controller.destroy()
                except Exception:pass
        for actor in actors:
            try:actor.destroy()
            except Exception:pass
        print("V0.6.12.8.2.2.76 test targets removed.")


if __name__=="__main__":main()
