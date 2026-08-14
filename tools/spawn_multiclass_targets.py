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


def spawn_walkers(world, world_map, center, count, rng):
    library = world.get_blueprint_library();blueprints = list(library.filter("walker.pedestrian.*"))
    try:locations = [x for x in world_map.get_crosswalks() if distance(x, center) <= 45.0]
    except Exception:locations = []
    if not locations:
        locations = [world.get_random_location_from_navigation() for _ in range(max(10, count * 5))]
        locations = [x for x in locations if x is not None and distance(x, center) <= 45.0]
    rng.shuffle(locations);actors=[]
    for loc in locations:
        if len(actors) >= count:break
        bp = rng.choice(blueprints)
        if bp.has_attribute("is_invincible"):bp.set_attribute("is_invincible", "false")
        if bp.has_attribute("role_name"):bp.set_attribute("role_name","rsu_test_walker")
        transform = carla.Transform(carla.Location(x=loc.x, y=loc.y, z=loc.z + 0.35))
        actor = world.try_spawn_actor(bp, transform)
        if actor is None:continue
        dx=float(center.x)-float(loc.x);dy=float(center.y)-float(loc.y);norm=max(.01,math.hypot(dx,dy))
        actor.apply_control(carla.WalkerControl(
            direction=carla.Vector3D(x=dx/norm,y=dy/norm,z=0.0),speed=1.2,jump=False))
        actors.append(actor)
    return actors


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
    parser=argparse.ArgumentParser(description="V0.6.12.8.2.2.8 deterministic walkers and road obstacles")
    parser.add_argument("--config",default="config/roadside.yaml")
    parser.add_argument("--walkers",type=int,default=12)
    parser.add_argument("--obstacles",type=int,default=6)
    parser.add_argument("--seed",type=int,default=42)
    args=parser.parse_args();rng=random.Random(args.seed);config=load_config(args.config);cc=config.get("carla",{})
    client=carla.Client(cc.get("host","127.0.0.1"),int(cc.get("port",2000)));client.set_timeout(float(cc.get("timeout",60.0)))
    world,world_map,center=junction_center(client,config)
    walkers=spawn_walkers(world,world_map,center,max(0,args.walkers),rng)
    obstacles=spawn_obstacles(world,world_map,center,max(0,args.obstacles),rng);actors=walkers+obstacles
    print("V0.6.12.8.2.2.8 targets active: seed=%d walkers=%d road_obstacles=%d"%(args.seed,len(walkers),len(obstacles)))
    for label,items in (("walker",walkers),("obstacle",obstacles)):
        for actor in items:
            loc=actor.get_location();print("  TARGET %s id=%d type=%s pos=(%.2f,%.2f,%.2f) range=%.2fm"%(label,actor.id,actor.type_id,loc.x,loc.y,loc.z,distance(loc,center)))
    print("Keep this process running; Ctrl+C removes only these test targets.")
    try:
        while True:time.sleep(1.0)
    except KeyboardInterrupt:pass
    finally:
        for actor in actors:
            try:actor.destroy()
            except Exception:pass
        print("V0.6.12.8.2.2.8 test targets removed.")


if __name__=="__main__":main()
