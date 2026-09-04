from __future__ import print_function

import argparse
import math
import os
import random
import sys
import signal
import time

# Direct execution sets sys.path[0] to tools/. Add the repository root so the
# documented `python3.7 tools/spawn_multiclass_targets.py` command can import
# the roadside package without requiring a separate PYTHONPATH entry for it.
PROJECT_ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:sys.path.insert(0,PROJECT_ROOT)

import carla
import yaml

from roadside.carla_station import CarlaRoadsideStation
from roadside.sim_ego import EGO_ROLE
from roadside.scenario_ego import ScenarioEgo
from roadside.ego_camera import launch_ego_viewer, stop_ego_viewer


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


def spawn_walkers(world, world_map, center, count, rng, mode="ai", speed=1.2, launch_interval=0.8, owned=None, owned_controllers=None):
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
        if owned is not None:owned.append(actor)
        if mode=="static":
            actors.append(actor)
            continue
        destination=opposite_crossing_destination(loc,locations,center,used_destinations)
        controller=None
        if controller_bp is not None:
            try:controller=world.try_spawn_actor(controller_bp,carla.Transform(),attach_to=actor)
            except Exception:controller=None
            if controller is not None and owned_controllers is not None:
                owned_controllers.append(controller)
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


def spawn_obstacles(world, world_map, center, count, rng, owned=None):
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
        if actor is not None:
            actors.append(actor)
            if owned is not None:owned.append(actor)
    return actors


def spawn_stopped_vehicles(world, world_map, center, count, rng, owned=None):
    blueprints=[]
    for bp in world.get_blueprint_library().filter("vehicle.*"):
        if bp.has_attribute("number_of_wheels") and int(bp.get_attribute(
                "number_of_wheels").as_int())==4:blueprints.append(bp)
    if not blueprints:
        print("WARNING: no four-wheel vehicle blueprints are available.")
        return []
    points=[]
    for wp in world_map.generate_waypoints(2.0):
        d=distance(wp.transform.location,center)
        if (15.0<=d<=32.0 and not wp.is_junction and
                all(distance(wp.transform.location,x.transform.location)>10.0
                    for x in points)):points.append(wp)
    rng.shuffle(points);actors=[]
    for wp in points:
        if len(actors)>=count:break
        bp=rng.choice(blueprints)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name","rsu_test_stopped_vehicle")
        transform=wp.transform;transform.location.z+=0.30
        actor=world.try_spawn_actor(bp,transform)
        if actor is None:continue
        if owned is not None:owned.append(actor)
        actor.set_autopilot(False)
        actor.apply_control(carla.VehicleControl(
            throttle=0.0,brake=1.0,hand_brake=True))
        actors.append(actor)
    return actors


def spawn_speeding_vehicles(world,world_map,center,count,rng,speed_kmh=55.0,owned=None):
    blueprints=[]
    for bp in world.get_blueprint_library().filter("vehicle.*"):
        if bp.has_attribute("number_of_wheels") and int(bp.get_attribute(
                "number_of_wheels").as_int())==4:blueprints.append(bp)
    if not blueprints:
        print("WARNING: no four-wheel vehicle blueprints are available for SLW.")
        return []
    points=[]
    for wp in world_map.generate_waypoints(2.0):
        d=distance(wp.transform.location,center)
        if 35.0<=d<=55.0 and not wp.is_junction:points.append(wp)
    rng.shuffle(points);items=[];speed_mps=max(0.1,float(speed_kmh)/3.6)
    for wp in points:
        if len(items)>=count:break
        bp=rng.choice(blueprints)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name","rsu_test_speeding_vehicle")
        transform=wp.transform;transform.location.z+=0.30
        actor=world.try_spawn_actor(bp,transform)
        if actor is None:continue
        if owned is not None:owned.append(actor)
        forward=transform.get_forward_vector()
        velocity=carla.Vector3D(x=float(forward.x)*speed_mps,
                                y=float(forward.y)*speed_mps,z=0.0)
        actor.set_autopilot(False);actor.set_target_velocity(velocity)
        items.append({"actor":actor,"velocity":velocity,"speed_kmh":float(speed_kmh)})
    return items


def main():
    parser=argparse.ArgumentParser(description="V0.6.12.8.2.2.84 on-demand warning scenarios")
    parser.add_argument("--config",default="config/roadside.yaml")
    parser.add_argument("--scenario",choices=("custom","vrucw","hlw","avw","slw"),
                        default="custom")
    parser.add_argument("--walkers",type=int,default=12)
    parser.add_argument("--obstacles",type=int,default=6)
    parser.add_argument("--stopped-vehicles",type=int,default=0)
    parser.add_argument("--speeding-vehicles",type=int,default=0)
    parser.add_argument("--ego-speed-kmh",type=float,default=None,
                        help="ego desired speed (default 25, SLW 55 km/h); 0 parks ego")
    parser.add_argument("--ego-role",default=None)
    parser.add_argument("--ego-view",action="store_true",help="open an independent ego camera window")
    parser.add_argument("--tm-port",type=int,default=8000)
    parser.add_argument("--seed",type=int,default=42)
    parser.add_argument("--walker-mode",choices=("static","manual","ai"),default="manual",
                        help="static holds walkers in place; manual moves across roads; ai requires a pedestrian navmesh")
    parser.add_argument("--walker-speed",type=float,default=1.2)
    parser.add_argument("--walker-launch-interval",type=float,default=0.8)
    args=parser.parse_args()
    if args.ego_speed_kmh is None:
        args.ego_speed_kmh=55.0 if args.scenario=="slw" else 25.0
    if not math.isfinite(args.ego_speed_kmh) or args.ego_speed_kmh<0:
        parser.error("--ego-speed-kmh must be finite and non-negative")
    if not 1<=args.tm_port<=65535:parser.error("invalid --tm-port")
    if args.scenario=="vrucw":
        args.walkers=12;args.obstacles=0;args.stopped_vehicles=0;args.speeding_vehicles=0
    elif args.scenario=="hlw":
        args.walkers=0;args.obstacles=6;args.stopped_vehicles=0;args.speeding_vehicles=0
    elif args.scenario=="avw":
        args.walkers=0;args.obstacles=0;args.stopped_vehicles=1;args.speeding_vehicles=0
    elif args.scenario=="slw":
        args.walkers=0;args.obstacles=0;args.stopped_vehicles=0;args.speeding_vehicles=0
    rng=random.Random(args.seed);config=load_config(args.config);cc=config.get("carla",{})
    client=carla.Client(cc.get("host","127.0.0.1"),int(cc.get("port",2000)));client.set_timeout(float(cc.get("timeout",60.0)))
    world,world_map,center=junction_center(client,config)
    role=args.ego_role or config.get("v2x_events",{}).get("test_ego_role",EGO_ROLE)
    if not role:parser.error("ego role must not be empty")
    ego=ScenarioEgo(world,role);viewer=None
    owned=[];controllers=[];movements=[]

    def stop(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    try:
        walkers,movements=spawn_walkers(world,world_map,center,max(0,args.walkers),rng,
            args.walker_mode,max(.1,args.walker_speed),max(0.0,args.walker_launch_interval),
            owned=owned,owned_controllers=controllers)
        obstacles=spawn_obstacles(world,world_map,center,max(0,args.obstacles),rng,owned=owned)
        stopped_vehicles=spawn_stopped_vehicles(world,world_map,center,
            max(0,args.stopped_vehicles),rng,owned=owned)
        # Legacy custom extra speeding targets remain available. Named SLW uses ego only.
        speeding=spawn_speeding_vehicles(world,world_map,center,
            max(0,args.speeding_vehicles),rng,args.ego_speed_kmh,owned=owned)
        ego.start(client,world_map,center,args.ego_speed_kmh,args.tm_port,
                  targets=stopped_vehicles)
        if args.ego_view:viewer=launch_ego_viewer(args.config,role)
        print("V0.6.12.8.2.2.84 scenario=%s ego=%d walkers=%d obstacles=%d stopped=%d" %
              (args.scenario.upper(),ego.actor.id,len(walkers),len(obstacles),len(stopped_vehicles)))
        for actor in owned:
            loc=actor.get_location()
            print("  TARGET id=%d type=%s pos=(%.2f,%.2f,%.2f)" %
                  (actor.id,actor.type_id,loc.x,loc.y,loc.z))
        print("Ctrl+C removes this script's ego/targets only; a reused ego remains with its owner.")
        started_at=time.time()
        while True:
            if not ego.actor.is_alive:
                print("[EGO] Reference vehicle disappeared; stop and restart this scenario.")
                break
            launch_due_walkers(movements,time.time()-started_at)
            for item in speeding:
                try:item["actor"].set_target_velocity(item["velocity"])
                except RuntimeError:pass
            if viewer is not None and viewer.poll() is not None:
                print("[EGO VIEW] Window closed (exit=%s); scenario continues." % viewer.returncode)
                viewer=None
            time.sleep(0.1)
    except KeyboardInterrupt:pass
    finally:
        stop_ego_viewer(viewer)
        for controller in controllers:
            try:controller.stop()
            except RuntimeError:pass
        for controller in controllers:
            try:controller.destroy()
            except RuntimeError:pass
        for actor in owned:
            try:actor.destroy()
            except RuntimeError:pass
        ego.close()
        print("V0.6.12.8.2.2.84 scenario actors removed.")


if __name__=="__main__":main()
