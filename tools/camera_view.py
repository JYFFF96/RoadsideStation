from __future__ import print_function

import argparse
import math
import time

import carla
import cv2
import numpy as np


def combine_transform(base, offset):
    return carla.Transform(
        carla.Location(x=base.location.x + float(offset.get("x", 0)),
                       y=base.location.y + float(offset.get("y", 0)),
                       z=base.location.z + float(offset.get("z", 0))),
        carla.Rotation(pitch=base.rotation.pitch + float(offset.get("pitch", 0)),
                       yaw=base.rotation.yaw + float(offset.get("yaw", 0)),
                       roll=base.rotation.roll + float(offset.get("roll", 0))))


def find_rsu_transform(world, junction_index=0, lateral=5.0, height=8.0):
    seen = set(); junctions = []
    for wp in world.get_map().generate_waypoints(2.0):
        if not wp.is_junction: continue
        j = wp.get_junction()
        if j is None or j.id in seen: continue
        seen.add(j.id); junctions.append(wp)
    wp = junctions[int(junction_index) % len(junctions)]
    loc = wp.transform.location; yaw = float(wp.transform.rotation.yaw); r = math.radians(yaw)
    return carla.Transform(carla.Location(x=loc.x-math.sin(r)*lateral,
                                          y=loc.y+math.cos(r)*lateral,
                                          z=loc.z+height),
                           carla.Rotation(yaw=yaw))


def project(world_xyz, camera_transform, width, height, fov):
    focal = width / (2.0 * math.tan(math.radians(fov) / 2.0))
    inv = np.asarray(camera_transform.get_inverse_matrix(), dtype=np.float64)
    p = np.dot(inv, np.array([world_xyz[0], world_xyz[1], world_xyz[2], 1.0]))
    if p[0] <= 0.1: return None
    u = focal * (p[1] / p[0]) + width / 2.0
    v = focal * (-p[2] / p[0]) + height / 2.0
    if 0 <= u < width and 0 <= v < height: return int(u), int(v), float(p[0])
    return None


def main():
    ap=argparse.ArgumentParser(description="V0.3.2 RGB + CARLA vehicle projection validation")
    ap.add_argument("--host",default="127.0.0.1"); ap.add_argument("--port",type=int,default=2000)
    ap.add_argument("--junction-index",type=int,default=0); ap.add_argument("--width",type=int,default=1280)
    ap.add_argument("--height",type=int,default=720); ap.add_argument("--fov",type=float,default=90.0)
    args=ap.parse_args(); client=carla.Client(args.host,args.port); client.set_timeout(10.0); world=client.get_world()
    base=find_rsu_transform(world,args.junction_index); cam_tf=combine_transform(base,{"x":0,"y":0,"z":0,"pitch":-20,"yaw":0,"roll":0})
    bp=world.get_blueprint_library().find("sensor.camera.rgb"); bp.set_attribute("image_size_x",str(args.width)); bp.set_attribute("image_size_y",str(args.height)); bp.set_attribute("fov",str(args.fov))
    camera=world.spawn_actor(bp,cam_tf); latest=[None]
    def callback(img):
        a=np.frombuffer(img.raw_data,dtype=np.uint8).reshape((img.height,img.width,4)); latest[0]=a[:,:,:3].copy()
    camera.listen(callback); print("Camera viewer started on %s. Press Q or ESC in the image window to exit."%world.get_map().name.split("/")[-1])
    try:
        while True:
            frame=latest[0]
            if frame is None: time.sleep(.02); continue
            view=frame.copy(); visible=0
            for actor in world.get_actors().filter("vehicle.*"):
                loc=actor.get_location(); p=project((loc.x,loc.y,loc.z+1.0),cam_tf,args.width,args.height,args.fov)
                if p is None: continue
                u,v,d=p; visible+=1
                cv2.circle(view,(u,v),6,(0,255,255),2)
                cv2.putText(view,"CARLA #%d %.1fm"%(actor.id,d),(max(0,u-55),max(20,v-12)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1,cv2.LINE_AA)
            cv2.putText(view,"V0.3.2 projection check | visible vehicles: %d"%visible,(20,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,0),2,cv2.LINE_AA)
            cv2.imshow("RoadsideStation Camera Projection",view)
            key=cv2.waitKey(1)&0xff
            if key in (27,ord('q')): break
    finally:
        try: camera.stop(); camera.destroy()
        except Exception: pass
        cv2.destroyAllWindows(); print("Camera viewer stopped.")
if __name__=="__main__": main()
