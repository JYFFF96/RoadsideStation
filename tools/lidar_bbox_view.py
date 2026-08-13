from __future__ import print_function
import argparse,itertools,time,yaml,cv2
from roadside.carla_station import CarlaRoadsideStation
from roadside.fusion import SimpleFusion
from roadside.camera_fusion import CameraProjector

def load_config(path):
    with open(path,"r") as fp:return yaml.safe_load(fp)

def box_corners(center,extent):
    x,y,z=center;ex,ey,ez=[max(.3,float(v)) for v in extent];hx=max(.5,ex*.6);hy=max(.5,ey*.6);hz=max(.5,ez*.6)
    return [(x+sx*hx,y+sy*hy,z+sz*hz) for sx,sy,sz in itertools.product((-1.,1.),repeat=3)]

def projected_rect(projector,center,extent,width,height):
    pixels=[]
    for p3 in box_corners(center,extent):
        p=projector.project(p3[0],p3[1],p3[2])
        if p is not None:pixels.append((int(p["u"]),int(p["v"])))
    if len(pixels)<2:return None
    xs=[p[0] for p in pixels];ys=[p[1] for p in pixels]
    return max(0,min(xs)),max(0,min(ys)),min(width-1,max(xs)),min(height-1,max(ys))

def main():
    ap=argparse.ArgumentParser(description="V0.3.6 stable tracked LiDAR boxes on RGB");ap.add_argument("--config",default="config/roadside.yaml");args=ap.parse_args();cfg=load_config(args.config)
    sid=cfg["station"]["id"];station=CarlaRoadsideStation(cfg);fusion=SimpleFusion(sid,cfg["fusion"]);station.start();fusion.set_world_transform(station.lidar_transform);fusion.set_candidate_validator(station.is_driving_roi)
    cc=cfg["camera"];width=int(cc.get("width",1280));height=int(cc.get("height",720));fov=float(cc.get("fov",90));projector=CameraProjector(width,height,fov,station.camera_transform)
    print("V0.3.6 started. Green rectangle uses smoothed tracked extent.")
    try:
        while True:
            camera,lidar,radar=station.cache.snapshot()
            if camera is None or lidar is None:time.sleep(.02);continue
            ol=fusion.fuse(lidar[1],radar[1] if radar else None,frame_id=lidar[0]);view=camera[1][:,:,:3].copy();drawn=0
            by_id={c.get("id"):c for c in fusion.last_tracked_candidates}
            for obj in ol.objects:
                cand=by_id.get(obj.object_id)
                if cand is None:continue
                p=projector.project(obj.x,obj.y,obj.z);rect=projected_rect(projector,(cand["x"],cand["y"],cand["z"]),cand.get("extent",[2,1,1]),width,height)
                if p is None or rect is None:continue
                x1,y1,x2,y2=rect;u,v=int(p["u"]),int(p["v"]);drawn+=1;cv2.rectangle(view,(x1,y1),(x2,y2),(0,255,0),2);cv2.drawMarker(view,(u,v),(0,0,255),cv2.MARKER_CROSS,16,2)
                speed=(obj.vx*obj.vx+obj.vy*obj.vy)**.5;hits=cand.get("track_hits",1);label="%s %.1fm/s %s h=%d"%(obj.object_id,speed,"+".join(obj.sources),hits);cv2.putText(view,label,(x1,max(20,y1-8)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1,cv2.LINE_AA)
            s=fusion.last_stats;bg="READY/%d"%s["background_cells"] if s["background_ready"] else "LEARNING %.1fs"%s["background_remaining"]
            title="V0.3.6 stable LiDAR boxes | tracks=%d boxes=%d BG=%s"%(len(ol.objects),drawn,bg);cv2.putText(view,title,(20,30),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2,cv2.LINE_AA);cv2.imshow("RoadsideStation V0.3.6",view)
            key=cv2.waitKey(1)&0xff
            if key in (27,ord('q')):break
            time.sleep(.01)
    finally:
        station.stop();cv2.destroyAllWindows();print("V0.3.6 viewer stopped cleanly.")
if __name__=="__main__":main()
