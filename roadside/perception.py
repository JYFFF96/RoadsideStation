from __future__ import print_function
from collections import defaultdict, deque
import math
import numpy as np


def _bounds(c):
    e=c.get("extent",[0,0,0]); return (c["x"]-e[0]/2,c["x"]+e[0]/2,c["y"]-e[1]/2,c["y"]+e[1]/2,c["z"]-e[2]/2,c["z"]+e[2]/2)


def _gap(a,b):
    A=_bounds(a);B=_bounds(b)
    gx=max(0.0,max(A[0],B[0])-min(A[1],B[1])); gy=max(0.0,max(A[2],B[2])-min(A[3],B[3])); gz=max(0.0,max(A[4],B[4])-min(A[5],B[5]))
    return math.sqrt(gx*gx+gy*gy+gz*gz)


def merge_lidar_clusters(clusters,max_gap=1.8,max_merged_length=14.0,max_merged_width=4.5,max_merged_height=4.5):
    """Merge nearby fragments while preventing adjacent lane vehicles from becoming one object."""
    items=[dict(c) for c in clusters]; changed=True
    while changed:
        changed=False
        for i in range(len(items)):
            for j in range(i+1,len(items)):
                if _gap(items[i],items[j])>float(max_gap): continue
                A=_bounds(items[i]);B=_bounds(items[j]); xmin=min(A[0],B[0]);xmax=max(A[1],B[1]);ymin=min(A[2],B[2]);ymax=max(A[3],B[3]);zmin=min(A[4],B[4]);zmax=max(A[5],B[5])
                ex=xmax-xmin;ey=ymax-ymin;ez=zmax-zmin
                if max(ex,ey)>float(max_merged_length) or min(ex,ey)>float(max_merged_width) or ez>float(max_merged_height): continue
                na=max(1,items[i].get("point_count",1));nb=max(1,items[j].get("point_count",1));n=na+nb
                items[i]={"x":(items[i]["x"]*na+items[j]["x"]*nb)/n,"y":(items[i]["y"]*na+items[j]["y"]*nb)/n,"z":(items[i]["z"]*na+items[j]["z"]*nb)/n,"point_count":n,"extent":[ex,ey,ez]}
                del items[j];changed=True;break
            if changed: break
    items.sort(key=lambda x:x.get("point_count",0),reverse=True);return items


def _cluster_array(pts,voxel_size,min_points,min_length,max_length,min_width,max_width,min_height,max_height,max_objects):
    if pts is None or len(pts)==0:return []
    size=float(voxel_size);keys=np.floor(pts/size).astype(np.int32);buckets=defaultdict(list)
    for i,k in enumerate(keys):buckets[(int(k[0]),int(k[1]),int(k[2]))].append(i)
    occupied=set(buckets);visited=set();clusters=[];offs=[(a,b,c) for a in (-1,0,1) for b in (-1,0,1) for c in (-1,0,1)]
    for start in occupied:
        if start in visited:continue
        visited.add(start);q=deque([start]);idx=[]
        while q:
            cell=q.popleft();idx.extend(buckets[cell]);cx,cy,cz=cell
            for dx,dy,dz in offs:
                n=(cx+dx,cy+dy,cz+dz)
                if n in occupied and n not in visited:visited.add(n);q.append(n)
        if len(idx)<int(min_points):continue
        cp=pts[idx];cen=cp.mean(axis=0);pmin=cp.min(axis=0);pmax=cp.max(axis=0);e=pmax-pmin;ex,ey,ez=map(float,e);hl=max(ex,ey);hs=min(ex,ey)
        if hl<float(min_length) or hl>float(max_length) or hs<float(min_width) or hs>float(max_width) or ez<float(min_height) or ez>float(max_height):continue
        clusters.append({"x":float(cen[0]),"y":float(cen[1]),"z":float(cen[2]),"point_count":len(idx),"extent":[ex,ey,ez]})
    clusters.sort(key=lambda x:x["point_count"],reverse=True);return clusters[:int(max_objects)]


def voxel_cluster_lidar(points,voxel_size=0.8,min_points=6,min_z=-7.5,max_z=2.0,max_range=70.0,min_length=.6,max_length=8.0,min_width=.4,max_width=4.0,min_height=.25,max_height=4.0,max_objects=80):
    if points is None or len(points)==0:return []
    pts=np.asarray(points,dtype=np.float32);r2=pts[:,0]*pts[:,0]+pts[:,1]*pts[:,1];mask=(pts[:,2]>=min_z)&(pts[:,2]<=max_z)&(r2<=max_range*max_range);pts=pts[mask]
    return _cluster_array(pts,voxel_size,min_points,min_length,max_length,min_width,max_width,min_height,max_height,max_objects)


def adaptive_voxel_cluster_lidar(points,bands,min_z=-7.5,max_z=2.0,max_range=80.0,max_length=8.0,max_width=4.0,max_height=4.0,max_objects=120):
    """Cluster near/mid/far point clouds with progressively larger voxels.

    High-mounted roadside LiDAR becomes sparse with range. A fixed 0.5 m voxel
    grid and one geometry threshold works well nearby but fragments distant
    vehicles. Bands are non-overlapping annuli and therefore do not duplicate
    points. Each band can relax voxel size, point count and minimum geometry.
    """
    if points is None or len(points)==0:return []
    pts=np.asarray(points,dtype=np.float32)
    r=np.sqrt(pts[:,0]*pts[:,0]+pts[:,1]*pts[:,1])
    base=(pts[:,2]>=float(min_z))&(pts[:,2]<=float(max_z))&(r<=float(max_range))
    pts=pts[base];r=r[base]
    if len(pts)==0:return []
    out=[];lower=0.0
    for band in bands or []:
        upper=min(float(band.get("max_range",max_range)),float(max_range))
        if upper<=lower:continue
        mask=(r>=lower)&(r<upper if upper<float(max_range) else r<=upper)
        bp=pts[mask]
        out.extend(_cluster_array(
            bp,
            band.get("voxel_size",0.5),band.get("min_points",4),
            band.get("min_length",0.4),max_length,
            band.get("min_width",0.25),max_width,
            band.get("min_height",0.15),max_height,
            max_objects))
        lower=upper
        if lower>=float(max_range):break
    out.sort(key=lambda x:x.get("point_count",0),reverse=True)
    return out[:int(max_objects)]


def associate_radar(clusters,radar_detections,max_distance=3.0):
    if not clusters:return []
    radar=radar_detections or [];used=set();out=[];md2=float(max_distance)**2
    for c in clusters:
        best=None;bi=None;bd=md2
        for i,d in enumerate(radar):
            if i in used:continue
            dd=(float(d["x"])-c["x"])**2+(float(d["y"])-c["y"])**2+(float(d["z"])-c["z"])**2
            if dd<bd:bd=dd;best=d;bi=i
        if bi is not None:used.add(bi)
        item=dict(c);item["radar"]=best;out.append(item)
    return out
