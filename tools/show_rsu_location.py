from __future__ import print_function

import argparse
import math
import sys
import time

import carla
import yaml


def load_config(path):
    with open(path, "r") as fp:
        return yaml.safe_load(fp)


def angle_diff(a, b):
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def junction_candidates(world_map):
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
            if all(angle_diff(h, b) > 35.0 for b in bins):
                bins.append(h)
        box = j.bounding_box
        area = max(1.0, float(box.extent.x) * 2.0) * max(1.0, float(box.extent.y) * 2.0)
        score = len(bins) * 1000.0 + min(area, 500.0)
        seen[j.id] = (score, j, wp, len(bins), area)
    items = list(seen.values())
    items.sort(key=lambda x: x[0], reverse=True)
    return items


def resolve_station(world_map, cfg):
    sc = cfg["station"]
    deployment = sc.get("deployment", "manual")
    if deployment in ("auto_junction", "auto_cross_junction"):
        candidates = junction_candidates(world_map)
        if not candidates:
            raise RuntimeError("No junction found")
        cross = [x for x in candidates if x[3] >= 4]
        pool = cross or candidates
        score, junction, wp, dirs, area = pool[int(sc.get("junction_index", 0)) % len(pool)]
        center = junction.bounding_box.location
        yaw = float(wp.transform.rotation.yaw)
        lateral = float(sc.get("lateral_offset", 7.0))
        height = float(sc.get("height", 8.0))
        r = math.radians(yaw)
        x = float(center.x) - math.sin(r) * lateral
        y = float(center.y) + math.cos(r) * lateral
        z = float(center.z) + height
        sensor_yaw = math.degrees(math.atan2(float(center.y) - y, float(center.x) - x))
        base = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=sensor_yaw))
        return base, carla.Location(x=center.x, y=center.y, z=center.z), junction.id, dirs, area
    t = sc["transform"]
    base = carla.Transform(carla.Location(x=float(t.get("x", 0)), y=float(t.get("y", 0)), z=float(t.get("z", 8))), carla.Rotation(pitch=float(t.get("pitch", 0)), yaw=float(t.get("yaw", 0)), roll=float(t.get("roll", 0))))
    return base, carla.Location(x=base.location.x, y=base.location.y, z=0.0), None, 0, 0.0


def combined(base, offset):
    return carla.Transform(carla.Location(x=base.location.x + float(offset.get("x", 0)), y=base.location.y + float(offset.get("y", 0)), z=base.location.z + float(offset.get("z", 0))), carla.Rotation(pitch=base.rotation.pitch + float(offset.get("pitch", 0)), yaw=base.rotation.yaw + float(offset.get("yaw", 0)), roll=base.rotation.roll + float(offset.get("roll", 0))))


def forward_endpoint(transform, distance):
    yaw = math.radians(transform.rotation.yaw)
    pitch = math.radians(transform.rotation.pitch)
    cp = math.cos(pitch)
    return carla.Location(x=transform.location.x + distance * cp * math.cos(yaw), y=transform.location.y + distance * cp * math.sin(yaw), z=transform.location.z + distance * math.sin(pitch))


def main():
    ap = argparse.ArgumentParser(description="Show RoadsideStation RSU location in CARLA")
    ap.add_argument("--config", default="config/roadside.yaml")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--height", type=float, default=55.0)
    args = ap.parse_args()
    cfg = load_config(args.config)
    cc = cfg.get("carla", {})
    client = carla.Client(cc.get("host", "127.0.0.1"), int(cc.get("port", 2000)))
    client.set_timeout(float(cc.get("timeout", 60.0)))
    world = client.get_world()
    world_map = world.get_map()
    base, center, junction_id, dirs, area = resolve_station(world_map, cfg)
    camera_t = combined(base, cfg.get("camera", {}).get("transform", {}))
    lidar_t = combined(base, cfg.get("lidar", {}).get("transform", {}))
    radar_t = combined(base, cfg.get("radar", {}).get("transform", {}))
    print("RoadsideStation RSU Location Viewer")
    print("Map: %s" % world_map.name.split("/")[-1])
    if junction_id is not None:
        print("Junction: id=%s directions=%d area=%.1f center=(%.2f, %.2f)" % (junction_id, dirs, area, center.x, center.y))
    print("RSU_001: x=%.2f y=%.2f z=%.2f yaw=%.1f" % (base.location.x, base.location.y, base.location.z, base.rotation.yaw))
    print("Camera yaw=%.1f pitch=%.1f | Radar yaw=%.1f pitch=%.1f | LiDAR=360deg" % (camera_t.rotation.yaw, camera_t.rotation.pitch, radar_t.rotation.yaw, radar_t.rotation.pitch))
    dx = base.location.x - center.x
    dy = base.location.y - center.y
    norm = max(0.1, math.hypot(dx, dy))
    sx = center.x + dx / norm * 28.0
    sy = center.y + dy / norm * 28.0
    sz = center.z + args.height
    horiz = math.hypot(center.x - sx, center.y - sy)
    yaw = math.degrees(math.atan2(center.y - sy, center.x - sx))
    pitch = -math.degrees(math.atan2(sz - center.z, max(0.1, horiz)))
    world.get_spectator().set_transform(carla.Transform(carla.Location(x=sx, y=sy, z=sz), carla.Rotation(pitch=pitch, yaw=yaw, roll=0.0)))
    life = max(1.0, args.seconds)
    dbg = world.debug
    ground_center = carla.Location(x=center.x, y=center.y, z=center.z + 0.5)
    rsu_mark = carla.Location(x=base.location.x, y=base.location.y, z=base.location.z)
    dbg.draw_point(ground_center, size=0.45, color=carla.Color(255, 255, 0), life_time=life)
    dbg.draw_string(carla.Location(x=center.x, y=center.y, z=center.z + 2.0), "JUNCTION CENTER", draw_shadow=True, color=carla.Color(255, 255, 0), life_time=life, persistent_lines=False)
    dbg.draw_line(carla.Location(x=base.location.x, y=base.location.y, z=center.z), rsu_mark, thickness=0.12, color=carla.Color(255, 0, 255), life_time=life)
    dbg.draw_point(rsu_mark, size=0.55, color=carla.Color(255, 0, 255), life_time=life)
    dbg.draw_string(carla.Location(x=rsu_mark.x, y=rsu_mark.y, z=rsu_mark.z + 1.0), "RSU_001", draw_shadow=True, color=carla.Color(255, 0, 255), life_time=life, persistent_lines=False)
    cam_end = forward_endpoint(camera_t, 35.0)
    radar_end = forward_endpoint(radar_t, 35.0)
    dbg.draw_line(camera_t.location, cam_end, thickness=0.15, color=carla.Color(0, 255, 0), life_time=life)
    dbg.draw_string(cam_end, "CAMERA ->", draw_shadow=True, color=carla.Color(0, 255, 0), life_time=life, persistent_lines=False)
    dbg.draw_line(radar_t.location, radar_end, thickness=0.15, color=carla.Color(255, 0, 0), life_time=life)
    dbg.draw_string(radar_end, "RADAR ->", draw_shadow=True, color=carla.Color(255, 0, 0), life_time=life, persistent_lines=False)
    for deg in range(0, 360, 30):
        r = math.radians(deg)
        end = carla.Location(x=lidar_t.location.x + 8.0 * math.cos(r), y=lidar_t.location.y + 8.0 * math.sin(r), z=lidar_t.location.z)
        dbg.draw_line(lidar_t.location, end, thickness=0.04, color=carla.Color(0, 255, 255), life_time=life)
    dbg.draw_string(carla.Location(x=lidar_t.location.x, y=lidar_t.location.y, z=lidar_t.location.z + 1.8), "LiDAR 360", draw_shadow=True, color=carla.Color(0, 255, 255), life_time=life, persistent_lines=False)
    print("Spectator moved above the RSU junction.")
    print("Markers: MAGENTA=RSU, GREEN=Camera, RED=Radar, CYAN=LiDAR, YELLOW=junction center")
    print("Markers remain visible for %.0f seconds." % life)
    time.sleep(1.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
