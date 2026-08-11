from __future__ import print_function

import time

import yaml

from roadside.carla_station import CarlaRoadsideStation
from roadside.fusion import SimpleFusion
from roadside.messages import encode_object_list, encode_rsm
from roadside.mqtt_pub import MqttPublisher


def load_config(path="config/roadside.yaml"):
    with open(path, "r") as fp:
        return yaml.safe_load(fp)


def main():
    config = load_config()
    station_id = config["station"]["id"]
    station = CarlaRoadsideStation(config)
    fusion = SimpleFusion(station_id, config["fusion"])
    publisher = MqttPublisher(config["mqtt"])

    print("RoadsideStation V0.2.1 starting...")
    station.start()
    fusion.set_world_transform(station.lidar_transform)
    fusion.set_candidate_validator(station.is_driving_roi)
    publisher.connect()
    print("CARLA roadside sensors started: %d" % len(station.sensors))
    if station.lidar_transform is not None:
        print("Object coordinates: CARLA world frame (LiDAR transformed to world)")
    print("Road ROI filter: enabled (CARLA driving-lane map used only as ROI)")

    last_debug = 0.0
    try:
        while True:
            camera, lidar, radar = station.cache.snapshot()
            lidar_points = lidar[1] if lidar else None
            radar_detections = radar[1] if radar else None
            object_list = fusion.fuse(lidar_points, radar_detections)

            object_json = encode_object_list(object_list)
            rsm_json = encode_rsm(object_list)

            now = time.time()
            if now - last_debug >= 1.0:
                stats = fusion.last_stats
                camera_frame = camera[0] if camera else "-"
                print("[RSU %s | %s] Camera:%s  LiDAR:%d pts/%d clusters -> ROI:%d  Radar:%d  Tracks:%d" % (
                    station_id,
                    station.map_name,
                    camera_frame,
                    stats["lidar_points"],
                    stats["lidar_clusters"],
                    stats["roi_candidates"],
                    stats["radar_detections"],
                    stats["tracked_objects"]))
                for obj in object_list.objects[:10]:
                    print("  %-12s Xw=%8.2f Yw=%8.2f Zw=%6.2f vx=%6.2f vy=%6.2f conf=%.2f src=%s" % (
                        obj.object_id, obj.x, obj.y, obj.z, obj.vx, obj.vy,
                        obj.confidence, "+".join(obj.sources)))
                if len(object_list.objects) > 10:
                    print("  ... %d more objects" % (len(object_list.objects) - 10))
                last_debug = now

            mqtt_cfg = config["mqtt"]
            publisher.publish(mqtt_cfg["topic_object_list"], object_json)
            publisher.publish(mqtt_cfg["topic_rsm"], rsm_json)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping RoadsideStation...")
    finally:
        publisher.close()
        station.stop()


if __name__ == "__main__":
    main()
