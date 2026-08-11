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
    fusion = SimpleFusion(station_id, config["fusion"]["max_match_distance"])
    publisher = MqttPublisher(config["mqtt"])

    print("RoadsideStation V0.1 starting...")
    station.start()
    publisher.connect()
    print("CARLA roadside sensors started: %d" % len(station.sensors))

    try:
        while True:
            camera, lidar, radar = station.cache.snapshot()
            lidar_points = lidar[1] if lidar else None
            radar_detections = radar[1] if radar else None
            object_list = fusion.fuse(lidar_points, radar_detections)

            object_json = encode_object_list(object_list)
            rsm_json = encode_rsm(object_list)
            print("Objects: %d | %s" % (len(object_list.objects), rsm_json))

            mqtt_cfg = config["mqtt"]
            publisher.publish(mqtt_cfg["topic_object_list"], object_json)
            publisher.publish(mqtt_cfg["topic_rsm"], rsm_json)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Stopping RoadsideStation...")
    finally:
        publisher.close()
        station.stop()


if __name__ == "__main__":
    main()
