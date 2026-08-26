from __future__ import print_function

import argparse
import os
import sys
import time

ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:sys.path.insert(0,ROOT)

from roadside.dachuan_rsu import DachuanRsuBridge
from roadside.fused_objects import FusedObject,FusedObjectList
from roadside.mqtt_pub import MqttPublisher


def main():
    parser=argparse.ArgumentParser(description="Dachuan RSU MQTT/PC5 input preflight")
    parser.add_argument("--scenario",choices=["vrucw","hlw","avw","slw"],required=True)
    parser.add_argument("--host",required=True);parser.add_argument("--port",type=int,default=1883)
    parser.add_argument("--username",default=None)
    parser.add_argument("--password-env",default="ROADSIDE_MQTT_PASSWORD")
    parser.add_argument("--latitude",type=float,required=True)
    parser.add_argument("--longitude",type=float,required=True)
    parser.add_argument("--slw-sign-type",type=int,default=None)
    parser.add_argument("--wait-seconds",type=float,default=3.0)
    args=parser.parse_args()
    mqtt_config={"enabled":True,"host":args.host,"port":args.port,
        "username":args.username,"password_env":args.password_env,"qos":2,
        "client_id":"roadside-rsu-preflight","response_topic":"command///res/#"}
    bridge=DachuanRsuBridge({"enabled":True,
        "reference_latitude_deg":args.latitude,
        "reference_longitude_deg":args.longitude,"slw_sign_type":args.slw_sign_type})
    bridge.set_world_origin(0,0,0);publisher=MqttPublisher(mqtt_config);publisher.connect()
    try:
        if args.scenario=="vrucw":
            obj=FusedObject("test_person_1",object_type="person",size=[.6,.5,1.7],
                            sources=["camera"])
            packet=bridge.build_rsm(FusedObjectList("RSU_001",[obj],time.time()))
        elif args.scenario=="avw":
            obj=FusedObject("test_stopped_vehicle_1",object_type="car",
                            size=[4.5,1.9,1.6],sources=["lidar","camera"])
            packet=bridge.build_rsm(FusedObjectList("RSU_001",[obj],time.time()))
        elif args.scenario=="hlw":
            packet=bridge.build_rsi({"type":"event","data":{"category":"HLW",
                "event_count":1,"event_type":37,"description":"道路存在障碍物"}})
        else:
            packet=bridge.build_rsi({"type":"event","data":{"category":"SLW",
                "event_count":1,"speed_limit":40,"description":"请注意限速"}})
        if packet is None:
            parser.error("scenario payload was suppressed; SLW requires --slw-sign-type")
        publisher.publish(packet[0],packet[1])
        print("[RSU MQTT PREFLIGHT TX] scenario=%s topic=%s"%(args.scenario.upper(),packet[0]))
        print("[RSU MQTT PREFLIGHT PAYLOAD] %s"%packet[1])
        print("Waiting %.1fs for command///res/{UUID}/200..."%max(0.0,args.wait_seconds))
        time.sleep(max(0.0,args.wait_seconds))
    finally:publisher.close()


if __name__=="__main__":main()
