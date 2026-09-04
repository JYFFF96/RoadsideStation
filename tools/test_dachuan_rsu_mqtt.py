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
    parser.add_argument("--host",default="localhost");parser.add_argument("--port",type=int,default=1883)
    parser.add_argument("--device-id",default="DC887-002047")
    parser.add_argument("--rsi-id",default="RSU_0001",
                        help="CSAE RSI id: exactly 8 ASCII bytes")
    parser.add_argument("--username",default=None)
    parser.add_argument("--password-env",default="ROADSIDE_MQTT_PASSWORD")
    parser.add_argument("--latitude",type=float,required=True)
    parser.add_argument("--longitude",type=float,required=True)
    parser.add_argument("--slw-sign-type",type=int,default=None)
    parser.add_argument("--avw-event-type",type=int,default=37)
    parser.add_argument("--wait-seconds",type=float,default=3.0)
    args=parser.parse_args()
    mqtt_config={"enabled":True,"host":args.host,"port":args.port,
        "username":args.username,"password_env":args.password_env,"qos":2,
        "client_id":"roadside-rsu-preflight","response_topic":"command///res/#"}
    bridge=DachuanRsuBridge({"enabled":True,
        "reference_latitude_deg":args.latitude,
        "reference_longitude_deg":args.longitude,"slw_sign_type":args.slw_sign_type,
        "avw_event_type":args.avw_event_type,
        "device_id":args.device_id,"rsi_id":args.rsi_id})
    bridge.set_world_origin(0,0,0);publisher=MqttPublisher(mqtt_config);publisher.connect()
    try:
        if args.scenario=="vrucw":
            obj=FusedObject("test_person_1",object_type="person",size=[.6,.5,1.7],
                            sources=["camera"])
            packet=bridge.build_rsm(FusedObjectList("RSU_001",[obj],time.time()))
        elif args.scenario=="avw":
            packet=bridge.build_rsi({"type":"event","data":{"category":"AVW",
                "event_count":1,"event_x":0.0,"event_y":0.0,"event_z":0.0,
                "description":"请注意前方异常车辆"}})
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
