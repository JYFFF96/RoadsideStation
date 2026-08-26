from __future__ import print_function

import json
import unittest

from roadside.dachuan_rsu import DachuanRsuBridge
from roadside.fused_objects import FusedObject,FusedObjectList


class DachuanRsuBridgeTest(unittest.TestCase):
    def _bridge(self,**overrides):
        config={"enabled":True,"reference_latitude_deg":39.0,
                "reference_longitude_deg":116.0,"rsm_publish_hz":10}
        config.update(overrides);bridge=DachuanRsuBridge(config)
        bridge.set_world_origin(10.0,20.0,1.0);return bridge

    def test_enabled_bridge_requires_surveyed_reference(self):
        with self.assertRaises(ValueError):DachuanRsuBridge({"enabled":True})

    def test_rsm_matches_vendor_envelope_and_units(self):
        bridge=self._bridge()
        obj=FusedObject("person_7",object_type="person",x=10.0,y=20.0,
            vx=1.0,vy=0.0,size=[.6,.5,1.7],sources=["camera"])
        topic,payload=bridge.build_rsm(FusedObjectList("R",[obj],12.5))
        message=json.loads(payload);ptc=message["value"]["participants"][0]
        self.assertTrue(topic.startswith("command/dachuan/DC887-002047/req/"))
        self.assertTrue(topic.endswith("/rsm"))
        self.assertEqual(("RSM","RSM"),(message["type"],message["value"]["category"]))
        self.assertEqual((3,7,3),(ptc["ptcType"],ptc["ptcId"],ptc["source"]))
        self.assertEqual((50,7200),(ptc["pos"]["speed"],ptc["pos"]["heading"]))
        self.assertEqual(-10,ptc["pos"]["offsetV"]["elevation"])
        self.assertEqual((50,60,34),(ptc["pos"]["size"]["width"],
            ptc["pos"]["size"]["length"],ptc["pos"]["size"]["height"]))

    def test_unknown_vehicle_sized_obstacle_is_not_sent_as_rsm_participant(self):
        bridge=self._bridge()
        obj=FusedObject("track_x",size=[4.5,1.9,1.6],x=10,y=20)
        payload=json.loads(bridge.build_rsm(FusedObjectList("R",[obj],1))[1])
        self.assertEqual([],payload["value"]["participants"])

    def test_rsm_filters_before_applying_sixteen_participant_limit(self):
        bridge=self._bridge()
        unknown=[FusedObject("unknown_%d"%i,x=10,y=20) for i in range(16)]
        people=[FusedObject("person_%d"%i,object_type="person",x=10,y=20,
                            size=[.6,.5,1.7],sources=["camera"])
                for i in range(20)]
        payload=json.loads(bridge.build_rsm(
            FusedObjectList("R",unknown+people,1))[1])
        participants=payload["value"]["participants"]
        self.assertEqual(16,len(participants))
        self.assertTrue(all(item["ptcType"]==3 for item in participants))

    def test_rsm_rate_limit(self):
        bridge=self._bridge();items=FusedObjectList("R",[],1.0)
        self.assertIsNotNone(bridge.build_rsm(items));items.timestamp=1.05
        self.assertIsNone(bridge.build_rsm(items));items.timestamp=1.11
        self.assertIsNotNone(bridge.build_rsm(items))

    def test_world_axis_rotation_changes_geodetic_and_heading(self):
        bridge=self._bridge(world_x_heading_from_east_deg=90)
        obj=FusedObject("person_1",object_type="person",x=11,y=20,
                        vx=1,vy=0,size=[.5,.5,1.7])
        message=json.loads(bridge.build_rsm(FusedObjectList("R",[obj],1))[1])
        ptc=message["value"]["participants"][0]
        self.assertGreater(ptc["pos"]["offsetLL"]["position_LatLon"]["lat"],390000000)
        self.assertEqual(0,ptc["pos"]["heading"])

    def test_hlw_becomes_rsi_rte(self):
        bridge=self._bridge()
        event={"type":"event","data":{"category":"HLW","event_count":4,
            "event_type":37,"description":"道路存在障碍物"}}
        topic,payload=bridge.build_rsi(event);message=json.loads(payload)
        self.assertTrue(topic.startswith("command/traffic/event/req/"))
        self.assertTrue(topic.endswith("/rsi"))
        rte=message["value"]["rtes"][0]
        self.assertEqual(("RSI",37,5),(message["type"],rte["eventType"],rte["eventSource"]))

    def test_slw_requires_vendor_sign_type(self):
        event={"data":{"category":"SLW","event_count":2,"speed_limit":40}}
        bridge=self._bridge();self.assertIsNone(bridge.build_rsi(event))
        self.assertEqual("missing_slw_sign_type",bridge.last_diagnostic["suppressed"])
        bridge=self._bridge(slw_sign_type=88)
        message=json.loads(bridge.build_rsi(event)[1])
        self.assertEqual((88,"限速40km/h"),(message["value"]["rtss"][0]["signType"],
            message["value"]["rtss"][0]["description"]))

    def test_participant_warning_is_carried_by_rsm_not_rsi(self):
        bridge=self._bridge()
        self.assertIsNone(bridge.build_rsi({"data":{"category":"VRUCW"}}))
        self.assertEqual("participant_warning_uses_rsm",
                         bridge.last_diagnostic["suppressed"])

    def test_field_device_id_is_configurable(self):
        bridge=self._bridge(device_id="RSU-0130")
        topic=bridge.build_rsm(FusedObjectList("R",[],1))[0]
        self.assertTrue(topic.startswith("command/dachuan/RSU-0130/req/"))

    def test_rsi_topic_can_be_overridden_independently_from_rsm(self):
        bridge=self._bridge(rsi_topic_template="command/custom/{uuid}/{message_type}")
        event={"data":{"category":"HLW","event_type":37}}
        topic=bridge.build_rsi(event)[0]
        self.assertTrue(topic.startswith("command/custom/"))
        self.assertTrue(topic.endswith("/rsi"))


if __name__=="__main__":unittest.main()
