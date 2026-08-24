from __future__ import print_function
import json
import unittest

from roadside.models import DetectedObject,ObjectList
from roadside.v2x_events import V2XEventEngine,encode_v2x_event


class V2XEventTest(unittest.TestCase):
    def test_vrucw_requires_presence_confirmation_and_matches_manual(self):
        engine=V2XEventEngine("RSU_001",{"enabled":True,
            "vrucw":{"enabled":True,"required_updates":2,"direction":1},
            "avw":{"enabled":False},"slw":{"enabled":False}})
        person=DetectedObject("person_fragment_1",1,2,object_type="person")
        self.assertEqual([],engine.update(ObjectList("RSU_001",[person],100.0)))
        event=engine.update(ObjectList("RSU_001",[person],100.1),
                            {"speed_kmh":5})[0]
        data=event["data"]
        self.assertEqual(("VRUCW",10,3),
                         (data["category"],data["event_sort"],data["ptc_type"]))
        self.assertEqual(3,data["spc_type"])
        self.assertEqual(5,data["speed"])
        self.assertEqual("road_presence",data["trigger_mode"])

    def test_vrucw_aggregates_fragmented_ids_into_one_event(self):
        engine=V2XEventEngine("R",{"enabled":True,"cooldown_seconds":5,
            "vrucw":{"enabled":True,"required_updates":1},
            "avw":{"enabled":False},"slw":{"enabled":False}})
        fragments=[DetectedObject("person_1",0,0,object_type="person"),
                   DetectedObject("person_9",.2,.1,object_type="person")]
        events=engine.update(ObjectList("R",fragments,10.0))
        self.assertEqual(1,len(events))
        self.assertEqual(2,events[0]["data"]["participant_count"])
        self.assertEqual([],engine.update(ObjectList("R",fragments,11.0)))

    def test_avw_requires_vehicle_dwell_and_uses_manual_fields(self):
        engine=V2XEventEngine("RSU_001",{"enabled":True,"cooldown_seconds":5,
            "avw":{"enabled":True,"dwell_seconds":3,"max_stationary_speed_mps":.5},
            "slw":{"enabled":False}})
        obj=DetectedObject("vehicle_1",1,2,vx=.1,object_type="vehicle")
        self.assertEqual([],engine.update(ObjectList("RSU_001",[obj],100.0)))
        events=engine.update(ObjectList("RSU_001",[obj],103.0))
        self.assertEqual(1,len(events));data=events[0]["data"]
        self.assertEqual(("AVW",6,"RSU_001"),(data["category"],data["event_sort"],data["sid"]))
        self.assertEqual("vehicle_1",data["object_id"])
        self.assertEqual("event",json.loads(encode_v2x_event(events[0]))["type"])

    def test_moving_vehicle_resets_avw_dwell(self):
        engine=V2XEventEngine("R",{"enabled":True,"avw":{"dwell_seconds":2},
                                    "slw":{"enabled":False}})
        stopped=DetectedObject("v",0,0,object_type="vehicle")
        moving=DetectedObject("v",0,0,vx=2,object_type="vehicle")
        engine.update(ObjectList("R",[stopped],1));engine.update(ObjectList("R",[moving],2))
        self.assertEqual([],engine.update(ObjectList("R",[stopped],3)))

    def test_slw_marks_overspeed(self):
        engine=V2XEventEngine("R",{"enabled":True,"avw":{"enabled":False},
            "slw":{"enabled":True,"speed_limit_kmh":40}})
        event=engine.update(ObjectList("R",[],10),{"speed_kmh":51})[0]
        self.assertEqual("SLW",event["data"]["category"])
        self.assertEqual(9,event["data"]["event_sort"])
        self.assertEqual(2,event["data"]["spd_Flag"])


if __name__=="__main__":unittest.main()
