from __future__ import print_function

import json
import os
import unittest

import yaml

from roadside.camera_objects import CameraObject
from roadside.fused_objects import build_fused_object_list
from roadside.messages import encode_object_list, encode_rsm
from roadside.v2x_events import V2XEventEngine


class FusedOutputBoundaryTest(unittest.TestCase):
    def _fused(self):
        tracks = [{
            "id": 20, "x": 12.0, "y": -1.5, "z": 0.8,
            "vx": 3.0, "vy": 4.0, "extent": [1.8, 4.4, 1.6],
            "confidence": .72, "sources": ["lidar", "radar"],
            "radar_radial_velocity": 4.5, "track_hits": 7,
            "track_state": "confirmed",
        }]
        cameras = [CameraObject("CAM_01", "car", .91, [10, 20, 110, 180])]
        pairs = [{"lidar_index": 0, "camera_index": 0,
                  "iou": .50, "center_distance": 8.0}]
        return build_fused_object_list(
            "RSU_001", tracks, 100.25, cameras, pairs,
            frame_id=123, coordinate_frame="carla_world")

    def test_object_list_publishes_post_association_fields(self):
        payload = json.loads(encode_object_list(self._fused()))
        self.assertEqual("FusedObjectList", payload["msgType"])
        self.assertEqual("V1.0", payload["version"])
        self.assertEqual(123, payload["frameId"])
        self.assertEqual(100250, payload["timestampMs"])
        obj = payload["objects"][0]
        self.assertEqual("car", obj["type"])
        self.assertEqual({"length": 4.4, "width": 1.8, "height": 1.6}, obj["size"])
        self.assertEqual(7, obj["age"])
        self.assertEqual(["lidar", "radar", "camera"], obj["sources"])
        self.assertEqual(5.0, obj["speedMps"])

    def test_local_rsm_is_derived_from_same_fused_objects(self):
        payload = json.loads(encode_rsm(self._fused()))
        self.assertEqual("V0.2-local-json", payload["version"])
        self.assertEqual(123, payload["frameId"])
        ptc = payload["participants"][0]
        self.assertEqual((1, "car"), (ptc["ptcType"], ptc["typeName"]))
        self.assertEqual(5.0, ptc["speedMps"])
        self.assertEqual(4.4, ptc["sizeM"]["length"])
        self.assertIn("camera", ptc["sources"])

    def test_event_engine_consumes_fused_type(self):
        fused = self._fused()
        fused.objects[0].vx = 0.0
        fused.objects[0].vy = 0.0
        engine = V2XEventEngine("RSU_001", {
            "enabled": True, "cooldown_seconds": 5,
            "avw": {"enabled": True, "dwell_seconds": 3,
                    "max_stationary_speed_mps": .5},
            "slw": {"enabled": False},
        })
        self.assertEqual([], engine.update(fused))
        fused.timestamp += 3.0
        events = engine.update(fused)
        self.assertEqual("20", events[0]["data"]["object_id"])

    def test_default_lidar_profile_matches_fairy_48tx(self):
        root = os.path.dirname(os.path.dirname(__file__))
        with open(os.path.join(root, "config", "roadside.yaml"), "r") as stream:
            lidar = yaml.safe_load(stream)["lidar"]
        self.assertEqual(48, lidar["channels"])
        self.assertEqual(690000, lidar["points_per_second"])
        self.assertEqual(10.0, lidar["rotation_frequency"])
        self.assertEqual((-15.84, 15.84),
                         (lidar["lower_fov"], lidar["upper_fov"]))


if __name__ == "__main__":
    unittest.main()
