from __future__ import print_function

import json
import os
import unittest

import yaml

from roadside.camera_objects import CameraObject
from roadside.fused_objects import build_fused_object_list
from roadside.fusion import SimpleFusion
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

    def test_default_profile_has_opposite_cameras_and_background_learning(self):
        root = os.path.dirname(os.path.dirname(__file__))
        with open(os.path.join(root, "config", "roadside.yaml"), "r") as stream:
            config = yaml.safe_load(stream)
        cameras=config["cameras"]
        self.assertEqual(["CAM_NORTH","CAM_SOUTH"],[x["id"] for x in cameras])
        self.assertEqual([0.0,180.0],[x["transform"]["yaw"] for x in cameras])
        self.assertTrue(config["fusion"]["background_filter_enabled"])

    def test_strongest_of_two_camera_associations_is_published(self):
        tracks=[{"id":1,"x":0,"y":0,"z":1,"extent":[4,2,1.5],
                 "sources":["lidar"],"confidence":.7}]
        cameras=[CameraObject("CAM_NORTH","car",.8,[0,0,20,20]),
                 CameraObject("CAM_SOUTH","truck",.9,[0,0,30,30])]
        pairs=[{"lidar_index":0,"camera_index":0,"iou":.1,"center_distance":5},
               {"lidar_index":0,"camera_index":1,"iou":.4,"center_distance":10}]
        fused=build_fused_object_list("R",tracks,1.0,cameras,pairs)
        self.assertEqual("truck",fused.objects[0].object_type)
        self.assertEqual("CAM_SOUTH",fused.objects[0].camera["cameraId"])

    def test_background_filter_uses_prefixed_runtime_config(self):
        fusion=SimpleFusion("R",{
            "background_calibration_seconds":2.5,
            "background_cell_size":.7,
            "background_occupancy_ratio":.6,
            "background_moving_radar_speed":1.8,
            "background_neighbor_radius_cells":1})
        self.assertEqual(2.5,fusion.background.calibration_seconds)
        self.assertEqual(.7,fusion.background.cell_size)
        self.assertEqual(.6,fusion.background.occupancy_ratio)
        self.assertEqual(1.8,fusion.background.moving_radar_speed)
        self.assertEqual(1,fusion.background.neighbor_radius_cells)


if __name__ == "__main__":
    unittest.main()
