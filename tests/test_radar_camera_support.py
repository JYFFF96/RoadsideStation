from __future__ import print_function

import unittest

from roadside.ground_truth_eval import GroundTruthEvaluator
from roadside.radar_camera_support import annotate_radar_camera_support


class _Projector(object):
    def project(self, x, y, z):
        return {"u":100.0+float(x)*5.0,"v":100.0+float(z)*5.0}


class _Center(object):
    x=0.0;y=0.0


class RadarCameraSupportTests(unittest.TestCase):
    def _candidate(self):
        return {"x":8.0,"y":0.0,"z":1.0,"extent":[4.5,1.8,1.6]}

    def test_either_camera_can_support_copy_without_mutating_input(self):
        candidate=self._candidate()
        views=[
            {"camera_id":"CAM_NORTH","projector":_Projector(),
             "width":1280,"height":720,"camera_objects":[]},
            {"camera_id":"CAM_SOUTH","projector":_Projector(),
             "width":1280,"height":720,"camera_objects":[
                 {"bbox":[125,95,155,115],"className":"person",
                  "confidence":.91}]},
        ]
        result=annotate_radar_camera_support(
            [candidate],views,camera_source="detector",max_center_distance=120.0)
        self.assertNotIn("radar_camera_supported",candidate)
        self.assertTrue(result[0]["radar_camera_visible"])
        self.assertTrue(result[0]["radar_camera_supported"])
        self.assertEqual("CAM_SOUTH",result[0]["radar_camera_id"])
        self.assertEqual("person",result[0]["radar_camera_class"])

    def test_evaluator_reports_supported_truth_and_false_positive(self):
        evaluator=GroundTruthEvaluator(None,lambda:_Center(),{
            "radius":80.0,"match_distance":2.0})
        evaluator.truth_objects=lambda:[
            {"x":8.0,"y":0.0,"object_type":"pedestrian"}]
        report=evaluator.observe_radar_camera_support([
            {"x":8.2,"y":0.0,"radar_camera_visible":True,
             "radar_camera_supported":True,"radar_camera_class":"person",
             "radar_camera_source":"detector"},
            {"x":20.0,"y":0.0,"radar_camera_visible":True,
             "radar_camera_supported":True,"radar_camera_class":"car",
             "radar_camera_source":"detector"}],frame_id=10)
        self.assertEqual(2,report["candidates"])
        self.assertEqual(1,report["supported_truth"])
        self.assertEqual(1,report["supported_fp"])
        self.assertEqual(.5,report["supported_precision"])
        repeated=evaluator.observe_radar_camera_support([],frame_id=10)
        self.assertEqual(2,repeated["candidates"])


if __name__ == "__main__":
    unittest.main()
