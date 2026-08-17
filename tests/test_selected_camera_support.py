from __future__ import print_function

import unittest

from roadside.camera_objects import CameraObject
from roadside.selected_camera_support import (annotate_selected_camera_support,
                                                selected_camera_rescue_passes)


class _Projector(object):
    def project(self, x, y, z):
        return {"u": 100.0 + float(x) * 10.0,
                "v": 100.0 + float(y) * 10.0}


class _RightProjector(object):
    def project(self,x,y,z):return {"u":300.0,"v":100.0}


class SelectedCameraSupportTest(unittest.TestCase):
    def test_visible_hold_gets_generic_camera_support_on_copy(self):
        original = {"x": 0.0, "y": 0.0, "z": 1.0,
                    "extent": [1.0, 1.0, 1.0]}
        camera = CameraObject("CAM_01", "person", .91, [93, 93, 107, 107])
        annotated, stats = annotate_selected_camera_support(
            [original], _Projector(), [camera], 200, 200,
            camera_source="detector", min_iou=.05, max_center_distance=20.0)
        self.assertNotIn("selected_track_admission_camera_visible", original)
        self.assertTrue(annotated[0]["selected_track_admission_camera_visible"])
        self.assertTrue(annotated[0]["selected_track_admission_camera_supported"])
        self.assertEqual("person", annotated[0]["selected_track_admission_camera_class"])
        self.assertEqual(.91, annotated[0]["selected_track_admission_camera_confidence"])
        self.assertEqual("person",annotated[0]["selected_track_admission_camera_nearest_class"])
        self.assertEqual(0.0,annotated[0]["selected_track_admission_camera_nearest_distance"])
        self.assertEqual({"held": 1, "visible": 1, "supported": 1,
                          "source": "detector"}, stats)

    def test_nearest_box_is_recorded_outside_support_gate(self):
        candidate={"x":0.0,"y":0.0,"z":1.0,"extent":[1.0,1.0,1.0]}
        camera=CameraObject("CAM_01","car",.77,[170,93,184,107])
        annotated,stats=annotate_selected_camera_support(
            [candidate],_Projector(),[camera],200,200,
            camera_source="detector",min_iou=.05,max_center_distance=20.0)
        self.assertTrue(annotated[0]["selected_track_admission_camera_visible"])
        self.assertFalse(annotated[0]["selected_track_admission_camera_supported"])
        self.assertEqual("car",annotated[0]["selected_track_admission_camera_nearest_class"])
        self.assertEqual(77.0,annotated[0]["selected_track_admission_camera_nearest_distance"])

    def test_no_projector_records_unsupported_source(self):
        annotated, stats = annotate_selected_camera_support(
            [{"x": 1.0}], None, [], 0, 0, camera_source="none")
        self.assertFalse(annotated[0]["selected_track_admission_camera_visible"])
        self.assertFalse(annotated[0]["selected_track_admission_camera_supported"])
        self.assertEqual(0, stats["visible"])

    def test_offscreen_projection_reason_is_recorded(self):
        candidate={"x":0.0,"y":0.0,"z":1.0,"extent":[1.0,1.0,1.0]}
        annotated,stats=annotate_selected_camera_support(
            [candidate],_RightProjector(),[],200,200,camera_source="detector")
        self.assertFalse(annotated[0]["selected_track_admission_camera_visible"])
        self.assertEqual("right",annotated[0]["selected_track_admission_camera_projection_rejection"])

    def test_strong_rescue_requires_person_and_close_or_overlapping_box(self):
        base = {"selected_track_admission_camera_supported": True,
                "selected_track_admission_camera_class": "person",
                "selected_track_admission_camera_iou": .01,
                "selected_track_admission_camera_center_distance": 35.0}
        self.assertTrue(selected_camera_rescue_passes(base, .05, 45.0))
        self.assertFalse(selected_camera_rescue_passes(base, .05, 30.0))
        overlap = dict(base);overlap["selected_track_admission_camera_iou"] = .1
        overlap["selected_track_admission_camera_center_distance"] = 90.0
        self.assertTrue(selected_camera_rescue_passes(overlap, .05, 30.0))
        vehicle = dict(overlap);vehicle["selected_track_admission_camera_class"] = "car"
        self.assertFalse(selected_camera_rescue_passes(vehicle, .05, 30.0))

    def test_detector_calibration_shadows_separate_observed_distances(self):
        person = {"selected_track_admission_camera_supported":True,
                  "selected_track_admission_camera_class":"person",
                  "selected_track_admission_camera_center_distance":94.74,
                  "selected_track_admission_camera_iou":0.0}
        first_fp = {"selected_track_admission_camera_supported":True,
                    "selected_track_admission_camera_class":"person",
                    "selected_track_admission_camera_center_distance":107.56,
                    "selected_track_admission_camera_iou":0.0}
        self.assertFalse(selected_camera_rescue_passes(person, .05, 90.0))
        self.assertTrue(selected_camera_rescue_passes(person, .05, 100.0))
        self.assertFalse(selected_camera_rescue_passes(first_fp, .05, 100.0))
        second_person = dict(person)
        second_person["selected_track_admission_camera_center_distance"] = 104.18
        first_fp["selected_track_admission_camera_center_distance"] = 108.90
        self.assertTrue(selected_camera_rescue_passes(second_person, .05, 105.0))
        self.assertFalse(selected_camera_rescue_passes(first_fp, .05, 105.0))
        self.assertTrue(selected_camera_rescue_passes(first_fp, .05, 110.0))


if __name__ == "__main__":
    unittest.main()
