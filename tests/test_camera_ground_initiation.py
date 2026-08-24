from __future__ import print_function

import unittest
import numpy as np

from roadside.camera_ground_initiation import (
    CameraGroundInitiationShadow, camera_box_ground_point)
from roadside.ground_truth_eval import GroundTruthEvaluator


class _Transform(object):
    def get_matrix(self):
        matrix=np.eye(4,dtype=np.float64);matrix[2,3]=8.0
        return matrix.tolist()


class _Projector(object):
    K=np.array([[640.0,0.0,640.0],[0.0,640.0,360.0],[0.0,0.0,1.0]])
    transform=_Transform()


class _Center(object):
    x=0.0;y=0.0


class CameraGroundInitiationTests(unittest.TestCase):
    def _config(self):
        return {"camera_ground_initiation_shadow_enabled":True,
                "camera_ground_initiation_min_range":2.0,
                "camera_ground_initiation_max_range":30.0,
                "camera_ground_initiation_min_confidence":.25,
                "camera_ground_initiation_dedupe_distance":3.0,
                "camera_ground_initiation_cross_camera_distance":2.0,
                "camera_ground_initiation_allowed_classes":["person"]}

    def _view(self, camera_id="CAM_NORTH"):
        return {"camera_id":camera_id,"camera_source":"detector",
                "projector":_Projector(),"frame_id":10,
                "camera_objects":[{"className":"person","confidence":.9,
                                   "bbox":[620,300,660,680]}]}

    def test_bottom_center_ray_intersects_ground_plane(self):
        point=camera_box_ground_point(_Projector(),[620,300,660,680],0.0)
        self.assertAlmostEqual(16.0,point[0]);self.assertAlmostEqual(0.0,point[1])
        self.assertAlmostEqual(0.0,point[2])

    def test_shadow_candidate_is_deduplicated_across_cameras(self):
        shadow=CameraGroundInitiationShadow(self._config())
        result=shadow.update([self._view(),self._view("CAM_SOUTH")],[],0.0,
                             validator=lambda unused:True,frame_token=10)
        self.assertEqual(1,len(result));self.assertEqual("person",result[0]["object_type"])
        self.assertEqual(1,shadow.report()["cross_camera_deduped"])
        self.assertEqual([],shadow.update([self._view()],[],0.0,frame_token=10))

    def test_lidar_dedupe_and_roi_remain_before_would_emit(self):
        duplicate=CameraGroundInitiationShadow(self._config())
        self.assertEqual([],duplicate.update(
            [self._view()],[{"x":16.2,"y":0.0}],0.0,
            validator=lambda unused:True,frame_token=10))
        self.assertEqual(1,duplicate.report()["lidar_deduped"])
        rejected=CameraGroundInitiationShadow(self._config())
        rejected.update([self._view()],[],0.0,validator=lambda unused:False,
                        frame_token=10)
        self.assertEqual(1,rejected.report()["roi_rejected"])

    def test_tuple_roi_result_is_unwrapped_and_grouped(self):
        shadow=CameraGroundInitiationShadow(self._config())
        result=shadow.update(
            [self._view()],[],0.0,
            validator=lambda item:(True,"ok",{"lateral":0.2}),frame_token=10)
        self.assertEqual(1,len(result))
        report=shadow.report()
        self.assertEqual(1,report["roi_groups"]["vru"]["accepted"])
        self.assertEqual(0,report["validator_errors"])

    def test_validator_error_is_visible_instead_of_silent(self):
        shadow=CameraGroundInitiationShadow(self._config())
        def broken(unused):
            raise TypeError("wrong callback signature")
        self.assertEqual([],shadow.update(
            [self._view()],[],0.0,validator=broken,frame_token=10))
        report=shadow.report()
        self.assertEqual(1,report["validator_errors"])
        self.assertEqual(1,report["roi_rejection_reasons"]["validator_error:TypeError"])

    def test_vru_lateral_margin_ablation_stays_separate_from_output(self):
        config=self._config()
        config["camera_ground_vru_roi_extra_margin_shadow"]=[1.0,2.0,3.0]
        shadow=CameraGroundInitiationShadow(config)
        result=shadow.update(
            [self._view()],[],0.0,
            validator=lambda item:(False,"lateral",{
                "lateral":6.25,"allowed_lateral":4.5}),frame_token=10)
        self.assertEqual([],result)
        self.assertEqual([],shadow.last_vru_roi_ablation_candidates[1.0])
        self.assertEqual(1,len(shadow.last_vru_roi_ablation_candidates[2.0]))
        self.assertEqual(1,len(shadow.last_vru_roi_ablation_candidates[3.0]))
        self.assertAlmostEqual(1.75,shadow.last_vru_roi_ablation_candidates[2.0][0][
            "vru_roi_lateral_excess"])

    def test_temporal_shadow_requires_two_distinct_frames(self):
        config=self._config()
        config.update({"camera_ground_temporal_shadow_enabled":True,
                       "camera_ground_temporal_required_frames":2,
                       "camera_ground_temporal_match_distance":2.0,
                       "camera_ground_temporal_vru_extra_margin":1.0})
        shadow=CameraGroundInitiationShadow(config)
        shadow.update([self._view()],[],0.0,validator=lambda item:(True,"ok",{}),
                      frame_token=10)
        self.assertEqual([],shadow.last_temporal_candidates)
        shadow.update([self._view()],[],0.0,validator=lambda item:(True,"ok",{}),
                      frame_token=11)
        self.assertEqual(1,len(shadow.last_temporal_candidates))
        self.assertEqual(2,shadow.last_temporal_candidates[0][
            "camera_ground_temporal_hits"])
        report=shadow.report()["temporal"]
        self.assertEqual(1,report["seeded"]);self.assertEqual(1,report["matched"])
        self.assertEqual(1,report["confirmed"])

    def test_truth_evaluation_is_isolated_and_counts_precision(self):
        evaluator=GroundTruthEvaluator(None,lambda:_Center(),{
            "radius":80.0,"match_distance":2.0})
        evaluator.truth_objects=lambda:[
            {"x":16.0,"y":0.0,"object_type":"person"}]
        report=evaluator.observe_camera_ground_initiation([
            {"x":16.2,"y":0.0,"camera_source":"detector"},
            {"x":25.0,"y":0.0,"camera_source":"detector"}],frame_id=10)
        self.assertEqual(2,report["candidates"]);self.assertEqual(1,report["matched"])
        self.assertEqual(1,report["fp"]);self.assertEqual(.5,report["precision"])

    def test_vru_roi_ablation_truth_is_reported_per_margin(self):
        evaluator=GroundTruthEvaluator(None,lambda:_Center(),{
            "radius":80.0,"match_distance":2.0})
        evaluator.truth_objects=lambda:[
            {"x":16.0,"y":0.0,"object_type":"person"}]
        report=evaluator.observe_camera_ground_vru_roi_ablation({
            1.0:[],2.0:[{"x":16.2,"y":0.0}],
            3.0:[{"x":16.2,"y":0.0},{"x":25.0,"y":0.0}]},frame_id=10)
        self.assertEqual(1,report["2"]["matched"])
        self.assertEqual(1.0,report["2"]["precision"])
        self.assertEqual(1,report["3"]["fp"])
        self.assertEqual(.5,report["3"]["precision"])

    def test_temporal_truth_evaluation_is_independent(self):
        evaluator=GroundTruthEvaluator(None,lambda:_Center(),{
            "radius":80.0,"match_distance":2.0})
        evaluator.truth_objects=lambda:[
            {"x":16.0,"y":0.0,"object_type":"person"}]
        report=evaluator.observe_camera_ground_temporal([
            {"x":16.2,"y":0.0,"camera_source":"detector"}],frame_id=11)
        self.assertEqual(1,report["matched"]);self.assertEqual(0,report["fp"])
        self.assertEqual(1.0,report["precision"])


if __name__ == "__main__":unittest.main()
