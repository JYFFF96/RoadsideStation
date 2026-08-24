from __future__ import print_function

import unittest
import numpy as np

from roadside.camera_ground_initiation import (
    CameraGroundInitiationShadow, camera_box_ground_point)
from roadside.ground_truth_eval import GroundTruthEvaluator
from roadside.fusion import SimpleFusion
from roadside.tracking import NearestTracker


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

    def test_counterfactual_reports_incremental_recall(self):
        evaluator=GroundTruthEvaluator(None,lambda:_Center(),{
            "radius":80.0,"match_distance":2.0})
        evaluator.truth_objects=lambda:[
            {"x":10.0,"y":0.0,"object_type":"car"},
            {"x":16.0,"y":0.0,"object_type":"person"}]
        report=evaluator.observe_camera_ground_counterfactual(
            [{"x":10.1,"y":0.0}],
            [{"x":16.2,"y":0.0,"camera_source":"detector"}],frame_id=11)
        self.assertEqual(1,report["base_matched"])
        self.assertEqual(1,report["incremental_matched"])
        self.assertEqual(1.0,report["combined_recall"])
        self.assertEqual(.5,report["recall_gain"])

    def test_deployment_verdict_blocks_carla_truth_source(self):
        evaluator=GroundTruthEvaluator(None,lambda:_Center(),{
            "radius":80.0,"match_distance":2.0,
            "camera_ground_deployment_min_candidates":1,
            "camera_ground_deployment_min_precision":.9,
            "camera_ground_deployment_min_recall_gain":.1})
        evaluator.truth_objects=lambda:[
            {"x":16.0,"y":0.0,"object_type":"person"}]
        evaluator.observe_camera_ground_counterfactual([], [
            {"x":16.1,"y":0.0,"camera_source":"carla_truth"}],frame_id=11)
        verdict=evaluator.camera_ground_deployment_verdict()
        self.assertEqual("BLOCKED_CARLA_TRUTH",verdict["status"])
        self.assertFalse(verdict["checks"]["detector_source"])

    def test_deployment_verdict_distinguishes_missing_detector_evidence(self):
        evaluator=GroundTruthEvaluator(None,lambda:_Center(),{
            "radius":80.0,"match_distance":2.0})
        evaluator.truth_objects=lambda:[]
        evaluator.observe_camera_ground_counterfactual([],[],frame_id=11)
        self.assertEqual("BLOCKED_NO_DETECTOR_EVIDENCE",
                         evaluator.camera_ground_deployment_verdict()["status"])

    def test_tracker_queue_fails_closed_for_carla_truth(self):
        fusion=SimpleFusion("RSU_TEST",{
            "camera_ground_temporal_enforce_enabled":True,
            "camera_ground_temporal_enforce_required_source":"detector"})
        candidate={"x":16.0,"y":0.0,"camera_source":"carla_truth",
                   "camera_ground_temporal_confirmed":True}
        self.assertEqual([],fusion.queue_camera_ground_initiations(
            [candidate],"carla_truth"))
        self.assertEqual([],fusion._consume_camera_ground_initiations([]))
        self.assertEqual(1,fusion.camera_ground_tracker_stats[
            "source_rejected_total"])

    def test_real_detector_candidate_is_queued_and_lidar_wins_dedupe(self):
        fusion=SimpleFusion("RSU_TEST",{
            "camera_ground_temporal_enforce_enabled":True,
            "camera_ground_temporal_enforce_required_source":"detector",
            "camera_ground_initiation_dedupe_distance":3.0})
        candidate={"x":16.0,"y":0.0,"z":.85,"extent":[.6,.6,1.7],
                   "object_type":"person","camera_source":"detector",
                   "camera_ground_temporal_confirmed":True}
        queued=fusion.queue_camera_ground_initiations([candidate],"detector")
        self.assertEqual(1,len(queued));self.assertEqual(["camera"],queued[0]["sources"])
        self.assertEqual([],fusion._consume_camera_ground_initiations([
            {"x":16.2,"y":0.0,"sources":["lidar"]}]))
        self.assertEqual(1,fusion.camera_ground_tracker_stats[
            "dedupe_rejected_total"])

    def test_unproven_detector_class_fails_closed(self):
        fusion=SimpleFusion("RSU_TEST",{
            "camera_ground_temporal_enforce_enabled":True,
            "camera_ground_temporal_enforce_required_source":"detector",
            "camera_ground_temporal_enforce_allowed_classes":["person"]})
        candidate={"x":16.0,"y":0.0,"object_type":"car",
                   "camera_source":"detector",
                   "camera_ground_temporal_confirmed":True}
        self.assertEqual([],fusion.queue_camera_ground_initiations(
            [candidate],"detector"))
        self.assertEqual(1,fusion.camera_ground_tracker_stats[
            "class_rejected_total"])

    def test_camera_only_measurement_updates_tracker_sensor_quality(self):
        tracker=NearestTracker()
        track=tracker.update([{
            "x":16.0,"y":0.0,"z":.85,"extent":[.6,.6,1.7],
            "confidence":.9,"sources":["camera"],"object_type":"person",
            "camera_ground_initiated":True,"camera_ground_tracker_enforced":True,
            "candidate_score_bypass":True,
            "sensor_range":16.0}],timestamp=1.0)[0]
        self.assertEqual("C",track["track_sensors"])
        self.assertEqual(1,track["track_camera_hits"])
        self.assertEqual(0,track["track_lidar_hits"])
        self.assertEqual("person",track["object_type"])
        self.assertTrue(track["track_camera_ground_origin"])
        lidar=dict(track);lidar.update({"x":16.1,"sources":["lidar"],
                                       "camera_ground_tracker_enforced":False})
        updated=tracker.update([lidar],timestamp=1.1)[0]
        self.assertTrue(updated["track_camera_ground_origin"])
        self.assertFalse(updated["track_camera_ground_current"])
        self.assertEqual(1,updated["track_camera_ground_enforced_hits"])
        self.assertEqual(1,updated["track_lidar_hits"])

    def test_fusion_consumes_camera_candidate_into_common_tracker(self):
        fusion=SimpleFusion("RSU_TEST",{
            "camera_ground_temporal_enforce_enabled":True,
            "camera_ground_temporal_enforce_required_source":"detector",
            "background_filter_enabled":False,"cluster_merge_enabled":False,
            "range_adaptive_clustering":False,"far_track_admission_enabled":False,
            "selected_track_admission_enabled":False})
        candidate={"x":16.0,"y":0.0,"z":.85,"extent":[.6,.6,1.7],
                   "confidence":.9,"sources":["camera"],"object_type":"person",
                   "camera_source":"detector","camera_ground_initiated":True,
                   "camera_ground_temporal_confirmed":True,"sensor_range":16.0}
        fusion.queue_camera_ground_initiations([candidate],"detector")
        fusion.fuse([],[],timestamp=1.0,frame_id=1)
        self.assertEqual(1,len(fusion.last_tracked_candidates))
        track=fusion.last_tracked_candidates[0]
        self.assertEqual("person",track["object_type"])
        self.assertEqual(["camera"],track["sources"])
        self.assertEqual("C",track["track_sensors"])
        self.assertTrue(track["camera_ground_tracker_enforced"])

    def test_actual_camera_origin_track_attribution_counts_identity_fragments(self):
        evaluator=GroundTruthEvaluator(None,lambda:_Center(),{
            "radius":80.0,"match_distance":2.0})
        evaluator.truth_objects=lambda:[
            {"actor_id":7,"x":16.0,"y":0.0,"object_type":"person"}]
        report=evaluator.observe_camera_ground_enforcement([
            {"id":"vehicle_1","x":16.1,"y":0.0,"track_state":"confirmed",
             "track_camera_ground_origin":True,"track_lidar_hits":0},
            {"id":"vehicle_2","x":16.3,"y":0.0,"track_state":"new",
             "track_camera_ground_origin":True,"track_lidar_hits":1},
            {"id":"vehicle_3","x":30.0,"y":0.0,
             "track_camera_ground_origin":False}],frame_id=10)
        self.assertEqual(2,report["track_samples"])
        self.assertEqual(1,report["matched"]);self.assertEqual(1,report["fp"])
        self.assertEqual(.5,report["precision"])
        self.assertEqual(1,report["duplicate_like_fp"])
        self.assertEqual(0,report["spatial_fp"])
        self.assertEqual(1,report["unique_actors"])
        self.assertEqual(1,report["current"]["camera_only"])
        self.assertEqual(1,report["current"]["lidar_takeover"])
        report=evaluator.observe_camera_ground_enforcement([
            {"id":"vehicle_2","x":16.2,"y":0.0,"track_state":"confirmed",
             "track_camera_ground_origin":True,"track_lidar_hits":1}],frame_id=11)
        self.assertEqual(1,report["fragmented_actors"])
        self.assertEqual(1,report["id_fragments"])
        self.assertEqual(0,report["identity_switch_tracks"])
        self.assertAlmostEqual(1.5,report["avg_track_frames"])
        self.assertEqual(2,report["max_track_frames"])

    def test_camera_identity_gate_ablation_is_evaluator_only(self):
        evaluator=GroundTruthEvaluator(None,lambda:_Center(),{
            "radius":80.0,"match_distance":4.0,
            "camera_ground_identity_match_gates":[1.0,2.0,3.0,4.0]})
        evaluator.truth_objects=lambda:[
            {"actor_id":7,"x":16.0,"y":0.0,"object_type":"person"}]
        report=evaluator.observe_camera_ground_enforcement([{
            "id":"vehicle_1","x":17.5,"y":0.0,"track_state":"confirmed",
            "track_camera_ground_origin":True,"track_lidar_hits":0}],frame_id=10)
        gates=report["identity_gates"]
        self.assertEqual(0,gates["1"]["matched"])
        self.assertEqual(1,gates["1"]["spatial_fp"])
        self.assertEqual(0.0,gates["1"]["precision"])
        for gate in ("2","3","4"):
            self.assertEqual(1,gates[gate]["matched"])
            self.assertEqual(0,gates[gate]["fp"])
            self.assertEqual(1.0,gates[gate]["precision"])
            self.assertAlmostEqual(1.5,gates[gate]["error_avg"])
        # The established report remains governed by match_distance, proving
        # that the parallel gates do not alter admission or primary metrics.
        self.assertEqual(1,report["matched"])
        self.assertEqual(1.0,report["precision"])


if __name__ == "__main__":unittest.main()
