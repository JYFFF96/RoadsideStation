from __future__ import print_function

import unittest

from roadside.fusion import SimpleFusion
from roadside.ground_truth_eval import GroundTruthEvaluator


class SelectedTrackAdmissionTest(unittest.TestCase):
    def _fusion(self, shadow=True):
        return SimpleFusion("test", {
            "selected_track_admission_enabled": True,
            "selected_track_admission_shadow_mode": shadow,
            "selected_track_admission_required_frames": 2,
            "selected_track_admission_match_gate": 2.5,
            "selected_track_admission_track_gate": 4.0,
            "selected_track_admission_ttl": 0.5,
        })

    @staticmethod
    def _selected(x=10.0, y=0.0):
        return {"x": x, "y": y, "z": 0.5,
                "road_object_selected_enforced": True,
                "candidate_score": 0.35}

    def test_shadow_holds_first_frame_but_preserves_tracker_input(self):
        fusion = self._fusion(True);items = [self._selected()]
        admitted, rejected, stats = fusion._gate_selected_new_tracks(
            items, [], 10.0, frame_id=100)
        self.assertEqual([], admitted);self.assertEqual(1, len(rejected))
        self.assertEqual(1, stats["held"])
        self.assertEqual(1, len(fusion._selected_admission_tracker_candidates(
            items, admitted)))

    def test_pending_repeat_precedes_shadow_created_track_support(self):
        fusion = self._fusion(True);item = self._selected()
        fusion._gate_selected_new_tracks([item], [], 10.0, frame_id=100)
        shadow_track = {"x": 10.1, "y": 0.0, "track_hits": 1}
        admitted, rejected, stats = fusion._gate_selected_new_tracks(
            [self._selected(10.2)], [shadow_track], 10.1, frame_id=101)
        self.assertEqual([], rejected);self.assertEqual(1, stats["confirmed"])
        self.assertEqual("repeat", admitted[0]["selected_track_admission_reason"])
        self.assertEqual(1, admitted[0]["selected_track_admission_pending_id"])

    def test_same_lidar_frame_cannot_confirm(self):
        fusion = self._fusion(True);item = self._selected()
        fusion._gate_selected_new_tracks([item], [], 10.0, frame_id=100)
        admitted, rejected, stats = fusion._gate_selected_new_tracks(
            [item], [], 10.1, frame_id=100)
        self.assertEqual([], admitted);self.assertEqual(1, len(rejected))
        self.assertEqual(0, stats["confirmed"])

    def test_existing_track_radar_and_non_selected_bypass(self):
        fusion = self._fusion(True)
        tracked, rejected, unused_stats = fusion._gate_selected_new_tracks(
            [self._selected()], [{"x": 10.2, "y": 0.0,
                                  "track_non_selected_hits": 1}], 10.0, frame_id=1)
        self.assertEqual([], rejected)
        self.assertEqual("existing_track", tracked[0]["selected_track_admission_reason"])
        radar = self._selected(20.0);radar["radar_radial_velocity"] = 1.0
        plain = {"x": 30.0, "y": 0.0, "z": 0.5}
        admitted, unused_rejected, stats = fusion._gate_selected_new_tracks(
            [radar, plain], [], 10.1, frame_id=2)
        self.assertEqual(2, len(admitted));self.assertEqual(1, stats["sensor_bypass"])

    def test_pending_expires_on_new_frame(self):
        fusion = self._fusion(True)
        fusion._gate_selected_new_tracks([self._selected()], [], 10.0, frame_id=1)
        unused_admitted, unused_rejected, stats = fusion._gate_selected_new_tracks(
            [], [], 10.7, frame_id=2)
        self.assertEqual(1, stats["expired"])
        self.assertEqual("expired", fusion.last_selected_track_admission_expired_candidates[0]["selected_track_admission_reason"])

    def test_evaluator_reports_truth_and_fp_without_duplicate_frame(self):
        center = type("Center", (), {"x": 0.0, "y": 0.0})()
        evaluator = GroundTruthEvaluator(None, lambda: center, {
            "selected_track_admission_profiling": True})
        evaluator.truth_objects = lambda: [
            {"x": 10.0, "y": 0.0, "object_type": "person"}]
        held = [self._selected(), self._selected(30.0)]
        self.assertTrue(evaluator.observe_selected_track_admission(
            held, [], [], frame_id=10))
        self.assertFalse(evaluator.observe_selected_track_admission(
            held, [], [], frame_id=10))
        report = evaluator.report_selected_track_admission()
        self.assertEqual(2, report["run"]["hold"]["candidates"])
        self.assertEqual(1, report["run"]["hold"]["matched"])
        self.assertEqual(1, report["run"]["hold"]["fp"])
        self.assertEqual({"person": 1}, report["run"]["hold"]["classes"])

    def test_evaluator_profiles_pending_transitions_and_actor_coverage(self):
        center = type("Center", (), {"x": 0.0, "y": 0.0})()
        evaluator = GroundTruthEvaluator(None, lambda: center, {
            "selected_track_admission_profiling": True,
            "selected_track_admission_camera_rescue_ablations": [
                {"name":"test_rule", "min_iou":.05,
                 "max_center_distance":30.0,
                 "allowed_classes":["person"]}]})
        evaluator.truth_objects = lambda: [
            {"actor_id": 7, "x": 10.0, "y": 0.0,
             "object_type": "person"}]
        truth_hold = self._selected();truth_hold.update({
            "selected_track_admission_pending_id": 11,
            "adaptive_hybrid_source": "near_baseline",
            "adaptive_hybrid_temporal_rescue": True,
            "extent": [0.5, 0.3, 1.5], "point_count": 6,
            "selected_track_admission_camera_visible": True,
            "selected_track_admission_camera_supported": True,
            "selected_track_admission_camera_source": "detector",
            "selected_track_admission_camera_class": "person",
            "selected_track_admission_camera_iou": .4,
            "selected_track_admission_camera_center_distance": 8.0,
            "selected_track_admission_camera_confidence": .9})
        fp_hold = self._selected(30.0);fp_hold.update({
            "selected_track_admission_pending_id": 12,
            "adaptive_hybrid_source": "far_ranked",
            "extent": [0.8, 0.4, 0.1], "point_count": 3})
        evaluator.observe_selected_track_admission(
            [truth_hold, fp_hold], [], [], frame_id=20)
        truth_confirm = dict(truth_hold);truth_confirm["selected_track_admission_reason"] = "repeat"
        fp_confirm = dict(fp_hold);fp_confirm["selected_track_admission_reason"] = "repeat"
        evaluator.observe_selected_track_admission(
            [], [truth_confirm, fp_confirm], [], frame_id=21)
        report = evaluator.report_selected_track_admission()
        transition = report["transitions"]["confirm"]
        self.assertEqual(2, transition["total"])
        self.assertEqual(1, transition["origin_truth"])
        self.assertEqual(1, transition["origin_fp"])
        self.assertEqual(1, transition["same_truth_actor"])
        self.assertEqual(1, transition["stable_fp"])
        self.assertEqual(0, transition["changed_label_or_actor"])
        self.assertEqual({"actors": 1, "classes": {"person": 1}},
                         report["coverage"]["confirm"])
        self.assertEqual(1, report["actor_outcomes"]["confirm_only"]["actors"])
        self.assertEqual(0, report["actor_outcomes"]["expired_only"]["actors"])
        self.assertEqual(1.0, report["actor_outcomes"]["confirmation_coverage"])
        person_profile = report["outcome_features"]["confirm"]["person"]
        fp_profile = report["outcome_features"]["confirm"]["false_positive"]
        self.assertEqual(1, person_profile["samples"])
        self.assertEqual(0.35, person_profile["scores"]["p50"])
        self.assertEqual({"near": 1, "far": 0, "strict": 0, "rescue": 1},
                         person_profile["paths"])
        self.assertEqual(1, person_profile["camera"]["visible"])
        self.assertEqual(1, person_profile["camera"]["supported"])
        self.assertEqual(1.0, person_profile["camera"]["support_rate"])
        self.assertEqual({"person": 1}, person_profile["camera"]["classes"])
        self.assertEqual(.4, person_profile["camera"]["iou"]["p50"])
        self.assertEqual(8.0, person_profile["camera"]["center_distance"]["p50"])
        self.assertEqual(.9, person_profile["camera"]["confidence"]["p50"])
        self.assertEqual({"test_rule": 1},
                         person_profile["camera"]["rescue_ablations"])
        rescue = report["camera_rescue_shadow"]["test_rule"]
        self.assertEqual(1, rescue["confirm_person_samples_kept"])
        self.assertEqual(0, rescue["confirm_fp_samples_kept"])
        self.assertEqual({"near": 0, "far": 1, "strict": 1, "rescue": 0},
                         fp_profile["paths"])

    def test_selected_only_shadow_track_is_not_existing_track_support(self):
        fusion = self._fusion(True)
        shadow_track = {"x": 10.1, "y": 0.0,
                        "track_non_selected_hits": 0,
                        "track_selected_enforced_ever": True}
        admitted, rejected, stats = fusion._gate_selected_new_tracks(
            [self._selected()], [shadow_track], 10.0, frame_id=1)
        self.assertEqual([], admitted);self.assertEqual(1, len(rejected))
        self.assertEqual(0, stats["track_bypass"])

    def test_camera_rescue_shadow_counts_unique_expired_only_person(self):
        center = type("Center", (), {"x": 0.0, "y": 0.0})()
        evaluator = GroundTruthEvaluator(None, lambda: center, {
            "selected_track_admission_profiling": True,
            "selected_track_admission_camera_rescue_ablations": [
                {"name":"close_person", "min_iou":.05,
                 "max_center_distance":30.0,
                 "allowed_classes":["person"]}]})
        evaluator.truth_objects = lambda: [
            {"actor_id": 9, "x": 10.0, "y": 0.0,
             "object_type": "person"}]
        hold = self._selected();hold.update({
            "selected_track_admission_pending_id": 21,
            "selected_track_admission_camera_supported": True,
            "selected_track_admission_camera_class": "person",
            "selected_track_admission_camera_iou": .01,
            "selected_track_admission_camera_center_distance": 20.0})
        evaluator.observe_selected_track_admission([hold], [], [], frame_id=30)
        expired = dict(hold);expired["selected_track_admission_reason"] = "expired"
        evaluator.observe_selected_track_admission([], [], [expired], frame_id=31)
        result = evaluator.report_selected_track_admission()[
            "camera_rescue_shadow"]["close_person"]
        self.assertEqual(1, result["expired_only_actors"])
        self.assertEqual(1, result["expired_only_person_actors"])
        self.assertEqual(1, result["expired_only_actors_rescued"])
        self.assertEqual(1, result["expired_only_person_actors_rescued"])

    def test_delayed_reappearance_confirms_after_base_ttl_only_in_shadow(self):
        fusion = SimpleFusion("test", {
            "selected_track_admission_enabled": True,
            "selected_track_admission_shadow_mode": True,
            "selected_track_admission_required_frames": 2,
            "selected_track_admission_match_gate": 2.5,
            "selected_track_admission_ttl": .5,
            "selected_track_admission_delayed_reappearance_shadow": True,
            "selected_track_admission_delayed_reappearance_ablations": [
                {"name":"long", "ttl":1.0, "match_gate":2.5}]})
        fusion._gate_selected_new_tracks(
            [self._selected()], [], 10.0, frame_id=1)
        admitted, rejected, unused_stats = fusion._gate_selected_new_tracks(
            [self._selected(10.2)], [], 10.7, frame_id=2)
        self.assertEqual([], admitted)
        self.assertEqual(1, len(rejected))
        delayed = fusion.last_selected_delayed_reappearance_candidates["long"]
        self.assertEqual(1, len(delayed))
        self.assertAlmostEqual(.7, delayed[0][
            "selected_delayed_reappearance_time_gap"])

    def test_delayed_reappearance_evaluator_reports_expired_only_actor(self):
        center = type("Center", (), {"x": 0.0, "y": 0.0})()
        evaluator = GroundTruthEvaluator(None, lambda: center, {
            "selected_track_admission_profiling": True})
        evaluator.truth_objects = lambda: [
            {"actor_id": 12, "x": 10.0, "y": 0.0,
             "object_type": "person"}]
        hold = self._selected();hold["selected_track_admission_pending_id"] = 31
        evaluator.observe_selected_track_admission([hold], [], [], frame_id=40)
        expired = dict(hold);expired["selected_track_admission_reason"] = "expired"
        evaluator.observe_selected_track_admission([], [], [expired], frame_id=41)
        event = self._selected();event.update({
            "selected_delayed_reappearance_time_gap":.8,
            "selected_delayed_reappearance_match_distance":.4})
        evaluator.observe_selected_delayed_reappearance(
            {"long":[event]}, frame_id=41)
        result = evaluator.report_selected_track_admission()[
            "delayed_reappearance_shadow"]["long"]
        self.assertEqual(1, result["matched"])
        self.assertEqual(1, result["expired_only_person_actors_rescued"])


if __name__ == "__main__":
    unittest.main()
