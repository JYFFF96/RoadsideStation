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
            "selected_track_admission_profiling": True})
        evaluator.truth_objects = lambda: [
            {"actor_id": 7, "x": 10.0, "y": 0.0,
             "object_type": "person"}]
        truth_hold = self._selected();truth_hold["selected_track_admission_pending_id"] = 11
        fp_hold = self._selected(30.0);fp_hold["selected_track_admission_pending_id"] = 12
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

    def test_selected_only_shadow_track_is_not_existing_track_support(self):
        fusion = self._fusion(True)
        shadow_track = {"x": 10.1, "y": 0.0,
                        "track_non_selected_hits": 0,
                        "track_selected_enforced_ever": True}
        admitted, rejected, stats = fusion._gate_selected_new_tracks(
            [self._selected()], [shadow_track], 10.0, frame_id=1)
        self.assertEqual([], admitted);self.assertEqual(1, len(rejected))
        self.assertEqual(0, stats["track_bypass"])


if __name__ == "__main__":
    unittest.main()
