from __future__ import print_function

import unittest

from roadside.fusion import SimpleFusion


class FarTrackAdmissionShadowTest(unittest.TestCase):
    def _fusion(self, shadow):
        return SimpleFusion("test", {
            "far_track_admission_enabled": True,
            "far_track_admission_shadow_mode": shadow,
            "far_track_admission_min_range": 50.0,
            "far_track_admission_required_frames": 2,
            "far_track_admission_match_gate": 2.5,
            "far_track_admission_ttl": 0.5,
            "far_track_admission_strong_min_points": 10,
            "far_track_admission_strong_min_score": 0.72,
        })

    def _weak_far_candidate(self):
        return {"x": 60.0, "y": 0.0, "z": 0.5,
                "point_count": 4, "candidate_score": 0.50}

    def test_shadow_mode_observes_hold_but_keeps_tracker_input(self):
        fusion = self._fusion(True)
        dynamic = [self._weak_far_candidate()]
        admitted, rejected, stats = fusion._gate_far_new_tracks(
            dynamic, [], 10.0, frame_id=100)

        self.assertEqual([], admitted)
        self.assertEqual(1, len(rejected))
        self.assertEqual(1, stats["held"])
        tracker_candidates = fusion._far_admission_tracker_candidates(dynamic, admitted)
        self.assertEqual(1, len(tracker_candidates))

    def test_enforcing_mode_still_holds_weak_candidate(self):
        fusion = self._fusion(False)
        dynamic = [self._weak_far_candidate()]
        admitted, _, _ = fusion._gate_far_new_tracks(
            dynamic, [], 10.0, frame_id=100)

        tracker_candidates = fusion._far_admission_tracker_candidates(dynamic, admitted)
        self.assertEqual([], tracker_candidates)

    def test_confirmation_requires_a_new_lidar_frame(self):
        fusion = self._fusion(True)
        dynamic = [self._weak_far_candidate()]
        fusion._gate_far_new_tracks(dynamic, [], 10.0, frame_id=100)
        admitted, rejected, _ = fusion._gate_far_new_tracks(
            dynamic, [], 10.1, frame_id=100)
        self.assertEqual([], admitted)
        self.assertEqual(1, len(rejected))

        admitted, rejected, stats = fusion._gate_far_new_tracks(
            dynamic, [], 10.2, frame_id=101)
        self.assertEqual(1, len(admitted))
        self.assertEqual([], rejected)
        self.assertEqual(1, stats["confirmed"])
        self.assertAlmostEqual(0.0, admitted[0]["far_track_admission_match_distance"])
        self.assertAlmostEqual(0.2, admitted[0]["far_track_admission_time_gap"])
        self.assertAlmostEqual(1.0, admitted[0]["far_track_admission_frame_gap"])

    def test_pending_expires_only_on_a_new_lidar_frame(self):
        fusion = self._fusion(True)
        dynamic = [self._weak_far_candidate()]
        fusion._gate_far_new_tracks(dynamic, [], 10.0, frame_id=100)

        _, _, stats = fusion._gate_far_new_tracks([], [], 10.6, frame_id=100)
        self.assertEqual(0, stats["expired"])
        self.assertEqual(1, stats["pending"])

        _, _, stats = fusion._gate_far_new_tracks([], [], 10.7, frame_id=101)
        self.assertEqual(1, stats["expired"])
        self.assertEqual(1, len(fusion.last_far_admission_expired_candidates))


if __name__ == "__main__":
    unittest.main()
