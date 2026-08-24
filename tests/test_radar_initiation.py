from __future__ import print_function

import unittest

from roadside.radar_initiation import NearRadarTrackInitiator
from roadside.fusion import SimpleFusion
from roadside.tracking import NearestTracker


class NearRadarTrackInitiatorTests(unittest.TestCase):
    def _config(self, **overrides):
        config = {
            "radar_initiation_enabled": True,
            "radar_initiation_shadow_mode": False,
            "radar_initiation_min_range": 2.0,
            "radar_initiation_max_range": 30.0,
            "radar_initiation_cluster_radius": 1.5,
            "radar_initiation_cluster_z_gate": 1.5,
            "radar_initiation_min_points": 2,
            "radar_initiation_required_frames": 2,
            "radar_initiation_match_gate": 2.5,
            "radar_initiation_ttl": .6,
            "radar_initiation_min_abs_speed": .6,
            "radar_initiation_dedupe_distance": 3.0,
        }
        config.update(overrides)
        return config

    def _points(self, x=8.0, velocity=2.0):
        return [
            {"x": x, "y": 0.0, "z": 1.0, "velocity": velocity,
             "los_x": 1.0, "los_y": 0.0, "sensor_range": x},
            {"x": x + .4, "y": .2, "z": 1.1, "velocity": velocity + .2,
             "los_x": 1.0, "los_y": 0.0, "sensor_range": x + .4},
        ]

    def test_moving_cluster_emits_after_two_distinct_frames(self):
        initiator = NearRadarTrackInitiator(self._config())
        self.assertEqual([], initiator.update(
            self._points(), [], 1.0, frame_id=10, validator=lambda unused: True))
        emitted = initiator.update(
            self._points(8.2), [], 1.1, frame_id=11, validator=lambda unused: True)
        self.assertEqual(1, len(emitted))
        self.assertEqual(["radar"], emitted[0]["sources"])
        self.assertEqual(2, emitted[0]["radar_initiation_frames"])
        self.assertEqual(1, initiator.last_stats["emitted"])

    def test_repeated_frame_does_not_confirm(self):
        initiator = NearRadarTrackInitiator(self._config())
        initiator.update(self._points(), [], 1.0, frame_id=10)
        self.assertEqual([], initiator.update(self._points(), [], 1.1, frame_id=10))
        self.assertEqual(1, initiator.last_stats["pending"])

    def test_confirmed_speed_shadow_profiles_lower_thresholds(self):
        initiator = NearRadarTrackInitiator(self._config())
        initiator.update(self._points(velocity=.25), [], 1.0, frame_id=10)
        self.assertEqual([], initiator.update(
            self._points(8.1, velocity=.25), [], 1.1, frame_id=11))
        stats = initiator.last_stats
        self.assertAlmostEqual(.35, stats["confirmed_abs_speed_p50"])
        self.assertAlmostEqual(.35, stats["confirmed_abs_speed_max"])
        self.assertEqual({"0.10": 1, "0.20": 1, "0.40": 0, "0.60": 0},
                         stats["speed_shadow_counts"])
        self.assertEqual(1, stats["static_rejected"])

    def test_static_and_existing_lidar_targets_are_not_emitted(self):
        static = NearRadarTrackInitiator(self._config())
        static.update(self._points(velocity=.1), [], 1.0, frame_id=1)
        self.assertEqual([], static.update(
            self._points(8.1, velocity=.1), [], 1.1, frame_id=2))
        self.assertEqual(1, static.last_stats["static_rejected"])

        duplicate = NearRadarTrackInitiator(self._config())
        duplicate.update(self._points(), [], 1.0, frame_id=1)
        emitted = duplicate.update(
            self._points(8.1), [{"x": 8.0, "y": 0.0}], 1.1, frame_id=2)
        self.assertEqual([], emitted)
        self.assertEqual(1, duplicate.last_stats["dedupe_rejected"])

    def test_shadow_and_roi_modes_preserve_diagnostics(self):
        shadow = NearRadarTrackInitiator(self._config(
            radar_initiation_shadow_mode=True))
        shadow.update(self._points(), [], 1.0, frame_id=1)
        self.assertEqual([], shadow.update(self._points(), [], 1.1, frame_id=2))
        self.assertEqual(1, len(shadow.last_shadow_candidates))
        self.assertEqual(1, shadow.last_stats["emitted"])

        rejected = NearRadarTrackInitiator(self._config())
        rejected.update(self._points(), [], 1.0, frame_id=1)
        self.assertEqual([], rejected.update(
            self._points(), [], 1.1, frame_id=2, validator=lambda unused: False))
        self.assertEqual(1, rejected.last_stats["roi_rejected"])

    def test_tracker_reports_radar_only_source(self):
        tracker = NearestTracker()
        track = tracker.update([{
            "x": 8.0, "y": 0.0, "z": 1.0, "extent": [4.5, 1.8, 1.6],
            "confidence": .82, "sources": ["radar"],
            "radar_radial_velocity": 2.0, "radar_los_x": 1.0,
            "radar_los_y": 0.0, "candidate_score": .82,
            "candidate_score_bypass": True, "sensor_range": 8.0,
        }], timestamp=1.0)[0]
        self.assertEqual("R", track["track_sensors"])
        self.assertEqual(0, track["track_lidar_hits"])
        self.assertEqual(1, track["track_radar_hits"])

    def test_fusion_feeds_confirmed_radar_candidate_to_common_tracker(self):
        config = self._config(background_filter_enabled=False,
                              range_adaptive_clustering=False,
                              cluster_merge_enabled=False)
        fusion = SimpleFusion("RSU_TEST", config)
        fusion.radar_matrix = [[1.0, 0.0, 0.0, 0.0],
                               [0.0, 1.0, 0.0, 0.0],
                               [0.0, 0.0, 1.0, 0.0],
                               [0.0, 0.0, 0.0, 1.0]]
        fusion.radar_origin = (0.0, 0.0, 0.0)
        fusion.fuse([], self._points(), timestamp=1.0,
                    frame_id=100, radar_frame_id=10)
        self.assertEqual([], fusion.last_tracked_candidates)
        fusion.fuse([], self._points(8.2), timestamp=1.1,
                    frame_id=101, radar_frame_id=11)
        self.assertEqual(1, len(fusion.last_tracked_candidates))
        track = fusion.last_tracked_candidates[0]
        self.assertEqual(["radar"], track["sources"])
        self.assertEqual("R", track["track_sensors"])
        self.assertEqual(1, fusion.last_stats["radar_initiation_emitted"])


if __name__ == "__main__":
    unittest.main()
