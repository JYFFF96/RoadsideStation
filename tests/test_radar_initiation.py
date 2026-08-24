from __future__ import print_function

import unittest

from roadside.radar_initiation import NearRadarTrackInitiator
from roadside.fusion import SimpleFusion
from roadside.ground_truth_eval import GroundTruthEvaluator
from roadside.tracking import NearestTracker


class _Center(object):
    x = 0.0
    y = 0.0


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
            "radar_initiation_single_point_enabled": True,
            "radar_initiation_single_point_min_abs_speed": .2,
            "radar_initiation_single_point_required_frames": 3,
            "radar_initiation_single_point_ttl": 1.0,
            "radar_initiation_seed_bridge_shadow_enabled": True,
            "radar_initiation_seed_bridge_required_frames": [2, 3],
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

    def _single(self, x=8.0, velocity=.33):
        return [{"x": x, "y": 0.0, "z": 1.0, "velocity": velocity,
                 "los_x": 1.0, "los_y": 0.0, "sensor_range": x}]

    def test_moving_single_point_requires_three_distinct_frames(self):
        initiator = NearRadarTrackInitiator(self._config())
        self.assertEqual([], initiator.update(
            self._single(), [], 1.0, frame_id=10, validator=lambda unused: True))
        self.assertEqual([], initiator.update(
            self._single(8.1), [], 1.1, frame_id=11, validator=lambda unused: True))
        emitted = initiator.update(
            self._single(8.2), [], 1.2, frame_id=12, validator=lambda unused: True)
        self.assertEqual(1, len(emitted))
        self.assertEqual("single_moving", emitted[0]["radar_initiation_mode"])
        self.assertEqual(1, emitted[0]["radar_hits"])
        self.assertEqual(3, emitted[0]["radar_initiation_frames"])
        self.assertEqual(1, initiator.last_stats["single_point_confirmed"])
        self.assertEqual(1, initiator.last_stats["single_point_emitted"])

    def test_static_single_point_is_rejected_before_confirmation(self):
        initiator = NearRadarTrackInitiator(self._config())
        for frame in range(10, 14):
            self.assertEqual([], initiator.update(
                self._single(velocity=.0), [], frame / 10.0, frame_id=frame))
        self.assertEqual(0, initiator.last_stats["single_point_candidates"])
        self.assertEqual(0, initiator.last_stats["confirmed"])

    def test_single_point_does_not_confirm_with_cluster_mode(self):
        initiator = NearRadarTrackInitiator(self._config())
        initiator.update(self._single(), [], 1.0, frame_id=10)
        self.assertEqual([], initiator.update(
            self._points(8.1), [], 1.1, frame_id=11))
        self.assertEqual(0, initiator.last_stats["confirmed"])

    def test_confirmed_single_point_still_uses_lidar_dedupe(self):
        initiator = NearRadarTrackInitiator(self._config())
        initiator.update(self._single(), [], 1.0, frame_id=10)
        initiator.update(self._single(8.1), [], 1.1, frame_id=11)
        emitted = initiator.update(
            self._single(8.2), [{"x": 8.0, "y": 0.0}], 1.2, frame_id=12)
        self.assertEqual([], emitted)
        self.assertEqual(1, initiator.last_stats["dedupe_rejected"])

    def test_single_point_lifecycle_is_cumulative_across_unlogged_frames(self):
        initiator = NearRadarTrackInitiator(self._config())
        initiator.update(self._single(), [], 1.0, frame_id=10)
        initiator.update(self._single(8.1), [], 1.1, frame_id=11)
        initiator.update(self._single(8.2), [], 1.2, frame_id=12)
        cumulative = initiator.last_stats["cumulative"]
        self.assertEqual(3, cumulative["frames"])
        self.assertEqual(3, cumulative["single_point_components"])
        self.assertEqual(3, cumulative["single_point_candidates"])
        self.assertEqual(1, cumulative["single_point_started"])
        self.assertEqual(2, cumulative["single_point_matched"])
        self.assertEqual(1, cumulative["single_point_confirmed"])
        self.assertEqual(1, cumulative["single_point_emitted"])
        self.assertEqual(3, cumulative["single_point_speed_counts"]["0.20"])

    def test_lifecycle_profiles_below_speed_bridge_and_expired_hits(self):
        initiator = NearRadarTrackInitiator(self._config())
        initiator.update(self._single(), [], 1.0, frame_id=10)
        initiator.update(self._single(8.1, velocity=.0), [], 1.1, frame_id=11)
        self.assertEqual(
            1, initiator.last_stats["single_point_below_speed_near_pending"])
        initiator.update([], [], 2.2, frame_id=12)
        cumulative = initiator.last_stats["cumulative"]
        self.assertEqual(1, cumulative["single_point_expired"])
        self.assertEqual({"1": 1, "2": 0, "3+": 0},
                         cumulative["single_point_expired_hits"])
        self.assertEqual(1, cumulative["single_point_speed_counts"]["0.05"])

    def test_repeated_frame_does_not_change_cumulative_lifecycle(self):
        initiator = NearRadarTrackInitiator(self._config())
        initiator.update(self._single(), [], 1.0, frame_id=10)
        before = dict(initiator.last_stats["cumulative"])
        initiator.update(self._single(), [], 1.1, frame_id=10)
        self.assertEqual(before["frames"], initiator.last_stats["cumulative"]["frames"])
        self.assertEqual(before["single_point_candidates"],
                         initiator.last_stats["cumulative"]["single_point_candidates"])

    def test_lifecycle_profiles_moving_point_buried_in_multi_component(self):
        initiator = NearRadarTrackInitiator(self._config())
        points = self._points(velocity=.0)
        points[0]["velocity"] = .33
        points[1]["velocity"] = .0
        initiator.update(points, [], 1.0, frame_id=10)
        cumulative = initiator.last_stats["cumulative"]
        self.assertEqual(1, cumulative["mixed_moving_components"])
        self.assertEqual(1, cumulative["moving_points_in_multi_components"])
        self.assertEqual(0, cumulative["single_point_candidates"])

    def test_motion_seed_bridge_profiles_two_and_three_frame_rules(self):
        initiator = NearRadarTrackInitiator(self._config())
        initiator.update(self._single(velocity=.33), [], 1.0, frame_id=10,
                         validator=lambda unused: True)
        initiator.update(self._single(8.1, velocity=.0), [], 1.1, frame_id=11,
                         validator=lambda unused: True)
        stats = initiator.last_stats["seed_bridge_shadow"]
        self.assertEqual(1, stats["seeds"])
        self.assertEqual(1, stats["matches"])
        self.assertEqual(1, stats["below_speed_matches"])
        self.assertEqual(1, stats["rules"]["2"]["would_emit"])
        self.assertEqual(0, stats["rules"]["3"]["would_emit"])
        initiator.update(self._single(8.2, velocity=.0), [], 1.2, frame_id=12,
                         validator=lambda unused: True)
        stats = initiator.last_stats["seed_bridge_shadow"]
        self.assertEqual(1, stats["rules"]["2"]["confirmed"])
        self.assertEqual(1, stats["rules"]["3"]["confirmed"])
        self.assertEqual(1, stats["rules"]["3"]["would_emit"])

    def test_motion_seed_bridge_remains_shadow_and_profiles_gates(self):
        duplicate = NearRadarTrackInitiator(self._config())
        duplicate.update(self._single(), [], 1.0, frame_id=10)
        emitted = duplicate.update(
            self._single(8.1, velocity=.0), [{"x": 8.0, "y": 0.0}],
            1.1, frame_id=11, validator=lambda unused: True)
        self.assertEqual([], emitted)
        stats = duplicate.last_stats["seed_bridge_shadow"]["rules"]["2"]
        self.assertEqual(1, stats["dedupe_rejected"])
        self.assertEqual(0, stats["would_emit"])

        rejected = NearRadarTrackInitiator(self._config())
        rejected.update(self._single(), [], 1.0, frame_id=10)
        rejected.update(self._single(8.1, velocity=.0), [], 1.1, frame_id=11,
                        validator=lambda unused: False)
        stats = rejected.last_stats["seed_bridge_shadow"]["rules"]["2"]
        self.assertEqual(1, stats["roi_rejected"])
        self.assertEqual(0, stats["would_emit"])

    def test_static_singletons_cannot_seed_motion_bridge(self):
        initiator = NearRadarTrackInitiator(self._config())
        for frame in range(10, 14):
            initiator.update(self._single(velocity=.0), [], frame / 10.0,
                             frame_id=frame, validator=lambda unused: True)
        stats = initiator.last_stats["seed_bridge_shadow"]
        self.assertEqual(0, stats["seeds"])
        self.assertEqual(0, stats["rules"]["2"]["confirmed"])
        self.assertEqual(0, stats["rules"]["3"]["confirmed"])

    def test_motion_seed_bridge_truth_attribution_is_evaluation_only(self):
        evaluator = GroundTruthEvaluator(None, lambda: _Center(), {
            "radius": 80.0, "match_distance": 2.0})
        evaluator.truth_objects = lambda: [
            {"x": 8.0, "y": 0.0, "object_type": "pedestrian"}]
        report = evaluator.observe_radar_seed_bridge({
            "2": [{"x": 8.2, "y": 0.0}],
            "3": [{"x": 20.0, "y": 0.0}]}, frame_id=10)
        self.assertEqual(1, report["2"]["matched"])
        self.assertEqual(0, report["2"]["fp"])
        self.assertEqual(0, report["3"]["matched"])
        self.assertEqual(1, report["3"]["fp"])
        repeated = evaluator.observe_radar_seed_bridge({
            "2": [{"x": 8.1, "y": 0.0}]}, frame_id=10)
        self.assertEqual(1, repeated["2"]["candidates"])

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
