from __future__ import print_function

import unittest

from roadside.ground_truth_eval import GroundTruthEvaluator


class _Vector(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x;self.y = y;self.z = z


class _BoundingBox(object):
    def __init__(self):self.extent = _Vector(2.0, 1.0, 0.8)


class _Actor(object):
    def __init__(self, actor_id, x, y):
        self.id = actor_id;self.type_id = "vehicle.test"
        self.attributes = {"role_name": "autopilot"}
        self._location = _Vector(x, y, 0.0);self.bounding_box = _BoundingBox()

    def get_location(self):return self._location
    def get_velocity(self):return _Vector()


class _Actors(list):
    def filter(self, pattern):return self


class _World(object):
    def __init__(self, actors):self._actors = _Actors(actors)
    def get_actors(self):return self._actors


class FarAdmissionDecisionEvaluationTest(unittest.TestCase):
    def _evaluator(self):
        world = _World([_Actor(1, 60.0, 0.0)])
        return GroundTruthEvaluator(world, lambda: _Vector(), {
            "radius": 80.0, "match_distance": 4.0,
            "include_roles": ["autopilot"],
            "far_admission_edge_risk_shadow": True,
            "far_admission_edge_hard_ratio": 0.65,
            "far_admission_edge_soft_ratio": 0.35,
            "far_admission_edge_soft_score": 0.68})

    def test_truth_and_false_positive_attribution(self):
        evaluator = self._evaluator()
        held = [
            {"x": 61.0, "y": 0.0, "far_track_admission_match_distance": 1.2,
             "far_track_admission_time_gap": 0.1, "far_track_admission_frame_gap": 1,
             "candidate_score": 0.60, "point_count": 8,
             "extent": [4.0, 1.8, 1.5], "cluster_mode": "far_geometry_builder",
             "far_geometry_recovered": True,
             "roi_details": {"lateral": 0.2, "allowed_lateral": 4.0}},
            {"x": 70.0, "y": 8.0, "candidate_score": 0.40,
             "point_count": 3, "extent": [1.0, 0.2, 0.4],
             "cluster_mode": "bev_multiscale",
             "roi_details": {"lateral": 3.6, "allowed_lateral": 4.0}},
        ]
        confirmed = [{"x": 59.5, "y": 0.0,
                      "far_track_admission_reason": "repeat",
                      "far_track_admission_match_distance": 0.8,
                      "far_track_admission_time_gap": 0.2,
                      "far_track_admission_frame_gap": 2}]
        evaluator.observe_far_admission_decisions(held, confirmed, [], frame_id=10)
        report = evaluator.report_far_admission_decisions()

        self.assertEqual(2, report["would_hold"])
        self.assertEqual(1, report["would_hold_truth"])
        self.assertEqual(1, report["would_hold_fp"])
        self.assertEqual(1, report["would_confirm_truth"])
        self.assertEqual(2, report["candidate_jump"]["samples"])
        self.assertAlmostEqual(1.0, report["candidate_jump"]["mean"])
        profile = report["feature_profiles"]["would_hold"]
        self.assertEqual(1, profile["truth"]["count"])
        self.assertEqual(1, profile["fp"]["count"])
        self.assertAlmostEqual(0.60, profile["truth"]["scores"]["mean"])
        self.assertAlmostEqual(0.40, profile["fp"]["scores"]["mean"])
        self.assertEqual(1, profile["truth"]["recovery"])
        self.assertEqual({"far_geometry_builder": 1},
                         profile["truth"]["cluster_modes"])
        risk = report["edge_risk_shadow"]["would_hold"]
        self.assertEqual(1, risk["truth"]["kept"])
        self.assertEqual(1, risk["fp"]["rejected"])
        self.assertEqual(1, risk["fp"]["hard_edge"])

    def test_edge_risk_shadow_combines_edge_score_and_source(self):
        evaluator = self._evaluator()
        confirmed = [
            {"x": 60.5, "y": 0.0, "far_track_admission_reason": "repeat",
             "candidate_score": 0.45, "cluster_mode": "far_geometry_builder",
             "roi_details": {"lateral": 0.2, "allowed_lateral": 4.0}},
            {"x": 70.0, "y": 8.0, "far_track_admission_reason": "repeat",
             "candidate_score": 0.90, "cluster_mode": "bev@0.85",
             "roi_details": {"lateral": 3.2, "allowed_lateral": 4.0}},
            {"x": 72.0, "y": 8.0, "far_track_admission_reason": "repeat",
             "candidate_score": 0.50, "cluster_mode": "bev@0.55",
             "roi_details": {"lateral": 1.8, "allowed_lateral": 4.0}},
        ]
        evaluator.observe_far_admission_decisions([], confirmed, [], frame_id=11)
        risk = evaluator.report_far_admission_decisions()["edge_risk_shadow"]["would_confirm"]
        self.assertEqual(1, risk["truth"]["kept"])
        self.assertEqual(2, risk["fp"]["rejected"])
        self.assertEqual(1, risk["fp"]["hard_edge"])
        self.assertEqual(1, risk["fp"]["soft_risk"])

    def test_same_lidar_frame_is_counted_once(self):
        evaluator = self._evaluator()
        held = [{"x": 60.0, "y": 0.0}]
        self.assertTrue(evaluator.observe_far_admission_decisions(
            held, [], [], frame_id=20))
        self.assertFalse(evaluator.observe_far_admission_decisions(
            held, [], [], frame_id=20))
        report = evaluator.report_far_admission_decisions()
        self.assertEqual(1, report["frames"])
        self.assertEqual(1, report["would_hold"])

    def test_expired_truth_is_evaluation_only(self):
        evaluator = self._evaluator()
        expired = [{"x": 60.5, "y": 0.0,
                    "far_track_admission_reason": "expired",
                    "candidate_score": 0.35, "point_count": 3,
                    "extent": [1.2, 0.3, 0.5],
                    "cluster_mode": "bev_multiscale"}]
        evaluator.observe_far_admission_decisions([], [], expired, frame_id=30)
        report = evaluator.report_far_admission_decisions()
        self.assertEqual(1, report["expired"])
        self.assertEqual(1, report["expired_truth"])
        self.assertEqual(0, report["expired_fp"])
        profile = report["feature_profiles"]["expired"]["truth"]
        self.assertAlmostEqual(0.35, profile["scores"]["mean"])
        self.assertEqual({"bev_multiscale": 1}, profile["cluster_modes"])


if __name__ == "__main__":
    unittest.main()
