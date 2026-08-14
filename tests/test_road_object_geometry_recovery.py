from __future__ import print_function

import unittest

from roadside.road_object_geometry_recovery import RoadObjectGeometryRecovery
from roadside.fusion import SimpleFusion
from roadside.ground_truth_eval import GroundTruthEvaluator


class RoadObjectGeometryRecoveryTest(unittest.TestCase):
    def _config(self):
        return {"road_object_recovery_enabled":True,
                "ground_clearance":.30,
                "road_object_recovery_ground_clearance":.05,
                "road_object_recovery_min_range":1.0,
                "road_object_recovery_max_range":20.0,
                "road_object_recovery_cell_size":.25,
                "road_object_recovery_min_points":2,
                "road_object_recovery_max_points":20,
                "road_object_recovery_temporal_frames":2,
                "road_object_recovery_temporal_gate":.8,
                "road_object_recovery_dedupe_distance":.8,
                "road_object_recovery_max_candidates":4}

    def _points(self):
        return [[5.00,0.00,0.10],[5.18,0.08,0.35],[5.28,0.14,0.62],
                [3.00,3.00,0.00],[4.00,4.00,0.01]]

    def test_requires_temporal_confirmation_and_ignores_ground(self):
        recovery=RoadObjectGeometryRecovery();config=self._config()
        self.assertEqual([],recovery.update(self._points(),[],.30,config,frame_id=1))
        self.assertEqual([],recovery.update(self._points(),[],.30,config,frame_id=1))
        out=recovery.update(self._points(),[],.30,config,frame_id=2)
        self.assertEqual(1,len(out))
        self.assertTrue(out[0]["road_object_recovered"])
        self.assertEqual(2,out[0]["road_object_temporal_hits"])
        self.assertEqual(3,out[0]["point_count"])

    def test_deduplicates_existing_normal_geometry(self):
        recovery=RoadObjectGeometryRecovery();config=self._config()
        recovery.update(self._points(),[],.30,config)
        existing=[{"x":5.1,"y":.1}]
        self.assertEqual([],recovery.update(self._points(),existing,.30,config))
        self.assertEqual(1,recovery.last_stats["dedupe"])

    def test_stage_diagnostics_distinguish_candidate_cap(self):
        recovery=RoadObjectGeometryRecovery();config=self._config();config.update({
            "road_object_recovery_temporal_frames":1,"road_object_recovery_max_candidates":1})
        points=[[5.00,0.00,.10],[5.18,.08,.35],[5.28,.14,.62],
                [10.00,0.00,.10],[10.20,.10,.35]]
        out=recovery.update(points,[],.30,config,frame_id=1)
        self.assertEqual(1,len(out))
        self.assertEqual(2,len(recovery.last_stage_outputs["shape"]))
        self.assertEqual(2,len(recovery.last_stage_outputs["temporal"]))
        self.assertEqual(2,len(recovery.last_stage_outputs["dedupe_pass"]))
        self.assertEqual(1,len(recovery.last_stage_outputs["output"]))
        self.assertEqual(1,recovery.last_stats["cap_reject"])

    def test_balanced_cap_reserves_bands_and_refills_unused_quota(self):
        recovery=RoadObjectGeometryRecovery();items=[]
        for index,distance in enumerate([10,11,12,13,14,15,28,29,30,40]):
            items.append({"id":index,"x":float(distance),"y":0.0,"point_count":20-index})
        config={"road_object_recovery_balanced_cap_shadow":True,
                "road_object_recovery_min_range":5.0,
                "road_object_recovery_balanced_bands":[
                    {"max_range":25.0,"quota":2},{"max_range":35.0,"quota":2},
                    {"max_range":45.0,"quota":2}]}
        selected=recovery._balanced_cap(items,6,config)
        ranges=[recovery._sensor_range(item) for item in selected]
        self.assertEqual(3,sum(1 for value in ranges if value<25.0))
        self.assertEqual(2,sum(1 for value in ranges if 25.0<=value<35.0))
        self.assertEqual(1,sum(1 for value in ranges if 35.0<=value<45.0))

    def test_adaptive_temporal_shadow_combines_sparse_mid_range_frames(self):
        recovery=RoadObjectGeometryRecovery();config=self._config();config.update({
            "road_object_recovery_max_range":45.0,
            "road_object_recovery_adaptive_temporal_shadow":True,
            "road_object_recovery_adaptive_voxel_size":.05,
            "road_object_recovery_adaptive_temporal_bands":[
                {"min_range":25.0,"max_range":35.0,"history_frames":3,
                 "min_support_frames":2,"max_points":48}]})
        first=[[30.00,0.00,.10]];second=[[30.20,.05,.35],[30.10,.02,.20]]
        self.assertEqual([],recovery.update(first,[],.30,config,frame_id=1))
        self.assertEqual([],recovery.update(second,[],.30,config,frame_id=2))
        adaptive=recovery.last_stage_outputs["adaptive_output"]
        self.assertEqual(1,len(adaptive))
        self.assertGreaterEqual(adaptive[0]["support_frames"],2)
        self.assertGreaterEqual(adaptive[0]["current_point_count"],1)
        self.assertTrue(adaptive[0]["range_adaptive_temporal_shadow"])
        self.assertEqual(0,recovery.last_stats["built"])

    def test_adaptive_low_object_ranking_prefers_compact_height(self):
        recovery=RoadObjectGeometryRecovery();config={
            "road_object_recovery_adaptive_ranking_shadow":True,
            "road_object_recovery_balanced_cap_shadow":True,
            "road_object_recovery_min_range":5.0,
            "road_object_recovery_balanced_bands":[{"max_range":35.0,"quota":1}]}
        low={"id":"low","x":30.0,"y":0.0,"point_count":5,"current_point_count":2,
             "support_frames":3,"extent":[.5,.2,.08]}
        tall={"id":"tall","x":30.5,"y":0.0,"point_count":12,"current_point_count":5,
              "support_frames":4,"extent":[.8,.4,1.2]}
        selected=recovery._adaptive_ranked_cap([tall,low],1,config)
        self.assertEqual("low",selected[0]["id"])
        self.assertGreater(low["adaptive_rank_score"],tall["adaptive_rank_score"])

    def test_shadow_mode_does_not_feed_geometry_or_tracker(self):
        config=self._config();config.update({
            "road_object_recovery_shadow_mode":True,
            "ground_removal_enabled":True,"candidate_scoring_enabled":False,
            "range_adaptive_clustering":False,"cluster_min_points":20,
            "cluster_merge_enabled":False,"sparse_geometry_rescue_enabled":False,
            "far_geometry_builder_enabled":False,"far_sparse_discovery_enabled":False,
            "far_track_admission_enabled":False})
        fusion=SimpleFusion("test",config)
        fusion.world_transform={"x":0.0,"y":0.0,"z":0.0,"yaw":0.0}
        fusion.set_ground_reference(0.0)
        fusion.fuse(self._points(),[],frame_id=1)
        fusion.fuse(self._points(),[],frame_id=2)
        self.assertEqual(1,len(fusion.last_road_object_recovery_candidates))
        self.assertFalse(any(x.get("road_object_recovered",False)
                             for x in fusion.last_geometry_world))
        self.assertFalse(any(x.get("cluster_mode")=="road_object_low"
                             for x in fusion.last_dynamic_candidates))
        diagnostics=fusion.road_object_recovery_diagnostics_world()
        self.assertEqual(3,len(diagnostics["input_points"]))
        self.assertEqual(1,len(diagnostics["stages"]["output"]))

    def test_precision_profile_uses_percentiles_and_normalized_sides(self):
        items=[{"point_count":5,"extent":[.8,.2,.1],"range":10.0},
               {"point_count":15,"extent":[.3,1.2,.5],"range":30.0}]
        profile=GroundTruthEvaluator._geometry_profile(items)
        self.assertEqual(10.0,profile["points"]["p50"])
        self.assertEqual(1.0,profile["long_side"]["p50"])
        self.assertEqual(.25,profile["short_side"]["p50"])
        self.assertEqual(.3,profile["height"]["p50"])

    def test_precision_gate_is_shadow_only_and_accumulates_samples(self):
        evaluator=GroundTruthEvaluator(None,lambda:None,{
            "road_object_precision_gate_shadow":True,
            "road_object_gate_min_points":10,
            "road_object_gate_point_ablations":[8,9,10],
            "road_object_gate_max_height":.45,
            "road_object_gate_max_range":25.0})
        truth=[{"actor_id":42,"type_id":"static.prop.box02","role":"rsu_test_obstacle",
                "object_type":"unknown_obstacle","x":10.0,"y":0.0,"range":10.0}]
        evaluator.truth_objects=lambda:truth
        evaluator._detected_with_range=lambda items:list(items)
        good={"x":10.0,"y":0.0,"point_count":12,"extent":[.6,.3,.2],"range":10.0}
        false={"x":30.0,"y":0.0,"point_count":6,"extent":[.4,.4,.9],"range":30.0}
        report=evaluator.analyze_road_object_recovery([good,false])
        self.assertEqual(1,report["precision_gate_shadow"]["truth"]["kept"])
        self.assertEqual(1,report["precision_gate_shadow"]["fp"]["rejected"])
        report=evaluator.analyze_road_object_recovery([good,false])
        self.assertEqual(2,report["cumulative"]["classes"]["unknown_obstacle"]["matched_samples"])
        self.assertEqual(2,report["cumulative"]["precision_gate_shadow"]["fp"]["rejected"])
        actor=report["cumulative"]["actor_coverage"][0]
        self.assertEqual(42,actor["actor_id"])
        self.assertEqual(2,actor["visible_frames"])
        self.assertEqual(2,actor["matched_frames"])
        self.assertEqual(2,actor["gate_kept_frames"])

    def test_point_gate_ablation_and_overlapping_rejection_reasons(self):
        evaluator=GroundTruthEvaluator(None,lambda:None,{
            "road_object_precision_gate_shadow":True,"road_object_gate_min_points":10,
            "road_object_gate_point_ablations":[8,9,10],
            "road_object_gate_max_height":.45,"road_object_gate_max_range":25.0})
        borderline={"point_count":9,"extent":[.6,.3,.2],"range":20.0}
        false={"point_count":6,"extent":[.6,.3,.9],"range":30.0}
        reports=evaluator._road_object_ablations({"unknown_obstacle":[borderline]},[false])
        self.assertEqual(1,reports["8"]["truth"]["kept"])
        self.assertEqual(1,reports["9"]["truth"]["kept"])
        self.assertEqual(0,reports["10"]["truth"]["kept"])
        failures=reports["10"]["fp"]["failures"]
        self.assertEqual(1,failures["points"])
        self.assertEqual(1,failures["height"])
        self.assertEqual(1,failures["range"])

    def test_range_stage_attribution_profiles_raw_to_output(self):
        evaluator=GroundTruthEvaluator(None,lambda:None,{
            "road_object_raw_support_radius":1.5,"road_object_stage_match_distance":2.0,
            "road_object_recovery_min_range":5.0,"road_object_stage_range_bins":[25,35,45]})
        truth=[{"actor_id":1,"type_id":"static.prop.box02","role":"rsu_test_obstacle",
                "object_type":"unknown_obstacle","x":10.0,"y":0.0,"range":10.0},
               {"actor_id":2,"type_id":"static.prop.trashcan03","role":"rsu_test_obstacle",
                "object_type":"unknown_obstacle","x":30.0,"y":0.0,"range":30.0}]
        evaluator.truth_objects=lambda:truth
        near={"x":10.1,"y":0.0,"z":0.1};far={"x":30.1,"y":0.0,"z":0.1}
        report=evaluator.analyze_road_object_recovery_stages({
            "input_points":[near,far],"stages":{"component":[near],"shape":[near],
            "temporal":[near],"dedupe_pass":[near],"output":[near],
            "balanced_output":[near,far]}})
        actors=report["actors"]
        self.assertEqual(1,actors[0]["raw_frames"])
        self.assertEqual(1,actors[0]["stage_frames"]["output"])
        self.assertEqual(1,actors[1]["raw_frames"])
        self.assertEqual(0,actors[1]["stage_frames"]["component"])
        self.assertEqual(1,actors[1]["stage_frames"]["balanced_output"])
        self.assertEqual(1,report["range_bands"][0]["output"])
        self.assertEqual(0,report["range_bands"][1]["output"])
        self.assertEqual(1,report["range_bands"][1]["balanced_output"])
        evaluator._detected_with_range=lambda items:list(items)
        comparison=evaluator.analyze_road_object_cap_comparison([near],[near,far])
        self.assertEqual(1,comparison["baseline"]["matched"])
        self.assertEqual(2,comparison["balanced"]["matched"])
        comparison=evaluator.analyze_road_object_cap_comparison([near],[near,far])
        self.assertEqual(2,comparison["baseline_run"]["matched"])
        self.assertEqual(4,comparison["balanced_run"]["matched"])
        comparison=evaluator.analyze_road_object_cap_comparison([near],[near,far],[far],[far])
        self.assertEqual(1,comparison["adaptive"]["matched"])
        self.assertEqual(1,comparison["adaptive_run"]["matched"])
        self.assertEqual(1,comparison["adaptive_ranked"]["matched"])

    def test_adaptive_temporal_feature_profile_accumulates_truth_and_fp(self):
        evaluator=GroundTruthEvaluator(None,lambda:None,{
            "road_object_adaptive_feature_profiling":True})
        truth=[{"actor_id":42,"type_id":"static.prop.box02","role":"rsu_test_obstacle",
                "object_type":"unknown_obstacle","x":30.0,"y":0.0,"range":30.0}]
        evaluator.truth_objects=lambda:truth
        evaluator._detected_with_range=lambda items:list(items)
        good={"x":30.1,"y":0.0,"range":30.1,"point_count":5,
              "current_point_count":2,"temporal_point_count":3,"support_frames":3,
              "extent":[.5,.2,.3],"adaptive_band":"25-35m"}
        false={"x":40.0,"y":5.0,"range":40.3,"point_count":18,
               "current_point_count":8,"temporal_point_count":10,"support_frames":4,
               "extent":[1.0,.4,.8],"adaptive_band":"35-45m"}
        report=evaluator.analyze_road_object_adaptive_profile([good,false])
        self.assertEqual(1,report["frame"]["matched"])
        self.assertEqual(1,report["frame"]["fp"])
        profile=report["run"]["classes"]["unknown_obstacle"]["profile"]
        self.assertEqual(2.0,profile["current_points"]["mean"])
        self.assertEqual(3.0,profile["history_points"]["mean"])
        self.assertEqual(1,report["run"]["bands"]["25-35m"]["truth"])
        report=evaluator.analyze_road_object_adaptive_profile([good,false])
        self.assertEqual(4,report["run"]["candidates"])

    def test_benchmark_session_resets_pre_spawn_cumulative_metrics(self):
        evaluator=GroundTruthEvaluator(None,lambda:None,{
            "road_object_benchmark_session_isolation":True})
        evaluator._road_object_samples["false"].append({"point_count":4})
        evaluator._adaptive_temporal_samples["false"].append({"point_count":5})
        evaluator._road_object_cap_totals["baseline"]["candidates"]=7
        self.assertFalse(evaluator._sync_road_object_benchmark_session([])["reset"])
        first=[{"actor_id":42,"role":"rsu_test_obstacle"}]
        session=evaluator._sync_road_object_benchmark_session(first)
        self.assertTrue(session["reset"])
        self.assertEqual(1,session["generation"])
        self.assertEqual([],evaluator._road_object_samples["false"])
        self.assertEqual([],evaluator._adaptive_temporal_samples["false"])
        self.assertEqual(0,evaluator._road_object_cap_totals["baseline"]["candidates"])
        evaluator._road_object_cap_totals["baseline"]["candidates"]=3
        self.assertFalse(evaluator._sync_road_object_benchmark_session(first)["reset"])
        self.assertEqual(3,evaluator._road_object_cap_totals["baseline"]["candidates"])
        second=[{"actor_id":99,"role":"rsu_test_obstacle"}]
        session=evaluator._sync_road_object_benchmark_session(second)
        self.assertTrue(session["reset"])
        self.assertEqual(2,session["generation"])
        self.assertEqual(0,evaluator._road_object_cap_totals["baseline"]["candidates"])


if __name__=="__main__":unittest.main()
