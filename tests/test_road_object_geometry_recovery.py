from __future__ import print_function

import os
import unittest

import yaml

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

    def test_adaptive_stratified_ranking_reserves_elevated_candidates(self):
        recovery=RoadObjectGeometryRecovery();config={
            "road_object_recovery_adaptive_stratified_shadow":True,
            "road_object_recovery_min_range":5.0,
            "road_object_recovery_balanced_bands":[{"max_range":35.0,"quota":4}],
            "road_object_adaptive_stratified_height":.30,
            "road_object_adaptive_stratified_elevated_quota":2}
        items=[]
        for index,height in enumerate([.05,.08,.12,.70,.80,.90]):
            items.append({"id":index,"x":30.0+index*.01,"y":0.0,"point_count":5,
                          "current_point_count":2,"support_frames":4,
                          "extent":[.5,.2,height]})
        selected=recovery._adaptive_stratified_cap(items,4,config)
        heights=[item["extent"][2] for item in selected]
        self.assertEqual(2,sum(1 for value in heights if value<=.30))
        self.assertEqual(2,sum(1 for value in heights if value>.30))

    def test_adaptive_hybrid_combines_near_baseline_and_far_ranked(self):
        recovery=RoadObjectGeometryRecovery();config={
            "road_object_recovery_adaptive_hybrid_shadow":True,
            "road_object_recovery_balanced_cap_shadow":True,
            "road_object_recovery_min_range":5.0,
            "road_object_recovery_dedupe_distance":.5,
            "road_object_adaptive_hybrid_split_range":25.0,
            "road_object_recovery_balanced_bands":[
                {"max_range":25.0,"quota":4},{"max_range":35.0,"quota":4},
                {"max_range":45.0,"quota":4}]}
        baseline=[];adaptive=[]
        for index,distance in enumerate([10,12,14,16,18,20]):
            baseline.append({"id":"near%d"%index,"x":float(distance),"y":0.0,
                             "point_count":20-index,"extent":[.5,.2,.2]})
        for index,distance in enumerate([28,29,30,31,32,33,37,38,39,40,41,42]):
            adaptive.append({"id":"far%d"%index,"x":float(distance),"y":0.0,
                             "point_count":5,"current_point_count":2,
                             "support_frames":4,"extent":[.5,.2,.1]})
        selected=recovery._adaptive_hybrid_cap(baseline,adaptive,12,config)
        ranges=[recovery._sensor_range(item) for item in selected]
        self.assertEqual(4,sum(1 for value in ranges if value<25.0))
        self.assertEqual(4,sum(1 for value in ranges if 25.0<=value<35.0))
        self.assertEqual(4,sum(1 for value in ranges if 35.0<=value<45.0))
        self.assertEqual(4,sum(1 for item in selected
                               if item["adaptive_hybrid_source"]=="near_baseline"))

    def test_hybrid_source_aware_gate_uses_near_shape_and_far_support(self):
        recovery=RoadObjectGeometryRecovery();config={
            "road_object_recovery_adaptive_hybrid_gate_shadow":True,
            "road_object_hybrid_gate_near_min_points":8,
            "road_object_hybrid_gate_near_low_height":.45,
            "road_object_hybrid_gate_near_min_short_side":.50,
            "road_object_hybrid_gate_far_stable_frames":4,
            "road_object_hybrid_gate_far_close_range":32.0,
            "road_object_hybrid_gate_far_close_min_points":5,
            "road_object_hybrid_gate_far_close_current_points":2}
        items=[
            {"id":"near_wide","x":15.0,"y":0.0,"point_count":8,
             "extent":[.8,.6,.9],"adaptive_hybrid_source":"near_baseline"},
            {"id":"near_low","x":20.0,"y":0.0,"point_count":8,
             "extent":[.4,.2,.2],"adaptive_hybrid_source":"near_baseline"},
            {"id":"near_weak","x":18.0,"y":0.0,"point_count":7,
             "extent":[.8,.6,.9],"adaptive_hybrid_source":"near_baseline"},
            {"id":"far_stable","x":38.0,"y":0.0,"point_count":4,
             "current_point_count":1,"support_frames":4,"extent":[.5,.2,.1],
             "adaptive_hybrid_source":"far_ranked"},
            {"id":"far_close","x":30.0,"y":0.0,"point_count":5,
             "current_point_count":2,"support_frames":3,"extent":[.5,.2,.1],
             "adaptive_hybrid_source":"far_ranked"},
            {"id":"far_weak","x":38.0,"y":0.0,"point_count":5,
             "current_point_count":2,"support_frames":3,"extent":[.5,.2,.1],
             "adaptive_hybrid_source":"far_ranked"}]
        kept,reasons=recovery._adaptive_hybrid_gate(items,config)
        self.assertEqual(set(["near_wide","near_low","far_stable","far_close"]),
                         set(item["id"] for item in kept))
        self.assertEqual(1,reasons["near_points"])
        self.assertEqual(1,reasons["far_support"])

    def test_hybrid_temporal_rescue_restores_persistent_sparse_rejects(self):
        recovery=RoadObjectGeometryRecovery();config={
            "road_object_recovery_adaptive_hybrid_gate_shadow":True,
            "road_object_hybrid_gate_near_min_points":8,
            "road_object_hybrid_gate_near_low_height":.45,
            "road_object_hybrid_gate_near_min_short_side":.50,
            "road_object_hybrid_gate_far_stable_frames":4,
            "road_object_hybrid_gate_far_close_range":32.0,
            "road_object_hybrid_gate_far_close_min_points":5,
            "road_object_hybrid_gate_far_close_current_points":2,
            "road_object_recovery_adaptive_hybrid_rescue_shadow":True,
            "road_object_hybrid_rescue_near_temporal_hits":3,
            "road_object_hybrid_rescue_near_min_points":5,
            "road_object_hybrid_rescue_far_max_range":32.0,
            "road_object_hybrid_rescue_far_support_frames":3,
            "road_object_hybrid_rescue_far_min_points":4,
            "road_object_hybrid_rescue_far_current_points":1}
        items=[
            {"id":"near_keep","x":15.0,"y":0.0,"point_count":8,
             "road_object_temporal_hits":2,"extent":[.8,.6,.9],
             "adaptive_hybrid_source":"near_baseline"},
            {"id":"near_rescue","x":18.0,"y":0.0,"point_count":5,
             "road_object_temporal_hits":3,"extent":[.4,.2,.9],
             "adaptive_hybrid_source":"near_baseline"},
            {"id":"near_reject","x":19.0,"y":0.0,"point_count":5,
             "road_object_temporal_hits":2,"extent":[.4,.2,.9],
             "adaptive_hybrid_source":"near_baseline"},
            {"id":"far_keep","x":38.0,"y":0.0,"point_count":4,
             "current_point_count":1,"support_frames":4,"extent":[.5,.2,.1],
             "adaptive_hybrid_source":"far_ranked"},
            {"id":"far_rescue","x":30.0,"y":0.0,"point_count":4,
             "current_point_count":1,"support_frames":3,"extent":[.5,.2,.1],
             "adaptive_hybrid_source":"far_ranked"},
            {"id":"far_reject","x":38.0,"y":0.0,"point_count":4,
             "current_point_count":1,"support_frames":3,"extent":[.5,.2,.1],
             "adaptive_hybrid_source":"far_ranked"}]
        recovery._adaptive_hybrid_gate(items,config)
        output,sources=recovery._adaptive_hybrid_temporal_rescue(items,config)
        self.assertEqual(set(["near_keep","near_rescue","far_keep","far_rescue"]),
                         set(item["id"] for item in output))
        self.assertEqual({"near_baseline":1,"far_ranked":1},sources)
        self.assertTrue(next(item for item in output if item["id"]=="near_rescue")
                        ["adaptive_hybrid_temporal_rescue"])

    def test_hybrid_rescue_geometry_gate_filters_only_rescue_additions(self):
        recovery=RoadObjectGeometryRecovery();config={
            "road_object_recovery_adaptive_hybrid_gate_shadow":True,
            "road_object_hybrid_gate_near_min_points":8,
            "road_object_hybrid_gate_near_low_height":.45,
            "road_object_hybrid_gate_near_min_short_side":.50,
            "road_object_hybrid_gate_far_stable_frames":4,
            "road_object_hybrid_gate_far_close_range":32.0,
            "road_object_hybrid_gate_far_close_min_points":5,
            "road_object_hybrid_gate_far_close_current_points":2,
            "road_object_recovery_adaptive_hybrid_rescue_shadow":True,
            "road_object_hybrid_rescue_near_temporal_hits":3,
            "road_object_hybrid_rescue_near_min_points":5,
            "road_object_hybrid_rescue_far_max_range":35.0,
            "road_object_hybrid_rescue_far_support_frames":3,
            "road_object_hybrid_rescue_far_min_points":4,
            "road_object_hybrid_rescue_far_current_points":1,
            "road_object_recovery_adaptive_hybrid_geometry_gate_shadow":True,
            "road_object_hybrid_geometry_gate_near_min_area":.02,
            "road_object_hybrid_geometry_gate_far_max_range":32.0}
        items=[
            {"id":"strict","x":15.0,"y":0.0,"point_count":8,
             "extent":[.8,.6,.9],"adaptive_hybrid_source":"near_baseline"},
            {"id":"near_pass","x":18.0,"y":0.0,"point_count":5,
             "road_object_temporal_hits":3,"extent":[.2,.15,.9],
             "adaptive_hybrid_source":"near_baseline"},
            {"id":"near_fail","x":18.0,"y":0.0,"point_count":5,
             "road_object_temporal_hits":3,"extent":[.1,.1,.9],
             "adaptive_hybrid_source":"near_baseline"},
            {"id":"far_pass","x":31.0,"y":0.0,"point_count":4,
             "current_point_count":1,"support_frames":3,"extent":[.4,.2,.1],
             "adaptive_hybrid_source":"far_ranked"},
            {"id":"far_fail","x":33.0,"y":0.0,"point_count":4,
             "current_point_count":1,"support_frames":3,"extent":[.4,.2,.1],
             "adaptive_hybrid_source":"far_ranked"}]
        recovery._adaptive_hybrid_gate(items,config)
        rescued,_=recovery._adaptive_hybrid_temporal_rescue(items,config)
        output,reasons=recovery._adaptive_hybrid_rescue_geometry_gate(rescued,config)
        self.assertEqual(set(["strict","near_pass","far_pass"]),
                         set(item["id"] for item in output))
        self.assertEqual({"near_area":1,"far_range":1},reasons)

    def test_selected_shadow_output_resolves_named_policy_without_enforcement(self):
        stages={"baseline":[{"id":"baseline"}],
                "adaptive_hybrid_gated":[{"id":"gated"}],
                "adaptive_hybrid_rescued":[{"id":"rescued"}],
                "adaptive_hybrid_geometry_gated":[{"id":"selected"}]}
        output,policy=RoadObjectGeometryRecovery._selected_shadow_output(stages,{
            "road_object_recovery_selected_output_shadow":True,
            "road_object_recovery_selected_output_policy":"adaptive_hybrid_geometry_gated"})
        self.assertEqual("adaptive_hybrid_geometry_gated",policy)
        self.assertEqual("selected",output[0]["id"])
        fallback,policy=RoadObjectGeometryRecovery._selected_shadow_output(stages,{
            "road_object_recovery_selected_output_shadow":True,
            "road_object_recovery_selected_output_policy":"not-a-policy"})
        self.assertEqual("baseline",policy)
        self.assertEqual("baseline",fallback[0]["id"])
        disabled,policy=RoadObjectGeometryRecovery._selected_shadow_output(stages,{})
        self.assertEqual([],disabled)
        self.assertEqual("disabled",policy)

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
        self.assertGreater(diagnostics["stages"]["output"][0]["sensor_range"],0.0)

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

    def test_hybrid_selection_profile_splits_sources_truth_and_fp(self):
        evaluator=GroundTruthEvaluator(None,lambda:None,{
            "road_object_hybrid_feature_profiling":True})
        truth=[{"actor_id":42,"type_id":"static.prop.box02","role":"rsu_test_obstacle",
                "object_type":"unknown_obstacle","x":10.0,"y":0.0,"range":10.0},
               {"actor_id":43,"type_id":"walker.pedestrian.0001","role":"rsu_test_walker",
                "object_type":"person","x":30.0,"y":0.0,"range":30.0}]
        evaluator.truth_objects=lambda:truth
        evaluator._detected_with_range=lambda items:list(items)
        near={"x":10.1,"y":0.0,"range":10.1,"point_count":12,
              "extent":[.5,.3,.2],"adaptive_hybrid_source":"near_baseline"}
        far={"x":30.1,"y":0.0,"range":30.1,"point_count":6,
             "current_point_count":2,"temporal_point_count":4,"support_frames":3,
             "adaptive_rank_score":.7,"extent":[.4,.2,.8],
             "adaptive_hybrid_source":"far_ranked"}
        false={"x":40.0,"y":5.0,"range":40.3,"point_count":8,
               "adaptive_rank_score":.6,"extent":[1.0,.5,.6],
               "adaptive_hybrid_source":"far_ranked"}
        report=evaluator.analyze_road_object_hybrid_profile([near,far,false])
        self.assertEqual(1,report["frame"]["near_baseline"]["matched"])
        self.assertEqual(1,report["frame"]["far_ranked"]["matched"])
        self.assertEqual(1,report["frame"]["far_ranked"]["fp"])
        self.assertEqual(1,report["run"]["far_ranked"]["classes"]["person"]["samples"])
        report=evaluator.analyze_road_object_hybrid_profile([near,far,false])
        self.assertEqual(4,report["run"]["far_ranked"]["candidates"])

    def test_hybrid_rescue_profile_uses_only_incremental_rescues(self):
        evaluator=GroundTruthEvaluator(None,lambda:None,{
            "road_object_hybrid_rescue_feature_profiling":True,
            "road_object_rescue_near_area_ablations":[.08],
            "road_object_rescue_far_range_ablations":[28.0]})
        truth=[{"actor_id":42,"type_id":"static.prop.box02","role":"rsu_test_obstacle",
                "object_type":"unknown_obstacle","x":10.0,"y":0.0,"range":10.0},
               {"actor_id":43,"type_id":"walker.pedestrian.0001","role":"rsu_test_walker",
                "object_type":"person","x":30.0,"y":0.0,"range":30.0}]
        evaluator.truth_objects=lambda:truth
        evaluator._detected_with_range=lambda items:list(items)
        gated={"x":10.1,"y":0.0,"range":10.1,"point_count":12,
               "extent":[.5,.3,.2],"adaptive_hybrid_source":"near_baseline"}
        near={"x":10.2,"y":0.0,"range":10.2,"point_count":5,
              "road_object_temporal_hits":3,"extent":[.4,.2,.8],
              "adaptive_hybrid_source":"near_baseline",
              "adaptive_hybrid_temporal_rescue":True}
        far={"x":30.1,"y":0.0,"range":30.1,"sensor_range":27.0,"point_count":4,
             "current_point_count":1,"temporal_point_count":3,"support_frames":3,
             "extent":[.4,.2,.8],"adaptive_hybrid_source":"far_ranked",
             "adaptive_hybrid_temporal_rescue":True}
        false={"x":25.0,"y":8.0,"range":26.2,"sensor_range":31.0,"point_count":4,
               "current_point_count":1,"support_frames":3,"extent":[.5,.2,.4],
               "adaptive_hybrid_source":"far_ranked",
               "adaptive_hybrid_temporal_rescue":True}
        report=evaluator.analyze_road_object_hybrid_rescue_profile([gated,near,far,false])
        self.assertEqual(1,report["frame"]["near_baseline"]["matched"])
        self.assertEqual(1,report["frame"]["far_ranked"]["matched"])
        self.assertEqual(1,report["frame"]["far_ranked"]["fp"])
        self.assertEqual(3,sum(value["candidates"] for value in report["frame"].values()))
        near_gate=report["ablations_frame"]["near_baseline"]["area>=0.080"]
        self.assertEqual(1,near_gate["truth_kept"])
        far_gate=report["ablations_frame"]["far_ranked"]["sensor_range<=28.0"]
        self.assertEqual(1,far_gate["truth_kept"])
        self.assertEqual(1,far_gate["fp_rejected"])
        report=evaluator.analyze_road_object_hybrid_rescue_profile([gated,near,far,false])
        self.assertEqual(4,report["run"]["far_ranked"]["candidates"])

    def test_rescue_profiler_is_enabled_in_evaluation_config(self):
        path=os.path.join(os.path.dirname(__file__),"..","config","roadside.yaml")
        with open(path,"r") as stream:config=yaml.safe_load(stream)
        self.assertTrue(config["evaluation"]["road_object_hybrid_rescue_feature_profiling"])
        self.assertNotIn("road_object_hybrid_rescue_feature_profiling",config["fusion"])
        self.assertEqual(.02,config["fusion"]["road_object_hybrid_geometry_gate_near_min_area"])
        self.assertEqual(32.0,config["fusion"]["road_object_hybrid_geometry_gate_far_max_range"])

    def test_truth_lifecycle_distinguishes_boundary_exit_jump_and_disappearance(self):
        evaluator=GroundTruthEvaluator(None,lambda:None,{
            "radius":80.0,"truth_lifecycle_diagnostics":True,
            "truth_lifecycle_boundary_margin":10.0,
            "truth_lifecycle_teleport_distance":8.0})
        def actor(actor_id,x,role="rsu_test_walker"):
            return {"actor_id":actor_id,"role":role,"type_id":"walker.pedestrian.0001",
                    "x":float(x),"y":0.0,"range":abs(float(x))}
        first=evaluator.analyze_truth_lifecycle([actor(1,10),actor(2,75)])
        self.assertEqual(2,first["counts"]["entered"])
        second=evaluator.analyze_truth_lifecycle([actor(1,20),actor(3,30)])
        self.assertEqual(1,second["counts"]["entered"])
        self.assertEqual(1,second["counts"]["boundary_exit"])
        self.assertEqual(1,second["counts"]["teleport"])
        third=evaluator.analyze_truth_lifecycle([actor(3,31)])
        self.assertEqual(1,third["counts"]["unexpected_exit"])
        self.assertEqual(1,third["totals"]["boundary_exit"])
        self.assertEqual(1,third["totals"]["unexpected_exit"])

    def test_benchmark_session_resets_pre_spawn_cumulative_metrics(self):
        evaluator=GroundTruthEvaluator(None,lambda:None,{
            "road_object_benchmark_session_isolation":True})
        evaluator._road_object_samples["false"].append({"point_count":4})
        evaluator._adaptive_temporal_samples["false"].append({"point_count":5})
        evaluator._road_object_cap_totals["baseline"]["candidates"]=7
        self.assertFalse(evaluator._sync_road_object_benchmark_session([])["reset"])
        walkers=[{"actor_id":10,"role":"rsu_test_walker"}]
        pending=evaluator._sync_road_object_benchmark_session(walkers)
        self.assertTrue(pending["pending"])
        self.assertFalse(pending["reset"])
        first=[{"actor_id":42,"role":"rsu_test_obstacle"}]
        session=evaluator._sync_road_object_benchmark_session(first)
        self.assertTrue(session["reset"])
        self.assertEqual(1,session["generation"])
        self.assertEqual([],evaluator._road_object_samples["false"])
        self.assertEqual([],evaluator._adaptive_temporal_samples["false"])
        self.assertEqual({},evaluator._hybrid_rescue_samples)
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
