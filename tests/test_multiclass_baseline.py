from __future__ import print_function

import fnmatch
import unittest

from roadside.ground_truth_eval import GroundTruthEvaluator
from roadside.fusion import SimpleFusion
from roadside.object_taxonomy import carla_actor_class, object_group


class _Vector(object):
    def __init__(self,x=0.0,y=0.0,z=0.0):self.x=x;self.y=y;self.z=z


class _Box(object):
    def __init__(self,size):self.extent=_Vector(size[0]*.5,size[1]*.5,size[2]*.5)


class _Actor(object):
    def __init__(self,actor_id,type_id,x,y,size,role=""):
        self.id=actor_id;self.type_id=type_id;self._location=_Vector(x,y,0.0)
        self.bounding_box=_Box(size);self.attributes={"role_name":role}
    def get_location(self):return self._location
    def get_velocity(self):return _Vector()


class _Actors(list):
    def filter(self,pattern):return _Actors([x for x in self if fnmatch.fnmatch(x.type_id,pattern)])


class _World(object):
    def __init__(self,actors):self._actors=_Actors(actors)
    def get_actors(self):return self._actors


class MultiClassBaselineTest(unittest.TestCase):
    def test_taxonomy_covers_vehicle_vru_and_obstacle(self):
        self.assertEqual("person",carla_actor_class("walker.pedestrian.0001"))
        self.assertEqual("bicycle",carla_actor_class("vehicle.bh.crossbike"))
        self.assertEqual("motorcycle",carla_actor_class("vehicle.kawasaki.ninja"))
        self.assertEqual("unknown_obstacle",carla_actor_class("static.prop.trafficcone01"))
        self.assertEqual("vru",object_group("person"))

    def test_evaluator_reports_per_class_recall(self):
        world=_World([
            _Actor(1,"vehicle.tesla.model3",20.0,0.0,[4.5,1.8,1.5],"autopilot"),
            _Actor(2,"walker.pedestrian.0001",25.0,0.0,[.5,.5,1.8]),
            _Actor(3,"static.prop.trafficcone01",30.0,0.0,[.4,.4,.8]),
        ])
        evaluator=GroundTruthEvaluator(world,lambda:_Vector(),{
            "radius":80.0,"match_distance":2.0,"include_roles":["autopilot"],
            "include_walkers":True,"obstacle_actor_patterns":["static.prop.*"]})
        metrics=evaluator.evaluate_candidates([
            {"x":20.2,"y":0.0},{"x":25.1,"y":0.0},{"x":60.0,"y":10.0}])
        self.assertEqual(3,metrics["truth"])
        self.assertEqual(2,metrics["matched"])
        self.assertEqual(1,metrics["class_metrics"]["car"]["matched"])
        self.assertEqual(1,metrics["class_metrics"]["person"]["matched"])
        self.assertEqual(1,metrics["class_metrics"]["unknown_obstacle"]["missed"])

    def test_admission_truth_includes_person_instead_of_counting_fp(self):
        world=_World([_Actor(2,"walker.pedestrian.0001",60.0,0.0,[.5,.5,1.8])])
        evaluator=GroundTruthEvaluator(world,lambda:_Vector(),{
            "radius":80.0,"match_distance":2.0,"include_walkers":True,
            "far_admission_edge_risk_shadow":True})
        held=[{"x":60.1,"y":0.0,"candidate_score":.7,
               "roi_details":{"lateral":0.1,"allowed_lateral":4.0}}]
        evaluator.observe_far_admission_decisions(held,[],[],frame_id=1)
        report=evaluator.report_far_admission_decisions()
        self.assertEqual(1,report["would_hold_truth"])
        self.assertEqual(0,report["would_hold_fp"])
        self.assertEqual(1,report["edge_risk_classes"]["would_hold"]["person"]["kept"])

    def test_compact_geometry_receives_multiclass_score_support(self):
        fusion=SimpleFusion("test",{
            "multiclass_compact_geometry_enabled":True,
            "multiclass_compact_score_bonus":.14})
        item={"extent":[.60,.35,1.70],"point_count":2,"scale_votes":1,
              "roi_details":{"lateral":.2,"allowed_lateral":4.0}}
        score=fusion._candidate_score(item)
        self.assertTrue(item["multiclass_compact_geometry"])
        self.assertGreaterEqual(score,.46)

    def test_geometry_attribution_profiles_each_class_and_false_positive(self):
        world=_World([
            _Actor(1,"vehicle.tesla.model3",20.0,0.0,[4.5,1.8,1.5],"autopilot"),
            _Actor(2,"walker.pedestrian.0001",25.0,0.0,[.5,.5,1.8]),
        ])
        evaluator=GroundTruthEvaluator(world,lambda:_Vector(),{
            "radius":80.0,"match_distance":2.0,"include_roles":["autopilot"],
            "include_walkers":True})
        report=evaluator.analyze_geometry_attribution([
            {"x":25.1,"y":0.0,"point_count":4,"extent":[.6,.4,1.7],
             "cluster_mode":"3d","multiclass_compact_geometry":True},
            {"x":60.0,"y":10.0,"point_count":3,"extent":[.2,.2,1.2],
             "cluster_mode":"bev_multiscale"},
        ])
        self.assertEqual(1,report["classes"]["person"]["matched"])
        self.assertEqual(1,report["classes"]["car"]["no_geometry"])
        self.assertEqual(1,report["false_positive"])
        self.assertEqual(1,report["classes"]["person"]["profile"]["sources"]["compact"])

    def test_detection_drop_reasons_are_split_by_class(self):
        world=_World([
            _Actor(1,"vehicle.tesla.model3",20.0,0.0,[4.5,1.8,1.5],"autopilot"),
            _Actor(2,"walker.pedestrian.0001",25.0,0.0,[.5,.5,1.8]),
        ])
        evaluator=GroundTruthEvaluator(world,lambda:_Vector(),{
            "radius":80.0,"match_distance":2.0,"include_roles":["autopilot"],
            "include_walkers":True})
        person={"x":25.1,"y":0.0}
        report=evaluator.analyze_detection_drop_reasons([person],[person],[person],[person])
        self.assertEqual(1,report["class_counts"]["person"]["pass"])
        self.assertEqual(1,report["class_counts"]["car"]["no_geometry_candidate"])


if __name__=="__main__":unittest.main()
