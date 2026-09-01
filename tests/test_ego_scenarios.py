"""Offline contract tests; real CARLA rendering/physics still need an integration run."""
import math
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

from roadside.ego_camera import EgoCamera
from roadside.models import DetectedObject, ObjectList
from roadside.scenario_ego import (ScenarioEgo, _angle_delta_degrees,
                                   _relative_xy, _straightest_successor)
from roadside.sim_ego import EGO_ROLE, find_ego_actor, read_ego_state
from roadside.v2x_events import V2XEventEngine


def vector(x=0, y=0, z=0):
    return NS(x=x, y=y, z=z)


def actor(actor_id=7, role=EGO_ROLE):
    result = MagicMock()
    result.id = actor_id
    result.attributes = {"role_name": role}
    result.is_alive = True
    result.get_velocity.return_value = vector(3, 4)
    tf = result.get_transform.return_value
    tf.location = vector(10, 20)
    tf.rotation.yaw = 90.0
    tf.get_matrix.return_value = [[0, -1, 0, 10], [1, 0, 0, 20],
                                  [0, 0, 1, 0], [0, 0, 0, 1]]
    result.bounding_box.extent = vector(2.0, 1.0, .8)
    result.bounding_box.location = vector(.5, 0, .8)
    return result


def world_with(*actors):
    world = MagicMock()
    world.get_actors.return_value.filter.return_value = list(actors)
    return world


def fake_carla():
    return NS(Transform=MagicMock(), Location=MagicMock(), Rotation=MagicMock(),
              VehicleControl=lambda **kwargs:NS(**kwargs), AttachmentType=NS(Rigid="rigid"),
              TrafficLightState=NS(Red="red",Yellow="yellow"),
              LaneType=NS(Driving="driving"))


class EgoReferenceTests(unittest.TestCase):
    def test_read_rotated_bbox_and_speed_not_other_traffic(self):
        ego = actor()
        state = read_ego_state(world_with(actor(6, "autopilot"), ego))
        self.assertEqual(7, state["actor_id"])
        self.assertEqual(18, state["speed_kmh"])
        self.assertAlmostEqual(10, state["bbox_x"])
        self.assertAlmostEqual(20.5, state["bbox_y"])
        self.assertEqual(.8, state["bbox_z"])
        self.assertEqual(90, state["yaw_deg"])

    def test_missing_dead_duplicate_and_custom_role(self):
        dead = actor(); dead.is_alive = False
        self.assertEqual({}, read_ego_state(world_with(dead, actor(8, "autopilot"))))
        with self.assertRaisesRegex(ValueError, "Multiple ego"):
            find_ego_actor(world_with(actor(7), actor(9)))
        custom = actor(10, "my_ego")
        self.assertIs(custom, find_ego_actor(world_with(custom), "my_ego"))

    def test_parked_ego_not_avw_but_target_remains_and_input_not_mutated(self):
        ego = read_ego_state(world_with(actor()))
        own = DetectedObject("tracker_101", 10, 20.5, object_type="car")
        target = DetectedObject("tracker_202", 10, 40, object_type="car")
        engine = V2XEventEngine("R", {"enabled": True,
            "vrucw": {"enabled": False}, "hlw": {"enabled": False},
            "slw": {"enabled": False}, "avw": {"dwell_seconds": 0}})
        self.assertEqual([], engine.update(ObjectList("R", [own], 1), ego))
        original = ObjectList("R", [own, target], 2)
        event = engine.update(original, ego)[0]
        self.assertEqual("tracker_202", event["data"]["object_id"])
        self.assertEqual(1, event["data"]["vehicle_count"])
        self.assertEqual([own, target], original.objects)
        self.assertEqual(1, engine.last_diagnostics["ego"]["self_detections_excluded"])

    def test_adjacent_car_and_pedestrian_not_masked(self):
        ego = read_ego_state(world_with(actor()))
        # At yaw=90 the lateral direction is world X. Adjacent lane is outside bbox.
        adjacent = DetectedObject("next_lane", 13.5, 20.5, object_type="car")
        person = DetectedObject("person", 10, 20.5, object_type="person")
        engine = V2XEventEngine("R", {"enabled": True, "hlw": {"enabled": False},
            "vrucw": {"required_updates": 1}, "slw": {"enabled": False},
            "avw": {"dwell_seconds": 0}})
        events = engine.update(ObjectList("R", [adjacent, person], 1), ego)
        self.assertEqual({"VRUCW", "AVW"}, {e["data"]["category"] for e in events})
        self.assertEqual(0, engine.last_diagnostics["ego"]["self_detections_excluded"])

    def test_unknown_self_geometry_does_not_trigger_hlw(self):
        from roadside.fused_objects import FusedObject, FusedObjectList
        own = FusedObject("fragment", x=10, y=20.5, z=.8,
                          size=[1, .5, .5], confidence=.9, age=10,
                          perception_evidence={"roadObjectSelectedEver": True})
        engine = V2XEventEngine("R", {"enabled": True,
            "hlw": {"required_updates": 1}, "avw": {"enabled": False},
            "vrucw": {"enabled": False}, "slw": {"enabled": False}})
        data = FusedObjectList("R", [own], 1)
        self.assertEqual([], engine.update(data, read_ego_state(world_with(actor()))))
        self.assertEqual("HLW", engine.update(data, {})[0]["data"]["category"])


class ScenarioLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.patch = patch.dict(sys.modules, {"carla": fake_carla()})
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def test_reuse_never_changes_control_or_destroys_foreign_ego(self):
        car = actor()
        world = world_with(car)
        owner = ScenarioEgo(world)
        self.assertIs(car, owner.start(MagicMock(), MagicMock(), vector()))
        owner.close(); owner.close()
        car.set_autopilot.assert_not_called()
        car.destroy.assert_not_called()
        world.try_spawn_actor.assert_not_called()

    def setup_spawn(self):
        world = world_with()
        car = actor()
        world.try_spawn_actor.return_value = car
        bp = MagicMock(); bp.id = "vehicle.tesla.model3"
        bp.get_attribute.return_value.as_int.return_value = 4
        world.get_blueprint_library.return_value.filter.return_value = [bp]
        wp = MagicMock(); wp.is_junction = False; wp.road_id=1;wp.lane_id=-1
        wp.transform.location = MagicMock(x=-45, y=0, z=0)
        wp.transform.location.distance.return_value = 45
        wp.transform.rotation.pitch=0;wp.transform.rotation.yaw=0;wp.transform.rotation.roll=0
        wp.transform.get_forward_vector.return_value = vector(1, 0)
        route=MagicMock();route.road_id=1;route.lane_id=-1;route.lane_width=3.5
        route.transform.location=vector(10,20);route.transform.rotation.yaw=90
        successor=MagicMock();successor.road_id=2;successor.lane_id=-1
        successor.transform.location=vector(10,24);successor.transform.rotation.yaw=90
        route.next.return_value=[successor]
        world_map = MagicMock(); world_map.generate_waypoints.return_value = [wp]
        world_map.get_waypoint.return_value=route
        return world, car, world_map

    def test_owned_ego_uses_waypoint_control_and_is_destroyed_once(self):
        world, car, world_map = self.setup_spawn()
        client = MagicMock()
        owner = ScenarioEgo(world)
        owner.start(client, world_map, vector(), 55, 8003)
        client.get_trafficmanager.assert_not_called()
        car.set_autopilot.assert_called_once_with(False)
        self.assertEqual("lane_follow",owner.last_status["mode"])
        self.assertGreater(car.apply_control.call_args[0][0].throttle,0)
        owner.close(); owner.close()
        car.destroy.assert_called_once()
        world.tick.assert_not_called()
        world.apply_settings.assert_not_called()

    def test_failed_initial_control_cleans_spawned_ego(self):
        world, car, world_map = self.setup_spawn()
        client = MagicMock();car.apply_control.side_effect=RuntimeError("control failed")
        owner = ScenarioEgo(world)
        with self.assertRaisesRegex(RuntimeError, "control failed"):
            owner.start(client, world_map, vector())
        car.destroy.assert_called_once()
        self.assertIsNone(owner.actor)

    def test_all_spawn_points_blocked_reports_failure(self):
        world, car, world_map = self.setup_spawn()
        world.try_spawn_actor.return_value = None
        owner = ScenarioEgo(world)
        with self.assertRaisesRegex(RuntimeError, "Cannot spawn ego"):
            owner.start(MagicMock(), world_map, vector())
        self.assertFalse(owner.owned)
        car.destroy.assert_not_called()

    def test_reused_ego_update_never_applies_control(self):
        car=actor();owner=ScenarioEgo(world_with(car))
        owner.start(MagicMock(),MagicMock(),vector())
        self.assertEqual("reused_no_control",owner.update_control()["mode"])
        car.apply_control.assert_not_called()

    def test_front_vehicle_or_walker_triggers_full_brake(self):
        for kind in ("vehicle","walker"):
            world,car,world_map=self.setup_spawn();owner=ScenarioEgo(world)
            owner.start(MagicMock(),world_map,vector(),18);car.apply_control.reset_mock()
            hazard=actor(20 if kind=="vehicle" else 21,"target")
            hazard.get_location.return_value=vector(10,27)
            hazard.bounding_box.extent=vector(1,.5,1)
            owner.set_hazards([hazard]);status=owner.update_control()
            self.assertEqual("hazard_stop",status["mode"])
            self.assertEqual(hazard.id,status["hazard_id"])
            self.assertEqual(1.0,car.apply_control.call_args[0][0].brake)

    def test_adjacent_lane_hazard_does_not_trigger_brake(self):
        world,car,world_map=self.setup_spawn();owner=ScenarioEgo(world)
        owner.start(MagicMock(),world_map,vector(),18);car.apply_control.reset_mock()
        hazard=actor(30,"target");hazard.get_location.return_value=vector(14,27)
        hazard.bounding_box.extent=vector(1,.5,1);owner.set_hazards([hazard])
        self.assertEqual("lane_follow",owner.update_control()["mode"])
        self.assertEqual(0.0,car.apply_control.call_args[0][0].brake)

    def test_lane_departure_stops_instead_of_cutting_across_lanes(self):
        world,car,world_map=self.setup_spawn();owner=ScenarioEgo(world)
        owner.start(MagicMock(),world_map,vector(),18);car.apply_control.reset_mock()
        world_map.get_waypoint.return_value.transform.location=vector(15,20)
        status=owner.update_control()
        self.assertEqual("lane_departure_stop",status["mode"])
        self.assertEqual(1.0,car.apply_control.call_args[0][0].brake)

    def test_route_math_selects_straightest_successor(self):
        current=MagicMock();current.transform.rotation.yaw=170
        choices=[]
        for yaw in (-170,90,150):
            wp=MagicMock();wp.transform.rotation.yaw=yaw;wp.road_id=yaw;wp.lane_id=1
            choices.append(wp)
        self.assertIs(choices[0],_straightest_successor(current,choices))
        self.assertEqual(20,_angle_delta_degrees(-170,170))
        transform=NS(location=vector(0,0),rotation=NS(yaw=90))
        forward,lateral=_relative_xy(transform,vector(0,8))
        self.assertAlmostEqual(8,forward);self.assertAlmostEqual(0,lateral)

    def test_layout_selects_one_waypoint_per_junction_entry_lane(self):
        from tools.spawn_multiclass_targets import incoming_lane_waypoints
        center=vector(0,0,0);junction=MagicMock();junction.id=99
        junction.bounding_box.location=center
        sample=MagicMock();sample.is_junction=True;sample.get_junction.return_value=junction
        entries=[]
        for road,lane,location,forward,placed in (
                (1,-1,vector(-10,0),vector(1,0),vector(-20,0)),
                (2,-2,vector(0,-10),vector(0,1),vector(0,-20))):
            entry=MagicMock();entry.road_id=road;entry.section_id=0;entry.lane_id=lane
            entry.transform.location=location;entry.transform.get_forward_vector.return_value=forward
            placement=MagicMock();placement.road_id=road;placement.section_id=0;placement.lane_id=lane
            placement.transform.location=placed
            placement.transform.get_forward_vector.return_value=forward
            entry.previous.return_value=[placement];entries.append(entry)
        junction.get_waypoints.return_value=[(entries[0],MagicMock()),
                                             (entries[1],MagicMock()),
                                             (entries[0],MagicMock())]
        world_map=MagicMock();world_map.generate_waypoints.return_value=[sample]
        result=incoming_lane_waypoints(world_map,center,20)
        self.assertEqual(2,len(result))
        self.assertEqual({(1,-1),(2,-2)},
                         {(item.road_id,item.lane_id) for item in result})


class EgoCameraLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.patch = patch.dict(sys.modules, {"carla": fake_carla()})
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def test_detach_reattach_drops_stale_images_and_preserves_ego(self):
        world = world_with()
        first, second = MagicMock(), MagicMock()
        world.spawn_actor.side_effect = [first, second]
        car = actor()
        view = EgoCamera()
        view.attach(world, car)
        receive_old = first.listen.call_args[0][0]
        receive_old("old image")
        self.assertEqual("old image", view.latest[0])
        view.view = "driver"
        view.attach(world, car)
        first.stop.assert_called_once(); first.destroy.assert_called_once()
        receive_old("delayed image")
        self.assertIsNone(view.latest)
        second.listen.call_args[0][0]("new image")
        self.assertEqual("new image", view.latest[0])
        view.close(); view.close()
        second.destroy.assert_called_once()
        car.destroy.assert_not_called()
        world.tick.assert_not_called()
        world.get_spectator.assert_not_called()
        world.apply_settings.assert_not_called()

    def test_listener_failure_and_stop_failure_still_destroy_sensor(self):
        world = world_with()
        sensor = world.spawn_actor.return_value
        sensor.listen.side_effect = RuntimeError("listen failed")
        sensor.stop.side_effect = RuntimeError("stop failed")
        view = EgoCamera()
        with self.assertRaisesRegex(RuntimeError, "listen failed"):
            view.attach(world, actor())
        sensor.destroy.assert_called_once()
        self.assertIsNone(view.camera)


if __name__ == "__main__":
    unittest.main()
