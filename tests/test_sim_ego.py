from __future__ import print_function

import unittest

from roadside.sim_ego import find_test_ego_speed_kmh


class _Velocity(object):
    def __init__(self,x,y,z=0):self.x=x;self.y=y;self.z=z


class _Actor(object):
    def __init__(self,role,velocity):self.attributes={"role_name":role};self.velocity=velocity
    def get_velocity(self):return self.velocity


class _Actors(list):
    def filter(self,pattern):return self


class _World(object):
    def __init__(self,actors):self.actors=_Actors(actors)
    def get_actors(self):return self.actors


class SimEgoTest(unittest.TestCase):
    def test_reads_only_tagged_vehicle_speed(self):
        world=_World([_Actor("autopilot",_Velocity(20,0)),
                      _Actor("rsu_test_speeding_vehicle",_Velocity(3,4))])
        self.assertAlmostEqual(18.0,find_test_ego_speed_kmh(world),places=6)

    def test_missing_role_has_no_synthetic_speed(self):
        self.assertIsNone(find_test_ego_speed_kmh(
            _World([_Actor("autopilot",_Velocity(20,0))])))


if __name__=="__main__":unittest.main()
