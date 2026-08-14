from __future__ import print_function

import unittest

from roadside.road_object_geometry_recovery import RoadObjectGeometryRecovery


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


if __name__=="__main__":unittest.main()
