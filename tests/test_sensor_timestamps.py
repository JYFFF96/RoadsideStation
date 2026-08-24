from __future__ import print_function
import unittest
from roadside.sensors import SensorCache,level_mount_blind_ranges


class SensorTimestampTest(unittest.TestCase):
    def test_cache_preserves_sensor_timestamps(self):
        cache=SensorCache()
        cache.set_camera(11,"camera",1.25)
        cache.set_lidar(10,"lidar",1.20)
        cache.set_radar(9,"radar",1.15)
        camera,lidar,radar=cache.snapshot()
        self.assertEqual((11,"camera",1.25),camera)
        self.assertEqual((10,"lidar",1.20),lidar)
        self.assertEqual((9,"radar",1.15),radar)

    def test_timestamp_is_optional_for_compatibility(self):
        cache=SensorCache();cache.set_camera(1,"camera")
        self.assertEqual((1,"camera",None),cache.snapshot()[0])

    def test_aligned_snapshot_selects_newest_common_camera_lidar_frame(self):
        cache=SensorCache()
        cache.set_camera(20,"camera-20",2.0);cache.set_lidar(20,"lidar-20",2.0)
        cache.set_camera(21,"camera-21",2.1);cache.set_lidar(22,"lidar-22",2.2)
        camera,lidar,radar=cache.snapshot_aligned()
        self.assertEqual(20,camera[0]);self.assertEqual(20,lidar[0]);self.assertIsNone(radar)

    def test_aligned_snapshot_uses_nearest_radar_timestamp(self):
        cache=SensorCache()
        cache.set_radar(29,"radar-29",2.9);cache.set_radar(31,"radar-31",3.1)
        cache.set_camera(30,"camera-30",3.0);cache.set_lidar(30,"lidar-30",3.0)
        self.assertEqual(29,cache.snapshot_aligned()[2][0])

    def test_aligned_snapshot_falls_back_until_common_frame_arrives(self):
        cache=SensorCache();cache.set_camera(4,"camera-4",.4);cache.set_lidar(5,"lidar-5",.5)
        self.assertEqual(cache.snapshot(),cache.snapshot_aligned())

    def test_multi_camera_alignment_uses_lidar_frame_per_camera(self):
        cache=SensorCache()
        cache.set_camera(10,"north-10",1.0,"CAM_NORTH")
        cache.set_camera(11,"north-11",1.1,"CAM_NORTH")
        cache.set_camera(10,"south-10",1.0,"CAM_SOUTH")
        cache.set_camera(12,"south-12",1.2,"CAM_SOUTH")
        cache.set_lidar(10,"lidar-10",1.0)
        cameras,lidar,radar=cache.snapshot_all_aligned()
        self.assertEqual("north-10",cameras["CAM_NORTH"][1])
        self.assertEqual("south-10",cameras["CAM_SOUTH"][1])
        self.assertEqual(10,lidar[0]);self.assertIsNone(radar)
        self.assertEqual("north-11",cache.snapshot()[0][1])

    def test_fairy_height_explains_near_blind_zone(self):
        ground,target=level_mount_blind_ranges(8.5,-15.84,1.7)
        self.assertAlmostEqual(30.0,ground,places=1)
        self.assertAlmostEqual(24.0,target,places=1)


if __name__=="__main__":unittest.main()
