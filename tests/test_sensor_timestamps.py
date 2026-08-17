from __future__ import print_function
import unittest
from roadside.sensors import SensorCache


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


if __name__=="__main__":unittest.main()
