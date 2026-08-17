from __future__ import print_function

import unittest

from roadside.map_selection import (carla_map_short_name, is_town05_map,
                                     town05_switch_target)


class MapSelectionTest(unittest.TestCase):
    def test_normalizes_full_carla_asset_path(self):
        self.assertEqual("Town05_Opt", carla_map_short_name(
            "/Game/Carla/Maps/Town05_Opt"))

    def test_all_town05_variants_keep_current_world(self):
        self.assertTrue(is_town05_map("Town05"))
        self.assertTrue(is_town05_map("/Game/Carla/Maps/Town05_Opt"))
        self.assertIsNone(town05_switch_target("town05_custom", "Town05_Opt"))

    def test_non_town05_requests_configured_town05_target(self):
        self.assertEqual("Town05_Opt", town05_switch_target(
            "/Game/Carla/Maps/Town10HD_Opt", "Town05_Opt"))
        self.assertEqual("Town05_Opt", town05_switch_target(
            "Town03", "Town10HD_Opt"))


if __name__ == "__main__":
    unittest.main()
