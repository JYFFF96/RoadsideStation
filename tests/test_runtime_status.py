from __future__ import print_function

import unittest

from roadside.runtime_status import background_ready_banner


class RuntimeStatusTests(unittest.TestCase):
    def test_background_ready_banner_is_conspicuous(self):
        lines = background_ready_banner()
        self.assertEqual(18, len(lines))
        self.assertEqual(["|"] * 8, lines[:8])
        self.assertEqual("[BACKGROUND] Status:READY", lines[8])
        self.assertIn("START TEST TARGETS NOW", lines[9])
        self.assertEqual(["|"] * 8, lines[-8:])


if __name__ == "__main__":
    unittest.main()
