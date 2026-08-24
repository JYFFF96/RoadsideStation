from __future__ import print_function

import unittest

from roadside.runtime_status import background_ready_banner


class RuntimeStatusTests(unittest.TestCase):
    def test_background_ready_banner_is_conspicuous(self):
        lines = background_ready_banner()
        self.assertEqual(4, len(lines))
        self.assertEqual("-" * 72, lines[0])
        self.assertEqual(lines[0], lines[-1])
        self.assertEqual("[BACKGROUND] Status:READY", lines[1])
        self.assertIn("START TEST TARGETS NOW", lines[2])


if __name__ == "__main__":
    unittest.main()
