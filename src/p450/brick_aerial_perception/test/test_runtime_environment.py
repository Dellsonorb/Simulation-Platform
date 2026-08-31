#!/usr/bin/env python3

import pathlib
import unittest


WORKSPACE = pathlib.Path(__file__).resolve().parents[3]


class RuntimeEnvironmentTest(unittest.TestCase):
    def test_catkin_uses_ros_noetic_system_python(self):
        path = WORKSPACE / "build" / "brick_aerial_perception" / "CMakeCache.txt"
        if not path.is_file():
            self.skipTest("build cache is unavailable in a source-only checkout")
        cache = path.read_text()
        self.assertIn("PYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3", cache)


if __name__ == "__main__":
    unittest.main()
