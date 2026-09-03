#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/integrations/rm4d_sim_integration"
sys.path.insert(0, str(PACKAGE / "src"))


class Rm4dIntegrationRepositoryContractTest(unittest.TestCase):
    def test_package_exists_only_as_a_thin_integration(self):
        self.assertTrue((PACKAGE / "package.xml").is_file())
        self.assertTrue((PACKAGE / "CMakeLists.txt").is_file())
        self.assertFalse((PACKAGE / "src/rm4d").exists())

    def test_public_geometry_constant_is_frozen(self):
        from rm4d_sim_integration.geometry import (
            RM4D_LOCAL_Y_REGULARIZATION_RAD,
        )

        self.assertEqual(1e-6, RM4D_LOCAL_Y_REGULARIZATION_RAD)


if __name__ == "__main__":
    unittest.main()
