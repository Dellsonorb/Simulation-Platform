#!/usr/bin/env python3
import math
import unittest

from brick_aerial_perception.brick_contract import oriented_brick_contract


class BrickContractTest(unittest.TestCase):
    def test_flat_and_side_up_preserve_nominal_physical_dimensions(self):
        flat = oriented_brick_contract(0.240, 0.115, 0.053, "flat")
        side = oriented_brick_contract(0.240, 0.115, 0.053, "side_up")

        self.assertEqual((0.240, 0.115, 0.053), flat.nominal_dimensions)
        self.assertEqual((0.240, 0.115, 0.053), side.nominal_dimensions)
        self.assertEqual((0.240, 0.115), flat.top_dimensions)
        self.assertEqual((0.240, 0.053), side.top_dimensions)
        self.assertAlmostEqual(0.053, flat.vertical_height)
        self.assertAlmostEqual(0.115, side.vertical_height)
        self.assertAlmostEqual(0.115, flat.grasp_span)
        self.assertAlmostEqual(0.053, side.grasp_span)
        self.assertAlmostEqual(0.0265, flat.resting_center_z)
        self.assertAlmostEqual(0.0575, side.resting_center_z)

    def test_invalid_contract_fails_closed(self):
        invalid = (
            (0.240, 0.115, 0.053, "edge_up"),
            (math.nan, 0.115, 0.053, "flat"),
            (0.115, 0.240, 0.053, "flat"),
            (0.240, 0.0, 0.053, "side_up"),
        )
        for values in invalid:
            with self.assertRaises(ValueError):
                oriented_brick_contract(*values)


if __name__ == "__main__":
    unittest.main()
