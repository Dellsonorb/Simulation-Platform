#!/usr/bin/env python3

import unittest

import cv2
import numpy as np

from brick_aerial_perception.mask_frontend import (
    HSVAllFrontend,
    RGBShapeComponentFrontend,
    make_mask_frontend,
    projected_target_aspect_ratio,
)


def _config(**overrides):
    config = {
        "red_hue_low_1": 0,
        "red_hue_high_1": 12,
        "red_hue_low_2": 165,
        "red_hue_high_2": 179,
        "min_saturation": 80,
        "min_value": 55,
        "morph_kernel": 1,
        "min_component_area": 120.0,
        "max_component_area": 6000.0,
        "min_aspect_ratio": 2.5,
        "max_aspect_ratio": 7.0,
        "target_aspect_ratio": 4.5,
        "min_rectangularity": 0.75,
        "boundary_margin": 2,
        "min_score": 0.70,
        "ambiguity_margin": 0.08,
    }
    config.update(overrides)
    return config


class RGBShapeComponentFrontendTest(unittest.TestCase):
    def test_side_up_uses_calibrated_projected_aspect_without_lowering_gate(self):
        self.assertEqual(
            2.0,
            projected_target_aspect_ratio(
                "side_up", flat_target=4.53, side_up_target=2.0),
        )

        rgb = np.zeros((180, 320, 3), dtype=np.uint8)
        cv2.rectangle(rgb, (110, 80), (209, 129), (220, 20, 15), -1)
        result = RGBShapeComponentFrontend(
            _config(
                min_aspect_ratio=1.8,
                target_aspect_ratio=2.0,
                min_score=0.60,
            )
        ).segment(rgb)

        self.assertEqual("VALID", result.state)
        self.assertGreaterEqual(
            result.audit["selected_candidate"]["score"], 0.60)
        self.assertEqual(2.0, result.audit["model_target_aspect_ratio"])

    def test_flat_keeps_existing_projected_aspect_calibration(self):
        self.assertEqual(
            4.53,
            projected_target_aspect_ratio(
                "flat", flat_target=4.53, side_up_target=2.0),
        )

    def test_selects_brick_rectangle_and_excludes_red_robot_geometry(self):
        rgb = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.rectangle(rgb, (120, 105), (209, 124), (220, 20, 15), -1)
        cv2.rectangle(rgb, (20, 55), (80, 115), (220, 20, 15), -1)
        cv2.circle(rgb, (80, 85), 22, (220, 20, 15), -1)

        result = RGBShapeComponentFrontend(_config()).segment(rgb)

        self.assertEqual("VALID", result.state)
        self.assertEqual("", result.reason)
        self.assertEqual(np.uint8, result.mask.dtype)
        self.assertEqual(rgb.shape[:2], result.mask.shape)
        self.assertGreater(np.count_nonzero(result.mask[105:125, 120:210]), 1500)
        self.assertEqual(0, np.count_nonzero(result.mask[45:126, 10:105]))
        self.assertEqual(1, result.audit["accepted_candidate_count"])
        self.assertEqual(2, result.audit["proposal_count"])

    def test_rejects_when_no_red_candidate_exists(self):
        rgb = np.zeros((120, 160, 3), dtype=np.uint8)
        rgb[:, :, 1] = 180

        result = RGBShapeComponentFrontend(_config()).segment(rgb)

        self.assertEqual("REJECTED", result.state)
        self.assertEqual("NO_PLAUSIBLE_COMPONENT", result.reason)
        self.assertEqual(0, np.count_nonzero(result.mask))

    def test_rejects_component_touching_image_boundary(self):
        rgb = np.zeros((120, 200, 3), dtype=np.uint8)
        cv2.rectangle(rgb, (0, 45), (89, 64), (220, 20, 15), -1)

        result = RGBShapeComponentFrontend(_config()).segment(rgb)

        self.assertEqual("REJECTED", result.state)
        self.assertEqual("NO_PLAUSIBLE_COMPONENT", result.reason)
        self.assertEqual("IMAGE_BOUNDARY", result.audit["candidates"][0]["rejection"])

    def test_rejects_equally_plausible_components_as_ambiguous(self):
        rgb = np.zeros((180, 320, 3), dtype=np.uint8)
        cv2.rectangle(rgb, (30, 45), (119, 64), (220, 20, 15), -1)
        cv2.rectangle(rgb, (180, 105), (269, 124), (220, 20, 15), -1)

        result = RGBShapeComponentFrontend(_config()).segment(rgb)

        self.assertEqual("REJECTED", result.state)
        self.assertEqual("AMBIGUOUS_COMPONENTS", result.reason)
        self.assertEqual(2, result.audit["accepted_candidate_count"])
        self.assertEqual(0, np.count_nonzero(result.mask))

    def test_factory_rejects_unknown_mode(self):
        with self.assertRaises(ValueError):
            make_mask_frontend("object_id_shortcut", _config())

    def test_projected_aspect_rejects_unknown_orientation(self):
        with self.assertRaises(ValueError):
            projected_target_aspect_ratio(
                "diagonal", flat_target=4.53, side_up_target=2.0)


class LegacyHSVFrontendTest(unittest.TestCase):
    def test_legacy_mode_returns_union_of_all_red_pixels(self):
        rgb = np.zeros((100, 180, 3), dtype=np.uint8)
        cv2.rectangle(rgb, (10, 20), (49, 39), (220, 20, 15), -1)
        cv2.rectangle(rgb, (120, 60), (169, 79), (220, 20, 15), -1)

        result = HSVAllFrontend(_config()).segment(rgb)

        self.assertEqual("VALID", result.state)
        self.assertGreater(np.count_nonzero(result.mask[20:40, 10:50]), 0)
        self.assertGreater(np.count_nonzero(result.mask[60:80, 120:170]), 0)
        self.assertEqual("hsv_all", result.audit["mode"])


if __name__ == "__main__":
    unittest.main()
