"""Interchangeable RGB-only mask frontends for aerial Brick perception."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class MaskResult:
    mask: np.ndarray
    state: str
    reason: str
    audit: dict
    accepted_candidates: tuple = ()


def _integer(config, name):
    value = int(config[name])
    if value < 0:
        raise ValueError("{} must be non-negative".format(name))
    return value


def _finite_positive(config, name, allow_zero=False):
    value = float(config[name])
    if not math.isfinite(value) or value < 0.0 or (not allow_zero and value == 0.0):
        raise ValueError("{} must be finite and positive".format(name))
    return value


def projected_target_aspect_ratio(orientation_mode, flat_target, side_up_target):
    """Return the calibrated image-plane target without changing hard gates.

    The physical top-face ratio is not an image-plane invariant under the
    fixed oblique P450 camera.  Keep the legacy flat calibration intact and
    select a separately audited side-up projection calibration.
    """
    targets = {
        "flat": float(flat_target),
        "side_up": float(side_up_target),
    }
    if orientation_mode not in targets:
        raise ValueError("unsupported brick orientation mode {}".format(
            orientation_mode))
    target = targets[orientation_mode]
    if not math.isfinite(target) or target <= 0.0:
        raise ValueError("projected target aspect ratio must be finite and positive")
    return target


class _RedProposalFrontend:
    def __init__(self, config):
        self.hue_low_1 = _integer(config, "red_hue_low_1")
        self.hue_high_1 = _integer(config, "red_hue_high_1")
        self.hue_low_2 = _integer(config, "red_hue_low_2")
        self.hue_high_2 = _integer(config, "red_hue_high_2")
        self.min_saturation = _integer(config, "min_saturation")
        self.min_value = _integer(config, "min_value")
        self.morph_kernel = _integer(config, "morph_kernel")
        if not (0 <= self.hue_low_1 <= self.hue_high_1 <= 179 and
                0 <= self.hue_low_2 <= self.hue_high_2 <= 179 and
                self.min_saturation <= 255 and self.min_value <= 255):
            raise ValueError("HSV proposal configuration is invalid")

    def _red_proposals(self, rgb):
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("RGB input must be uint8 HxWx3")
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        lower_1 = np.array(
            [self.hue_low_1, self.min_saturation, self.min_value], np.uint8)
        upper_1 = np.array([self.hue_high_1, 255, 255], np.uint8)
        lower_2 = np.array(
            [self.hue_low_2, self.min_saturation, self.min_value], np.uint8)
        upper_2 = np.array([self.hue_high_2, 255, 255], np.uint8)
        mask = cv2.inRange(hsv, lower_1, upper_1) | cv2.inRange(
            hsv, lower_2, upper_2)
        if self.morph_kernel > 1:
            kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask


class HSVAllFrontend(_RedProposalFrontend):
    """Legacy diagnostic frontend that returns the union of red proposals."""

    def segment(self, rgb):
        mask = self._red_proposals(rgb)
        pixels = int(np.count_nonzero(mask))
        state = "VALID" if pixels else "REJECTED"
        reason = "" if pixels else "NO_RED_PIXELS"
        return MaskResult(
            mask=mask,
            state=state,
            reason=reason,
            audit={"mode": "hsv_all", "mask_pixels": pixels},
        )


class RGBShapeComponentFrontend(_RedProposalFrontend):
    """Select exactly one RGB red proposal with the expected Brick shape."""

    def __init__(self, config):
        super().__init__(config)
        self.min_component_area = _finite_positive(config, "min_component_area")
        self.max_component_area = _finite_positive(config, "max_component_area")
        self.min_aspect_ratio = _finite_positive(config, "min_aspect_ratio")
        self.max_aspect_ratio = _finite_positive(config, "max_aspect_ratio")
        self.target_aspect_ratio = _finite_positive(config, "target_aspect_ratio")
        self.min_rectangularity = _finite_positive(config, "min_rectangularity")
        self.boundary_margin = _integer(config, "boundary_margin")
        self.min_score = _finite_positive(config, "min_score", allow_zero=True)
        self.ambiguity_margin = _finite_positive(
            config, "ambiguity_margin", allow_zero=True)
        if self.min_component_area >= self.max_component_area:
            raise ValueError("component area limits are invalid")
        if self.min_aspect_ratio >= self.max_aspect_ratio:
            raise ValueError("aspect ratio limits are invalid")
        if not (0.0 < self.min_rectangularity <= 1.0 and
                0.0 <= self.min_score <= 1.0 and
                0.0 <= self.ambiguity_margin <= 1.0):
            raise ValueError("shape score limits are invalid")

    def _candidate(self, contour, image_shape):
        area = float(cv2.contourArea(contour))
        x, y, width, height = cv2.boundingRect(contour)
        rows, columns = image_shape
        audit = {
            "area": area,
            "bounding_rect": [int(x), int(y), int(width), int(height)],
            "rejection": "",
        }
        if (x <= self.boundary_margin or y <= self.boundary_margin or
                x + width >= columns - self.boundary_margin or
                y + height >= rows - self.boundary_margin):
            audit["rejection"] = "IMAGE_BOUNDARY"
            return audit
        if area < self.min_component_area or area > self.max_component_area:
            audit["rejection"] = "AREA"
            return audit
        rectangle = cv2.minAreaRect(contour)
        side_a, side_b = rectangle[1]
        short = min(float(side_a), float(side_b))
        long = max(float(side_a), float(side_b))
        if short <= 0.0 or long <= 0.0:
            audit["rejection"] = "DEGENERATE_RECTANGLE"
            return audit
        aspect_ratio = long / short
        box_area = long * short
        rectangularity = area / box_area
        aspect_score = math.exp(-abs(math.log(
            aspect_ratio / self.target_aspect_ratio)))
        score = 0.65 * aspect_score + 0.35 * min(1.0, rectangularity)
        audit.update({
            "aspect_ratio": aspect_ratio,
            "rectangularity": rectangularity,
            "score": score,
            "rotated_rect": {
                "center": [float(rectangle[0][0]), float(rectangle[0][1])],
                "size": [float(side_a), float(side_b)],
                "angle_deg": float(rectangle[2]),
            },
        })
        if not self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio:
            audit["rejection"] = "ASPECT_RATIO"
        elif rectangularity < self.min_rectangularity:
            audit["rejection"] = "RECTANGULARITY"
        elif score < self.min_score:
            audit["rejection"] = "SCORE"
        return audit

    def segment(self, rgb):
        proposals = self._red_proposals(rgb)
        contours = cv2.findContours(
            proposals, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours = sorted(
            contours, key=lambda item: cv2.boundingRect(item)[:2])
        records = []
        accepted = []
        for index, contour in enumerate(contours):
            audit = self._candidate(contour, proposals.shape)
            audit["index"] = index
            records.append(audit)
            if not audit["rejection"]:
                accepted.append((audit, contour))
        accepted.sort(key=lambda item: (-item[0]["score"], item[0]["index"]))
        common_audit = {
            "mode": "rgb_shape_component",
            "model_target_aspect_ratio": self.target_aspect_ratio,
            "proposal_count": len(records),
            "accepted_candidate_count": len(accepted),
            "candidates": records,
        }
        empty = np.zeros(proposals.shape, dtype=np.uint8)
        if not accepted:
            return MaskResult(empty, "REJECTED", "NO_PLAUSIBLE_COMPONENT",
                              common_audit)
        if (len(accepted) > 1 and
                accepted[0][0]["score"] - accepted[1][0]["score"] <
                self.ambiguity_margin):
            common_audit["top_score_margin"] = (
                accepted[0][0]["score"] - accepted[1][0]["score"])
            return MaskResult(
                empty, "REJECTED", "AMBIGUOUS_COMPONENTS", common_audit,
                tuple(item[0] for item in accepted))
        selected_audit, selected_contour = accepted[0]
        selected = np.zeros(proposals.shape, dtype=np.uint8)
        cv2.drawContours(selected, [selected_contour], -1, 255, thickness=-1)
        common_audit["selected_candidate"] = selected_audit
        common_audit["mask_pixels"] = int(np.count_nonzero(selected))
        if len(accepted) > 1:
            common_audit["top_score_margin"] = (
                accepted[0][0]["score"] - accepted[1][0]["score"])
        return MaskResult(
            selected, "VALID", "", common_audit,
            tuple(item[0] for item in accepted))


def make_mask_frontend(mode, config):
    if mode == "rgb_shape_component":
        return RGBShapeComponentFrontend(config)
    if mode == "hsv_all":
        return HSVAllFrontend(config)
    raise ValueError("unsupported mask frontend {}".format(mode))
