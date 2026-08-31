"""Orientation-derived geometry for the nominal physical Brick."""
from __future__ import division

import collections
import math


OrientedBrickGeometry = collections.namedtuple(
    'OrientedBrickGeometry',
    ('top_length', 'top_width', 'vertical_height', 'grasp_span',
     'resting_center_z'))


def oriented_brick_geometry(length, width, height, orientation_mode):
    values = tuple(float(value) for value in (length, width, height))
    if (not all(not math.isnan(value) and not math.isinf(value)
                for value in values) or
            values[0] <= values[1] or values[1] <= 0.0 or values[2] <= 0.0):
        raise ValueError('INVALID_BRICK_DIMENSIONS')
    length, width, height = values
    if orientation_mode == 'flat':
        top_width = width
        vertical_height = height
        grasp_span = width
    elif orientation_mode == 'side_up':
        top_width = height
        vertical_height = width
        grasp_span = height
    else:
        raise ValueError('INVALID_BRICK_ORIENTATION_MODE')
    return OrientedBrickGeometry(
        length, top_width, vertical_height, grasp_span,
        vertical_height * 0.5)


def min_cloud_points_for_geometry(reference_minimum, reference_top_area,
                                  geometry):
    reference_minimum = int(reference_minimum)
    reference_top_area = float(reference_top_area)
    actual_top_area = geometry.top_length * geometry.top_width
    values = (reference_top_area, actual_top_area)
    if (reference_minimum <= 0 or
            not all(not math.isnan(value) and not math.isinf(value)
                    for value in values) or
            reference_top_area <= 0.0 or actual_top_area <= 0.0):
        raise ValueError('INVALID_CLOUD_REFERENCE')
    return int(math.ceil(reference_minimum *
                         actual_top_area / reference_top_area))
