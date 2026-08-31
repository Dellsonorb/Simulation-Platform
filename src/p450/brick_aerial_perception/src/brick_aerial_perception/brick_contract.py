"""Physical Brick dimensions shared by aerial perception interfaces."""
import collections
import math


BrickContract = collections.namedtuple(
    "BrickContract",
    (
        "orientation_mode",
        "nominal_dimensions",
        "top_dimensions",
        "vertical_height",
        "grasp_span",
        "resting_center_z",
    ),
)


def oriented_brick_contract(length, width, height, orientation_mode):
    """Derive oriented geometry without changing nominal physical dimensions."""
    values = tuple(float(value) for value in (length, width, height))
    if (
        not all(math.isfinite(value) for value in values)
        or values[0] <= values[1]
        or values[1] <= 0.0
        or values[2] <= 0.0
    ):
        raise ValueError("INVALID_BRICK_DIMENSIONS")
    length, width, height = values
    if orientation_mode == "flat":
        top_width = width
        vertical_height = height
        grasp_span = width
    elif orientation_mode == "side_up":
        top_width = height
        vertical_height = width
        grasp_span = height
    else:
        raise ValueError("INVALID_BRICK_ORIENTATION_MODE")
    return BrickContract(
        orientation_mode,
        values,
        (length, top_width),
        vertical_height,
        grasp_span,
        0.5 * vertical_height,
    )
