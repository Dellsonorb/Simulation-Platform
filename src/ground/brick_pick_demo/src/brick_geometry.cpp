#include <brick_pick_demo/brick_geometry.h>

#include <cmath>

namespace brick_pick_demo {

bool deriveOrientedBrickGeometry(const BrickDimensions& dimensions,
                                 OrientedBrickGeometry* result,
                                 std::string* error) {
  if (!result) {
    if (error) *error = "INVALID_BRICK_GEOMETRY_OUTPUT";
    return false;
  }
  if (!std::isfinite(dimensions.length) ||
      !std::isfinite(dimensions.width) ||
      !std::isfinite(dimensions.height) ||
      dimensions.length <= dimensions.width ||
      dimensions.width <= 0.0 || dimensions.height <= 0.0) {
    if (error) *error = "INVALID_BRICK_DIMENSIONS";
    return false;
  }
  result->top_length = dimensions.length;
  if (dimensions.orientation_mode == "flat") {
    result->top_width = dimensions.width;
    result->vertical_height = dimensions.height;
    result->grasp_span = dimensions.width;
  } else if (dimensions.orientation_mode == "side_up") {
    result->top_width = dimensions.height;
    result->vertical_height = dimensions.width;
    result->grasp_span = dimensions.height;
  } else {
    if (error) *error = "INVALID_BRICK_ORIENTATION_MODE";
    return false;
  }
  result->resting_center_z = result->vertical_height * 0.5;
  if (error) error->clear();
  return true;
}

}  // namespace brick_pick_demo
