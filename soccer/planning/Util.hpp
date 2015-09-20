#pragma once

#include <Geometry2d/Point.hpp>

namespace Planning {

/// Returns a randomly-generated Point within the bounds of the field. Useful
/// for randomized planning (i.e. RRT)
Geometry2d::Point RandomFieldLocation();

}  // namespace Planning
