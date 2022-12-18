#pragma once

#include "planning/instant.hpp"
#include "planning/planner/motion_command.hpp"

using rj_geometry::Point;
using rj_geometry::Pose;
using rj_geometry::Twist;

namespace planning {

// definition is in conversion_tests.cpp because we do not have a
// LinearMotionInstant.hpp struct
bool operator==(const LinearMotionInstant& a, const LinearMotionInstant& b);

}  // namespace planning
