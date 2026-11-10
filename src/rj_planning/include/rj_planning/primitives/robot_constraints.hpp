#pragma once

#include "rj_planning/primitives/motion_constraints.hpp"
#include "rj_planning/primitives/rotation_constraints.hpp"

namespace planning {

struct RobotConstraints {
public:
    RobotConstraints() = default;
    RobotConstraints(MotionConstraints mot, RotationConstraints rot) : rot(rot), mot(mot){};
    RotationConstraints rot;
    MotionConstraints mot;
};

}  // namespace planning