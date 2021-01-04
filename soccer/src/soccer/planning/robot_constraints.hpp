#pragma once

#include "rotation_constraints.hpp"
#include "motion_constraints.hpp"

namespace Planning {

struct RobotConstraints {
public:
    RobotConstraints() = default;
    RobotConstraints(MotionConstraints mot, RotationConstraints rot) : rot(rot), mot(mot){};
    RotationConstraints rot;
    MotionConstraints mot;
};

} // namespace Planning