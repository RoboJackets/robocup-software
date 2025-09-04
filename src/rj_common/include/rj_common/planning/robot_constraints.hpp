#pragma once

#include "rj_common/planning/motion_constraints.hpp"
#include "rj_common/planning/rotation_constraints.hpp"

namespace planning {

struct RobotConstraints {
public:
    RobotConstraints() = default;
    RobotConstraints(MotionConstraints mot, RotationConstraints rot) : rot(rot), mot(mot){};
    RotationConstraints rot;
    MotionConstraints mot;
};

}  // namespace planning