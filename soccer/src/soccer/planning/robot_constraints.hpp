#pragma once

#include "rotation_constraints.hpp"
#include "motion_constraints.hpp"

struct RobotConstraints {
public:
    RobotConstraints(){};
    RobotConstraints(MotionConstraints mot, RotationConstraints rot)
        : rot(rot), mot(mot){};
    RotationConstraints rot;
    MotionConstraints mot;
};