#pragma once

#include "RotationConstraints.hpp"
#include "MotionConstraints.hpp"

struct RobotConstraints {
public:
    RobotConstraints(){};
    RobotConstraints(MotionConstraints mot, RotationConstraints rot)
        : rot(rot), mot(mot){};
    RotationConstraints rot;
    MotionConstraints mot;
};