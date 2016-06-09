#pragma once

#include <array>
#include "const-math.hpp"

class RobotModel {
public:
    std::array<float, 4> wheelAngles;
};

// TODO: get legit values for these
// TODO: account for how wheels are numbered
static const RobotModel RobotModel2015 = RobotModel({M_PI/4,2*M_PI/4,3*M_PI/4,4*M_PI/4});
