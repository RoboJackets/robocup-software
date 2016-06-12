#pragma once

#include <array>
#include "const-math.hpp"
#include <Eigen/Dense>
#include "Geometry2d/Util.hpp"

/// Model parameters for a robot.  Used by the controls system.
class RobotModel {
public:
    /// Radius of omni-wheel (in meters)
    float WheelRadius;

    /// Distance from center of robot to center of wheel
    float WheelDist;

    /// Wheel angles (in radians) measured between +x axis and wheel axle
    std::array<float, 4> WheelAngles;

    /// V_bot = wheelToBot * wheelSpeeds;
    mutable Eigen::Matrix<float, 3, 4> WheelToBot;

    void regenerateDerivedMatrices() const {
        const float pi2r = 2 * M_PI * WheelRadius;
        // clang-format off
        WheelToBot << -pi2r*sinf(WheelAngles[0]), -pi2r*sinf(WheelAngles[1]),
                         -pi2r*sinf(WheelAngles[2]), -pi2r*sinf(WheelAngles[3]),
                      pi2r*cosf(WheelAngles[0]), pi2r*cosf(WheelAngles[1]),
                           pi2r*cosf(WheelAngles[2]), pi2r*cosf(WheelAngles[3]),
                      pi2r/WheelDist, pi2r/WheelDist, pi2r/WheelDist, pi2r/WheelDist;
        // TODO: remove pi2 from bottom row?
        // clang-format on
    }
};

// Model parameters for 2015 robot
static const RobotModel RobotModel2015 = []() {
    RobotModel model;
    model.WheelRadius = 0.02856;
    // note: wheels are numbered clockwise, starting with the top-right
    model.WheelAngles = {
        DegreesToRadians(38),
        DegreesToRadians(315),
        DegreesToRadians(225),
        DegreesToRadians(142),
    };
    model.WheelDist = 0.0798576;
    model.regenerateDerivedMatrices();
    return model;
}();
