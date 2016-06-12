#pragma once

#include <array>
#include <Eigen/Dense>
#include "Geometry2d/Util.hpp"
#undef M_PI
#include "const-math.hpp"

/// Model parameters for a robot.  Used by the controls system.
class RobotModel {
public:
    /// Radius of omni-wheel (in meters)
    float WheelRadius;

    /// Distance from center of robot to center of wheel
    float WheelDist;

    /// Wheel angles (in radians) measured between +x axis and wheel axle
    std::array<float, 4> WheelAngles;

    /// V_bot = WheelToBot * wheelSpeeds;
    Eigen::Matrix<float, 3, 4> WheelToBot;

    /// wheelSpeeds = BotToWheel * V_bot
    /// BotToWheel = pseudoinverse(WheelToBot)
    Eigen::Matrix<float, 4, 3> BotToWheel;

    void regenerateDerivedMatrices() {
        const float pi2r = 2 * M_PI * WheelRadius;
        // clang-format off
        WheelToBot << -pi2r*sinf(WheelAngles[0]), -pi2r*sinf(WheelAngles[1]),
                         -pi2r*sinf(WheelAngles[2]), -pi2r*sinf(WheelAngles[3]),
                      pi2r*cosf(WheelAngles[0]), pi2r*cosf(WheelAngles[1]),
                           pi2r*cosf(WheelAngles[2]), pi2r*cosf(WheelAngles[3]),
                      pi2r/WheelDist, pi2r/WheelDist, pi2r/WheelDist, pi2r/WheelDist;
        // TODO: remove pi2 from bottom row?
        // clang-format on

        // TODO: compute BotToWheel!
    }

    /// (wheel rad/s desired) * DutyCycleMultiplier = duty cycle
    /// Note that this is an approximation, as the relationship isn't exactly linear
    float DutyCycleMultiplier;
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
    model.DutyCycleMultiplier = 10; // TODO: this value is garbage
    model.regenerateDerivedMatrices();
    return model;
}();
