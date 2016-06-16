#pragma once

#define EIGEN_HAS_CXX11_MATH 0
#include <Eigen/Dense>
#include <array>
#include "Geometry2d/Util.hpp"
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

    /// wheelSpeeds = BotToWheel * V_bot
    Eigen::Matrix<float, 4, 3> BotToWheel;

    /// This should be called when any of the other parameters are changed
    void recalculateBotToWheel() {
        // See this paper for more info on how this matrix is derived:
        // http://people.idsia.ch/~foerster/2006/1/omnidrive_kiart_preprint.pdf

        // clang-format off
        BotToWheel <<
            sinf(WheelAngles[0]), cosf(WheelAngles[0]), -WheelDist,
            sinf(WheelAngles[1]), cosf(WheelAngles[1]), -WheelDist,
            sinf(WheelAngles[2]), cosf(WheelAngles[2]), -WheelDist,
            sinf(WheelAngles[3]), cosf(WheelAngles[3]), -WheelDist;
        BotToWheel /= WheelRadius;
        // clang-format on
    }

    /// (wheel rad/s desired) * DutyCycleMultiplier = duty cycle
    /// Note that this is an approximation, as the relationship isn't exactly
    /// linear
    float DutyCycleMultiplier;
};

/// Model parameters for 2015 robot
static const RobotModel RobotModel2015 = []() {
    RobotModel model;
    model.WheelRadius = 0.02856;
    // note: wheels are numbered clockwise, starting with the top-right
    model.WheelAngles = {
        DegreesToRadians(38), DegreesToRadians(315), DegreesToRadians(225),
        DegreesToRadians(142),
    };
    model.WheelDist = 0.0798576;

    // @125 duty cycle, 1260rpm @ no load
    model.DutyCycleMultiplier = 6; // TODO: tune this value

    model.recalculateBotToWheel();

    return model;
}();
