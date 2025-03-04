#pragma once

#ifndef EIGEN_HAS_CXX11_MATH
#define EIGEN_HAS_CXX11_MATH 0
#endif
#include <Eigen/Dense>
#include <array>
#include <cmath>


/// Model parameters for a robot.  Used by the controls system.
class RobotModel {

static constexpr double DegreesToRadians(double val) { return val * M_PI / 180.0; }

private:
    RobotModel() {
        WheelRadius = 0.02786;
        
        RearWheelDist = 0.077874;
        FrontWheelDist = 0.078089;
        
        FrontAngle = 30;
        BackAngle = 45;
        
        WheelAngles = {
            DegreesToRadians(180 - FrontAngle),  // M1
            DegreesToRadians(180 + BackAngle),  // M2
            DegreesToRadians(360 - BackAngle),  // M3
            DegreesToRadians(0 + FrontAngle),    // M4
        };

        // = 0.0779815
        WheelDist = (FrontWheelDist + RearWheelDist) / 2.0;

        recalculateBotToWheel();
    }
    
    double RearWheelDist;
    double FrontWheelDist;
    int BackAngle;
    int FrontAngle;

public:
    // singleton pattern
    static RobotModel& get() {
        static RobotModel instance; // Guaranteed to be destroyed.
        // Instantiated on first use.
        return instance;
    }

    RobotModel(RobotModel const&) = delete;
    void operator=(RobotModel const&) = delete;

    /// Radius of omni-wheel (in meters)
    double WheelRadius;

    /// Distance from center of robot to center of wheel
    double WheelDist;

    /// Wheel angles (in radians) measured between +x axis and wheel axle
    std::array<double, 4> WheelAngles;

    /// wheelSpeeds = BotToWheel * V_bot
    Eigen::Matrix<double, 4, 3> BotToWheel;
    Eigen::Matrix<double, 3, 4> WheelToBot;

    /// This should be called when any of the other parameters are changed
    void recalculateBotToWheel() {
        // See this paper for more info on how this matrix is derived:
        // http://people.idsia.ch/~foerster/2006/1/omnidrive_kiart_preprint.pdf

        // Factor WheelDist (R) into this matrix
        // clang-format off
        BotToWheel <<
            -sinf(WheelAngles[0]), cosf(WheelAngles[0]), WheelDist,
            -sinf(WheelAngles[1]), cosf(WheelAngles[1]), WheelDist,
            -sinf(WheelAngles[2]), cosf(WheelAngles[2]), WheelDist,
            -sinf(WheelAngles[3]), cosf(WheelAngles[3]), WheelDist;
        // Invert because our wheels spin opposite to paper
        BotToWheel *= -1;
        BotToWheel /= WheelRadius;
        // clang-format on
        WheelToBot = (BotToWheel.transpose() * BotToWheel).inverse() * BotToWheel.transpose();
    }

    // Convert rad/s to duty cycle
    // Choosen empirically on a no load robot
    // doing the average ratio between commanded speed
    // and output speed
    static constexpr float kDutyCycleToSpeed = 125;
    static constexpr float kSpeedToDutyCycle = 1 / kDutyCycleToSpeed;
};

/// Model parameters for robot.  See RobotModel.cpp for values.
extern const RobotModel RobotModelControl;
