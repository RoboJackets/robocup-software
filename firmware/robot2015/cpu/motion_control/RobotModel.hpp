/**
 * This file contains the structures that define the dynamics model of our
 * robots.  Right now each robot has the exact same model parameters, although
 * in the future each one may be configured separately.
 *
 * Robot Coordinate System
 *      +y
 *       ^
 *       |
 *       |
 * w_1/-----\w_0
 *   / mouth \
 *  |         | ---> +x
 *   \       /
 * w_2\-----/w_3
 */

#pragma once

#include <Eigen/Dense>

/// Contains robot model parameters that we care about / account for.
struct RobotModelParams {
    /// Mass of bot in Kg
    float M_bot;
    /// Moment of inertia of bot about the vertical axis through the center of
    /// mass.  In (Kg*m^2)
    float I_bot;
    /// Gear ratio of motor to wheel.  w_motor * g = w_wheel
    float g;
    /// Radius of omni-wheels (meters)
    float r;
    /// distance from the center of wheel to the center of the robot (meters)
    float L;
    /// Resistance of motor from terminal to terminal (ohms)
    float Rt;
    /// Back-emf constant of motor (V/(rad/s))
    /// Maxon lists the "Speed Constant" in rpm/V as 380
    /// invert this to get 1/380 V/rpm
    /// multiply by 60 to get 60/380 V/rps
    /// divide by 2pi to get 30/380pi V/(rad/s)
    float K_e;
    /// Torque constant of motor (N*m/A)
    float K_t;
    /// Viscous friction coefficient of wheel assembly (N*m/(rad/s)).
    /// Tau_friction = w_motor * K_f
    float K_f;
    /// Moment of inertia of wheel assembly.  Tau_accel = I_asm*w_dot_motor
    float I_asm;
    /// Max voltage applied to the motor phases (volts)
    float V_max;
    /// Wheel angles relative to the +x axis, which runs through the right side
    /// of the bot
    float WheelAngles[4];
};


// The RobotModelParams values are compacted down into this class, which
// contains three matrices which capture the dynamics in a form that is used in
// controls calculations.
//
// The state equation of the robot is:
// X_dot = (A1 + A2*dPhiDt)*X + B*u
// where X is the body velocity of the robot.
struct RobotModel {
    /// A2 and A2 are 3x3 matrices
    typedef Eigen::Matrix<float, 3, 3> AType;
    /// B is a 3x4 matrix
    typedef Eigen::Matrix<float, 3, 4> BType;

    RobotModel(const AType &a1, const AType &a2, const BType &b);
    RobotModel(const RobotModelParams &params);

    AType A1, A2;
    BType B;
};
