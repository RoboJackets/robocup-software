#pragma once

#include "planning/RotationCommand.hpp"
#include "planning/MotionCommand.hpp"
#include <Constants.hpp>
typedef std::array<float, Num_Shells> RobotMask;

struct ControlSetpoints {
    float xVelocity;
    float yVelocity;
    float aVelocity;
};
struct RobotIntent {
    enum ShootMode { KICK, CHIP };
    enum TriggerMode { STAND_DOWN, IMMEDIATE, ON_BREAK_BEAM };

    std::unique_ptr<Planning::MotionCommand> _motionCommand;
    std::unique_ptr<Planning::RotationCommand> _rotationCommand;

    /// set of obstacles added by plays
    Geometry2d::ShapeSet _local_obstacles;

    /// masks for obstacle avoidance
    RobotMask _opp_avoid_mask;
    float _avoidBallRadius;  /// radius of ball obstacle

    ShootMode shootMode;
    TriggerMode triggerMode;
    int kcStrength;
    float dVelocity;

    ControlSetpoints setpoints;
};