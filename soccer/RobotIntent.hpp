#pragma once

#include <planning/RotationCommand.hpp>
#include <planning/MotionCommand.hpp>
enum ShootMode
{
    KICK = 0;
    CHIP = 1;
};

enum TriggerMode
{
    STAND_DOWN = 0;
    IMMEDIATE = 1;
    ON_BREAK_BEAM = 2;
};



struct RobotIntent {
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
};