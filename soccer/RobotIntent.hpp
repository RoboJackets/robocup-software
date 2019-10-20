#pragma once

#include <Constants.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include "planning/MotionCommand.hpp"
#include "planning/RotationCommand.hpp"

typedef std::array<float, Num_Shells> RobotMask;

struct ControlSetpoints {
    float xvelocity;
    float yvelocity;
    float avelocity;
};
struct RobotIntent {
    enum class ShootMode { KICK, CHIP };
    enum class TriggerMode { STAND_DOWN, IMMEDIATE, ON_BREAK_BEAM };
    enum class Song { STOP, CONTINUE, FIGHT_SONG };

    std::unique_ptr<Planning::MotionCommand> _motionCommand;
    std::unique_ptr<Planning::RotationCommand> _rotationCommand;

    /// set of obstacles added by plays
    Geometry2d::ShapeSet _local_obstacles;

    /// masks for obstacle avoidance
    RobotMask _opp_avoid_mask;
    float _avoidBallRadius;  /// radius of ball obstacle

    ShootMode shootmode;
    TriggerMode triggermode;
    Song song;
    int kcstrength;
    float dvelocity;

    ControlSetpoints setpoints;

    void clear() {
        dvelocity = setpoints.xvelocity = setpoints.yvelocity =
            setpoints.avelocity = 0;
        kcstrength = 255u;
        shootmode = ShootMode::KICK;
        triggermode = TriggerMode::STAND_DOWN;
        song = Song::CONTINUE;
    }
};