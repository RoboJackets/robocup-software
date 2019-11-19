#pragma once

#include <Constants.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include "planning/planner/MotionCommand.hpp"

typedef std::array<float, Num_Shells> RobotMask;

struct RobotIntent {
    enum class ShootMode { KICK, CHIP };
    enum class TriggerMode { STAND_DOWN, IMMEDIATE, ON_BREAK_BEAM };
    enum class Song { STOP, CONTINUE, FIGHT_SONG };

    std::unique_ptr<Planning::MotionCommand> motion_command; //todo(Ethan) get rid of this unique_ptr?

    /// set of obstacles added by plays
    Geometry2d::ShapeSet local_obstacles;

    /// masks for obstacle avoidance
    RobotMask opp_avoid_mask;
    float avoid_ball_radius;  /// radius of ball obstacle

    ShootMode shoot_mode;
    TriggerMode trigger_mode;
    Song song;
    int kcstrength;
    float dvelocity;

    bool is_active;

    void clear() {
        dvelocity = 0;
        kcstrength = 255;
        shoot_mode = ShootMode::KICK;
        trigger_mode = TriggerMode::STAND_DOWN;
        song = Song::CONTINUE;
    }

    RobotIntent() : is_active(false) { clear(); }
};