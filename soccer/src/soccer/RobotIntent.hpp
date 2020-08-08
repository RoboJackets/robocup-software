#pragma once

#include <Geometry2d/ShapeSet.hpp>
#include <rj_constants/constants.hpp>

#include "planning/RotationCommand.hpp"
#include "planning/planner/MotionCommand.hpp"

using RobotMask = std::array<float, kNumShells>;

struct RobotIntent {
    enum class ShootMode { KICK, CHIP };
    enum class TriggerMode { STAND_DOWN, IMMEDIATE, ON_BREAK_BEAM };
    enum class Song { STOP, CONTINUE, FIGHT_SONG };

    Planning::MotionCommand motion_command;

    /// set of obstacles added by plays
    Geometry2d::ShapeSet local_obstacles;

    /// masks for obstacle avoidance
    RobotMask opp_avoid_mask{};
    float avoid_ball_radius{};  /// radius of ball obstacle

    ShootMode shoot_mode = ShootMode::KICK;
    TriggerMode trigger_mode = TriggerMode::STAND_DOWN;
    Song song = Song::STOP;
    int kcstrength = 0;
    float dvelocity = 0;

    bool is_active = false;

    int8_t priority = 0;

    void clear() {
        dvelocity = 0;
        kcstrength = 255;
        shoot_mode = ShootMode::KICK;
        trigger_mode = TriggerMode::STAND_DOWN;
        song = Song::CONTINUE;

        priority = 0;

        motion_command = Planning::EmptyCommand();
    }

    RobotIntent() : motion_command(Planning::EmptyCommand{}) { clear(); }
};
