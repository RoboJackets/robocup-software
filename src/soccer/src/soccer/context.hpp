#pragma once

#include <set>

#include <rj_constants/constants.hpp>
#include <rj_protos/referee.pb.h>

#include "control/motion_setpoint.hpp"
#include "debug_drawer.hpp"
#include "game_settings.hpp"
#include "game_state.hpp"
#include "logger.hpp"
#include "planning/robot_constraints.hpp"
#include "planning/trajectory.hpp"
#include "radio/robot_status.hpp"
#include "robot_intent.hpp"
#include "team_info.hpp"
#include "world_state.hpp"

struct Context {
    Context() : debug_drawer(this) {}

    // Delete copy, copy-assign, move, and move-assign because
    // many places are expected to hold Context* pointers.
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    Context(Context&&) = delete;
    Context& operator=(Context&&) = delete;

    // Gameplay -> Planning, Radio
    std::array<RobotIntent, kNumShells> robot_intents;
    // Motion control -> Radio
    std::array<MotionSetpoint, kNumShells> motion_setpoints;
    // Planning -> Motion control
    std::array<planning::Trajectory, kNumShells> trajectories;
    // Radio -> Gameplay
    std::array<RobotStatus, kNumShells> robot_status;
    // Coach -> Positions
    // TODO(sid-parikh) Delete Robot_Positions UI stuff
    std::array<uint32_t, kNumShells> robot_positions;
    // MainWindow -> Manual control
    std::array<bool, kNumShells> is_joystick_controlled{};
    /** \brief Whether at least one joystick is connected */
    bool joystick_valid = false;

    rj_geometry::ShapeSet global_obstacles;
    rj_geometry::ShapeSet def_area_obstacles;

    PlayState play_state = PlayState::halt();
    MatchState match_state;

    TeamInfo our_info;
    TeamInfo their_info;
    bool blue_team = true;
    DebugDrawer debug_drawer;

    std::vector<Referee> referee_packets;
    std::vector<SSL_WrapperPacket> raw_vision_packets;

    WorldState world_state;

    FieldDimensions field_dimensions;

    GameSettings game_settings;

    Logs logs;
    std::string behavior_tree;
};
