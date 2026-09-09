#pragma once

#include <set>

#include <rj_constants/constants.hpp>
#include <rj_protos/referee.pb.h>
#include <rj_protos/ssl_vision_wrapper.pb.h>

#include "rj_common/control/motion_setpoint.hpp"
#include "rj_common/debug_drawer.hpp"
#include "rj_common/game_settings.hpp"
#include "rj_common/game_state.hpp"
#include "rj_common/planning/robot_constraints.hpp"
#include "rj_common/planning/trajectory.hpp"
#include "rj_common/radio/robot_status.hpp"
#include "rj_common/robot_intent.hpp"
#include "rj_common/team_info.hpp"
#include "rj_common/world_state.hpp"

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

    std::string behavior_tree;
};
