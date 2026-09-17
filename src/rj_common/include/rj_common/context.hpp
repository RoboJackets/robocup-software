#pragma once

#include <memory>
#include <set>

#include <rj_constants/constants.hpp>
#include <rj_protos/referee.pb.h>

#include "rj_common/control/motion_setpoint.hpp"
#include "rj_common/debug_drawer.hpp"
#include "rj_common/game_settings.hpp"
#include "rj_common/game_state.hpp"
#include "rj_common/live_frame.hpp"
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

    WorldState world_state;

    FieldDimensions field_dimensions;

    GameSettings game_settings;

    std::string behavior_tree;

    std::vector<std::shared_ptr<rj_common::LiveFrame>> frames;

    RJ::Time start_time;
};

static std::shared_ptr<rj_common::LiveFrame> create_log_frame(const Context& context) {
    // Add everything to the log frame.
    auto frame = std::make_shared<rj_common::LiveFrame>();

    // Debug drawing
    frame->debug_draw_frame = context.debug_drawer.published_frame();

    frame->blue = context.blue_team;

    // Our robots
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const auto& state = context.world_state.our_robots.at(shell);
        const auto& status = context.robot_status.at(shell);

        if (RJ::now() - status.timestamp < RJ::Seconds(0.5)) {
            frame->radio_rx_.push_back(status);
        }

        if (!state.visible) {
            continue;
        }

        rj_common::LiveFrame::fill_robot(&frame->self_.emplace_back(), shell, state, &status);
    }

    // Opponent robots
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const auto& state = context.world_state.their_robots.at(shell);
        if (!state.visible) {
            continue;
        }

        rj_common::LiveFrame::fill_robot(&frame->opp_.emplace_back(), shell, state, nullptr);
    }

    // Ball
    if (context.world_state.ball.visible) {
        frame->ball_state = {context.world_state.ball.position, context.world_state.ball.velocity};
    }

    // Field
    frame->manual = context.game_settings.joystick_config.manual_id;
    frame->defend_plus = context.game_settings.defend_plus_x;
    frame->use_our = context.game_settings.use_our_half;
    frame->use_opponent = context.game_settings.use_their_half;

    // Team names
    if (context.blue_team) {
        frame->yellow_name = context.their_info.name;
        frame->blue_name = context.our_info.name;
    } else {
        frame->yellow_name = context.our_info.name;
        frame->blue_name = context.their_info.name;
    }

    return frame;
}
