#pragma once

#include <memory>
#include <set>

#include <rj_constants/constants.hpp>
#include <rj_protos/referee.pb.h>

#include "rj_common/control/motion_setpoint.hpp"
#include "rj_common/debug_drawer.hpp"
#include "rj_common/game_settings.hpp"
#include "rj_common/game_state.hpp"
#include "rj_common/planning/robot_constraints.hpp"
#include "rj_common/planning/trajectory.hpp"
#include "rj_common/radio/robot_status.hpp"
#include "rj_common/robot_intent.hpp"
#include "rj_common/team_info.hpp"
#include "rj_common/ui_frame.hpp"
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

    std::vector<std::shared_ptr<rj_common::UIFrame>> frames;

    RJ::Time start_time;
};

inline rj_common::UIRobot create_ui_robot(int shell_id, const RobotState& state,
                    const std::optional<RobotStatus>& status) {
    rj_common::UIRobot out;
    out.shell_id = shell_id;

    out.position = state.pose.position();
    out.heading = state.pose.heading();

    if (status != std::nullopt) {
        out.has_ball = status->has_ball;
        out.motors.resize(5);
        for (int i = 0; i < 5; i++) {
            out.motors.at(i) =
                (status->motors_healthy[i] ?
                    rj_common::MotorStatus::kGood : rj_common::MotorStatus::kFault);
        }
        out.kicker_ok = status->kicker != RobotStatus::KickerState::kFailed;
        out.battery = static_cast<float>(status->battery_voltage);
    }
    return out;
}

inline std::shared_ptr<rj_common::UIFrame> create_ui_frame(const Context& context) {
    auto frame = std::make_shared<rj_common::UIFrame>();
    
    frame->debug_draw_frame = context.debug_drawer.published_frame();

    frame->blue = context.blue_team;

    for (size_t shell = 0; shell < kNumShells; shell++) {
        const auto& state = context.world_state.our_robots.at(shell);
        const auto& status = context.robot_status.at(shell);

        if (RJ::now() - status.timestamp < RJ::Seconds(0.5)) {
            frame->radio_rx.push_back(status);
        }

        if (!state.visible) {
            continue;
        }

        frame->self.push_back(create_ui_robot(shell, state, status));
    }

    for (size_t shell = 0; shell < kNumShells; shell++) {
        const auto& state = context.world_state.their_robots.at(shell);
        if (!state.visible) {
            continue;
        }

        frame->opp.push_back(create_ui_robot(shell, state, std::nullopt));
    }

    if (context.world_state.ball.visible) {
        frame->ball_state = {context.world_state.ball.position, context.world_state.ball.velocity};
    }

    frame->manual_id = context.game_settings.joystick_config.manual_id;
    frame->defend_plus_x = context.game_settings.defend_plus_x;
    frame->use_our_half = context.game_settings.use_our_half;
    frame->use_their_half = context.game_settings.use_their_half;

    if (context.blue_team) {
        frame->yellow_name = context.their_info.name;
        frame->blue_name = context.our_info.name;
    } else {
        frame->yellow_name = context.our_info.name;
        frame->blue_name = context.their_info.name;
    }

    return frame;
}
