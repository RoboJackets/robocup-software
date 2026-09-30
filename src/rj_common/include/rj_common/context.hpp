#pragma once

#include <memory>
#include <set>

#include <rj_protos/referee.pb.h>

#include "rj_common/control/motion_setpoint.hpp"
#include "rj_common/debug_drawer.hpp"
#include "rj_common/game_constants.hpp"
#include "rj_common/game_settings.hpp"
#include "rj_common/game_state.hpp"
#include "rj_common/planning/robot_constraints.hpp"
#include "rj_common/planning/trajectory.hpp"
#include "rj_common/radio/robot_status.hpp"
#include "rj_common/robot_intent.hpp"
#include "rj_common/team_info.hpp"
#include "rj_common/time.hpp"
#include "rj_common/ui_frame.hpp"
#include "rj_common/world_state.hpp"

constexpr size_t kMaxLogFrames = 60 * 60 * 30;
constexpr RJ::Seconds kDropRobotThreshold = RJ::Seconds(0.5);
class Context {
public:
    Context() {}

    // Delete copy, copy-assign, move, and move-assign because
    // many places are expected to hold Context* pointers.
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    Context(Context&&) = delete;
    Context& operator=(Context&&) = delete;

    // Creates a UIFrame based on the current state of Context
    // and adds it to the frames vector.
    void create_ui_frame() {
        auto frame = std::make_shared<rj_common::UIFrame>();

        frame->debug_draw_frame = debug_drawer.published_frame();

        frame->blue = blue_team;

        for (size_t shell = 0; shell < kNumShells; shell++) {
            const auto& state = world_state.our_robots.at(shell);
            const auto& status = robot_status.at(shell);

            // Only add the robot to the frame if it recently sent a radio packet
            if (RJ::now() - status.timestamp < kDropRobotThreshold) {
                frame->radio_rx.push_back(status);
            }

            if (!state.visible) {
                continue;
            }

            frame->self.push_back(rj_common::UIRobot(shell, state, status));
        }

        for (size_t shell = 0; shell < kNumShells; shell++) {
            const auto& state = world_state.their_robots.at(shell);
            if (!state.visible) {
                continue;
            }

            frame->opp.push_back(rj_common::UIRobot(shell, state, std::nullopt));
        }

        if (world_state.ball.visible) {
            frame->ball_state = {world_state.ball.position, world_state.ball.velocity};
        }

        frame->manual_id = game_settings.joystick_config.manual_id;
        frame->defend_plus_x = game_settings.defend_plus_x;
        frame->use_our_half = game_settings.use_our_half;
        frame->use_their_half = game_settings.use_their_half;

        if (blue_team) {
            frame->yellow_name = their_info.name;
            frame->blue_name = our_info.name;
        } else {
            frame->yellow_name = our_info.name;
            frame->blue_name = their_info.name;
        }

        // Only store last 30 minutes of frames
        frames.push_back(frame);
        if (frames.size() > kMaxLogFrames) {
            frames.erase(frames.begin());
        }
    }

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
