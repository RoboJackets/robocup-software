#pragma once

#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/referee.pb.h>

#include <rj_constants/constants.hpp>
#include <set>

#include "debug_drawer.hpp"
#include "game_settings.hpp"
#include "game_state.hpp"
#include "logger.hpp"
#include "robot_config.hpp"
#include "robot_intent.hpp"
#include "system_state.hpp"
#include "team_info.hpp"
#include "world_state.hpp"
#include "joystick/gamepad_message.hpp"
#include "motion/motion_setpoint.hpp"
#include "planning/robot_constraints.hpp"
#include "planning/trajectory.hpp"
#include "radio/robot_status.hpp"

struct Context {
    Context() : state(this), debug_drawer(this) {}

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
    std::array<Planning::Trajectory, kNumShells> trajectories;
    // Radio -> Gameplay
    std::array<RobotStatus, kNumShells> robot_status;
    // MainWindow -> Manual control
    std::array<bool, kNumShells> is_joystick_controlled;
    /** \brief Whether at least one joystick is connected */
    bool joystick_valid;

    std::array<RobotLocalConfig, kNumShells> local_configs;
    std::array<RobotConstraints, kNumShells> robot_constraints;
    std::unique_ptr<RobotConfig> robot_config;

    rj_geometry::ShapeSet global_obstacles;
    rj_geometry::ShapeSet goal_zone_obstacles;

    SystemState state;
    GameState game_state;
    TeamInfo our_info;
    TeamInfo their_info;
    bool blue_team;
    DebugDrawer debug_drawer;

    /** \brief Vector of unique IDs of gamepads. First is oldest to connect. */
    std::vector<int> gamepads;
    std::vector<joystick::GamepadMessage> gamepad_messages;

    std::vector<SSL_Referee> referee_packets;
    std::vector<SSL_WrapperPacket> raw_vision_packets;

    WorldState world_state;

    FieldDimensions field_dimensions;

    std::optional<grSim_Packet> grsim_command;

    std::optional<QPointF> ball_command;
    std::optional<rj_geometry::TransformMatrix> screen_to_world_command;

    GameSettings game_settings;

    Logs logs;
    std::string behavior_tree;
};
