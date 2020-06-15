#pragma once

#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/referee.pb.h>

#include <rj_constants/constants.hpp>
#include <planning/trajectory/Trajectory.hpp>
#include <set>

#include "DebugDrawer.hpp"
#include "GameSettings.hpp"
#include "GameState.hpp"
#include "Logger.hpp"
#include "RobotConfig.hpp"
#include "RobotIntent.hpp"
#include "SystemState.hpp"
#include "WorldState.hpp"
#include "joystick/GamepadMessage.hpp"
#include "motion/MotionSetpoint.hpp"
#include "planning/RobotConstraints.hpp"
#include "radio/RobotStatus.hpp"
#include "vision/VisionPacket.hpp"

struct Context {
    Context() : state(this), debug_drawer(this) {}

    // Delete copy, copy-assign, move, and move-assign because
    // many places are expected to hold Context* pointers.
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    Context(Context&&) = delete;
    Context& operator=(Context&&) = delete;

    // Gameplay -> Planning, Radio
    std::array<RobotIntent, Num_Shells> robot_intents;
    // Planning -> Motion control
    std::array<MotionSetpoint, Num_Shells> motion_setpoints;
    // Planning -> Motion control
    std::array<Trajectory::Trajectory, Num_Shells> trajectories;
    // Radio -> Gameplay
    std::array<RobotStatus, Num_Shells> robot_status;
    // MainWindow -> Manual control
    std::array<bool, Num_Shells> is_joystick_controlled;
    /** \brief Whether at least one joystick is connected */
    bool joystick_valid;

    std::array<RobotLocalConfig, Num_Shells> local_configs;
    std::array<RobotConstraints, Num_Shells> robot_constraints;
    std::unique_ptr<RobotConfig> robot_config;

    Geometry2d::ShapeSet globalObstacles;
    Geometry2d::ShapeSet goalZoneObstacles;

    SystemState state;
    GameState game_state;
    DebugDrawer debug_drawer;

    /** \brief Vector of unique IDs of gamepads. First is oldest to connect. */
    std::vector<int> gamepads;
    std::vector<joystick::GamepadMessage> gamepad_messages;

    std::vector<std::unique_ptr<VisionPacket>> vision_packets;

    std::vector<SSL_Referee> referee_packets;
    std::vector<SSL_WrapperPacket> raw_vision_packets;

    WorldState world_state;

    Field_Dimensions field_dimensions;

    std::optional<grSim_Packet> grsim_command;

    std::optional<QPointF> ball_command;
    std::optional<Geometry2d::TransformMatrix> screen_to_world_command;

    GameSettings game_settings;

    Logs logs;
    std::string behavior_tree;
};
