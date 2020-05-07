#pragma once

#include <protobuf/grSim_Packet.pb.h>
#include <protobuf/referee.pb.h>

#include <Constants.hpp>
#include <set>

#include "DebugDrawer.hpp"
#include "GameSettings.hpp"
#include "GameState.hpp"
#include "RobotConfig.hpp"
#include "RobotIntent.hpp"
#include "SystemState.hpp"
#include "WorldState.hpp"
#include "motion/MotionSetpoint.hpp"
#include "planning/RobotConstraints.hpp"
#include "vision/VisionPacket.hpp"

struct Context {
    Context() : state(this), debug_drawer(this) {}

    // Delete copy, copy-assign, move, and move-assign because
    // many places are expected to hold Context* pointers.
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    Context(Context&&) = delete;
    Context& operator=(Context&&) = delete;

    std::array<RobotIntent, Num_Shells> robot_intents;
    std::array<MotionSetpoint, Num_Shells> motion_setpoints;
    std::array<Planning::AngleFunctionPath, Num_Shells> paths;
    std::array<RobotStatus, Num_Shells> robot_status;
    std::array<RobotConstraints, Num_Shells> robot_constraints;

    std::array<bool, Num_Shells> is_joystick_controlled;

    std::unique_ptr<RobotConfig> robot_config;

    SystemState state;
    GameState game_state;
    DebugDrawer debug_drawer;

    std::vector<std::unique_ptr<VisionPacket>> vision_packets;
    std::vector<SSL_Referee> referee_packets;

    std::shared_ptr<Packet::LogFrame> logFrame;

    WorldState world_state;

    Field_Dimensions field_dimensions;

    std::optional<grSim_Packet> grsim_command;

    std::optional<QPointF> ball_command;
    std::optional<Geometry2d::TransformMatrix> screen_to_world_command;

    GameSettings game_settings;
};
