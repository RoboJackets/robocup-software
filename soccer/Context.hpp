#pragma once

#include <Constants.hpp>
#include <set>
#include "DebugDrawer.hpp"
#include "GameState.hpp"
#include "RobotIntent.hpp"
#include "SystemState.hpp"
#include "WorldState.hpp"
#include "motion/MotionSetpoint.hpp"
#include "vision/VisionPacket.hpp"

struct Context {
    Context() : state(this), debug_drawer(this), goalie_id(1337) {}

    // Delete copy, copy-assign, move, and move-assign because
    // many places are expected to hold Context* pointers.
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    Context(Context&&) = delete;
    Context& operator=(Context&&) = delete;

    std::array<RobotIntent, Num_Shells> robot_intents;
    std::array<MotionSetpoint, Num_Shells> motion_setpoints;

    unsigned int goalie_id;

    SystemState state;
    GameState game_state;
    DebugDrawer debug_drawer;

    std::vector<std::unique_ptr<VisionPacket>> vision_packets;
    WorldState world_state;
};
