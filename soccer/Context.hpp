#pragma once

#include <Constants.hpp>
#include <motion/MotionControl.hpp>
#include "DebugDrawer.hpp"
#include "GameState.hpp"
#include "RobotIntent.hpp"
#include "SystemState.hpp"
#include "WorldState.hpp"
#include "vision/VisionPacket.hpp"

struct Context {
    Context() : state(this), debug_drawer(this) {}

    // Delete copy, copy-assign, move, and move-assign because
    // many places are expected to hold Context* pointers.
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    Context(Context&&) = delete;
    Context& operator=(Context&&) = delete;

    SystemState state;
    GameState game_state;
    DebugDrawer debug_drawer;

    std::vector<std::unique_ptr<VisionPacket>> vision_packets;
    WorldState world_state;

    // construct these right away so they can be used from the SystemState
    // constructor
    std::array<RobotIntent, Num_Shells> robotIntents;
    std::array<MotionSetpoint, Num_Shells> motionSetpoints;
};
