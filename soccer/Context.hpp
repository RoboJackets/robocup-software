#pragma once

#include "DebugDrawer.hpp"
#include "GameState.hpp"
#include "SystemState.hpp"

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
};
