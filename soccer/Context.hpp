#pragma once

#include "SystemState.hpp"

struct Context {
    Context() : state(this) {}

    SystemState state;
};
