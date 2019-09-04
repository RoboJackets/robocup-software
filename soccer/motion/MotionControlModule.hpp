#pragma once

#include <vector>
#include "MotionControl.hpp"
#include "Constants.hpp"

/**
 * Handles motion control for all robots. Calling this once will run motion
 * control on all robots.
 */
class MotionControlModule {
public:
    explicit MotionControlModule(Context* context);

    void run(bool force_stop);

private:
    Context* _context;
    std::vector<std::optional<MotionControl>> _controllers;
};