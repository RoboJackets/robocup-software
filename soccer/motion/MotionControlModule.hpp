#pragma once

#include <vector>
#include "Constants.hpp"
#include "Module.hpp"
#include "MotionControl.hpp"

/**
 * Handles motion control for all robots. Calling this once will run motion
 * control on all robots.
 */
class MotionControlModule : public Module {
public:
    explicit MotionControlModule(Context* context);

    void run() override;

private:
    Context* _context;
    std::vector<std::optional<MotionControl>> _controllers;
};