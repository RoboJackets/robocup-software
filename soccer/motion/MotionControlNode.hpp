#pragma once

#include <vector>
#include "Constants.hpp"
#include "MotionControl.hpp"
#include "Node.hpp"

/**
 * Handles motion control for all robots. Calling this once will run motion
 * control on all robots.
 */
class MotionControlNode : public Node {
public:
    explicit MotionControlNode(Context* context);

    void run() override;

private:
    Context* _context;
    std::vector<std::optional<MotionControl>> _controllers;
};