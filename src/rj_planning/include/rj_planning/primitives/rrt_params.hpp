#pragma once

/**
 * Struct representing params passed into RRT utils.
 */
namespace planning {

struct RrtParams {
    RrtParams()
        : enable_debug_drawing(false),
          goal_bias(0.3),
          step_size(0.15),
          waypoint_bias(0.5),
          max_iterations(250),
          min_iterations(0) {}

    bool enable_debug_drawing;
    double goal_bias;
    double step_size;
    double waypoint_bias;
    int max_iterations;
    int min_iterations;
};

}  // namespace planning