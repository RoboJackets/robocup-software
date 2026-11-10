#pragma once

namespace planning {

/// How CreatePath::intermediate picks intermediate points around obstacles.
struct IntermediateParams {
    double min_scale;  // m
    double max_scale;  // m
    double min_angle;  // deg
    double max_angle;  // deg
    int num_intermediates;
    double step_size;  // m
};

}  // namespace planning
