#pragma once

#include <planning/Path.hpp>
#include <planning/PlanRequest.hpp>
#include "SystemState.hpp"
#include "planning/DynamicObstacle.hpp"

#include <map>
#include <memory>

namespace Planning {

/**
 * @brief Interface for Path Planners that plan paths for a set of robots.
 */
class MultiRobotPathPlanner {
public:
    virtual std::map<int, std::unique_ptr<Path>> run(
        std::map<int, PlanRequest> requests) = 0;
};

}  // namespace Planning
