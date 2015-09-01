#pragma once

#include "MultiRobotPathPlanner.hpp"
#include "SingleRobotPathPlanner.hpp"

namespace Planning {

/**
 * @brief Interface for Path Planners that plan paths for a set of robots.
 */
class IndependentMultiRobotPathPlanner : public MultiRobotPathPlanner {
public:
    virtual std::map<int, std::unique_ptr<Path>> run(
        std::map<int, PlanRequest> requests) override;

private:
    /// Map of shell id -> planner
    std::map<int, std::unique_ptr<SingleRobotPathPlanner>> _planners;
};

}  // namespace Planning
