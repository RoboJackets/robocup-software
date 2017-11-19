#pragma once

#include "MultiRobotPathPlanner.hpp"
#include "SingleRobotPathPlanner.hpp"

namespace Planning {

/// Plans paths for a collection of robots using a SingleRobotPathPlanner for
/// each.  This planner doesn't take other robots' paths into account when
/// planning, which means that occasionally the planned paths will collide.
class IndependentMultiRobotPathPlanner : public MultiRobotPathPlanner {
public:
    virtual std::map<int, std::unique_ptr<Path>> run(
        std::map<int, PlanRequest> requests) override;

private:
    /// Map of shell id -> planner
    std::map<int, std::unique_ptr<SingleRobotPathPlanner>> _planners;
};

}  // namespace Planning
