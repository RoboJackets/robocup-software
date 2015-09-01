#pragma once

#include <planning/MotionCommand.hpp>
#include <planning/MotionConstraints.hpp>
#include <planning/MotionInstant.hpp>
#include <planning/Path.hpp>

#include <map>
#include <memory>

namespace Planning {

struct PlanRequest {
    PlanRequest(MotionInstant start, MotionCommand command,
                MotionConstraints constraints, std::unique_ptr<Path> prevPath,
                std::shared_ptr<const Geometry2d::ShapeSet> obstacles)
        : start(start),
          command(command),
          constraints(constraints),
          prevPath(std::move(prevPath)),
          obstacles(obstacles) {}

    PlanRequest() {}

    MotionInstant start;
    MotionCommand command;
    MotionConstraints constraints;
    std::unique_ptr<Path> prevPath;
    std::shared_ptr<const Geometry2d::ShapeSet> obstacles;
};

/**
 * @brief Interface for Path Planners that plan paths for a set of robots.
 */
class MultiRobotPathPlanner {
public:
    virtual std::map<int, std::unique_ptr<Path>> run(
        std::map<int, PlanRequest> requests) = 0;
};

}  // namespace Planning
