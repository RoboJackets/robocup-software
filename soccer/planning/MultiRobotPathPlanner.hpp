#pragma once

#include <planning/MotionCommand.hpp>
#include <planning/MotionConstraints.hpp>
#include <planning/MotionInstant.hpp>
#include <planning/Path.hpp>

#include <map>
#include <memory>

namespace Planning {

/// The PlanRequest encapsulates all information that the planner needs to know
/// about an individual robot in order to generate a path for it.
struct PlanRequest {
    PlanRequest(MotionInstant start, std::unique_ptr<MotionCommand> command,
                MotionConstraints constraints, std::unique_ptr<Path> prevPath,
                std::shared_ptr<const Geometry2d::ShapeSet> obs)
        : start(start),
          motionCommand(std::move(command)),
          constraints(constraints),
          prevPath(std::move(prevPath)),
          obstacles(obs) {}

    PlanRequest() {}

    MotionInstant start;
    std::unique_ptr<MotionCommand> motionCommand;
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
