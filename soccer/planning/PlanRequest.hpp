#pragma once

#include <planning/MotionCommand.hpp>
#include <planning/MotionConstraints.hpp>
#include <planning/MotionInstant.hpp>
#include <planning/Path.hpp>
#include "Context.hpp"
#include "planning/DynamicObstacle.hpp"

#include <map>
#include <memory>
#include "RobotConstraints.hpp"

namespace Planning {

/**
 * @brief Encapsulates information needed for planner to make a path
 *
 * @details This struct contains ALL information necessary for a single
 * robot path to be planned.
 */
struct PlanRequest {
    PlanRequest(Context* context, MotionInstant start,
                std::unique_ptr<MotionCommand> command,
                RobotConstraints constraints, std::unique_ptr<Path> prevPath,
                Geometry2d::ShapeSet obs, std::vector<DynamicObstacle> dObs,
                unsigned shellID, int8_t priority = 0)
        : context(context),
          start(start),
          motionCommand(std::move(command)),
          constraints(constraints),
          prevPath(std::move(prevPath)),
          obstacles(obs),
          dynamicObstacles(dObs),
          shellID(shellID) {}

    Context* context;         /**< Allows debug drawing, position info */
    MotionInstant start;      /**< Starting state of the robot */
    std::unique_ptr<MotionCommand>
        motionCommand;              /**< Specific type of motion desired */
    RobotConstraints constraints;   /**< Constraint parameters on motion */
    std::unique_ptr<Path> prevPath; /**< Last path planned or nullptr */
    Geometry2d::ShapeSet obstacles; /**< Static obstacles */
    std::vector<DynamicObstacle> dynamicObstacles; /**< Dynamic obstacles */
    unsigned shellID; /**< Shell ID used for debug drawing */
    int8_t priority;  /**< Higher priority planned first */
};
}
