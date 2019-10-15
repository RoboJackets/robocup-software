#pragma once

#include <planning/planner/MotionCommand.hpp>
#include <planning/MotionConstraints.hpp>
#include "Context.hpp"

#include <map>
#include <memory>
#include <planning/trajectory/Trajectory.hpp>
#include "planning/RobotConstraints.hpp"

namespace Planning {

/**
 * @brief Encapsulates information needed for planner to make a path
 *
 * @details This struct contains ALL information necessary for a single
 * robot path to be planned.
 */
struct PlanRequest {
    PlanRequest(Context* context, RobotState start,
                MotionCommand command,
                RobotConstraints constraints, Trajectory&& prevTrajectory,
                Geometry2d::ShapeSet obs, unsigned shellID, int8_t priority = 0)
        : context(context),
          start(start),
          motionCommand(command),
          constraints(constraints),
          prevTrajectory(prevTrajectory),
          obstacles(obs),
          shellID(shellID),
          priority(priority) {}

    /**
     * The system context. Used for debug drawing and robot state information.
     */
    Context* context;

    /**
     * The robot's starting state.
     */
    RobotState start;

    /**
     * The goal to plan for.
     */
    MotionCommand motionCommand;

    /**
     * Angular and linear acceleration and velocity constraints on the robot.
     */
    RobotConstraints constraints;

    /**
     * The previous trajectory planned, or nullptr.
     */
    Trajectory prevTrajectory;

    /**
     * The list of static obstacles.
     *
     * For now, we treat all obstacles as static.
     */
    Geometry2d::ShapeSet obstacles;

    /**
     * The robot's shell ID. Used for debug drawing.
     */
    unsigned shellID;

    /**
     * The priority of this plan request.
     */
    int8_t priority;
};

}
