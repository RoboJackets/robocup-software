#pragma once

#include <planning/MotionConstraints.hpp>
#include <planning/planner/MotionCommand.hpp>
#include "Context.hpp"
#include <map>
#include <memory>
#include <planning/trajectory/Trajectory.hpp>
#include <utility>
#include "planning/DynamicObstacle.hpp"
#include "planning/RobotConstraints.hpp"
#include "WorldState.hpp"

namespace Planning {

/**
 * @brief Encapsulates information needed for planner to make a path
 *
 * @details This struct contains ALL information necessary for a single
 * robot path to be planned.
 */
struct PlanRequest {
    PlanRequest(RobotInstant start, MotionCommand command,
                RobotConstraints constraints, Trajectory&& prevTrajectory,
                Geometry2d::ShapeSet statics,
                std::vector<DynamicObstacle> dynamics, unsigned shellID,
                const WorldState* world_state,
                int8_t priority = 0,
                DebugDrawer* debug_drawer = nullptr)
        : start(start),
          motionCommand(command),
          constraints(constraints),
          prevTrajectory(prevTrajectory),
          static_obstacles(std::move(statics)),
          dynamic_obstacles(std::move(dynamics)),
          shellID(shellID),
          priority(priority),
          world_state(world_state),
          debug_drawer(debug_drawer) {}

    /**
     * return a copy with no history
     */
    PlanRequest copyNoHistory() const {
        return PlanRequest(start,
                           motionCommand,
                           constraints,
                           Trajectory{{}},
                           static_obstacles,
                           dynamic_obstacles,
                           shellID,
                           world_state,
                           priority,
                           debug_drawer);
    }

    /**
     * The robot's starting state.
     */
    RobotInstant start;

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
     */
    Geometry2d::ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;

    /**
     * The robot's shell ID. Used for debug drawing.
     */
    unsigned shellID;

    /**
     * The state of the world, containing robot and ball states.
     *
     * For obstacle-avoidance purposes, obstacles should be used instead. This
     * can be used for lookup of robots/balls by ID.
     */
    const WorldState* world_state;

    /**
     * The priority of this plan request.
     */
    int8_t priority;

    /**
     * Allows debug drawing in the world. If this is nullptr, no debug drawing
     * should be performed.
     */
    DebugDrawer* debug_drawer;
};

}  // namespace Planning
