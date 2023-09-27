#pragma once

#include "motion_command.hpp"
#include "plan_request.hpp"
#include "planning/global_state.hpp"
#include "planning/instant.hpp"
#include "planning/trajectory.hpp"

namespace planning {

/**
 * Virtual base class for all PathPlanners. An implementation of PathPlanner must take
 * in a PlanRequest from PlannerNode and output a Trajectory for the robot to
 * take. PathPlanners can be state machines internally, they simply must always
 * output a valid Trajectory.
 *
 * PathPlanners should also detail which fields of MotionCommand.msg they use, and
 * for what purpose in the docstring.
 */
class PathPlanner {
public:
    explicit PathPlanner(std::string name) : name_(std::move(name)) {}
    virtual ~PathPlanner() = default;

    PathPlanner(PathPlanner&&) noexcept = default;
    PathPlanner& operator=(PathPlanner&&) noexcept = default;
    PathPlanner(const PathPlanner&) = default;
    PathPlanner& operator=(const PathPlanner&) = default;

    /**
     * Plan a trajectory for this request. This is guaranteed to be a request
     * that this PathPlanner is able to handle (name in MotionCommand must
     * match this PathPlanner's name).
     *
     * @param request The request to plan for.
     * @return A Trajectory for the robot (or empty Trajectory if planning
     * fails).
     */
    virtual Trajectory plan(const GlobalState& global_state, const RobotIntent& robot_intent, const DebugDrawer& debug_draw) = 0;

    /**
     * Reset this planner. Called if this planner fails to generate a valid
     * Trajectory.
     */
    virtual void reset() {}

    /*
     * @return true if planner is done, false otherwise.
     */
    [[nodiscard]] virtual bool is_done() const = 0;

    /**
     * Get a human-readable name. RobotIntents will come with strings matching
     * this name.
     *
     * (We have to do it this way because ROS msgs only allow us to pass basic
     * types or other ROS msgs.)
     */
    [[nodiscard]] std::string name() const { return name_; }

private:
    std::string name_;
};

}  // namespace planning
