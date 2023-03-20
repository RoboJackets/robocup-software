#pragma once

#include "motion_command.hpp"
#include "plan_request.hpp"
#include "planning/instant.hpp"
#include "planning/trajectory.hpp"

namespace planning {

/**
 * Virtual base class for all Planners. An implementation of Planner must take
 * in a PlanRequest from PlannerNode and output a Trajectory for the robot to
 * take. Planners can be state machines internally, they simply must always
 * output a valid Trajectory.
 *
 * Planners should also detail which fields of MotionCommand.msg they use, and
 * for what purpose in the docstring.
 */
class Planner {
public:
    explicit Planner(std::string name) : name_(std::move(name)) {}
    virtual ~Planner() = default;

    Planner(Planner&&) noexcept = default;
    Planner& operator=(Planner&&) noexcept = default;
    Planner(const Planner&) = default;
    Planner& operator=(const Planner&) = default;

    /**
     * Plan a trajectory for this request. This is guaranteed to be a request
     * that this planner is able to handle (according to @ref is_applicable)
     *
     * @param request The request to plan.
     * @return A trajectory for the robot to follow.
     */
    virtual Trajectory plan(const PlanRequest& request) = 0;

    /**
     * Reset this planner. Called after the planner is _not_ used to handle a
     * given command for the robot.
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
