#pragma once

#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/motion_command.hpp>
#include <rj_common/planning/trajectory.hpp>

#include "rclcpp/node.hpp"
#include "rj_planning/plan_request.hpp"
#include "rj_planning/primitives/intermediate_params.hpp"
#include "rj_planning/primitives/rrt_params.hpp"

namespace planning {

struct ConstraintsParams {
    double max_rotational_accel;
    double max_rotational_speed;
    double max_translational_accel;
    double max_translational_speed;
};

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
    PathPlanner(rclcpp::Node* node, std::string name);
    virtual ~PathPlanner();

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
    virtual Trajectory plan(const PlanRequest& request) = 0;

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

protected:
    rclcpp::Node* node_;

    double replanner_off_path_threshold;
    double replanner_partial_replan_lead_time;
    double replanner_pos_change_threshold;
    double replanner_vel_change_threshold;

    RrtParams rrt_params_;
    IntermediateParams intermediate_params_;

private:
    std::string name_;

    RrtParams build_rrt_params();
    std::shared_ptr<rclcpp::Node::OnSetParametersCallbackHandle> rrt_params_callback_;

    IntermediateParams build_intermediate_params();
    std::shared_ptr<rclcpp::Node::OnSetParametersCallbackHandle> intermediate_params_callback_;

    std::shared_ptr<rclcpp::Node::OnSetParametersCallbackHandle> replanner_params_callback_;
};

}  // namespace planning
