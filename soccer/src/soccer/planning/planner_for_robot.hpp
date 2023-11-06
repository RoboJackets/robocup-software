#pragma once

#include <memory>
#include <optional>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <rj_common/time.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_msgs/srv/plan_hypothetical_path.hpp>
#include "planning/planner/escape_obstacles_path_planner.hpp"

#include "global_state.hpp"
#include "planning/planner/escape_obstacles_path_planner.hpp"
#include "planning/planner/path_planner.hpp"
#include "planning/planner/plan_request.hpp"
#include "planning/trajectory.hpp"
#include "planning/trajectory_collection.hpp"
#include "robot_intent.hpp"

namespace planning {

/**
 * @brief Handles one robot's planning, which for us is a translation of RobotIntent to
 * Trajectory. Bundles together all relevant ROS pub/subs and forwards
 * RobotIntents to the appropriate PathPlanner (see path_planner.hpp).
 *
 * In general, prefers default behavior and WARN-level logs over crashing.
 *
 * (PlannerNode makes N PlannerForRobots and handles them all, see
 * PlannerNode below.)
 */
class PlannerForRobot {
public:
    PlannerForRobot(int robot_id, rclcpp::Node* node, TrajectoryCollection* robot_trajectories,
                    const GlobalState& global_state);

    PlannerForRobot(PlannerForRobot&&) = delete;
    const PlannerForRobot& operator=(PlannerForRobot&&) = delete;
    PlannerForRobot(const PlannerForRobot&) = delete;
    const PlannerForRobot& operator=(const PlannerForRobot&) = delete;

    ~PlannerForRobot() = default;

    /**
     * Entry point for the planner node's ActionServer.
     *
     * Creates and publishes a Trajectory based on the given RobotIntent. Also
     * publishes a ManipulatorSetpoint to control kicker/dribbler/chipper.
     */
    void execute_intent(const RobotIntent& intent);

    /*
     * @brief estimate the amount of time it would take for a robot to execute a robot intent
     * (SERVICE).
     *
     * @param request Requested RobotIntent resulting in the hypothetical robot path.
     * @param response The response object that will contain the resultant time to completion of a
     * hypothetical path.
     */
    // TODO(Kevin): I broke this, sorry
    void plan_hypothetical_robot_path(
        const std::shared_ptr<rj_msgs::srv::PlanHypotheticalPath::Request>& request,
        std::shared_ptr<rj_msgs::srv::PlanHypotheticalPath::Response>& response);

    /**
     * @return RJ::Seconds time left for the trajectory to complete
     *
     * Note: RJ::Seconds is an alias for std::chrono::duration<double>.
     */
    [[nodiscard]] std::optional<RJ::Seconds> get_time_left() const;

    /*
     * @return true if current planner is done, false otherwise.
     */
    [[nodiscard]] bool is_done() const;

private:
    /**
     * @brief Create a PlanRequest based on the given RobotIntent.
     *
     * @details This is how gameplay's RobotIntents end up in the right planner (e.g.
     * how a Pivot skill goes to PivotPath planner).
     *
     * @param intent RobotIntent msg
     *
     * @return PlanRequest based on input RobotIntent
     */
    PlanRequest make_request(const RobotIntent& intent);

    /*
     * @brief Get a Trajectory based on the string name given in MotionCommand.
     * Guaranteed to output a valid Trajectory: defaults to
     * EscapeObstaclesPathPlanner if requested planner fails, and gives WARN
     * logs.
     *
     * @details Uses unsafe_plan_for_robot() to get a plan, handling any
     * Exceptions that come up.
     *
     * @param request PlanRequest to create a Trajectory from
     *
     * @return Trajectory (timestamped series of poses & twists) that satisfies
     * the PlanRequest as well as possible
     */
    Trajectory safe_plan_for_robot(const planning::PlanRequest& request);

    /*
     * @brief Get a Trajectory based on the string name given in MotionCommand.
     *
     * @details Differs from safe_plan_for_robot() in that it will throw Exceptions if
     * planners fail (which safe_plan_for_robot() handles).
     *
     * @param request PlanRequest to create a Trajectory from
     *
     * @return Trajectory (timestamped series of poses & twists) that satisfies
     * the PlanRequest as well as possible
     */
    Trajectory unsafe_plan_for_robot(const planning::PlanRequest& request);

    /*
     * @brief Check that robot is visible in world_state and that world_state has been
     * updated recently.
     */
    [[nodiscard]] bool robot_alive() const;

    rclcpp::Node* node_;

    // unique_ptrs here because we don't want to transfer ownership of
    // thePathPlanner objects anywhere else
    // (must be some type of ptr for polymorphism)
    std::unordered_map<std::string, std::unique_ptr<PathPlanner>> path_planners_;
    // EscapeObstaclesPathPlanner is our default path planner
    // because when robots start inside of an obstacle, all other planners will fail
    // TODO(Kevin): make EscapeObstaclesPathPlanner default start of all planner FSM so this doesn't
    // happen
    std::unique_ptr<PathPlanner> default_path_planner_{
        std::make_unique<EscapeObstaclesPathPlanner>()};

    // raw ptr here because current_path_planner_ should not take ownership
    // from any of the unique_ptrs to PathPlanners
    PathPlanner* current_path_planner_{default_path_planner_.get()};

    int robot_id_;
    TrajectoryCollection* robot_trajectories_;
    const GlobalState& global_state_;

    bool had_break_beam_ = false;

    rclcpp::Subscription<RobotIntent::Msg>::SharedPtr intent_sub_;
    rclcpp::Subscription<rj_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Publisher<Trajectory::Msg>::SharedPtr trajectory_topic_;
    rclcpp::Publisher<rj_msgs::msg::ManipulatorSetpoint>::SharedPtr manipulator_pub_;
    rclcpp::Service<rj_msgs::srv::PlanHypotheticalPath>::SharedPtr hypothetical_path_service_;

    rj_drawing::RosDebugDrawer debug_draw_;
};

}  // namespace planning
