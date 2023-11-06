#pragma once

#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// for ROS actions
#include <rclcpp_action/rclcpp_action.hpp>

#include <context.hpp>
#include <rj_common/time.hpp>
#include <rj_msgs/action/robot_move.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "node.hpp"
#include "planner/path_planner.hpp"
#include "planner/plan_request.hpp"
#include "planning/trajectory_collection.hpp"
#include "planning_params.hpp"
#include "robot_intent.hpp"
#include "trajectory.hpp"
#include "world_state.hpp"
#include "planner_for_robot.hpp"

namespace planning {

/**
 * ROS node that spawns many PlannerForRobots and helps coordinate them.
 */
class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

    using RobotMove = rj_msgs::action::RobotMove;
    using GoalHandleRobotMove = rclcpp_action::ServerGoalHandle<RobotMove>;

private:
    std::vector<std::unique_ptr<PlannerForRobot>> robot_planners_;
    TrajectoryCollection robot_trajectories_;
    GlobalState global_state_;
    ::params::LocalROS2ParamProvider param_provider_;
    // setup ActionServer for RobotMove.action
    // follows the standard AS protocol, see ROS2 docs & RobotMove.action
    rclcpp_action::Server<RobotMove>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const RobotMove::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleRobotMove> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleRobotMove> goal_handle);

    /*
     * @brief Upon being given a RobotIntent, publish an appropriate
     * Trajectory, send time remaining as feedback, and return success when
     * done. Blocking (as in, will loop until complete).
     */
    void execute(const std::shared_ptr<GoalHandleRobotMove> goal_handle);

    /*
     * @brief Track the current state of a robot's task. This is how
     * PlannerNode ensures each robot only has one task running.
     */
    struct ServerTaskState {
        ServerTaskState() = default;
        ~ServerTaskState() = default;
        // disallow copy/move operators
        ServerTaskState(const ServerTaskState& state) = delete;
        ServerTaskState& operator=(const ServerTaskState& state) = delete;
        ServerTaskState(const ServerTaskState&& state) = delete;
        ServerTaskState& operator=(const ServerTaskState&& state) = delete;

        volatile std::atomic_bool is_executing{false};
        volatile std::atomic_bool new_task_waiting_signal{false};
    };

    // create an array, kNumShells long, of ServerTaskState structs for
    // PlannerNode to use
    std::array<ServerTaskState, kNumShells> server_task_states_;
};

}  // namespace planning
