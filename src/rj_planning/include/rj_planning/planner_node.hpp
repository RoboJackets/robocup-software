#pragma once

#include <unordered_map>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/context.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_common/robot_intent.hpp>
#include <rj_common/ros_debug_drawer.hpp>
#include <rj_common/time.hpp>
#include <rj_common/world_state.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/action/robot_move.hpp>
#include <rj_param_utils/planning/planning_params.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "rj_planning/plan_request.hpp"
#include "rj_planning/planner_for_robot.hpp"
#include "rj_planning/planners/path_planner.hpp"
#include "rj_planning/trajectory_collection.hpp"

namespace planning {

struct ConstraintsParams {
    double max_rotational_accel;
    double max_rotational_speed;
    double max_translational_accel;
    double max_translational_speed;
};

struct RrtParams {
    bool enable_debug_drawing;
    double goal_bias;
    double step_size;
    double waypoint_bias;
    int max_iterations;
    int min_iterations;
};

struct LineTestParams {
    double start_x;
    double start_y;
    double end_x;
    double end_y;
    int robot_id;
};

/**
 * ROS node that spawns many PlannerForRobots and helps coordinate them.
 */
class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

    using RobotMove = rj_msgs::action::RobotMove;
    using GoalHandleRobotMove = rclcpp_action::ServerGoalHandle<RobotMove>;

    double replanner_off_path_threshold;
    double replanner_partial_replan_lead_time;
    double replanner_pos_change_threshold;
    double replanner_vel_change_threshold;

private:
    // All parameters put in the ROS2 node
    // TODO: extract to individual path planners and add callbacks
    ConstraintsParams constraints_params_;
    double pivot_radius_multiplier_param_;
    RrtParams rrt_params_;
    LineTestParams line_test_params_;
    double timeout_param_;
    bool use_sim_time_param_;

    // builders for params - Perform get_parameter_or on each param
    ConstraintsParams build_constraints_params();
    RrtParams build_rrt_params();
    LineTestParams build_line_test_params();

    std::vector<std::unique_ptr<PlannerForRobot>> robot_planners_;
    std::shared_ptr<TrajectoryCollection> robot_trajectories_ = nullptr;
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
