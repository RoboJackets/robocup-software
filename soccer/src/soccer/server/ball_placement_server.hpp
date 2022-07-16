#pragma once

#include <functional>
#include <memory>
#include <thread>
#include <vector>

// ros2 action includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// rj includes
#include <planning/planner/motion_command.hpp>
#include <radio/robot_status.hpp>
#include <rj_common/utils.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/ball_placement.hpp>
#include <rj_msgs/msg/is_done.hpp>
#include <rj_msgs/msg/robot_intent.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>
#include <world_state.hpp>

#include "planning/instant.hpp"
#include "planning/planner/plan_request.hpp"
#include "planning/planner_node.hpp"
#include "planning/trajectory.hpp"
#include "rj_geometry/point.hpp"
/* #include "robot_intent.hpp" */

namespace server {
class BallPlacementServer : public rclcpp::Node {
public:
    // TODO: plan here, change while loop condition if needed
    // see this GH comment chain:
    // https://github.com/RoboJackets/robocup-software/pull/1909#discussion_r901173122
    //
    // each of these steps blocks the future steps in the chain
    // 0) find ball->goal_pt vector
    // 0b) make other robots move out of vector path?? (check rules on if allowed)
    // 1) move robot A to behind ball + move robot B to right behind ball placement pt
    // 2) line kick robot A->B + turn on B dribbler (maybe should consider doing the backwards
    // capture drive here?) 3) have B capture ball onto point if necessary (in case of bounce)
    // 4) send SUCCESS to client
    //
    // if there is a bot in the way, debug print for now (later can dribble ball to side and
    // then pass)
    //
    // also, while doing it, send some kind of feedback to client
    enum BallPlacementState { INIT, MOVING_TO_SPOTS, PASSING, ADJUSTING, DONE };

    using BallPlacement = rj_msgs::action::BallPlacement;
    using GoalHandleBallPlacement = rclcpp_action::ServerGoalHandle<BallPlacement>;

    BallPlacementServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~BallPlacementServer() = default;

private:
    // TODO(Alex): docstrings
    rclcpp_action::Server<BallPlacement>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const BallPlacement::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleBallPlacement> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleBallPlacement> goal_handle);
    void execute(const std::shared_ptr<GoalHandleBallPlacement> goal_handle);

    // to keep track of duplicate goal pt requests
    rj_geometry::Point curr_goal_pt_;
    bool was_given_goal_pt_ = false;

    BallPlacementState current_state_ = BallPlacementState::INIT;

    // TODO: check RobotStatus?

    // this should all be gameplay lmao
    rclcpp::Publisher<rj_msgs::msg::RobotIntent>::SharedPtr robot_intent_0_pub_;
    rclcpp::Publisher<rj_msgs::msg::RobotIntent>::SharedPtr robot_intent_1_pub_;

    rclcpp::Subscription<rj_msgs::msg::IsDone>::SharedPtr is_done_0_sub_;
};
}  // namespace server
