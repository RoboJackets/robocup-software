#pragma once
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rj_common/robot_intent.hpp>
#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/robot_status.hpp>

class Robot {
public:
    using RobotMove = rj_msgs::action::RobotMove;
    using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;
    using RobotStatus = rj_msgs::msg::RobotStatus;
    Robot(int id, rclcpp::Node* node);
    Robot(Robot& other);

    int getId();
    bool equals(Robot& other);
    void sendMoveIntent(RobotIntent& intent);
    void sendManipulateIntent(RobotIntent& intent);
    bool getIsMoveFinished();
    bool getIsKickFinished();

    void robotStatusCallback(RobotStatus::SharedPtr msg);
    const std::string move_Req;
    const std::string kick_Req;
    std::optional<RobotStatus::SharedPtr> getLatestStatus();

private:
    void result_callback(const GoalHandleRobotMove::WrappedResult& result);
    void goal_response_callback(GoalHandleRobotMove::SharedPtr goal_handle);

    void kick_result_callback(const GoalHandleRobotMove::WrappedResult& result);
    void kick_goal_response_callback(GoalHandleRobotMove::SharedPtr goal_handle);

    int id;
    bool isMoveFinished = false;
    bool isKickFinished = false;
    std::shared_ptr<RobotIntent> intent_;
    rclcpp_action::Client<RobotMove>::SharedPtr client_ptr_;
    rclcpp_action::Client<RobotMove>::SharedPtr man_ptr_;

    rclcpp::Subscription<RobotStatus>::SharedPtr statusSub;
    RobotStatus::SharedPtr latestStatus;
    bool isStatusInitialized;
};
