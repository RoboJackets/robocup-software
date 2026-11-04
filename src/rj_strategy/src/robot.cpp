#include "rj_strategy/robot.hpp"

#include <spdlog/spdlog.h>

using std::placeholders::_1;

Robot::Robot(int id, rclcpp::Node* node)
    : move_Req("move_" + std::to_string(id)), kick_Req("kick_" + std::to_string(id)) {
    this->id = id;
    this->client_ptr_ = rclcpp_action::create_client<RobotMove>(node, "robot_move");
    this->man_ptr_ = rclcpp_action::create_client<RobotMove>(node, "robot_manipulator");
    this->statusSub = node->create_subscription<rj_msgs::msg::RobotStatus>(
        "radio/robot_status/robot_" + std::to_string(id), 10,
        [this](RobotStatus::SharedPtr msg) { this->robotStatusCallback(msg); });

    SPDLOG_INFO("=======================Robot {} Move Req: {}===================", id, move_Req);
    isKickFinished = false;
    isMoveFinished = false;
    isStatusInitialized = false;
    //   /radio/robot_status/robot_0
}
Robot::Robot(Robot& other)
    : move_Req("move_" + std::to_string(other.id)), kick_Req("kick_" + std::to_string(other.id)) {
    // SPDLOG_ERROR("==============COPIED ROBOT=============");
}

bool Robot::equals(Robot& other) { return this->id == other.getId(); }

int Robot::getId() { return this->id; }

void Robot::robotStatusCallback(RobotStatus::SharedPtr msg) {
    // SPDLOG_INFO("============================Initializing Status========================");
    this->latestStatus = msg;
    // SPDLOG_INFO("==========================Status Initialized 1: {}======================",
    // this->isStatusInitialized);
    this->isStatusInitialized = true;
}

void Robot::goal_response_callback(GoalHandleRobotMove::SharedPtr goal_handle) {
    if (goal_handle) {
        // SPDLOG_INFO("==========================================Motion Command
        // Accepted===============================");
    } else {
        // SPDLOG_INFO("==========================================Motion Command
        // Denied===============================");
    }
}

void Robot::result_callback(const GoalHandleRobotMove::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            isMoveFinished = true;
            SPDLOG_INFO("========================Move command Succeeded=====================");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            isMoveFinished = true;
            SPDLOG_INFO("========================Move command Aborted=====================");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            SPDLOG_INFO("========================Move command Cancelled=====================");
            isMoveFinished = true;
            return;
        default:
            return;
    }
}

void Robot::kick_goal_response_callback(GoalHandleRobotMove::SharedPtr goal_handle) {
    if (goal_handle) {
        // SPDLOG_INFO("==========================================Motion Command
        // Accepted===============================");
    } else {
        // SPDLOG_INFO("==========================================Motion Command
        // Denied===============================");
    }
}

void Robot::kick_result_callback(const GoalHandleRobotMove::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            isKickFinished = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            isKickFinished = true;
            return;
        case rclcpp_action::ResultCode::CANCELED:
            isKickFinished = true;
            return;
        default:
            return;
    }
}

void Robot::sendMoveIntent(RobotIntent& intent) {
    // SPDLOG_INFO("===============================Start Send Move
    // Intent=============================");
    auto goal_msg = RobotMove::Goal();
    intent.robot_id = this->id;

    // SPDLOG_INFO("===============================Inititializing Goal
    // Msg=============================");
    goal_msg.robot_intent = rj_convert::convert_to_ros(intent);
    auto send_goal_options = rclcpp_action::Client<RobotMove>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](auto arg) { goal_response_callback(arg); };

    send_goal_options.result_callback = [this](auto arg) { result_callback(arg); };

    // SPDLOG_INFO("===============================Sending New Goal=============================");
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    // SPDLOG_INFO("===============================Sent New Goal=============================");

    isMoveFinished = false;
    // SPDLOG_INFO("===============================Is Move Finished: {} =============",
    // isMoveFinished);
}

void Robot::sendManipulateIntent(RobotIntent& intent) {
    auto goal_msg = RobotMove::Goal();
    intent.robot_id = this->id;
    goal_msg.robot_intent = rj_convert::convert_to_ros(intent);
    auto send_goal_options = rclcpp_action::Client<RobotMove>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](auto arg) {
        kick_goal_response_callback(arg);
    };

    send_goal_options.result_callback = [this](auto arg) { kick_result_callback(arg); };
    man_ptr_->async_send_goal(goal_msg, send_goal_options);

    isKickFinished = false;
}

std::optional<rj_msgs::msg::RobotStatus::SharedPtr> Robot::getLatestStatus() {
    // SPDLOG_INFO("==========================Status Initialized 2: {}======================",
    // this->isStatusInitialized);
    return this->isStatusInitialized
               ? std::optional<rj_msgs::msg::RobotStatus::SharedPtr>(latestStatus)
               : std::nullopt;
}

bool Robot::getIsMoveFinished() {
    // SPDLOG_INFO("========================Is Move Finished: {} ===============", isMoveFinished);
    return isMoveFinished;
}

bool Robot::getIsKickFinished() { return isKickFinished; }
