#include "rj_strategy/commands/move_command.hpp"

namespace commands {
MoveCommand::MoveCommand(::Robot robot, planning::MotionCommand motion_command,
                         rclcpp_action::Client<RobotMove>::SharedPtr client_ptr)
    : Command("move_command", {robot}) {
    this->client_ptr = client_ptr;

    this->motion_command = motion_command;
}

void MoveCommand::initialize() {
    SPDLOG_INFO(
        "===============================Inititializing Motion "
        "Command=============================");
    RobotIntent intent = RobotIntent{};
    intent.robot_id = this->getRequirements()[0].getId();
    intent.motion_command = this->motion_command;

    auto goal_msg = RobotMove::Goal();
    goal_msg.robot_intent = rj_convert::convert_to_ros(intent);

    auto send_goal_options = rclcpp_action::Client<RobotMove>::SendGoalOptions();
    SPDLOG_INFO(
        "==========================================Motion Command "
        "Init===============================");
    send_goal_options.goal_response_callback = [this](auto arg) { goal_response_callback(arg); };
    send_goal_options.result_callback = [this](auto arg) { result_callback(arg); };
    client_ptr->async_send_goal(goal_msg, send_goal_options);

    this->finished = false;
}

bool MoveCommand::is_done() { return finished; }

void MoveCommand::goal_response_callback(GoalHandleRobotMove::SharedPtr goal_handle) {
    if (goal_handle) {
        SPDLOG_INFO(
            "==========================================Motion Command "
            "Accepted===============================");
    } else {
        SPDLOG_INFO(
            "==========================================Motion Command "
            "Denied===============================");
    }
}

void MoveCommand::result_callback(const GoalHandleRobotMove::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            finished = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            finished = true;
            return;
        case rclcpp_action::ResultCode::CANCELED:
            finished = true;
            return;
        default:
            return;
    }
}

}  // namespace commands