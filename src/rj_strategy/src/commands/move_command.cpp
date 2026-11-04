#include "rj_strategy/commands/move_command.hpp"

#include <string>

namespace commands {
MoveCommand::MoveCommand(std::shared_ptr<Robot> robot, planning::MotionCommand motion_command)
    : commands::Command("move_command" + std::to_string(robot->getId())) {
    this->robot = robot;
    addRequirements(robot->move_Req);
    this->motion_command = motion_command;
}

void MoveCommand::initialize() {
    SPDLOG_INFO(
        "===============================Inititializing Motion "
        "Command=============================");
    RobotIntent intent = RobotIntent{};
    intent.robot_id = this->robot->getId();
    intent.motion_command = this->motion_command;
    SPDLOG_INFO("===============================Built Intent=============================");
    this->robot->sendMoveIntent(intent);
    // this->robot->getIsKickFinished();
    SPDLOG_INFO("===============================Sent Intent=============================");
}

bool MoveCommand::is_done() { return this->robot->getIsMoveFinished(); }
}  // namespace commands