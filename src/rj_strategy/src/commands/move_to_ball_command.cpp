#include "rj_strategy/commands/move_to_ball_command.hpp"

#include <spdlog/spdlog.h>

#include <rj_common/robot_intent.hpp>
#include <rj_convert/ros_convert.hpp>

namespace commands {

MoveToBallCommand::MoveToBallCommand(
    std::deque<rj_msgs::msg::WorldState::SharedPtr>& world_state_cache,
    std::shared_ptr<Robot> robot)
    : Command("move_command") {
    this->world_state_cache = &world_state_cache;
    this->robot = robot;
    addRequirements(this->robot->move_Req);
    addRequirements(this->robot->kick_Req);
}

void MoveToBallCommand::followBall() {
    // SPDLOG_INFO("==============================Starting Initialize, Cache Size
    // {}====================", world_state_cache->size()); WorldState wState =
    // rj_convert::convert_from_ros<>
    if (world_state_cache->empty()) return;
    // SPDLOG_INFO("==============================World State Not Empty====================");
    BallState ball = rj_convert::convert_from_ros(world_state_cache->front()->ball);
    // BallState* ball = nullptr;
    // SPDLOG_INFO("==============================Grabbed Ball====================");
    auto motion_command = planning::MotionCommand{
        "path_target",
        planning::LinearMotionInstant{rj_geometry::Point{ball.position.x(), ball.position.y()},
                                      rj_geometry::Point{0, 0}},
        planning::FaceTarget(), true};
    motion_command.face_option = planning::FaceBall{};
    // SPDLOG_INFO("==============================Built Motion Command====================");
    RobotIntent intent = RobotIntent{};
    intent.motion_command = motion_command;
    // SPDLOG_INFO("==============================Built Intent====================");
    if (ball.visible) {
        // SPDLOG_INFO("==============================Ball Visible====================");
        robot->sendMoveIntent(intent);
        // SPDLOG_INFO("==============================Sent Intent====================");
    } else {
        // SPDLOG_INFO("Ball Invisible");
    }
}

void MoveToBallCommand::initialize() {
    followBall();
    RobotIntent intent{};
    intent.trigger_mode = RobotIntent::TriggerMode::STAND_DOWN;
    intent.dribbler_mode = RobotIntent::DribblerMode::ON;
    intent.shoot_mode = RobotIntent::ShootMode::CHIP;
    robot->sendManipulateIntent(intent);
}

void MoveToBallCommand::execute() { followBall(); }

bool MoveToBallCommand::is_done() {
    auto yes = robot->getLatestStatus();
    if (yes.has_value()) {
        // SPDLOG_INFO("===================Optional has value=================");
        return yes.value()->has_ball_sense;
    } else {
        // SPDLOG_INFO("==================Optional has no value=================");
        return false;
    }
}

void MoveToBallCommand::exit(bool interrupted) {
    SPDLOG_INFO("========================Move to ball exit====================");
}

}  // namespace commands