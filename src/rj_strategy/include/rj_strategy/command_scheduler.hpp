#pragma once
#include <list>

#include <queue>

#include "rj_strategy/command.hpp"

namespace commands {
/**
ROS node that schedules and runs commands
*/

class CommandScheduler {
public:
    CommandScheduler();
    bool scheduleCommand(std::shared_ptr<Command> command);
    // static void schedule(const std::shared_ptr<rj_msgs::srv::ScheduleCommand::Request> request,
    //   std::shared_ptr<rj_msgs::srv::ScheduleCommand::Response>      response);
    // rclcpp::Service<rj_msgs::srv::ScheduleCommand>::SharedPtr service;

    void tick();

private:
    std::list<std::shared_ptr<Command>> toExecute;
};

}  // namespace commands