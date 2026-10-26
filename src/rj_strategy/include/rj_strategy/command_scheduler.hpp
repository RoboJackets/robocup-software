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

    void tick();

private:
    std::list<std::shared_ptr<Command>> toExecute;
};

}  // namespace commands