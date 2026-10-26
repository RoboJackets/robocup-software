#include "rj_strategy/command_scheduler.hpp"

#include <chrono>

#include <spdlog/spdlog.h>

#include "rj_strategy/commands/infinite_command.hpp"
#include "rj_strategy/commands/wait_command.hpp"

using namespace std::chrono_literals;
;

namespace commands {

CommandScheduler::CommandScheduler() {}

bool CommandScheduler::scheduleCommand(std::shared_ptr<Command> command) {
    std::list<std::list<std::shared_ptr<Command>>::iterator> toErase;
    // SPDLOG_INFO("===================================================Attempt
    // std::shared_ptr<Command> command_ptr = std::make_shared<Command>(command);
    for (auto it = toExecute.begin(); it != toExecute.end(); ++it) {
        if (command->requirements_overlap(**it)) {
            if (!(*it)->is_interruptible()) {
                return false;
            } else {
                toErase.push_back(it);
            }
        }
    }

    for (auto it = toExecute.begin(); it != toExecute.end();) {
        if (toErase.empty()) {
            break;
        }

        auto eraseIt = toErase.begin();
        if ((*it).get() == (**eraseIt).get()) {  // Skeptical of this line
            toExecute.erase(it);
        } else {
            ++it;
        }
    }
    command->initialize();
    toExecute.push_back(command);

    return true;
}

void CommandScheduler::tick() {
    // SPDLOG_INFO("================Ticking Length: {} ============", toExecute.size());
    for (auto it = toExecute.begin(); it != toExecute.end();) {
        // SPDLOG_INFO("=============Looping Execute===============");
        (*it)->execute();
        if ((*it)->is_done()) {
            (*it)->exit(false);
            it = toExecute.erase(it);
        } else {
            it++;
        }
    }
}

}  // namespace commands
