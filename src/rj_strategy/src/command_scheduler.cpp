#include "rj_strategy/command_scheduler.hpp"

#include <chrono>

#include <spdlog/spdlog.h>

#include "rj_strategy/commands/infinite_command.hpp"
#include "rj_strategy/commands/wait_command.hpp"

using namespace std::chrono_literals;
;

namespace commands {

CommandScheduler::CommandScheduler() {
    std::shared_ptr<InfiniteCommand> infinite_command = std::make_shared<InfiniteCommand>();
    // toExecute.push_back(infinite_command);
}

bool CommandScheduler::scheduleCommand(std::shared_ptr<Command> command) {
    std::list<std::list<std::shared_ptr<Command>>::iterator> toErase;
    SPDLOG_INFO(
        "===================================================Attempt Schedule Start "
        "=========================");
    for (auto it = toExecute.begin(); it != toExecute.end(); ++it) {
        SPDLOG_INFO(
            "=================================================== Start Loop "
            "=========================");
        if (command->requirements_overlap(**it)) {
            SPDLOG_INFO(
                "===================================================Reqs overlap start "
                "=========================");
            if (!(*it)->is_interruptible()) {
                SPDLOG_INFO(
                    "===================================================Uninterruptible "
                    "=========================");
                return false;
            } else {
                SPDLOG_INFO(
                    "===================================================Interruptible "
                    "=========================");
                toErase.push_back(it);
            }
            SPDLOG_INFO(
                "===================================================End Loop "
                "=========================");
        }
    }
    SPDLOG_INFO(
        "=================================================== compared requirements "
        "=========================");
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
    SPDLOG_INFO(
        "===================================================Before initialize "
        "=========================");
    command->initialize();
    SPDLOG_INFO(
        "===================================================After Initialize "
        "=========================");
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
